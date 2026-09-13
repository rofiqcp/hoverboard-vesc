#!/usr/bin/env python3
"""Hardware qualification that proves process-D opposes measured logical steering motion."""
import argparse,json,re,time
from pathlib import Path
from vesc_dual import VescDual
def d32(a,b): return (b-a)&0xffffffff

def stop(v):
    try: v.terminal('stop',False)
    except Exception: pass
    time.sleep(.15)

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--arm',action='store_true')
    ap.add_argument('--angles',default='-20,20,-10,10,0'); ap.add_argument('--hold',type=float,default=1.2); ap.add_argument('--hz',type=float,default=50.0)
    ap.add_argument('--max-current',type=float,default=5.0); ap.add_argument('--min-vin',type=float,default=35.0); ap.add_argument('--output',default='data/esc/position_process_d.json'); args=ap.parse_args()
    if not args.arm: raise SystemExit('ARM_REQUIRED: steering process-D qualification moves LEFT steering')
    angles=[float(x) for x in args.angles.split(',') if x.strip()]; v=VescDual(args.port,115200,timeout=1.0); rep={'angles':angles,'samples':[]}
    try:
        rep['platform']=v.require_platform_compatible(True); enc=v.terminal('encoder',False); tun=v.terminal('tuning',False)
        if not all(x in enc for x in ('cal=1','homed=1','sync=1')): raise RuntimeError(f'steering not qualified: {enc.strip()}')
        m=re.search(r'kdproc\s+([0-9.eE+-]+)',tun)
        if not m or float(m.group(1))<=0: raise RuntimeError(f'process-D disabled: {tun.strip()}')
        rep['kd_proc']=float(m.group(1)); p0=v.isr_profile(reset=True); prev=None; oppose=same=usable=0
        for target in angles:
            end=time.monotonic()+args.hold
            while time.monotonic()<end:
                v.set_steering_deg(target); v.alive(False); d=v.position_d_state(False)
                if prev is not None:
                    delta=d['position']-prev
                    if delta and abs(d['dproc_q15'])>=4:
                        usable+=1
                        if delta*d['dproc_q15']<0: oppose+=1
                        elif delta*d['dproc_q15']>0: same+=1
                prev=d['position']; rep['samples'].append(dict(target_deg=target,**d))
                if len(rep['samples'])%5==0:
                    x=v.values(False)
                    if x.fault: raise RuntimeError(f'fault={x.fault}')
                    if max(abs(x.current_motor),abs(x.iq),abs(x.id))>args.max_current: raise RuntimeError('current safety limit')
                    if x.vin<args.min_vin: raise RuntimeError(f'vin low {x.vin:.2f}')
                time.sleep(1.0/args.hz)
            stop(v)
        p=v.isr_profile(False); rep['isr']=p; rep.update(usable=usable,opposing=oppose,same_direction=same,opposing_ratio=(oppose/usable if usable else 0.0))
        dma_delta=d32(p0['dma_tc_pending_exit'],p['dma_tc_pending_exit']); rep['dma_tc_pending_exit_delta']=dma_delta
        if p['deadline_miss'] or dma_delta or p['slot_sequence_errors']: raise RuntimeError('ISR integrity fail')
        if usable<5 or same>0 or oppose/usable<.95: raise RuntimeError(f'process-D sign FAIL usable={usable} oppose={oppose} same={same}')
    finally:
        stop(v); v.close()
    out=Path(args.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(rep,indent=2,sort_keys=True)+'\n'); print('POSITION_PROCESS_D_PASS',out)
if __name__=='__main__': main()
