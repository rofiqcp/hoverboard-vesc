#!/usr/bin/env python3
"""Non-persistent A/B qualification for VESC decoupling and speed-source modes."""
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,json,math,re,statistics,time
from vesc_common import u32_delta
from vesc_dual import VescDual

def parse_cfg(txt):
    pats={k:rf'{k}=([0-9.+-]+)' for k in ('R','L','flux','dec','speed_src')}
    out={}
    for k,p in pats.items():
        m=re.search(p,txt); out[k]=float(m.group(1)) if m else None
    if out['dec'] is not None: out['dec']=int(out['dec'])
    if out['speed_src'] is not None: out['speed_src']=int(out['speed_src'])
    return out

def stop(v,right):
    try: v.terminal('stop',right)
    except Exception: pass
    time.sleep(.15)

def set_ram(v,right,key,val):
    stop(v,right)
    r=v.terminal(f'set {key} {val}',right)
    if 'OK' not in r: raise RuntimeError(f'{key}={val} rejected: {r.strip()}')
    got=parse_cfg(v.terminal('config',right))
    name='dec' if key=='decoupling' else 'speed_src'
    if got[name] != int(val): raise RuntimeError(f'{key} verify failed: {got}')

def run_target(v,right,target,hold,hz,max_current,min_vin):
    p0=v.isr_profile(reset=True); rows=[]; dt=1.0/hz; end=time.monotonic()+hold
    while time.monotonic()<end:
        v.set_rpm_one(int(target),right); x=v.values(right)
        rows.append((x.rpm,x.current_motor,x.iq,x.id,x.duty,x.vin,x.fault))
        if x.fault: raise RuntimeError(f'fault={x.fault}')
        if max(abs(x.current_motor),abs(x.iq),abs(x.id))>max_current: raise RuntimeError('current safety limit')
        if x.vin<min_vin: raise RuntimeError(f'vin low {x.vin:.2f}')
        time.sleep(dt)
    stop(v,right); prof=v.isr_profile(False)
    dma_delta=u32_delta(p0['dma_tc_pending_exit'],prof['dma_tc_pending_exit'])
    if prof['deadline_miss'] or dma_delta or prof['slot_sequence_errors']:
        raise RuntimeError(f'ISR integrity fail dma_delta={dma_delta} profile={prof}')
    actual=[r[0] for r in rows]; err=[a-target for a in actual]
    return dict(target=target,mean=statistics.mean(actual),rmse=math.sqrt(statistics.mean([e*e for e in err])),
                steady_error=target-statistics.mean(actual[-max(3,len(actual)//4):]),
                peak_current=max(max(abs(r[1]),abs(r[2]),abs(r[3])) for r in rows),
                peak_duty=max(abs(r[4]) for r in rows),isr_max=prof['total_max'])

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--arm',action='store_true')
    ap.add_argument('--kind',choices=('speed','decoupling','all'),default='all')
    ap.add_argument('--motor',choices=('left','right'),default='right')
    ap.add_argument('--targets',default='-100,100,-200,200,-500,500,-1000,1000,-2000,2000,-4000,4000,-6000,6000,-8000,8000')
    ap.add_argument('--hold',type=float,default=.8); ap.add_argument('--hz',type=float,default=25.0)
    ap.add_argument('--max-current',type=float,default=7.0); ap.add_argument('--min-vin',type=float,default=35.0)
    ap.add_argument('--output',default='data/esc/control_mode_qualification.json'); args=ap.parse_args()
    if not args.arm: raise SystemExit('ARM_REQUIRED: A/B qualification moves the selected motor')
    right=args.motor=='right'; targets=[int(x) for x in args.targets.split(',') if x.strip()]
    v=VescDual(args.port,115200,timeout=1.0); report={'motor':args.motor,'targets':targets,'results':{}}
    try:
        report['platform']=v.require_platform_compatible(True); cfg0=parse_cfg(v.terminal('config',right)); report['initial']=cfg0
        if cfg0['dec'] is None or cfg0['speed_src'] is None: raise RuntimeError('firmware lacks RAM mode observability')
        if args.kind in ('speed','all'):
            rr={}
            for src in (0,1):
                set_ram(v,right,'speed_src',src); cases=[]
                for t in targets: cases.append(run_target(v,right,t,args.hold,args.hz,args.max_current,args.min_vin))
                rr[str(src)]=dict(cases=cases,score=statistics.mean([c['rmse']/max(100,abs(c['target'])) for c in cases]))
            report['results']['speed_source']=rr
        if args.kind in ('decoupling','all'):
            rr={}; r_ok=(cfg0['R'] or 0)>0; l_ok=(cfg0['L'] or 0)>0; f_ok=(cfg0['flux'] or 0)>0
            for mode in range(4):
                if (mode in (1,3) and not l_ok) or (mode in (2,3) and not f_ok):
                    rr[str(mode)]={'status':'REJECT_MODEL'}; continue
                set_ram(v,right,'decoupling',mode); cases=[]
                for t in targets: cases.append(run_target(v,right,t,args.hold,args.hz,args.max_current,args.min_vin))
                rr[str(mode)]=dict(status='PASS',cases=cases,score=statistics.mean([c['rmse']/max(100,abs(c['target'])) for c in cases]))
            report['results']['decoupling']=rr
    finally:
        try:
            stop(v,right)
            if 'initial' in report:
                set_ram(v,right,'decoupling',report['initial']['dec']); set_ram(v,right,'speed_src',report['initial']['speed_src'])
                report['restored']=parse_cfg(v.terminal('config',right))
        finally: v.close()
    out=Path(args.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    print('CONTROL_MODE_QUALIFICATION_PASS',out)
if __name__=='__main__': main()
