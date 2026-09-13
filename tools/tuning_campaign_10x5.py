#!/usr/bin/env python3
"""Qualification-only 10x5 system campaign. Never changes or stores PID/FOC gains."""
from __future__ import annotations
import argparse,csv,json,math,statistics,time
from pathlib import Path
from vesc_dual import VescDual
def d32(a,b): return (b-a)&0xffffffff
CASES=[(180.0,1000),(180.0,-1000),(120.0,2000),(240.0,-2000),(60.0,4000),(300.0,-4000),(0.0,6000),(360.0,-6000),(90.0,8000),(270.0,-8000)]

def stop(link):
    for r in (False,True):
        try: link.terminal('stop',r)
        except Exception: pass
    time.sleep(.15)

def run_case(link,idx,rep,pos,rpm,hold,dt):
    stop(link); p0=link.isr_profile(reset=True); t0=time.monotonic(); rows=[]
    for _ in range(max(5,int(.35/dt))):
        link.set_pos_one(180.0,False); link.set_rpm_one(0,True); time.sleep(dt)
    start=time.monotonic()
    while time.monotonic()-start<hold:
        link.set_pos_one(pos,False); link.set_rpm_one(rpm,True)
        vl=link.values(False); vr=link.values(True); pl=link.isr_profile(False)
        dma_delta=d32(p0['dma_tc_pending_exit'],pl['dma_tc_pending_exit'])
        rows.append(dict(t=time.monotonic()-t0,pos=vl.position,pos_target=pos,rpm=vr.rpm,rpm_target=rpm,left_i=vl.current_motor,right_i=vr.current_motor,left_iq=vl.iq,right_iq=vr.iq,left_fault=vl.fault,right_fault=vr.fault,isr_max=pl['total_max'],deadline_miss=pl['deadline_miss'],dma_pending=dma_delta))
        if vl.fault or vr.fault or pl['deadline_miss'] or dma_delta: break
        time.sleep(dt)
    stop(link)
    tail=rows[-max(3,min(10,len(rows))):]
    pe=statistics.mean([abs(x['pos_target']-x['pos']) for x in tail]) if tail else 999.0
    re=statistics.mean([abs(x['rpm_target']-x['rpm']) for x in tail]) if tail else 99999.0
    peak=max([max(abs(x['left_i']),abs(x['right_i'])) for x in rows],default=999.0)
    ok=bool(rows) and all(not x['left_fault'] and not x['right_fault'] and not x['deadline_miss'] and not x['dma_pending'] and x['isr_max']<3000 for x in rows) and pe<=8.0 and re<=max(150.0,.08*abs(rpm))
    return {"case":idx,"repeat":rep,"pos_target":pos,"rpm_target":rpm,"pass":ok,"steady_pos_abs_error":pe,"steady_rpm_abs_error":re,"peak_motor_current":peak,"samples":rows}

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--arm',action='store_true'); ap.add_argument('--repeat',type=int,default=5); ap.add_argument('--hold',type=float,default=2.0); ap.add_argument('--dt',type=float,default=.04); ap.add_argument('--output',default='/home/otomasi/agv/data/esc/qualification_10x5_latest.json'); a=ap.parse_args()
    if not a.arm: raise SystemExit('ARM_REQUIRED: 10x5 moves both motors')
    if a.repeat!=5: raise SystemExit('--repeat must remain 5 for the production 10x5 gate')
    link=VescDual(a.port,115200,timeout=1.0); result={"runs":[]}
    try:
        result['platform']=link.require_platform_compatible(require_build=True); before={"left":link.get_tuning(False).physical,"right":link.get_tuning(True).physical}
        result['locked_tuning']=before
        for i,(pos,rpm) in enumerate(CASES,1):
            for rep in range(1,6):
                r=run_case(link,i,rep,pos,rpm,a.hold,a.dt); result['runs'].append(r)
                print('QUAL10X5',i,rep,'PASS' if r['pass'] else 'FAIL',f"poserr={r['steady_pos_abs_error']:.2f}",f"rpmerr={r['steady_rpm_abs_error']:.1f}",flush=True)
        after={"left":link.get_tuning(False).physical,"right":link.get_tuning(True).physical}; result['tuning_unchanged']=(after==before); result['pass']=all(r['pass'] for r in result['runs']) and result['tuning_unchanged'] and len(result['runs'])==50
        out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
        if not result['pass']: raise SystemExit(f'QUALIFICATION_10X5_FAIL output={out}')
        print(f'QUALIFICATION_10X5_PASS output={out}',flush=True)
    finally: stop(link); link.close()
if __name__=='__main__': main()
