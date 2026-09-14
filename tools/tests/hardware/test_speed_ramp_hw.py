#!/usr/bin/env python3
"""Guarded RAM-only speed-ramp qualification.

Speed gains use HB_SET_TUNING(store=False); speed_ramp uses the RAM-only terminal
setter. The original tuning and ramp are restored in finally, with no EEPROM
write during candidate evaluation.
"""
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,csv,json,re,time
from vesc_dual import VescDual
from test_speed_pid_sweep_hw import speed_candidate,run_case,release

def get_ramp(link,right):
    txt=link.terminal('tuning',right)
    m=re.search(r'ramp=([0-9.+-]+)ERPM/s',txt)
    if not m: raise RuntimeError(f'cannot parse speed ramp from: {txt.strip()}')
    return float(m.group(1))

def set_ramp_ram(link,right,ramp):
    release(link,right); time.sleep(.08)
    txt=link.terminal(f'set speed_ramp {float(ramp):.3f}',right)
    if 'OK RAM' not in txt: raise RuntimeError(f'speed_ramp={ramp} rejected: {txt.strip()}')
    got=get_ramp(link,right)
    if abs(got-float(ramp))>1.0: raise RuntimeError(f'speed_ramp readback {got} != {ramp}')

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('--port',default='auto')
    ap.add_argument('--arm',action='store_true',help='required to actuate the motor'); a=ap.parse_args()
    if not a.arm: ap.error('motor actuation requires --arm')
    stamp=time.strftime('%Y%m%d_%H%M%S'); out=TOOLS_DIR/'results'/'speed_pid'/stamp; out.mkdir(parents=True,exist_ok=True)
    rawp=out/'ramp_raw.csv'; sump=out/'ramp_summary.csv'
    configs={'left':(.009,.020,0.0),'right':(.0095,.022,0.0)}; ramps=[600,900,1200,1500]
    fields=['case','motor','kp','ki','kd','target','phase','t','erpm','iq','id','imotor','iin','duty','vq','vd','fault']
    link=VescDual(a.port,115200,timeout=.65); orig_tuning={}; orig_ramp={}; res=[]
    try:
        link.require_platform_compatible(True)
        for right,motor in ((False,'left'),(True,'right')):
            orig_tuning[motor]=link.get_tuning(right); orig_ramp[motor]=get_ramp(link,right)
        with rawp.open('w',newline='') as f:
            wr=csv.DictWriter(f,fieldnames=fields);wr.writeheader()
            for right,motor in ((False,'left'),(True,'right')):
                kp,ki,kd=configs[motor]
                candidate=speed_candidate(orig_tuning[motor],kp,ki,kd)
                link.set_tuning(candidate,right,store=False)
                for ramp in ramps:
                    set_ramp_ram(link,right,ramp)
                    for rep in (1,2):
                        for target in (300,-300):
                            cid=f'{motor}_ramp{ramp}_{target:+d}_r{rep}'
                            m=run_case(link,right,motor,kp,ki,kd,target,2.2,.8,18,1000,1.2,wr,cid)
                            m['ramp']=ramp;m['repeat']=rep;res.append(m);f.flush();print(json.dumps(m,sort_keys=True),flush=True);time.sleep(.2)
                link.set_tuning(orig_tuning[motor],right,store=False); set_ramp_ram(link,right,orig_ramp[motor]); release(link,right)
    finally:
        for right,motor in ((False,'left'),(True,'right')):
            try:
                if motor in orig_tuning: link.set_tuning(orig_tuning[motor],right,store=False)
                if motor in orig_ramp: set_ramp_ram(link,right,orig_ramp[motor])
                release(link,right)
            except Exception as e: print('RESTORE_WARN',motor,e,flush=True)
        link.close()
    keys=sorted({k for x in res for k in x})
    with sump.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(res)
    print('RAM_ONLY_SPEED_RAMP_PASS'); print('RAW',rawp);print('SUMMARY',sump)
if __name__=='__main__':main()
