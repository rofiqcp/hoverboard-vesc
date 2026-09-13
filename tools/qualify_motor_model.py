#!/usr/bin/env python3
"""Repeatable non-persistent R/L/flux qualification for both hoverboard motors."""
from __future__ import annotations
import argparse,json,statistics,time
from pathlib import Path
from vesc_dual import VescDual

def stat(v):
    m=statistics.mean(v); med=statistics.median(v); sd=statistics.pstdev(v) if len(v)>1 else 0.0
    return {"mean":m,"median":med,"stdev":sd,"cv":(sd/abs(m) if abs(m)>1e-15 else 999.0),"min":min(v),"max":max(v)}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('port',nargs='?',default='auto')
    ap.add_argument('--arm',action='store_true',help='required: measurements energize/spin the selected motor')
    ap.add_argument('--motor',choices=('left','right','both'),default='both')
    ap.add_argument('--repeat',type=int,default=5)
    ap.add_argument('--flux-current',type=float,default=1.0)
    ap.add_argument('--flux-ramp-erpm-s',type=float,default=1800.0)
    ap.add_argument('--max-cv-r',type=float,default=0.08); ap.add_argument('--max-cv-l',type=float,default=0.10); ap.add_argument('--max-cv-flux',type=float,default=0.10)
    ap.add_argument('--output',default='/home/otomasi/agv/data/esc/model_qualification_latest.json')
    a=ap.parse_args()
    if not a.arm: raise SystemExit('ARM_REQUIRED: rerun with --arm only when motors are safe to move')
    if not 3<=a.repeat<=12: raise SystemExit('--repeat must be 3..12')
    link=VescDual(a.port,115200,timeout=1.0); result={"motors":{},"repeat":a.repeat}
    try:
        result['platform']=link.require_platform_compatible(require_build=True)
        choices=[('left',False),('right',True)] if a.motor=='both' else [(a.motor,a.motor=='right')]
        for name,right in choices:
            rl=[]
            for i in range(a.repeat):
                x=link.measure_r_l(right); rl.append(x); print(f'MODEL_RL {name} {i+1}/{a.repeat} {x}',flush=True); time.sleep(.15)
            sr=stat([x['r_ohm'] for x in rl]); sl=stat([x['l_h'] for x in rl]); sld=stat([x['ld_lq_h'] for x in rl])
            r,l=sr['median'],sl['median']; flux=[]
            for i in range(a.repeat):
                # Wire command retains a duty field for VESC compatibility, but this F103 worker intentionally
                # identifies at a bounded 600-ERPM operating point. Keep the protocol placeholder fixed.
                f=link.measure_flux_openloop(a.flux_current,a.flux_ramp_erpm_s,0.35,r,l,right)
                flux.append(f); print(f'MODEL_FLUX {name} {i+1}/{a.repeat} {f:.9g}',flush=True); time.sleep(.2)
            sf=stat(flux)
            passed=(0.0<r<=2.0 and 5e-6<=l<=0.02 and 1e-4<=sf['median']<=1.0 and sr['cv']<=a.max_cv_r and sl['cv']<=a.max_cv_l and sf['cv']<=a.max_cv_flux)
            result['motors'][name]={"r_ohm":sr,"l_h":sl,"ld_lq_h":sld,"flux_wb":sf,"raw_rl":rl,"raw_flux":flux,
              "pass":passed,"force_dq_equal":True,"reason_dq":"standalone identification does not prove repeatable Ld/Lq saliency"}
            print(f'MODEL_{"PASS" if passed else "FAIL"} {name} Rcv={sr["cv"]:.4f} Lcv={sl["cv"]:.4f} fluxcv={sf["cv"]:.4f}',flush=True)
        result['pass']=all(x['pass'] for x in result['motors'].values())
        out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
        if not result['pass']: raise SystemExit(f'MOTOR_MODEL_QUALIFICATION_FAIL output={out}')
        print(f'MOTOR_MODEL_QUALIFICATION_PASS output={out}',flush=True)
    finally:
        try: link.shutdown_safe(False); link.shutdown_safe(True)
        except Exception: pass
        link.close()
if __name__=='__main__': main()
