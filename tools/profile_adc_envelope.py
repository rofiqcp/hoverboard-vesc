#!/usr/bin/env python3
"""Read-only ADC PWM-window envelope logger. It never sends a motor setpoint."""
from __future__ import annotations
import argparse,csv,json,time
from pathlib import Path
from vesc_dual import VescDual

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--seconds',type=float,default=10.0); ap.add_argument('--hz',type=float,default=50.0); ap.add_argument('--output',default='/home/otomasi/agv/data/esc/adc_envelope_latest.csv'); a=ap.parse_args()
    link=VescDual(a.port,115200,timeout=.8); rows=[]
    try:
        plat=link.require_platform_compatible(require_build=True); start=time.monotonic(); period=1/max(1.0,a.hz); nxt=start
        base={False:link.adc_validity(False)['invalid_count'],True:link.adc_validity(True)['invalid_count']}
        while time.monotonic()-start<a.seconds:
            for name,right in (('left',False),('right',True)):
                d=link.adc_validity(right); v=link.values(right)
                rows.append(dict(t_s=time.monotonic()-start,motor=name,duty=v.duty,erpm=v.rpm,ccr_a=d['ccr_a'],ccr_b=d['ccr_b'],ccr_c=d['ccr_c'],zero_window=d['zero_window'],min_window=d['min_window'],guard=d['guard'],adc_phase=d['adc_phase'],sector=d['sector'],window_valid=d['window_valid'],offset_valid=d['offset_valid'],driven_offset_valid=d['driven_offset_valid'],bridge_settled=d['bridge_settled'],invalid_count=d['invalid_count']))
            nxt+=period; time.sleep(max(0,nxt-time.monotonic()))
        out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True)
        with out.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else ['t_s']); w.writeheader(); w.writerows(rows)
        summary={}
        for name,right in (('left',False),('right',True)):
            rr=[x for x in rows if x['motor']==name]; final=rr[-1]['invalid_count'] if rr else base[right]
            valid=[abs(x['duty']) for x in rr if x['window_valid'] and x['offset_valid'] and x['bridge_settled']]
            summary[name]={"samples":len(rr),"invalid_delta":final-base[right],"max_observed_valid_abs_duty":max(valid,default=0.0),"min_window":min((x['zero_window'] for x in rr),default=0),"guard":rr[-1]['guard'] if rr else 0}
        Path(str(out)+'.json').write_text(json.dumps({"platform":plat,"summary":summary},indent=2,sort_keys=True)+'\n')
        print('ADC_ENVELOPE_READONLY_PASS',json.dumps(summary,sort_keys=True),flush=True)
    finally: link.close()
if __name__=='__main__': main()
