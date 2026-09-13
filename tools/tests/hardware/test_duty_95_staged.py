#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import argparse,csv,time
from vesc_dual import VescDual,COMM_SET_DUTY,COMM_ALIVE
from vesc_debug import send_one,release_one

def main():
 p=argparse.ArgumentParser(description='Staged VESC duty test with fault/Hall safety logging')
 p.add_argument('port',nargs='?',default='auto'); p.add_argument('--motor',choices=['left','right'],default='left')
 p.add_argument('--stages',default='0.20,0.40,0.60,0.80,0.90,0.95'); p.add_argument('--stage-s',type=float,default=0.8)
 p.add_argument('--max-erpm',type=float,default=6000.0); p.add_argument('--max-current',type=float,default=3.0)
 p.add_argument('--hz',type=float,default=50.0); p.add_argument('--arm',action='store_true'); p.add_argument('--csv',default='')
 a=p.parse_args(); stages=[float(x) for x in a.stages.split(',') if x.strip()]
 if any(x<=0 or x>0.95 for x in stages): raise SystemExit('stages must be 0 < duty <= 0.95')
 if not a.arm:
  print('DRY RUN: add --arm only on a mechanically safe rig/dyno'); print('stages=',stages,'max_erpm=',a.max_erpm,'max_current=',a.max_current); return 0
 right=a.motor=='right'; link=VescDual(a.port,115200,timeout=max(.15,2/a.hz)); link.require_platform_compatible(True); rows=[]; reason=''
 try:
  b=link.diag(right); base=(b.hall_invalid,b.hall_sequence_rejects or 0,b.current_trips,b.phase_trip_count or 0,b.dc_trip_count or 0,b.rx_queue_drops or 0)
  last_alive=0.0
  for duty in stages:
   send_one(link,a.motor,COMM_SET_DUTY,round(duty*100000)); t0=time.monotonic()
   while time.monotonic()-t0<a.stage_s:
    now=time.monotonic()
    if now-last_alive>.2:
     with link.io_lock: link.send(link.fwd(bytes((COMM_ALIVE,))) if right else bytes((COMM_ALIVE,)))
     last_alive=now
    v=link.values(right); rows.append((now,duty,v.duty,v.rpm,v.current_motor,v.current_in,v.id,v.iq,v.vd,v.vq,v.fault))
    if v.fault: reason=f'fault={v.fault}'; break
    if abs(v.current_motor)>a.max_current: reason=f'current={v.current_motor:.2f}A'; break
    if abs(v.rpm)>a.max_erpm: reason=f'erpm={v.rpm:.0f}'; break
    time.sleep(max(0.0,1/a.hz))
   if rows:
    r=rows[-1]; print(f'cmd={100*duty:.1f}% actual={100*r[2]:.1f}% erpm={r[3]:.0f} I={r[4]:.2f}A Id={r[6]:.2f} Iq={r[7]:.2f} fault={r[10]}')
   if reason: print('ABORT',reason); break
 finally:
  try: release_one(link,a.motor)
  except Exception as e: print('release warning:',e)
  time.sleep(.25)
  try:
   d=link.diag(right); print(f'END fault={d.fault} trips={d.current_trips} phase={d.phase_trip_count} dc={d.dc_trip_count} hall_bad={d.hall_invalid} seq={d.hall_sequence_rejects} qdrop={d.rx_queue_drops}')
  except Exception as e: print('end diag warning:',e)
  link.close()
 if a.csv:
  path=Path(a.csv); path.parent.mkdir(parents=True,exist_ok=True)
  with path.open('w',newline='') as f:
   w=csv.writer(f); w.writerow(['t','cmd_duty','actual_duty','erpm','imotor','iin','id','iq','vd','vq','fault']); w.writerows(rows)
  print('CSV',path)
 return 2 if reason else 0
if __name__=='__main__': raise SystemExit(main())
