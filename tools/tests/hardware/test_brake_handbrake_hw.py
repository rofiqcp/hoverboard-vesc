#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,csv,struct,time
from vesc_dual import VescDual,COMM_SET_RPM,COMM_SET_CURRENT_BRAKE,COMM_SET_HANDBRAKE,COMM_ALIVE
def send(l,r,c,val,scale):
 p=bytes([c])+struct.pack('>i',round(val*scale));
 with l.io_lock:l.send(l.fwd(p) if r else p)
def alive(l,r):
 with l.io_lock:l.send(l.fwd(bytes([COMM_ALIVE])) if r else bytes([COMM_ALIVE]))
def release(l,r): send(l,r,COMM_SET_RPM,0,1);alive(l,r)
def sample(l,r,phase,t0,row):
 v=l.values(r); row.update(phase=phase,t=time.monotonic()-t0,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault);return v
def main():
 ap=argparse.ArgumentParser(description='Guarded brake and handbrake hardware test.')
 ap.add_argument('--port',default='auto')
 ap.add_argument('--arm',action='store_true',help='required to actuate the motor')
 a=ap.parse_args()
 if not a.arm: ap.error('motor actuation requires --arm')
 stamp=time.strftime('%Y%m%d_%H%M%S');out=TOOLS_DIR/'results'/'speed_pid'/stamp;out.mkdir(parents=True,exist_ok=True);fpath=out/'brake_handbrake.csv'
 L=VescDual(a.port,921600,timeout=.7);L.require_platform_compatible(True);rows=[]
 try:
  # Audit memakai konfigurasi motor aktif apa adanya; test harness tidak boleh
  # menulis MC config karena gain eksperimental dapat membuat hasil brake palsu.
  for r,_m in [(False,'left'),(True,'right')]:release(L,r)
  for r,m in [(False,'left'),(True,'right')]:
   for sign in (1,-1):
    for ba in (.2,.4):
     # accelerate with safe speed PID
     t0=time.monotonic();send(L,r,COMM_SET_RPM,450*sign,1)
     while time.monotonic()-t0<1.3:
      alive(L,r);v=sample(L,r,'drive',t0,dict(test='brake',motor=m,sign=sign,amps=ba),);rows.append(dict(test='brake',motor=m,sign=sign,amps=ba,phase='drive',t=time.monotonic()-t0,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault));
      if v.fault or abs(v.rpm)>900 or abs(v.current_motor)>1.2:break
      time.sleep(.055)
     send(L,r,COMM_SET_CURRENT_BRAKE,ba,1000);tb=time.monotonic()
     while time.monotonic()-tb<1.2:
      alive(L,r);v=L.values(r);rows.append(dict(test='brake',motor=m,sign=sign,amps=ba,phase='brake',t=time.monotonic()-tb,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault));time.sleep(.055)
     release(L,r);time.sleep(.25)
     br=[x for x in rows if x['test']=='brake' and x['motor']==m and x['sign']==sign and x['amps']==ba and x['phase']=='brake'];norm=[sign*x['erpm'] for x in br]
     print('BRAKE',m,sign,ba,'start',round(norm[0],1) if norm else None,'min',round(min(norm),1) if norm else None,'final',round(norm[-1],1) if norm else None,'peakIq',round(max(abs(x['iq']) for x in br),3) if br else None,'fault',max(x['fault'] for x in br) if br else None)
   # handbrake at standstill, no preceding drive
   for ha in (.1,.2,.4):
    release(L,r);time.sleep(.35);send(L,r,COMM_SET_HANDBRAKE,ha,1000);th=time.monotonic();hs=[]
    while time.monotonic()-th<.9:
     alive(L,r);v=L.values(r);x=dict(test='handbrake',motor=m,sign=0,amps=ha,phase='hold',t=time.monotonic()-th,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault);rows.append(x);hs.append(x);time.sleep(.055)
    release(L,r);time.sleep(.25)
    print('HANDBRAKE',m,ha,'peakERPM',max(abs(x['erpm']) for x in hs),'finalERPM',hs[-1]['erpm'],'peakIq',max(abs(x['iq']) for x in hs),'fault',max(x['fault'] for x in hs))
 finally:
  for r,m in [(False,'left'),(True,'right')]:
   try: release(L,r)
   except Exception as e: print('release warn',m,e)
  L.close()
 with fpath.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=['test','motor','sign','amps','phase','t','erpm','iq','imotor','duty','fault']);w.writeheader();w.writerows(rows)
 print('CSV',fpath)
if __name__=='__main__':main()
