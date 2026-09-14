#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,csv,struct,time
from vesc_dual import VescDual
COMM_SET_DUTY=5
COMM_SET_CURRENT=6
COMM_ALIVE=30

def send(link,right,cmd,payload=b''):
    pkt=bytes([cmd])+payload
    with link.io_lock: link.send(link.fwd(pkt) if right else pkt)

def set_duty(link,right,d): send(link,right,COMM_SET_DUTY,struct.pack('>i',int(round(d*100000.0))))
def release(link,right): send(link,right,COMM_SET_CURRENT,struct.pack('>i',0))
def alive(link,right): send(link,right,COMM_ALIVE)
def stop_all(link):
    # COMM_SET_CURRENT(0) maps to mcpwm_foc_release_motor(). Do not use
    # COMM_SET_DUTY(0): VESC defines zero duty as an actively driven zero-vector.
    for r in (False,True):
        try: release(link,r); alive(link,r)
        except Exception: pass

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('port',nargs='?',default='auto')
    ap.add_argument('--motor',choices=['left','right','both'],default='both')
    ap.add_argument('--hold',type=float,default=0.8)
    ap.add_argument('--hz',type=float,default=20.0)
    ap.add_argument('--max-current',type=float,default=15.5)
    ap.add_argument('--max-input-current',type=float,default=15.5)
    ap.add_argument('--max-erpm',type=float,default=15000.0)
    ap.add_argument('--out',default='tools/results_duty100.csv')
    ap.add_argument('--arm',action='store_true',help='required for the +/-100% duty sweep')
    a=ap.parse_args()
    if not a.arm: ap.error('motor actuation requires --arm on a mechanically safe rig/dyno')
    steps=[0.0,.05,.10,.15,.20,.25,.30,.35,.40,.45,.50,.55,.60,.65,.70,.75,.80,.85,.90,.95,1.00,.95,.90,.85,.80,.75,.70,.65,.60,.55,.50,.45,.40,.35,.30,.25,.20,.15,.10,.05,0.0,-.05,-.10,-.15,-.20,-.25,-.30,-.35,-.40,-.45,-.50,-.55,-.60,-.65,-.70,-.75,-.80,-.85,-.90,-.95,-1.00,-.95,-.90,-.85,-.80,-.75,-.70,-.65,-.60,-.55,-.50,-.45,-.40,-.35,-.30,-.25,-.20,-.15,-.10,-.05,0.0]
    motors=[]
    if a.motor in ('left','both'): motors.append((False,'left'))
    if a.motor in ('right','both'): motors.append((True,'right'))
    link=VescDual(a.port,115200,timeout=.5)
    link.require_platform_compatible(True)
    rows=[]
    try:
        stop_all(link); time.sleep(.25)
        for right,name in motors:
            d0=link.diag(right)
            if d0.fault: raise RuntimeError(f'{name}: pre-existing fault={d0.fault}')
            print(f'=== {name.upper()} START ===')
            for duty in steps:
                if duty == 0.0:
                    set_duty(link,right,0.0); alive(link,right); time.sleep(.35)
                set_duty(link,right,duty); t0=time.monotonic(); peak_i=peak_erpm=0.0
                while time.monotonic()-t0<a.hold:
                    alive(link,right); v=link.values(right); dg=link.diag(right)
                    diag_i=(dg.iq_a*dg.iq_a+dg.id_a*dg.id_a)**0.5 if dg.state==2 else 0.0; peak_i=max(peak_i,diag_i); peak_erpm=max(peak_erpm,abs(v.rpm))
                    row=dict(motor=name,cmd_duty=duty,t=time.monotonic()-t0,actual_duty=v.duty,erpm=v.rpm,imotor=v.current_motor,ibatt=v.current_in,id=v.id,iq=v.iq,vin=v.vin,fault=v.fault,trips=dg.current_trips or 0,qdrop=dg.rx_queue_drops or 0,crc=dg.rx_crc_errors or 0)
                    rows.append(row)
                    if v.fault or (dg.current_trips or 0)>0: raise RuntimeError(f'{name}: fault/trip at {duty*100:.0f}% fault={v.fault} trips={dg.current_trips}')
                    if abs(v.current_in)>a.max_input_current: raise RuntimeError(f'{name}: DC/input current guard {abs(v.current_in):.2f}A at {duty*100:.0f}%')
                    if peak_i>a.max_current: raise RuntimeError(f'{name}: RUNNING current-vector guard {peak_i:.2f}A at {duty*100:.0f}%')
                    if peak_erpm>a.max_erpm: raise RuntimeError(f'{name}: ERPM guard {peak_erpm:.0f} at {duty*100:.0f}%')
                    time.sleep(max(0.0,1.0/a.hz))
                print(f'{name} cmd={duty*100:5.1f}% actual={v.duty*100:6.2f}% erpm={v.rpm:8.1f} Im={v.current_motor:6.3f}A Ib={v.current_in:6.3f}A Id={v.id:6.3f}A Iq={v.iq:6.3f}A fault={v.fault} trips={dg.current_trips or 0}')
            release(link,right); alive(link,right); time.sleep(.5)
            print(f'=== {name.upper()} PASS +/-100% NORMALIZED (EFeru physical ceiling) ===')
    finally:
        stop_all(link); link.close()
        if rows:
            p=Path(a.out); p.parent.mkdir(parents=True,exist_ok=True)
            with p.open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            print('CSV',p)
if __name__=='__main__': main()
