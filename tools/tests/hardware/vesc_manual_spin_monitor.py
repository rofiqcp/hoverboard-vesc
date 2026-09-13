#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import argparse, csv, time
from vesc_dual import VescDual

def sample(link, right):
    v = link.values(right)
    return dict(
        t=time.time(), motor='right' if right else 'left', erpm=v.rpm,
        id=v.id, iq=v.iq, imotor=v.current_motor, ibattery=v.current_in,
        duty=v.duty, vin=v.vin, fault=v.fault)

def main():
    ap=argparse.ArgumentParser(description='Passive/manual-spin VESC telemetry monitor')
    ap.add_argument('--port',default='auto')
    ap.add_argument('--motor',choices=['left','right','both'],default='both')
    ap.add_argument('--seconds',type=float,default=10.0)
    ap.add_argument('--hz',type=float,default=20.0)
    ap.add_argument('--csv',default='tools/results/response_lab/manual_spin.csv')
    a=ap.parse_args(); link=VescDual(a.port,115200,timeout=.65)
    rows=[]; period=1.0/max(a.hz,1.0); end=time.monotonic()+a.seconds
    motors=[False,True] if a.motor=='both' else [a.motor=='right']
    try:
        while time.monotonic()<end:
            line=[]
            for right in motors:
                r=sample(link,right); rows.append(r)
                line.append(f"{r['motor'][0].upper()}: ERPM={r['erpm']:8.1f} Id={r['id']:7.3f}A Iq={r['iq']:7.3f}A Im={r['imotor']:7.3f}A Ib={r['ibattery']:7.3f}A duty={100*r['duty']:6.2f}% fault={r['fault']}")
            print(' | '.join(line), flush=True)
            time.sleep(period)
    finally:
        link.close()
    p=Path(a.csv); p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else ['t']); w.writeheader(); w.writerows(rows)
    print(p)

if __name__=='__main__': main()
