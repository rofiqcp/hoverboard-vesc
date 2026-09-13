#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
"""Hardware check for VESC COMM_SET_DETECT -> 100 Hz COMM_ROTOR_POSITION."""
import argparse, statistics, struct, time
from vesc_dual import frame, PacketDecoder, open_transport
from test_vesc_tool_rt50 import parse_values
COMM_SET_DETECT=11; COMM_ROTOR_POSITION=22; COMM_GET_VALUES=4; COMM_FORWARD_CAN=34
MODE_OBSERVER=2; MODE_PID_POS=4

def wrapped_diff(a,b): return ((a-b+180.0)%360.0)-180.0

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--seconds',type=float,default=1.2)
    a=ap.parse_args(); s=open_transport(a.port,115200,timeout=.001); dec=PacketDecoder(); s.reset_input_buffer()
    def send(p): s.write(frame(bytes(p))); s.flush()
    ok=True
    try:
        for right in (False,True):
            prefix=[COMM_FORWARD_CAN,2] if right else []
            name="RIGHT-ID2" if right else "LEFT-ID1"
            for mode,label in ((MODE_OBSERVER,"OBSERVER"),(MODE_PID_POS,"PID_POS")):
                send(prefix+[COMM_SET_DETECT,mode]); ts=[]; pos=[]; end=time.monotonic()+a.seconds
                while time.monotonic()<end:
                    for p in dec.feed(s.read(s.in_waiting or 1)):
                        if p and p[0]==COMM_ROTOR_POSITION and len(p)==5:
                            ts.append(time.monotonic()); pos.append(struct.unpack_from('>i',p,1)[0]/100000.0)
                send(prefix+[COMM_GET_VALUES]); v=None; deadline=time.monotonic()+.3
                while time.monotonic()<deadline and v is None:
                    for p in dec.feed(s.read(s.in_waiting or 1)):
                        if p and p[0]==COMM_GET_VALUES: v=parse_values(p,False); break
                send(prefix+[COMM_SET_DETECT,0]); time.sleep(.04)
                hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0.0
                periods=[(b-c)*1000 for c,b in zip(ts,ts[1:])]
                finite_range=bool(pos) and all(-0.01 <= x <= 360.01 for x in pos)
                diff=wrapped_diff(pos[-1],v.position) if pos and v and mode==MODE_PID_POS else 0.0
                good=len(pos)>=int(a.seconds*85) and 85<=hz<=115 and v is not None and v.fault==0 and finite_range
                if mode==MODE_PID_POS: good = good and abs(diff)<1.0
                print(f'{name} {label}: n={len(pos)} rate={hz:.2f}Hz avg_period={statistics.mean(periods) if periods else 0:.2f}ms stream={pos[-1] if pos else 0:.5f}deg values={v.position if v else 0:.5f}deg diff={diff:.5f} fault={v.fault if v else -1} RESULT={"PASS" if good else "FAIL"}')
                ok &= good
    finally:
        try: send([COMM_SET_DETECT,0]); send([COMM_FORWARD_CAN,2,COMM_SET_DETECT,0])
        except Exception: pass
        s.close()
    print('VESC_ROTOR_POSITION_PERIODIC_PASS' if ok else 'VESC_ROTOR_POSITION_PERIODIC_FAIL')
    return 0 if ok else 1
if __name__=='__main__': raise SystemExit(main())
