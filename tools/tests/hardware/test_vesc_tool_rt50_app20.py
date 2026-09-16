#!/usr/bin/env python3
"""Measure real VESC Tool-style RT motor 50 Hz + RT App Data 20 Hz at the wire."""
import argparse, statistics, struct, sys, time
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == "tools")
if str(TOOLS_DIR) not in sys.path: sys.path.insert(0, str(TOOLS_DIR))
from vesc_dual import (open_transport, PacketDecoder, frame, parse_fw, parse_diag,
                       COMM_FW_VERSION, COMM_GET_VALUES, COMM_FORWARD_CAN,
                       COMM_GET_DECODED_PPM, COMM_GET_DECODED_ADC, COMM_GET_DECODED_CHUK,
                       COMM_ALIVE, COMM_SET_CURRENT, COMM_CUSTOM_APP_DATA, HB_MAGIC, HB_VERSION, HB_GET_DIAG)
from test_vesc_tool_rt50 import parse_values, validate
RIGHT_ID=2

def fwd(p:bytes)->bytes: return bytes((COMM_FORWARD_CAN,RIGHT_ID))+p

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('port'); ap.add_argument('--baud',type=int,default=921600)
    ap.add_argument('--seconds',type=float,default=5.0); ap.add_argument('--rt-hz',type=float,default=50.0); ap.add_argument('--app-hz',type=float,default=20.0)
    ap.add_argument('--standby',action='store_true',help='put both motors in driven-zero standby before rate test')
    a=ap.parse_args(); ser=open_transport(a.port,a.baud,timeout=.001); dec=PacketDecoder()
    def request_one(payload, expected, timeout=.5):
        ser.write(frame(payload)); ser.flush(); end=time.monotonic()+timeout
        while time.monotonic()<end:
            for q in dec.feed(ser.read(ser.in_waiting or 1)):
                if q and q[0]==expected:return q
        raise TimeoutError(expected)
    def diag(right=False):
        q=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_VERSION,HB_GET_DIAG))
        return parse_diag(request_one(fwd(q) if right else q,COMM_CUSTOM_APP_DATA,.8))
    try:
        print('FWL',parse_fw(request_one(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION)))
        print('FWR',parse_fw(request_one(fwd(bytes((COMM_FW_VERSION,))),COMM_FW_VERSION)))
        if a.standby:
            zero=bytes((COMM_SET_CURRENT,))+struct.pack('>i',0)
            ser.write(frame(zero)); ser.write(frame(fwd(zero)))
            ser.write(frame(bytes((COMM_ALIVE,)))); ser.write(frame(fwd(bytes((COMM_ALIVE,))))); ser.flush()
            time.sleep(.08)
            print('STANDBY_ARMED')
        d0=(diag(False),diag(True)); ser.reset_input_buffer(); dec=PacketDecoder()
        counts={'L':0,'R':0,'ppm':0,'adc':0,'chuk':0}; times={k:[] for k in counts}; bad=[]; last_motor={'L':None,'R':None}
        rt_req=bytes((COMM_GET_VALUES,)); ppm=bytes((COMM_GET_DECODED_PPM,)); adc=bytes((COMM_GET_DECODED_ADC,)); chuk=bytes((COMM_GET_DECODED_CHUK,)); alive=bytes((COMM_ALIVE,))
        t0=time.monotonic(); end=t0+a.seconds; next_rt=t0; next_app=t0; next_alive=t0
        rt_sent=0; app_cycles=0
        while time.monotonic()<end:
            now=time.monotonic()
            while now>=next_rt and next_rt<end:
                ser.write(frame(rt_req)); ser.write(frame(fwd(rt_req))); rt_sent+=1; next_rt+=1.0/a.rt_hz
                now=time.monotonic()
            while now>=next_app and next_app<end:
                # Same three commands queued by VESC Tool mPollAppTimer.
                ser.write(frame(ppm)); ser.write(frame(adc)); ser.write(frame(chuk)); app_cycles+=1; next_app+=1.0/a.app_hz
                now=time.monotonic()
            if now>=next_alive:
                ser.write(frame(alive)); ser.write(frame(fwd(alive))); next_alive+=.2
            ser.flush()
            chunk=ser.read(ser.in_waiting or 1)
            for q in dec.feed(chunk):
                ts=time.monotonic()
                try:
                    if not q: continue
                    if q[0]==COMM_GET_VALUES:
                        v=parse_values(q,False); validate(v,v.vesc_id)
                        key='L' if v.vesc_id==1 else 'R' if v.vesc_id==2 else None
                        if key is None: raise ValueError(f'bad id {v.vesc_id}')
                        last_motor[key]=v
                    elif q[0]==COMM_GET_DECODED_PPM:
                        if len(q)!=9: raise ValueError(f'PPM len {len(q)}')
                        key='ppm'
                    elif q[0]==COMM_GET_DECODED_ADC:
                        if len(q)!=17: raise ValueError(f'ADC len {len(q)}')
                        vals=[struct.unpack_from('>i',q,1+4*i)[0]/1e6 for i in range(4)]
                        if not all(-1000.0<x<1000.0 for x in vals): raise ValueError('ADC nonfinite/range')
                        key='adc'
                    elif q[0]==COMM_GET_DECODED_CHUK:
                        if len(q)!=5: raise ValueError(f'CHUK len {len(q)}')
                        key='chuk'
                    else: continue
                    counts[key]+=1; times[key].append(ts)
                except Exception as e: bad.append(str(e))
            time.sleep(.0005)
        drain=time.monotonic()+1.2
        while time.monotonic()<drain and (counts['L']<rt_sent or counts['R']<rt_sent or counts['adc']<app_cycles or counts['ppm']<app_cycles or counts['chuk']<app_cycles):
            for q in dec.feed(ser.read(ser.in_waiting or 1)):
                ts=time.monotonic()
                try:
                    if q and q[0]==COMM_GET_VALUES:
                        v=parse_values(q,False); key='L' if v.vesc_id==1 else 'R' if v.vesc_id==2 else None
                        if key: last_motor[key]=v
                    elif q and q[0]==COMM_GET_DECODED_PPM:key='ppm'
                    elif q and q[0]==COMM_GET_DECODED_ADC:key='adc'
                    elif q and q[0]==COMM_GET_DECODED_CHUK:key='chuk'
                    else: continue
                    if key: counts[key]+=1; times[key].append(ts)
                except Exception as e: bad.append(str(e))
        def rate(k,period):
            t=times[k]; return len(t)/((t[-1]-t[0])+period) if len(t)>1 else 0.0
        rates={'L':rate('L',1/a.rt_hz),'R':rate('R',1/a.rt_hz),'ppm':rate('ppm',1/a.app_hz),'adc':rate('adc',1/a.app_hz),'chuk':rate('chuk',1/a.app_hz)}
        d1=(diag(False),diag(True))
        print('SENT rt_cycles',rt_sent,'app_cycles',app_cycles)
        print('COUNTS',counts); print('RATES', {k:round(v,2) for k,v in rates.items()}); print('ERRORS',bad[:5])
        print('DROP L',d0[0].rx_queue_drops,'->',d1[0].rx_queue_drops,'CRC',d0[0].rx_crc_errors,'->',d1[0].rx_crc_errors)
        print('DROP R',d0[1].rx_queue_drops,'->',d1[1].rx_queue_drops,'CRC',d0[1].rx_crc_errors,'->',d1[1].rx_crc_errors)
        standby_ok=True
        if a.standby:
            for key,d in (('L',d1[0]),('R',d1[1])):
                v=last_motor[key]
                print('STANDBY',key,'mode',d.control_mode,'state',d.state,'fault',d.fault,
                      'driven',d.driven_offset_valid,'cal',d.driven_offset_calibrating,
                      'Imotor/Id/Iq',None if v is None else (v.current_motor,v.id,v.iq),
                      'isr',d.foc_isr_cycles,'gap',d.process_gap_max_ms)
                standby_ok=standby_ok and d.control_mode==2 and d.state==2 and d.fault==0 and d.driven_offset_valid and not d.driven_offset_calibrating and v is not None and abs(v.current_motor)<=0.02 and abs(v.id)<=0.02 and abs(v.iq)<=0.02
        ok=(not bad and standby_ok and counts['L']==rt_sent and counts['R']==rt_sent and all(counts[k]==app_cycles for k in ('ppm','adc','chuk')) and rates['L']>=a.rt_hz*.98 and rates['R']>=a.rt_hz*.98 and all(rates[k]>=a.app_hz*.98 for k in ('ppm','adc','chuk')) and d1[0].rx_queue_drops==d0[0].rx_queue_drops and d1[0].rx_crc_errors==d0[0].rx_crc_errors and d1[1].rx_queue_drops==d0[1].rx_queue_drops and d1[1].rx_crc_errors==d0[1].rx_crc_errors)
        print('VESC_RT50_APP20_PASS' if ok else 'VESC_RT50_APP20_FAIL')
        return 0 if ok else 1
    finally: ser.close()
if __name__=='__main__': raise SystemExit(main())
