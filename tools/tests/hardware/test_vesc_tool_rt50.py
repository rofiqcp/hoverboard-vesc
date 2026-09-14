#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

"""Hardware RT-data audit matching VESC Tool COMM_GET_VALUES parser semantics."""
import argparse, math, statistics, struct, sys, time
from dataclasses import dataclass
from vesc_dual import frame, PacketDecoder, parse_fw, parse_diag, open_transport, HB_MAGIC, HB_VERSION, HB_GET_DIAG

COMM_FW_VERSION=0
COMM_GET_VALUES=4
COMM_FORWARD_CAN=34
COMM_CUSTOM_APP_DATA=36
COMM_GET_VALUES_SELECTIVE=50
RIGHT_ID=2
ALL_VALUE_MASK=(1<<22)-1

@dataclass
class RtValues:
    temp_mos:float=0.0; temp_motor:float=0.0
    current_motor:float=0.0; current_in:float=0.0
    id:float=0.0; iq:float=0.0; duty:float=0.0; rpm:float=0.0; vin:float=0.0
    ah_used:float=0.0; ah_charged:float=0.0; wh_used:float=0.0; wh_charged:float=0.0
    tachometer:int=0; tachometer_abs:int=0; fault:int=0; position:float=0.0; vesc_id:int=255
    temp_mos_1:float=0.0; temp_mos_2:float=0.0; temp_mos_3:float=0.0; vd:float=0.0; vq:float=0.0
    has_timeout:bool=False; kill_sw_active:bool=False

def parse_values(payload:bytes, selective=True)->RtValues:
    expected=COMM_GET_VALUES_SELECTIVE if selective else COMM_GET_VALUES
    if not payload or payload[0]!=expected: raise ValueError(f'wrong reply id {payload[:1].hex()}')
    i=1; mask=0xffffffff
    if selective:
        if len(payload)<5: raise ValueError('short selective reply')
        mask=struct.unpack_from('>I',payload,i)[0]; i+=4
        if mask!=ALL_VALUE_MASK: raise ValueError(f'mask 0x{mask:08x}')
    v=RtValues()
    def i16(scale):
        nonlocal i
        x=struct.unpack_from('>h',payload,i)[0]/scale; i+=2; return x
    def i32(scale):
        nonlocal i
        x=struct.unpack_from('>i',payload,i)[0]/scale; i+=4; return x
    for b in range(22):
        if not(mask&(1<<b)): continue
        if b==0:v.temp_mos=i16(10)
        elif b==1:v.temp_motor=i16(10)
        elif b==2:v.current_motor=i32(100)
        elif b==3:v.current_in=i32(100)
        elif b==4:v.id=i32(100)
        elif b==5:v.iq=i32(100)
        elif b==6:v.duty=i16(1000)
        elif b==7:v.rpm=i32(1)
        elif b==8:v.vin=i16(10)
        elif b==9:v.ah_used=i32(10000)
        elif b==10:v.ah_charged=i32(10000)
        elif b==11:v.wh_used=i32(10000)
        elif b==12:v.wh_charged=i32(10000)
        elif b==13:v.tachometer=int(i32(1))
        elif b==14:v.tachometer_abs=int(i32(1))
        elif b==15:v.fault=payload[i]; i+=1
        elif b==16:v.position=i32(1000000)
        elif b==17:v.vesc_id=payload[i]; i+=1
        elif b==18:v.temp_mos_1=i16(10); v.temp_mos_2=i16(10); v.temp_mos_3=i16(10)
        elif b==19:v.vd=i32(1000)
        elif b==20:v.vq=i32(1000)
        elif b==21:
            st=payload[i]; i+=1; v.has_timeout=bool(st&1); v.kill_sw_active=bool(st&2)
    if i!=len(payload): raise ValueError(f'VESC parser consumed {i}, packet has {len(payload)} bytes')
    return v

class Link:
    def __init__(self,port,baud=115200,timeout=.06):
        self.ser=open_transport(port,baud,timeout=.001); self.timeout=timeout; self.dec=PacketDecoder()
        self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
    def close(self):
        if self.ser.is_open:
            self.ser.close()
    @staticmethod
    def fwd(p): return bytes((COMM_FORWARD_CAN,RIGHT_ID))+p
    def send(self,p,right=False):
        self.ser.write(frame(self.fwd(p) if right else p)); self.ser.flush()
    def recv(self,cmd,deadline):
        while time.monotonic()<deadline:
            d=self.ser.read(self.ser.in_waiting or 1)
            for p in self.dec.feed(d):
                if p and p[0]==cmd:return p
        raise TimeoutError(f'no COMM {cmd}')
    def request(self,p,cmd,right=False):
        self.send(p,right); return self.recv(cmd,time.monotonic()+self.timeout)
    def fw(self,right=False): return parse_fw(self.request(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION,right))
    def values(self,right=False):
        # VESC Tool realtime page uses Commands::getValues(): COMM_GET_VALUES.
        # Keep selective parsing support above for protocol regression only.
        p=bytes((COMM_GET_VALUES,))
        return parse_values(self.request(p,COMM_GET_VALUES,right),False)
    def diag(self,right=False):
        p=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_VERSION,HB_GET_DIAG))
        return parse_diag(self.request(p,COMM_CUSTOM_APP_DATA,right))

def validate(v:RtValues,vid:int):
    nums=(v.temp_mos,v.temp_motor,v.current_motor,v.current_in,v.id,v.iq,v.duty,v.rpm,v.vin,
          v.ah_used,v.ah_charged,v.wh_used,v.wh_charged,v.position,v.vd,v.vq)
    if not all(math.isfinite(x) for x in nums): raise ValueError('non-finite RT field')
    if v.vesc_id!=vid: raise ValueError(f'VESC ID {v.vesc_id}, expected {vid}')
    if not -40.0<=v.temp_mos<=150.0: raise ValueError(f'temp_mos implausible {v.temp_mos}')
    if not 5.0<=v.vin<=60.0: raise ValueError(f'Vin implausible {v.vin}')
    if not -1.05<=v.duty<=1.05: raise ValueError(f'duty {v.duty}')

def poll_one(link,right,count,hz):
    """Emulasikan polling realtime VESC Tool: QTimer mengirim tiap 20 ms secara
    asynchronous, bukan request lalu menunggu reply sebelum tick berikutnya."""
    period=1.0/hz; replies=[]; reply_times=[]; send_times=[]; fail=[]
    # Diagnostic payload besar (~100 ms pada CH341) bukan bagian RT polling.
    old_timeout=link.timeout; link.timeout=max(old_timeout,0.25)
    d0=link.diag(right)
    link.timeout=old_timeout
    req=bytes((COMM_GET_VALUES,))
    wire=frame(link.fwd(req) if right else req)
    started=time.monotonic(); next_send=started
    sent=0; drain_deadline=None
    while True:
        now=time.monotonic()
        while sent<count and now>=next_send:
            link.ser.write(wire); link.ser.flush(); send_times.append(time.monotonic())
            sent+=1; next_send=started+sent*period; now=time.monotonic()
        chunk=link.ser.read(link.ser.in_waiting or 1)
        if chunk:
            for payload in link.dec.feed(chunk):
                if payload and payload[0]==COMM_GET_VALUES:
                    try:
                        value=parse_values(payload,False); validate(value,RIGHT_ID if right else 1)
                        replies.append(value); reply_times.append(time.monotonic())
                    except Exception as exc:
                        fail.append(str(exc))
        if sent>=count:
            if len(replies)>=count: break
            if drain_deadline is None: drain_deadline=time.monotonic()+1.5
            if time.monotonic()>=drain_deadline: break
        else:
            sleep_for=next_send-time.monotonic()
            if sleep_for>0.001: time.sleep(min(0.001,sleep_for))
    finished=time.monotonic()
    link.timeout=max(old_timeout,0.25)
    d1=link.diag(right)
    link.timeout=old_timeout
    # FIFO preserves order, sehingga request/reply dapat dipasangkan berdasarkan indeks.
    paired=min(len(send_times),len(reply_times))
    lat=[(reply_times[i]-send_times[i])*1000.0 for i in range(paired)]
    send_rate=(count/((send_times[-1]-send_times[0])+period)) if len(send_times)>1 else 0.0
    reply_rate=((len(reply_times)-1)/(reply_times[-1]-reply_times[0])) if len(reply_times)>1 else 0.0
    return (replies[-1] if replies else None),lat,fail,d0,d1,finished-started,send_rate,reply_rate,len(replies)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--baud',type=int,default=115200); ap.add_argument('--hz',type=float,default=50.0); ap.add_argument('--seconds',type=float,default=5.0)
    a=ap.parse_args(); count=max(1,round(a.hz*a.seconds))
    link=Link(a.port, baud=a.baud, timeout=(0.50 if str(a.port).startswith('tcp://') else 0.06))
    try:
        print('FW:',link.fw(False),'|',link.fw(True))
        for right in (False,True):
            name='RIGHT-ID2' if right else 'LEFT-ID1'; last,lat,fail,d0,d1,elapsed,send_rate,reply_rate,replies=poll_one(link,right,count,a.hz)
            p99=sorted(lat)[max(0,int(len(lat)*.99)-1)] if lat else 0
            print(f'{name}: {replies}/{count} PASS, send={send_rate:.2f}Hz reply={reply_rate:.2f}Hz, avgLat={statistics.mean(lat):.2f}ms p99={p99:.2f}ms, qdrop {d0.rx_queue_drops}->{d1.rx_queue_drops}, crc {d0.rx_crc_errors}->{d1.rx_crc_errors}')
            if last: print(f'  Imotor={last.current_motor:.2f}A Ibatt={last.current_in:.2f}A Id={last.id:.2f}A Iq={last.iq:.2f}A Vd={last.vd:.2f}V Vq={last.vq:.2f}V ERPM={last.rpm:.0f} duty={100*last.duty:.2f}% Vin={last.vin:.1f}V T={last.temp_mos:.1f}C rotor={last.position:.2f}deg hall={d1.hall} fault={last.fault} Ah={last.ah_used:.4f}/{last.ah_charged:.4f} Wh={last.wh_used:.4f}/{last.wh_charged:.4f}')
            if fail: print('  errors:',fail[:3])
            if replies != count or send_rate < a.hz*0.98 or reply_rate < a.hz*0.98:
                print(f'  ERROR RT throughput: replies={replies}/{count} send={send_rate:.2f}Hz reply={reply_rate:.2f}Hz')
                return 1
            if fail or d1.rx_queue_drops!=d0.rx_queue_drops or d1.rx_crc_errors!=d0.rx_crc_errors: return 1
        print('VESC_TOOL_RT50_PASS')
        return 0
    finally: link.close()
if __name__=='__main__': sys.exit(main())
