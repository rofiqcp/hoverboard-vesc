#!/usr/bin/env python3
import importlib.util, pathlib, struct, sys
p=next(x for x in pathlib.Path(__file__).resolve().parents if x.name=='tools')/'vesc_dual.py'
spec=importlib.util.spec_from_file_location('vd',p);vd=importlib.util.module_from_spec(spec);sys.modules['vd']=vd;spec.loader.exec_module(vd)
pl=bytes([vd.COMM_SET_CURRENT])+struct.pack('>i',2500)
f=vd.frame(pl)
assert f[0]==2 and f[1]==len(pl) and f[-1]==3
assert ((f[-3]<<8)|f[-2])==vd.crc16(pl)
d=vd.PacketDecoder(); out=[]
for chunk in (f[:2],f[2:5],f[5:]): out.extend(d.feed(chunk))
assert out==[pl]
fw=vd.VescDual.fwd(bytes([vd.COMM_FW_VERSION]))
assert fw==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_FW_VERSION])
pos_payload=bytes([vd.COMM_SET_POS])+struct.pack('>i',30000000)
assert vd.VescDual.fwd(pos_payload)==bytes([vd.COMM_FORWARD_CAN,2])+pos_payload
hall_payload=bytes([vd.COMM_DETECT_HALL_FOC])+struct.pack('>i',1000)
assert vd.COMM_DETECT_HALL_FOC==28 and vd.VescDual.fwd(hall_payload)==bytes([vd.COMM_FORWARD_CAN,2])+hall_payload
mask=vd.VALUE_MASK
payload=bytearray([vd.COMM_GET_VALUES_SELECTIVE])+bytearray(struct.pack('>I',mask))
# bits 2,3,4,5
for x in (150,-20,200,0): payload += struct.pack('>i',x)
# bit6 duty, bit7 rpm, bit8 vin
payload += struct.pack('>h',123)+struct.pack('>i',321)+struct.pack('>h',481)
# bit15 fault, bit16 position, bit17 id
payload += bytes([0])+struct.pack('>i',45000000)+bytes([2])
# bit19 vd, bit20 vq
payload += struct.pack('>i',1200)+struct.pack('>i',-5000)
v=vd.parse_selective(bytes(payload))
assert abs(v.current_motor-1.5)<1e-9 and v.vesc_id==2 and v.rpm==321 and abs(v.vq+5)<1e-9 and abs(v.position-45.0)<1e-9

# Exact VESC Tool commands.cpp packet-builder parity for motor commands.
import threading
class FakeSerial:
    def __init__(self): self.frames=[]
    def write(self,b): self.frames.append(bytes(b)); return len(b)
    def flush(self): pass
def decoded_payload(fr):
    dec=vd.PacketDecoder(); out=dec.feed(fr); assert len(out)==1; return out[0]
link=vd.VescDual.__new__(vd.VescDual); link.ser=FakeSerial(); link.io_lock=threading.Lock(); link.timeout=0.1; link.dec=vd.PacketDecoder()
link.set_duty(-0.125,0.25); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_DUTY])+struct.pack('>i',-12500); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_DUTY])+struct.pack('>i',25000)
link.set_current(-2.5,3.25); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_CURRENT])+struct.pack('>i',-2500); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_CURRENT])+struct.pack('>i',3250)
link.brake(-1.25,2.0); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_CURRENT_BRAKE])+struct.pack('>i',-1250); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_CURRENT_BRAKE])+struct.pack('>i',2000)
link.handbrake(-0.75,1.5); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_HANDBRAKE])+struct.pack('>i',-750); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_HANDBRAKE])+struct.pack('>i',1500)
link.set_rpm(-1234,5678); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_RPM])+struct.pack('>i',-1234); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_RPM])+struct.pack('>i',5678)
link.set_pos(-15.0,390.0); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_POS])+struct.pack('>i',-15000000); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_POS])+struct.pack('>i',390000000)
link.set_current_rel(-1.25,1.10); p0,p1=map(decoded_payload,link.ser.frames[-2:]); assert p0==bytes([vd.COMM_SET_CURRENT_REL])+struct.pack('>i',-125000); assert p1==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_CURRENT_REL])+struct.pack('>i',110000)
mt=vd.McconfTemp(0.5,0.6,-1000,2000,-0.1,0.9,-500,600,-12,13,30,1.0,0.1)
link.set_mcconf_temp(mt,store=True,forward=False,ack=False,divide_by_controllers=True,right=False)
pm=decoded_payload(link.ser.frames[-1]); assert pm[:5]==bytes([vd.COMM_SET_MCCONF_TEMP,1,0,0,1]) and len(pm)==37
vals=struct.unpack('>8f',pm[5:]); exp=(0.5,0.6,-1000.0,2000.0,-0.1,0.9,-500.0,600.0); assert all(abs(a-b)<1e-5 for a,b in zip(vals,exp))

# Remaining VESC Tool packet builders implemented by the firmware.
link.alive(False); assert decoded_payload(link.ser.frames[-1])==bytes([vd.COMM_ALIVE])
link.alive(True); assert decoded_payload(link.ser.frames[-1])==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_ALIVE])
link.set_detect(2,True); assert decoded_payload(link.ser.frames[-1])==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_SET_DETECT,2])
link.disable_app_output(-1,forward_can=True,right=False); pa=decoded_payload(link.ser.frames[-1]); assert pa==bytes([vd.COMM_APP_DISABLE_OUTPUT,1])+struct.pack('>i',-1)
# detect_encoder request/reply framing. Override transact only for this unit case.
cap=[]
def fake_transact(req,expected,timeout=None):
    cap.append((req,expected,timeout)); return bytes([vd.COMM_DETECT_ENCODER])+struct.pack('>iiB',17250000,15000000,1)
link.transact=fake_transact
off,ratio,inv=link.detect_encoder(1.25,False); assert cap[-1][0]==bytes([vd.COMM_DETECT_ENCODER])+struct.pack('>i',1250) and cap[-1][1]==vd.COMM_DETECT_ENCODER
assert abs(off-17.25)<1e-9 and abs(ratio-15.0)<1e-9 and inv

# Stage-2 communication health parser ABI.
health_link=vd.VescDual.__new__(vd.VescDual); health_link.timeout=0.1
health_vals=tuple(range(100,120))
health_payload=bytes([vd.COMM_CUSTOM_APP_DATA,0x48,0x42,vd.HB_VERSION,vd.HB_GET_COMMS_HEALTH,0])+struct.pack(">20I",*health_vals)
health_link.custom_transact=lambda *a,**k: health_payload
h=health_link.comms_health()
assert h["rx_ok"]==100 and h["tx_queue_highwater"]==108 and h["uart_forced_recovery"]==116 and h["main_tail_max_cycles"]==119

# Auto transport contract: direct USB-UART candidates are positively probed.
orig_candidates,orig_probe=vd._direct_serial_candidates,vd._probe_f103_uart
try:
    vd._direct_serial_candidates=lambda:[('/dev/mock0','other USB'),('/dev/mock1','F103 USB-UART')]
    calls=[]
    class FakeSerial:
        pass
    chosen=FakeSerial()
    def fake_probe(dev,baud,timeout):
        calls.append(dev)
        if dev=='/dev/mock0': raise TimeoutError('not F103')
        return chosen,'motor_left'
    vd._probe_f103_uart=fake_probe
    tr=vd.open_transport('auto'); assert tr is chosen and calls==['/dev/mock0','/dev/mock1']
finally:
    vd._direct_serial_candidates,vd._probe_f103_uart=orig_candidates,orig_probe

# Explicit device paths are also positively probed; merely naming /dev/ttyUSBx is not trust.
orig_probe=vd._probe_f103_uart
try:
    calls=[]
    chosen=FakeSerial()
    def explicit_probe(dev,baud,timeout):
        calls.append((dev,baud)); return chosen,'motor_left'
    vd._probe_f103_uart=explicit_probe
    tr=vd.open_transport('/dev/mock-explicit',vd.DEFAULT_BAUD); assert tr is chosen
    assert calls==[('/dev/mock-explicit',115200)]
finally:
    vd._probe_f103_uart=orig_probe

print(f'PY_VESC_DUAL_PACKET_PASS crc=0x{vd.crc16(pl):04x} forward_can_id={fw[1]} values_id={v.vesc_id} pos={v.position:.1f} tool_builders=exact auto_direct_uart=1')
