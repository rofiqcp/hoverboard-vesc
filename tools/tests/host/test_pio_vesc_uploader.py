#!/usr/bin/env python3
import importlib.util,os,struct,tempfile,types
from pathlib import Path
from _test_utils import unframe
R=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('upl',R/'tools/pio_vesc_upload.py'); upl=importlib.util.module_from_spec(spec); spec.loader.exec_module(upl)


class FakeDirectF103:
    def __init__(self,fw):
        self.args=types.SimpleNamespace(transport='serial',serial_port='/dev/mock-f103',baud=115200,fault_stop_after=0)
        self.buf=bytearray(); self.mode='app'; self.updated=False; self.state=0; self.size=0; self.crc=0
        self.stage=bytearray(); self.written=0; self.writes=0; self.confirmed_reads=0; self.fw=fw
    def close(self): pass
    def reconnect_transport(self): pass
    def transact(self,p,expected,timeout=3.0):
        c=p[0]
        if c==upl.COMM_FW_VERSION:
            name=b'f103rc_bootloader\0' if self.mode=='boot' else (b'motor_left_updated\0' if self.updated else b'motor_left\0')
            return bytes((c,6,0))+name
        if c==upl.COMM_CUSTOM_APP_DATA:
            op=p[4]
            if op==upl.HB_CUSTOM_BOOT_HANDOFF:
                self.mode='boot'; return bytes((c,))+upl.HB_MAGIC+bytes((upl.HB_APP_VERSION,op,0))
            if op==upl.HB_BOOT_GET_INFO:
                return bytes((c,))+upl.HB_MAGIC+bytes((upl.HB_BOOT_VERSION,op,0))+struct.pack('>IIIHIH',upl.MAX_FW,self.state,self.size,self.crc if self.state else 0,self.written,0xffff)+bytes((0,))
            if op==upl.HB_CUSTOM_GET_FW_UPDATE_STATE:
                self.confirmed_reads+=1
                return bytes((c,))+upl.HB_MAGIC+bytes((upl.HB_APP_VERSION,op,0))+struct.pack('>IIH',self.state,self.size,self.crc)
        if c==upl.COMM_ERASE_NEW_APP:
            self.size,self.crc=struct.unpack('>IH',p[1:7]); self.state=upl.STATE_STREAM; self.stage=bytearray(b'\xff'*self.size); self.written=0
            return bytes((c,1))+struct.pack('>I',0)
        if c==upl.COMM_WRITE_NEW_APP_DATA:
            off=struct.unpack('>I',p[1:5])[0]; data=p[5:]
            if off==0:
                sz,cr=struct.unpack('>IH',data[:6]); assert (sz,cr)==(self.size,self.crc); data=data[6:]; appoff=0
            else: appoff=off-6
            assert appoff==self.written,(appoff,self.written)
            self.stage[appoff:appoff+len(data)]=data; self.written+=len(data); self.writes+=1
            return bytes((c,1))+struct.pack('>I',off)
        raise AssertionError(('transact',p[:8]))
    def write(self,raw):
        p=unframe(raw)
        if p[0]==upl.COMM_JUMP_TO_BOOTLOADER:
            assert self.state==upl.STATE_STREAM and self.written==self.size and upl.crc16(bytes(self.stage))==self.crc
            self.state=upl.STATE_CONFIRMED; self.mode='app'; self.updated=True
        else: raise AssertionError(('write',p))

fw=bytes((i*37+11)&255 for i in range(2049)); link=FakeDirectF103(fw)
with tempfile.TemporaryDirectory() as td:
    old=os.environ.get('F103_LKG_ROOT'); os.environ['F103_LKG_ROOT']=td
    try: app=upl.upload(link,fw)
    finally:
        if old is None: os.environ.pop('F103_LKG_ROOT',None)
        else: os.environ['F103_LKG_ROOT']=old
assert app=='motor_left_updated'; assert link.updated and link.writes>=6 and link.confirmed_reads>=1
print(f'PIO_VESC_UPLOADER_MOCK_PASS direct_uart=1 bytes={len(fw)} writes={link.writes} crc=0x{upl.crc16(fw):04x} confirmed=1')

# Initial port resolution must tolerate a board that is still booting after ST-Link reset.
_orig_candidates,_orig_link,_orig_fw=upl._serial_candidates,upl.Link,upl.fw_version
_probe_calls={'n':0}
class _ProbeLink:
    def __init__(self,args): self.args=args
    def close(self): pass
def _flaky_fw(_link,_timeout=2.0):
    _probe_calls['n']+=1
    if _probe_calls['n']<3: raise TimeoutError('startup')
    return 'motor_left'
try:
    upl._serial_candidates=lambda:[('/dev/mock-f103','mock USB-UART')]
    upl.Link=_ProbeLink; upl.fw_version=_flaky_fw
    _ns=types.SimpleNamespace(serial_port='auto',baud=115200,firmware='fw.bin')
    _port=upl.resolve_serial_port(_ns,1.5)
    assert _port=='/dev/mock-f103' and _probe_calls['n']>=3
finally:
    upl._serial_candidates,upl.Link,upl.fw_version=_orig_candidates,_orig_link,_orig_fw
print(f'PIO_VESC_PORT_STARTUP_GRACE_PASS attempts={_probe_calls["n"]}')
