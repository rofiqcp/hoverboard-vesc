#!/usr/bin/env python3
import importlib.util,os,struct,tempfile,types
from pathlib import Path
R=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('upl',R/'tools/pio_vesc_upload.py'); upl=importlib.util.module_from_spec(spec); spec.loader.exec_module(upl)
def unframe(raw):
    h=2 if raw[0]==2 else 3; n=raw[1] if h==2 else (raw[1]<<8)|raw[2]; return raw[h:h+n]
class Fake:
    def __init__(self,old):
        self.args=types.SimpleNamespace(transport='serial',serial_port='/dev/mock',baud=115200,fault_stop_after=0)
        self.buf=bytearray(); self.mode='app'; self.active=old; self.state=upl.STATE_CONFIRMED; self.size=len(old); self.crc=upl.crc16(old); self.written=0; self.stage=bytearray(); self.installs=0
    def close(self): pass
    def reconnect_transport(self): pass
    def transact(self,p,expected,timeout=3):
        c=p[0]
        if c==upl.COMM_FW_VERSION:
            return bytes((c,6,0))+(b'f103rc_bootloader\0' if self.mode=='boot' else b'motor_left\0')
        if c==upl.COMM_CUSTOM_APP_DATA:
            op=p[4]
            if op==upl.HB_CUSTOM_BOOT_HANDOFF:
                self.mode='boot'; return bytes((c,))+upl.HB_MAGIC+bytes((2,op,0))
            if op==upl.HB_BOOT_GET_INFO:
                return bytes((c,))+upl.HB_MAGIC+bytes((1,op,0))+struct.pack('>IIIHIH',upl.MAX_FW,self.state,self.size,self.crc,self.written,0xffff)+bytes((1,))
            if op==upl.HB_BOOT_READ_APP:
                off,want=struct.unpack('>IH',p[5:11]); data=self.active[off:off+want]
                return bytes((c,))+upl.HB_MAGIC+bytes((1,op,0))+struct.pack('>IH',off,len(data))+data
            if op==upl.HB_CUSTOM_GET_FW_UPDATE_STATE:
                return bytes((c,))+upl.HB_MAGIC+bytes((2,op,0))+struct.pack('>IIH',self.state,self.size,self.crc)
        if c==upl.COMM_ERASE_NEW_APP:
            self.size,self.crc=struct.unpack('>IH',p[1:7]); self.state=upl.STATE_STREAM; self.stage=bytearray(b'\xff'*self.size); self.written=0
            return bytes((c,1))+struct.pack('>I',0)
        if c==upl.COMM_WRITE_NEW_APP_DATA:
            off=struct.unpack('>I',p[1:5])[0]; data=p[5:]
            if off==0: data=data[6:]; appoff=0
            else: appoff=off-6
            assert appoff==self.written
            self.stage[appoff:appoff+len(data)]=data; self.written+=len(data)
            return bytes((c,1))+struct.pack('>I',off)
        raise AssertionError(p[:8])
    def write(self,raw):
        p=unframe(raw); assert p[0]==upl.COMM_JUMP_TO_BOOTLOADER
        assert self.state==upl.STATE_STREAM and self.written==self.size
        self.active=bytes(self.stage); self.state=upl.STATE_CONFIRMED; self.mode='app'; self.installs+=1
old=bytes((i*11+3)&255 for i in range(3072)); cand=bytes((i*29+7)&255 for i in range(4097)); link=Fake(old)
orig_wait=upl._wait_application
with tempfile.TemporaryDirectory() as td:
    oldenv=os.environ.get('F103_LKG_ROOT'); os.environ['F103_LKG_ROOT']=td
    def fake_wait(l,label,timeout=65):
        if label=='candidate': raise RuntimeError('intentional candidate boot failure')
        return 'motor_left_rollback'
    upl._wait_application=fake_wait
    try:
        try: upl.upload(link,cand); raise AssertionError('expected rollback exception')
        except RuntimeError as e: assert 'rollback completed' in str(e),e
    finally:
        upl._wait_application=orig_wait
        if oldenv is None: os.environ.pop('F103_LKG_ROOT',None)
        else: os.environ['F103_LKG_ROOT']=oldenv
assert link.installs==2 and link.active==old
print(f'PIO_VESC_UPLOADER_RECOVERY_PASS direct_uart=1 installs={link.installs} rollback_bytes={len(old)}')
