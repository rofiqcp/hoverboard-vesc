#!/usr/bin/env python3
import importlib.util,struct,types
from pathlib import Path
R=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('upl',R/'tools/pio_vesc_upload.py'); upl=importlib.util.module_from_spec(spec); spec.loader.exec_module(upl)
class Fake:
    def __init__(self,fw):
        self.args=types.SimpleNamespace(transport='serial',serial_port='/dev/mock',baud=115200,fault_stop_after=8193)
        self.buf=bytearray(); self.mode='boot'; self.fw=fw; self.state=0; self.size=0; self.crc=0; self.stage=bytearray(); self.written=0; self.connections=1; self.resume_seen=0
    def reconnect_transport(self): self.connections+=1
    def transact(self,p,expected,timeout=3):
        c=p[0]
        if c==upl.COMM_FW_VERSION: return bytes((c,6,0))+b'f103rc_bootloader\0'
        if c==upl.COMM_ERASE_NEW_APP:
            size,cr=struct.unpack('>IH',p[1:7])
            if self.state==upl.STATE_STREAM and (size,cr)==(self.size,self.crc):
                self.resume_seen=self.written+6; return bytes((c,1))+struct.pack('>I',self.resume_seen)
            self.size,self.crc=size,cr; self.state=upl.STATE_STREAM; self.stage=bytearray(b'\xff'*size); self.written=0
            return bytes((c,1))+struct.pack('>I',0)
        if c==upl.COMM_WRITE_NEW_APP_DATA:
            off=struct.unpack('>I',p[1:5])[0]; data=p[5:]
            if off==0: data=data[6:]; appoff=0
            else: appoff=off-6
            assert appoff==self.written,(off,appoff,self.written)
            self.stage[appoff:appoff+len(data)]=data; self.written+=len(data)
            return bytes((c,1))+struct.pack('>I',off)
        raise AssertionError(p[:8])
fw=bytes((i*17+5)&255 for i in range(12289)); link=Fake(fw)
try: upl._stage_once(link,fw,1); raise AssertionError('expected intentional stop')
except upl.IntentionalStreamStop: stop=link.written
assert stop>=8193
link.args.fault_stop_after=0
upl._recover_bootloader_transport(link,'resume-test')
upl._stage_once(link,fw,2)
assert bytes(link.stage)==fw and link.written==len(fw) and link.resume_seen>0 and link.connections>=2
print(f'EXTERNAL_STREAM_RESUME_PASS direct_uart=1 stop_written={stop} resume={link.resume_seen} connections={link.connections}')
