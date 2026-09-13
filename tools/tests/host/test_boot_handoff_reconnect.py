#!/usr/bin/env python3
import importlib.util,types
from pathlib import Path
R=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('upl',R/'tools/pio_vesc_upload.py'); upl=importlib.util.module_from_spec(spec); spec.loader.exec_module(upl)
class Fake:
    def __init__(self):
        self.args=types.SimpleNamespace(transport='serial',serial_port='/dev/mock',baud=115200); self.buf=bytearray(); self.phase='app'; self.failures=0; self.reconnects=0
    def transact(self,p,expected,timeout=3):
        if p[0]==upl.COMM_CUSTOM_APP_DATA and p[4]==upl.HB_CUSTOM_BOOT_HANDOFF:
            self.phase='resetting'; return bytes((p[0],))+upl.HB_MAGIC+bytes((2,p[4],0))
        if p[0]==upl.COMM_FW_VERSION:
            if self.phase=='app': return bytes((0,6,0))+b'motor_left\0'
            if self.phase=='resetting': self.failures+=1; raise TimeoutError('target reset')
            return bytes((0,6,0))+b'f103rc_bootloader\0'
        raise AssertionError(p)
    def reconnect_transport(self): self.reconnects+=1; self.phase='boot'
    def write(self,b): raise AssertionError('legacy fallback unexpected')
link=Fake(); hw=upl.fw_version(link,1); got=upl.wait_for_bootloader(link,hw)
assert 'bootloader' in got and link.reconnects==1 and link.failures>=2
print(f'BOOT_HANDOFF_AUTORECONNECT_PASS direct_uart=1 reconnects={link.reconnects}')
