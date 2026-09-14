#!/usr/bin/env python3
from pathlib import Path
import importlib.util, struct, sys

R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
sys.path.insert(0,str(R/'tools'))
ini=(R/'platformio.ini').read_text()
u=(R/'tools/pio_stlink_upload.py').read_text()
installer=(R/'tools/install_bootloader_stlink.sh').read_text()

assert '--confirmed-meta-address 0x0803E800' in ini
for token in ('flash write_image erase', 'verify_image', 'reset run', 'adapter speed'):
    assert token in u, token
for forbidden in ('verify_f103_target', 'wait_for_application_runtime', 'normal_attach_stable',
                  'post-run', 'retrying at safer', 'connect_assert_srst', 'dump_image'):
    assert forbidden not in u, forbidden
for forbidden in ('dump_image', 'BACKUP=', 'reg pc', 'stlink_target_guard'):
    assert forbidden not in installer, forbidden

spec=importlib.util.spec_from_file_location('stup',R/'tools/pio_stlink_upload.py')
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
app=b'\xF0\xBF\x00\x20\x01\x29\x00\x08'+bytes(range(64))
meta=m.confirmed_meta(app)
assert len(meta)==36
magic,state,size,size_inv,crc,crc_inv,ver,ver_inv,attempt,attempt_inv,r0,r1=struct.unpack('<IIIIHHHHHHII',meta)
assert magic==0x56455343 and state==0x434E464D
assert size==len(app) and size_inv==((~size)&0xffffffff)
assert crc==m.crc16(app) and crc_inv==((~crc)&0xffff)
assert ver==2 and ver_inv==0xfffd
assert attempt==attempt_inv==0xffff and r0==r1==0xffffffff

m.validate_partition(0x08000000,0x1000,0x2800,None)
m.validate_partition(0x08002800,0x1000,0x3C000,0x0803E800)
for args in ((0x08002800,0x1000,0x3C000,None),
             (0x08000000,0x3000,0x2800,None),
             (0x08001000,0x100,0x1000,None)):
    try: m.validate_partition(*args)
    except SystemExit: pass
    else: raise AssertionError(f'invalid partition accepted: {args}')

print('STLINK_STANDARD_UPLOAD_PASS program=1 verify=1 reset=1 metadata=1 eeprom_guard=1 extra_runtime_checks=0')
