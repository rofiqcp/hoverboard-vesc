#!/usr/bin/env python3
from pathlib import Path
import importlib.util, struct, sys
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
sys.path.insert(0,str(R/'tools'))
ini=(R/'platformio.ini').read_text()
u=(R/'tools/pio_stlink_upload.py').read_text()
g=(R/'tools/stlink_target_guard.py').read_text()
assert '--confirmed-meta-address 0x0803E800' in ini
assert "normal SWD attach PASS" in u
assert '--rescue-under-reset' not in u
assert 'automatic connect-under-reset' not in u
assert 'normal_attach_stable=3/3' in u
assert 'app_image_verified=' in u and 'app_runtime_verified=' in u and 'wait_for_application_runtime' in u
assert 'expected_vtor=APP_BASE' not in u and 'pc_min=APP_BASE' not in u and 'pc_max=META_BASE' not in u
assert 'resume_before_shutdown=True' not in u
assert 'verify_f103_target(openocd, scripts, 100)' in u
assert 'need_halt =' in g and 'actions = ["init"]' in g
assert 'under_reset' not in g and 'connect_assert_srst' not in g
assert 'resume_before_shutdown: bool = False' in g
# Import uploader and validate exact metadata ABI expected by F103 bootloader.
spec=importlib.util.spec_from_file_location('stup',R/'tools/pio_stlink_upload.py'); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
app=b'\xF0\xBF\x00\x20\x01\x29\x00\x08' + bytes(range(64))
meta=m.confirmed_meta(app)
assert len(meta)==36
magic,state,size,size_inv,crc,crc_inv,ver,ver_inv,attempt,attempt_inv,r0,r1=struct.unpack('<IIIIHHHHHHII',meta)
assert magic==0x56455343 and state==0x434E464D
assert size==len(app) and size_inv==((~size)&0xffffffff)
assert crc==m.crc16(app) and crc_inv==((~crc)&0xffff)
assert ver==2 and ver_inv==0xfffd
assert attempt==attempt_inv==0xffff and r0==r1==0xffffffff

# Partition/vector guard must reject malformed or misaddressed images before OpenOCD.
good_boot=struct.pack('<II',0x2000BFF0,0x08000101)+bytes(0x180)
good_app=struct.pack('<II',0x2000BFF0,0x08002901)+bytes(0x180)
m.validate_project_image(good_boot,0x08000000,0x2800,None)
m.validate_project_image(good_app,0x08002800,0x3C000,0x0803E800)
for args in ((good_boot,0x08002800,0x3C000,0x0803E800),(good_app,0x08002800,0x3C000,None),(b'bad',0x08000000,0x2800,None)):
    try: m.validate_project_image(*args)
    except RuntimeError: pass
    else: raise AssertionError('invalid ST-Link image/partition accepted')
normal=m.openocd_program_cmd(Path('/x/openocd'),Path('/x/scripts'),'swd',Path('/tmp/app.bin'),0x08002800,100,Path('/tmp/meta.bin'),0x0803E800)
assert not any('connect_assert_srst' in x for x in normal) and 'connect_assert_srst' not in u
assert any('0x0803E800' in x and 'meta.bin' in x for x in normal)

print('STLINK_UPDATE_SAFETY_PASS normal_primary=1 under_reset_disabled=1 postrun_normal_probe=3 noninvasive_postrun=1 confirmed_meta=1 vector_guard=1 partition_guard=1')
