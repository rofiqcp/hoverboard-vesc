#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile,shutil,sys
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
ccs=[c for c in ('gcc','clang') if shutil.which(c)]
for cc in ccs:
    with tempfile.TemporaryDirectory(prefix='eeprom-v14-') as td:
        exe=Path(td)/'t'
        cmd=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-I{R}',f'-I{R/"Src"}',f'-I{R/"tools/support/host_stubs"}',
             str(R/'tools/tests/host/test_eeprom_persistence.c'),str(R/'Src/motor/mcpwm_foc.c'),str(R/'Src/motor/foc_math.c'),str(R/'Src/encoder/encoder.c'),str(R/'Src/encoder/enc_abi.c'),str(R/'Src/encoder/encoder_cfg.c'),str(R/'Src/motor/mc_interface.c'),'-lm','-o',str(exe)]
        x=subprocess.run(cmd,text=True,capture_output=True)
        if x.returncode:
            print(x.stdout+x.stderr);sys.exit(x.returncode)
        x=subprocess.run([str(exe)],text=True,capture_output=True)
        if x.returncode:
            print(x.stdout+x.stderr);sys.exit(x.returncode)
        print(cc,x.stdout.strip())
print('EEPROM_DUAL_PERSISTENCE_GCC_CLANG_PASS')
