#!/usr/bin/env python3
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

R = next(p for p in Path(__file__).resolve().parents if (p / 'platformio.ini').exists())
config = (R / 'Src/config.h').read_text()

def define(name: str) -> str:
    match = re.search(rf'^#define\s+{re.escape(name)}\s+([0-9]+)u?\b', config, re.M)
    if not match:
        raise SystemExit(f'missing numeric config define: {name}')
    return match.group(1)

duty_scale = define('VESC_DUTY_PHYSICAL_SCALE_PERMILLE')
vector_full = define('FOC_SVPWM_VECTOR_FULL_SAFE')
a2bit = define('A2BIT_CONV')

for cc in [c for c in ('gcc', 'clang') if shutil.which(c)]:
    with tempfile.TemporaryDirectory() as td:
        exe = Path(td) / 't'
        cmd = [
            cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
            f'-I{R}', f'-I{R / "Src"}',
            f'-DA2BIT_CONV={a2bit}',
            f'-DVESC_DUTY_PHYSICAL_SCALE_PERMILLE={duty_scale}',
            f'-DFOC_SVPWM_VECTOR_FULL_SAFE={vector_full}',
            '-DFOC_SVPWM_VECTOR_MAX=((FOC_SVPWM_VECTOR_FULL_SAFE*VESC_DUTY_PHYSICAL_SCALE_PERMILLE)/1000)',
            str(R / 'tools/tests/host/test_foc_math.c'),
            str(R / 'Src/motor/foc_math.c'), '-lm', '-o', str(exe),
        ]
        result = subprocess.run(cmd, text=True, capture_output=True)
        if result.returncode:
            print(result.stdout + result.stderr)
            sys.exit(result.returncode)
        result = subprocess.run([str(exe)], text=True, capture_output=True)
        if result.returncode:
            print(result.stdout + result.stderr)
            sys.exit(result.returncode)
        print(cc, result.stdout.strip())

print('FOC_GCC_CLANG_RUNTIME_PASS')
