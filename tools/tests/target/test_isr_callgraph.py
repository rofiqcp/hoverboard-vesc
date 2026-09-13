#!/usr/bin/env python3
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
ELF_STLINK = ROOT / '.pio/build/APP_STLINK/firmware.elf'
ELF_USART = ROOT / '.pio/build/APP_USART_PC/firmware.elf'
ELF = ELF_STLINK if ELF_STLINK.exists() else ELF_USART
OBJDUMP = shutil.which('arm-none-eabi-objdump')
NM = shutil.which('arm-none-eabi-nm')
if not OBJDUMP:
    candidate = Path.home() / '.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objdump'
    if candidate.exists():
        OBJDUMP = str(candidate)
    nm_candidate = Path.home() / '.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm'
    if not NM and nm_candidate.exists():
        NM = str(nm_candidate)

if not OBJDUMP or not NM:
    raise SystemExit('FAIL arm-none-eabi-objdump/nm not found')
if not ELF.exists():
    raise SystemExit('FAIL firmware.elf missing; run pio run for APP_STLINK/APP_USART_PC first')

text = subprocess.check_output([OBJDUMP, '-d', '-C', str(ELF)], text=True)
graph = {}
current = None
for line in text.splitlines():
    m = re.match(r'^([0-9a-f]+) <([^>]+)>:', line)
    if m:
        current = m.group(2)
        graph.setdefault(current, set())
        continue
    if current:
        m = re.search(r'\bbl(?:x)?\b[^<]*<([^>]+)>', line)
        if m:
            graph[current].add(m.group(1).split('+')[0])

# Build LTO memakai trampoline assembly kuat agar vector CMSIS tidak jatuh ke
# weak Default_Handler. Audit simbol ini wajib sebelum menganalisis call graph.
nm_text = subprocess.check_output([NM, '-C', str(ELF)], text=True)
required_handlers = {
    'SysTick_Handler', 'DMA1_Channel1_IRQHandler', 'DMA1_Channel2_IRQHandler',
    'DMA1_Channel3_IRQHandler', 'USART3_IRQHandler',
}
strong = set()
for line in nm_text.splitlines():
    m = re.match(r'^[0-9a-fA-F]+\s+([Tt])\s+(\S+)$', line.strip())
    if m:
        strong.add(m.group(2))
missing = sorted(required_handlers - strong)
if missing:
    raise SystemExit('FAIL IRQ vector handler is not strong under LTO: ' + ', '.join(missing))

root = 'f103_DMA1_Channel1_IRQHandler_impl'
if root not in graph:
    raise SystemExit(f'FAIL {root} not found in ELF')

reachable = set()
stack = [root]
while stack:
    fn = stack.pop()
    if fn in reachable:
        continue
    reachable.add(fn)
    stack.extend(graph.get(fn, ()))

forbidden_exact = {
    '__aeabi_ldivmod', '__aeabi_uldivmod',
    'sqrtf', 'sqrt', 'atan2f', 'atan2', 'sinf', 'cosf',
}
forbidden = []
for fn in sorted(reachable):
    if fn in forbidden_exact or fn.startswith('__aeabi_f') or fn.startswith('__aeabi_d'):
        forbidden.append(fn)

if forbidden:
    raise SystemExit('FAIL forbidden ISR calls: ' + ', '.join(forbidden))
# ARMv7-M SDIV/UDIV are hardware instructions and are allowed. The F103 ISR
# budget only forbids software float/double and 64-bit division helpers.
print(
    'ISR_CALLGRAPH_PASS',
    f'reachable={len(reachable)}',
    'forbidden=0',
    'vectors=strong',
)
