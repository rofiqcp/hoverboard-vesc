#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile,shutil,sys
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
HAL='''#pragma once\n#include <stdint.h>\n#include <stddef.h>\ntypedef struct {void*hdmatx;void*hdmarx;void*Instance;uint32_t gState;} UART_HandleTypeDef;\n#define HAL_OK 0\n#define HAL_UART_STATE_READY 0u\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t);\nint HAL_UART_Transmit(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);\nint HAL_UART_Transmit_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);\nvoid __disable_irq(void);\nvoid __enable_irq(void);\n'''
for cc in [c for c in ('gcc','clang') if shutil.which(c)]:
  with tempfile.TemporaryDirectory() as td:
    td=Path(td);(td/'stm32f1xx_hal.h').write_text(HAL)
    exe=td/'p'
    cmd=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-I{ROOT}',f'-I{ROOT/"Src"}',f'-I{ROOT/"Src/vesc"}',f'-I{td}',str(ROOT/'tools/tests/host/test_vesc_protocol_host.c'),str(ROOT/'Src/vesc/vesc_protocol.c'),str(ROOT/'Src/vesc/app_vesc.c'),str(ROOT/'Src/vesc/buffer.c'),str(ROOT/'Src/vesc/crc.c'),str(ROOT/'Src/vesc/mcconf_serial.c'),'-lm','-o',str(exe)]
    r=subprocess.run(cmd,text=True,capture_output=True)
    if r.returncode: print(r.stdout+r.stderr);sys.exit(r.returncode)
    r=subprocess.run([str(exe)],text=True,capture_output=True)
    if r.returncode: print(r.stdout+r.stderr);sys.exit(r.returncode)
    print(cc, r.stdout.strip())
print('VESC_PROTOCOL_GCC_CLANG_PASS')
