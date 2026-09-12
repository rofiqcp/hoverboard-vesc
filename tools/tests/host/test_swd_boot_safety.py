#!/usr/bin/env python3
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
eeprom=(R/'Src/eeprom.h').read_text(); setup=(R/'Src/setup.c').read_text(); main=(R/'Src/main.c').read_text(); irq=(R/'Src/stm32f1xx_it.c').read_text(); boot=(R/'Src/bootloader/main.c').read_text(); layout=(R/'Src/vesc/f103_boot_layout.h').read_text(); ini=(R/'platformio.ini').read_text(); ld=(R/'STM32F103RCTx_APP.ld').read_text()
assert '__HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE' not in setup

defines=(R/'Src/defines.h').read_text()
# TIM1 complementary outputs are PB13/PB14/PB15; PA13/PA14 must remain SWDIO/SWCLK.
assert '#define RIGHT_TIM_UL_PORT GPIOB' in defines and '#define RIGHT_TIM_VL_PORT GPIOB' in defines
assert '#define RIGHT_TIM_WL_PORT GPIOB' in defines
assert 'AFIO_MAPR_ADC1_ETRGREG_REMAP' in setup and 'AFIO_MAPR_SWJ_CFG_Msk' in setup and 'AFIO_MAPR_SWJ_CFG_RESET' in setup
assert 'f103_debug_keepalive' in main and main.count('f103_debug_keepalive();') >= 2
assert '__HAL_DBGMCU_FREEZE_IWDG()' in main and '__HAL_DBGMCU_FREEZE_IWDG()' in boot
assert 'f103_fault_to_recovery' in irq
for h in ('HardFault','MemManage','BusFault','UsageFault'):
    assert f'f103_{h}_Handler_impl' in irq
assert 'F103_BOOT_REQUEST_MAGIC' in irq and 'NVIC_SystemReset();' in irq
assert '#define F103_BOOT_BASE_ADDR        0x08000000u' in layout
assert '#define F103_BOOT_SIZE             0x00002800u' in layout
assert '#define F103_APP_BASE_ADDR         0x08002800u' in layout
assert '#define F103_APP_REGION_SIZE       0x0003C000u' in layout
assert 'ORIGIN = 0x8002800, LENGTH = 240K' in ld
assert '-DVECT_TAB_OFFSET=0x00002800U' in ini
assert '--address 0x08002800 --max-size 0x3C000 --confirmed-meta-address 0x0803E800' in ini
assert '--rescue-under-reset' not in ini
stlink=(R/'tools/pio_stlink_upload.py').read_text(); factory=(R/'tools/build_factory_image.py').read_text()
assert '--rescue-under-reset' in stlink and 'normal_attach_stable=3/3' in stlink
assert 'app_runtime_verified=' in stlink and 'expected_vtor=APP_BASE' in stlink
assert 'META_STATE_CONFIRMED = 0x434E464D' in stlink and 'confirmed_meta(image_bytes)' in stlink
assert 'META_STATE_CONFIRMED = 0x434E464D' in factory and 'confirmed_meta(app)' in factory
assert '[env:BOOTLOADER_STAGE2_UART]' not in ini
assert 'EEPROM_START_ADDRESS == F103_EEPROM_BASE_ADDR' in eeprom
print('SWD_BOOT_SAFETY_STATIC_PASS swd_pins_reserved=1 swd_preserved=1 unsafe_afio_remap=0 iwdg_debug_freeze=1 fault_to_recovery=1 single_stage=1 app=240K stlink_meta=1 normal_only=1')
