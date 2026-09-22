#!/usr/bin/env python3
from pathlib import Path
import re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
layout=(R/'Src/vesc/f103_boot_layout.h').read_text()
boot=(R/'Src/bootloader/main.c').read_text()
upd=(R/'Src/vesc/flash_update_f103.c').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
ini=(R/'platformio.ini').read_text()
app_ld=(R/'STM32F103RCTx_APP.ld').read_text(); boot_ld=(R/'STM32F103RCTx_BOOTLOADER.ld').read_text()

def hx(name):
    m=re.search(rf'#define\s+{name}\s+0x([0-9A-Fa-f]+)u?',layout)
    assert m,name; return int(m.group(1),16)
base=hx('F103_FLASH_BASE_ADDR'); total=hx('F103_FLASH_TOTAL_SIZE'); page=hx('F103_FLASH_PAGE_SIZE')
bb=hx('F103_BOOT_BASE_ADDR'); bs=hx('F103_BOOT_SIZE'); ab=hx('F103_APP_BASE_ADDR'); aps=hx('F103_APP_REGION_SIZE')
mb=hx('F103_META_BASE_ADDR'); ms=hx('F103_META_REGION_SIZE'); eb=hx('F103_EEPROM_BASE_ADDR'); es=hx('F103_EEPROM_REGION_SIZE')
assert base==bb==0x08000000 and total==0x40000 and page==0x800
assert bb+bs==ab and ab+aps==mb and mb+ms==eb and eb+es==base+total
for x in (bb,bs,ab,aps,mb,ms,eb,es): assert x%page==0,hex(x)
assert aps==240*1024 and bs==10*1024 and ms==2*1024 and es==4*1024
assert re.search(r'ORIGIN\s*=\s*0x8002800,\s*LENGTH\s*=\s*240K',app_ld)
assert re.search(r'ORIGIN\s*=\s*0x8000000,\s*LENGTH\s*=\s*10K',boot_ld)
for env in ('APP_STLINK','APP_USART_PC','BOOTLOADER_STLINK'): assert f'[env:{env}]' in ini
assert 'board_build.ldscript = STM32F103RCTx_APP.ld' in ini and 'board_build.ldscript = STM32F103RCTx_BOOTLOADER.ld' in ini
assert '-DVECT_TAB_OFFSET=0x00002800U' in ini
for token in ('COMM_ERASE_NEW_APP','COMM_WRITE_NEW_APP_DATA','COMM_JUMP_TO_BOOTLOADER'):
    assert f'case {token}:' in vp,token
for token in ('f103_fw_erase_staging','f103_fw_write_staging'):
    assert token in upd and token in vp,token
assert 'f103_fw_reset_to_bootloader' in upd and 'f103_fw_reset_to_bootloader();' in vp
assert 'F103_BOOT_REQUEST_ADDR' in layout and 'F103_BOOT_REQUEST_MAGIC_INV' in layout
assert 'boot_request[0] == F103_BOOT_REQUEST_MAGIC' in boot and 'force_recovery' in boot
assert 'F103_BOOT_REQUEST_ADDR' in upd and 'NVIC_SystemReset();' in upd
assert 'f103_fw_mark_pending_or_recovery' not in upd and 'f103_fw_mark_pending_or_recovery' not in vp
for token in ('F103_UPDATE_STATE_STREAM','F103_UPDATE_STATE_TEST','stream_begin','stream_write','stream_finalize','journal_mark_page','app_vector_valid','jump_app','F103_APP_REGION_SIZE'):
    assert token in boot,token
assert 'recv_payload(RECOVERY_BOOT_WINDOW_MS' not in boot
assert '__disable_irq();' in boot and 'cpsie i' not in boot
app=(R/'Src/main.c').read_text()
assert '__enable_irq();' in app and app.index('__enable_irq();') > app.index('HAL_ADC_Start(&hadc1)') and app.index('__enable_irq();') > app.index('HAL_ADC_Start(&hadc2)')
assert ('branch_to_app' in boot and '__attribute__((naked, noreturn))' in boot) or '__set_MSP(sp);' in boot
assert 'void SysTick_Handler(void)' in boot and 'HAL_IncTick();' in boot
assert 'SystemCoreClockUpdate();' in boot and boot.index('SystemCoreClockUpdate();') < boot.index('HAL_Init();')
assert 'boot_clock_init()' in boot and 'RCC_PLL_MUL16' in boot and 'RCC_HCLK_DIV2' in boot
assert boot.index('boot_clock_init()') < boot.index('uart_init();')
assert 'F103_STAGE_BASE_ADDR' not in boot and 'F103_STAGE_REGION_SIZE' not in boot
assert 'F103_STAGE_BASE_ADDR' not in upd and 'F103_STAGE_REGION_SIZE' not in upd
assert 'erase_pages(F103_APP_BASE_ADDR, F103_APP_REGION_SIZE)' not in boot
assert 'HB_BOOT_READ_APP' in boot and 'HB_BOOT_GET_INFO' in boot
assert 'f103_fw_test_pending' in upd and 'f103_fw_confirm_running_image' in upd
print('BOOTLOADER_LAYOUT_STATIC_PASS app=240K host_external_stage=1 boot=10K meta=2K eeprom=4K journal_resume=1 test_confirm=1')

assert 'uart_recv_byte' in boot and 'uart_send_bytes' in boot and 'USART_SR_ORE' in boot
assert 'HAL_UART_Receive(&huart3' not in boot and 'HAL_UART_Transmit(&huart3' not in boot

assert 'default_envs = APP_USART_PC' in ini  # canonical production default: 921600-baud VESC serial updater; ST-Link remains explicit recovery
