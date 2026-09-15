#ifndef F103_BOOT_LAYOUT_H_
#define F103_BOOT_LAYOUT_H_

#include <stdint.h>

#define F103_FLASH_BASE_ADDR      0x08000000u
#define F103_FLASH_TOTAL_SIZE     0x00040000u /* 256 KiB */
#define F103_FLASH_PAGE_SIZE      0x00000800u /* 2 KiB for STM32F103xE */

/* Native VESC transport: F103 USART3 PB10/PB11 <-> CH340 USB-UART <-> PC/NUC.
 * Runtime application and resident recovery bootloader both use 921600 baud
 * for fast VESC traffic and firmware streaming over CH340 USB-UART. The host
 * uploader retains a 115200 legacy fallback for one-time bootloader migration. */
#define F103_VESC_UART_BAUD       921600u
#define F103_BOOT_UART_BAUD       921600u

/* Top 16 bytes of SRAM are reserved in BOTH linker scripts. A two-word magic
 * makes application -> resident-bootloader entry independent of flash writes.
 * SRAM survives NVIC_SystemReset but not a real power loss, which is desired. */
#define F103_BOOT_REQUEST_ADDR       0x2000BFF0u
#define F103_BOOT_REQUEST_MAGIC      0x46574F54u /* 'FWOT' */
#define F103_BOOT_REQUEST_MAGIC_INV  ((uint32_t)~F103_BOOT_REQUEST_MAGIC)

/* Remaining two reserved SRAM words form a reset black-box. They survive
 * NVIC_SystemReset and are outside .bss/.data by linker reservation. */
#define F103_RESET_REASON_ADDR       0x2000BFF8u
#define F103_RESET_STAGE_ADDR        0x2000BFFCu
#define F103_RESET_REASON_REBOOT     0x52454254u /* 'REBT' */
#define F103_RESET_REASON_FW_UPDATE  0x46575550u /* 'FWUP' */
#define F103_RESET_REASON_TEST_OK    0x544F4B21u /* 'TOK!' app probation passed; bootloader commits CONFIRMED */

/* Reset-stage black-box values used by the TEST probation state machine.
 * Low byte of GATE/TIMEOUT stores a bitmask: bit0 elapsed, bit1 main-loop,
 * bit2 watchdog-feed progress, bit3 USART3-DMA healthy, bit4 host protocol seen. */
#define F103_STAGE_PROBATION_GATE_BASE       0x50524700u /* 'PRG'+bits */
#define F103_STAGE_PROBATION_TIMEOUT_BASE    0x50525400u /* 'PRT'+bits */
#define F103_STAGE_PROBATION_CONFIRM_OK      0x50524F4Bu /* 'PROK' */
#define F103_STAGE_PROBATION_VECTOR_FAIL     0x50525646u /* 'PRVF' */
#define F103_STAGE_PROBATION_CRC_FAIL        0x50524346u /* 'PRCF' */

#define F103_BOOT_BASE_ADDR        0x08000000u
#define F103_BOOT_SIZE             0x00002800u /* resident recovery bootloader, 10 KiB */
#define F103_APP_BASE_ADDR         0x08002800u
#define F103_APP_REGION_SIZE       0x0003C000u /* 240 KiB up to metadata */
#define F103_META_BASE_ADDR       0x0803E800u
#define F103_META_REGION_SIZE     0x00000800u /* 2 KiB / 1 page */
#define F103_EEPROM_BASE_ADDR     0x0803F000u
#define F103_EEPROM_REGION_SIZE   0x00001000u /* 4 KiB / 2 pages */

#define F103_VESC_IMAGE_HEADER_SIZE 6u
#define F103_MAX_FW_IMAGE_SIZE      F103_APP_REGION_SIZE

/* Host-backed external staging. The candidate and last-known-good image
 * live outside F103; internal flash only contains bootloader + active app. */
#define F103_UPDATE_META_MAGIC        0x56455343u /* 'VESC' */
#define F103_UPDATE_STATE_STREAM      0x5354524Du /* 'STRM' */
#define F103_UPDATE_STATE_TEST        0x54455354u /* 'TEST' */
#define F103_UPDATE_STATE_RECOVERY    0x52454356u /* 'RECV' */
#define F103_UPDATE_STATE_CONFIRMED   0x434E464Du /* 'CNFM' */
#define F103_UPDATE_META_VERSION      2u
#define F103_TEST_ATTEMPT_MAGIC       0xB007u
#define F103_UPDATE_JOURNAL_OFFSET    64u
#define F103_UPDATE_JOURNAL_MARK_BASE 0xA500u

typedef struct {
    uint32_t magic;
    uint32_t state;
    uint32_t size;
    uint32_t size_inv;
    uint16_t crc16;
    uint16_t crc16_inv;
    uint16_t version;
    uint16_t version_inv;
    uint16_t test_attempt;
    uint16_t test_attempt_inv;
    uint32_t reserved0;
    uint32_t reserved1;
} f103_update_meta_t;

#endif
