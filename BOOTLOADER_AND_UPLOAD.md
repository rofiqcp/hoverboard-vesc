# STM32F103RCT6 VESC Bootloader and Upload Paths

Production communication is direct: **PC/NUC USB-UART <-> STM32F103 USART3 PB10/PB11 at 115200 baud**. There is no intermediate MCU gateway, maintenance proxy, or TCP firmware route.

## Flash layout

- `0x08000000..0x080027FF`: resident recovery bootloader, 10 KiB
- `0x08002800..0x0803E7FF`: active application, 240 KiB
- `0x0803E800..0x0803EFFF`: update metadata/journal, 2 KiB
- `0x0803F000..0x0803FFFF`: emulated EEPROM, 4 KiB

The candidate image is staged on the PC/NUC and streamed directly to the resident bootloader. The F103 does not reserve a second 120-KiB application slot.

## Normal application upload

Wire the USB-UART directly to F103 USART3:

- USB-UART RX <- `PB10` F103 TX
- USB-UART TX -> `PB11` F103 RX
- GND <-> GND

Then use:

```bash
pio run -e APP_USART_PC -t upload --upload-port /dev/ttyUSBX
```

`APP_USART_PC` is the default PlatformIO environment. The uploader positively identifies the F103 with `COMM_FW_VERSION`, asks the running application for an ACKed boot handoff, waits for its UART TX to drain, follows the automatic MCU reset into the resident bootloader, streams the image, verifies TEST/CONFIRMED metadata, and reconnects the same USB-UART automatically when needed. **No manual RESET press is part of the normal flow.**

## ST-Link

`APP_STLINK` and `BOOTLOADER_STLINK` are direct recovery/development paths. Production ST-Link tooling is normal-SWD only; it has no automatic connect-under-reset fallback.

```bash
pio run -e APP_STLINK -t upload
pio run -e BOOTLOADER_STLINK -t upload
```

The resident bootloader is intentionally not self-updatable. Keep an ST-Link header available for exceptional recovery, but normal application updates should use the direct USART3 USB-UART path above.

## Safety guarantees

Before an application-to-bootloader handoff, both bridges are forced off. The ACK is transmitted first; reset is allowed only after the UART queue, DMA state, and USART transmission-complete state are drained. UART corruption recovery never escalates to an MCU reset.

Firmware streaming is resumable and idempotent. A candidate boots in TEST state and must confirm itself before it becomes the accepted image. If a candidate fails and a host last-known-good image exists, the uploader restores it through the same direct USART3 path.
