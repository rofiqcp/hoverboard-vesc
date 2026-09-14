# Firmware Tools

`vesc_tool.py` adalah satu-satunya CLI operasional. `vesc_dual.py` adalah backend protokol VESC 6.00 dan tidak mempunyai REPL sendiri. Frontend legacy `vesc_debug.py`/`hoverserial.py` sudah dihapus agar tidak ada implementasi ganda.

## Struktur

```text
tools/
├── vesc_tool.py         # CLI interaktif utama: control, telemetry, config, tuning, detect
├── vesc_dual.py         # framing/CRC/COMM_* VESC 6.00 + virtual CAN ID 2
├── run_all_checks.py    # regression host non-aktuatif
├── tests/               # host / target / hardware tests
├── support/             # host compile stubs
└── results/             # hasil pengujian
```

## Interactive CLI

```bash
cd /home/otomasi/agv/hoverboard-vesc
python3 tools/vesc_tool.py auto
```

Default baud 115200 dan transport diprobe positif sebagai F103 VESC sebelum command dikirim. LEFT adalah local VESC ID 1; RIGHT adalah virtual CAN ID 2 via `COMM_FORWARD_CAN`.

Contoh di dalam terminal:

```text
help
target left
telemetry on 10
set pos 180
steering status
tuning get left
tuning set pos 0.100 0.030 0.004 left store
term steering zero @left
target right
set rpm 2000
stop right
detect hall 4 right store
config save config/leftencoder_righthall.yaml
quit
```

`prompt_toolkit.patch_stdout` menjaga prompt dan teks yang sedang diketik tetap rapi saat telemetry terus turun ke baris baru. Seperti VESC Tool upstream, telemetry tetap dipoll saat motor idle/released dan `COMM_ALIVE` dikirim ke LEFT+RIGHT setiap 200 ms selama koneksi terbuka. Setpoint aktif tetap direfresh default 50 Hz secara terpisah; idle tidak disimulasikan dengan setpoint nol palsu.

## One-shot

```bash
python3 tools/vesc_tool.py auto --no-telemetry --exec 'values both'
python3 tools/vesc_tool.py auto --no-telemetry --exec 'tuning get both'
python3 tools/vesc_tool.py --selftest
```

## Regression

```bash
python3 tools/run_all_checks.py
```

Hardware tests yang khusus eksperimen tetap berada di `tests/hardware/`; script campaign/profiling bukan CLI umum dan tidak menggandakan `vesc_tool.py`.
