# hoverboard-vesc — STM32F103RCT6 VESC 6.00 Dual FOC

Firmware dual-motor FOC bare-metal untuk board hoverboard STM32F103RCT6 dengan
protokol dan perilaku kontrol yang diarahkan kompatibel dengan VESC 6.00/VESC Tool.
Jalur ADC dual-DMA, PWM TIM8/TIM1, dan ISR FOC 16 kHz tetap memakai basis EFeru
yang sudah digunakan pada hardware ini. USART3 PB10/PB11 production = 115200 baud dan bersifat VESC-exclusive; legacy raw hoverboard serial sudah tidak didukung.

Repository utama: `https://github.com/rofiqcp/hoverboard-vesc`.
Di workspace AGV, repository ini dipasang sebagai Git submodule pada
`/home/otomasi/ros/hoverboard-vesc` sehingga riwayat firmware tetap terpisah dari ROS 2.

## Git submodule

Setelah clone repository AGV, inisialisasi firmware dengan:

```bash
git submodule update --init --recursive
```

Untuk mengambil commit terbaru branch `v1` dari submodule:

```bash
git submodule update --remote --merge hoverboard-vesc
```

Setiap perubahan firmware harus di-commit dan di-push dari folder `hoverboard-vesc`
terlebih dahulu, kemudian commit pointer submodule yang baru pada repository AGV.

## Mapping motor

- LEFT = local VESC, ID 1
- RIGHT = virtual VESC/CAN ID 2
- `COMM_FORWARD_CAN,2,<packet>` mengeksekusi packet pada motor kanan secara lokal.
- Tidak membutuhkan CAN fisik untuk motor kanan.

Arah motor kanan dinormalisasi pada boundary protocol sehingga nilai user positif
memiliki arah kendaraan yang sama dengan motor kiri.

## Perbaikan runtime V16

### 1. SET_CURRENT 3 A

Jalur standar VESC tetap:

```text
COMM_SET_CURRENT
int32 / 1000
3.000 A -> 3000 -> Iq_target 3.000 A
```

Command VESC memiliki ownership per motor selama 500 ms sehingga command legacy
tidak menimpa setpoint pada ISR berikutnya. Bridge kiri/kanan juga sekarang
digating berdasarkan ownership masing-masing motor, bukan flag global.

Masalah utama V14 bukan scaling 3 A, tetapi Hall detector fisik: fixed electrical
phase detector ditimpa kembali oleh updater rotating-openloop di ISR. Akibatnya
tabel Hall hasil deteksi hardware dapat salah, sehingga Iq 3 A tidak menghasilkan
orientasi torsi yang benar. V16 mempertahankan pemisahan `CONTROL_MODE_OPENLOOP_PHASE`: pada
mode ini ISR hanya meramp Id dan tidak pernah mengubah phase yang diperintahkan
detector.

### 2. SET_RPM 50 ERPM

`COMM_SET_RPM` tetap menggunakan ERPM VESC.

Dengan 15 pole-pair:

```text
50 ERPM = 3.333333 mechanical RPM
```

V14 mengubahnya menjadi integer 3 mechanical RPM sehingga control target efektif
sekitar 45 ERPM. V16 menyimpan target mechanical RPM dalam Q16 fractional dan
speed PI menghitung error terhadap estimasi Hall Q16 fractional.

Hall timeout juga dinaikkan dari 2000 tick (125 ms) menjadi 8000 tick (500 ms).
Pada 50 ERPM satu Hall transition sekitar 200 ms, sehingga V14 memang dapat
menganggap speed nol sebelum transition berikutnya. Telemetry ERPM sekarang
dihitung langsung dari Hall period sehingga 50 ERPM tidak dikuantisasi menjadi 45.

### 3. Position

Standard VESC tetap single-turn:

```text
COMM_SET_POS
0 .. 360 degree rotor electrical
```

Python menyediakan API terpisah melalui `COMM_CUSTOM_APP_DATA` untuk posisi
multi-turn Hall-count signed int32:

```text
min     = signed int32 bebas
max     = signed int32 bebas
target  = signed int32 bebas
```

Contoh:

```bash
python3 tools/vesc_tool.py /dev/ttyUSB0 --no-telemetry --exec 'poscount limits -1000000 2000000 left'
python3 tools/vesc_tool.py /dev/ttyUSB0 --no-telemetry --exec 'poscount set -250 left'
```

Position min/max custom adalah konfigurasi runtime; V16 belum mengklaim persistence
min/max position ke EEPROM. Hall table, current limit, PID yang diimplementasikan,
speed ramp, dan speed release tetap dipersist per motor seperti V14.

### 4. Realtime data VESC Tool 50 Hz

Tidak ada lagi `LIVEON`, `LIVEOFF`, atau parameter `LIVE`.

- USART3 hanya membawa frame protokol VESC; telemetri legacy 72-byte sudah dihapus.
- Tidak ada byte debug/legacy yang disisipkan ke stream VESC Tool.
- RX VESC memakai FIFO 4 paket agar burst request tidak hilang karena satu slot pending.
- `COMM_GET_VALUES`, `COMM_GET_VALUES_SELECTIVE`, `COMM_GET_VALUES_SETUP`, dan
  `COMM_GET_VALUES_SETUP_SELECTIVE` tetap selalu mendapat reply langsung.
- Request realtime terakhir juga mengaktifkan pengiriman paket dengan ID/mask yang sama
  setiap 20 ms (50 Hz) selama link VESC aktif. Jadi hanya frame VESC ber-CRC yang keluar,
  tidak ada byte legacy yang disisipkan ke stream VESC Tool.

Contoh verifikasi host:

```bash
python3 tools/vesc_tool.py /dev/ttyUSB0
# lalu di prompt: telemetry on 50
```

Regression host memeriksa `COMM_GET_VALUES` dan `COMM_GET_VALUES_SETUP`: reply langsung
serta paket periodik baru muncul saat interval 20 ms tercapai.

### 5. Hall estimator / getar di sekitar 450 ERPM

Tabel hasil Hall detect adalah **sector center** (0..199 = 0..360° electrical). V15
memulai interpolasi dari center Hall baru lalu menambahkan hingga satu sektor lagi. Pada
15 pole-pair, threshold 30 RPM mekanik tepat sama dengan 450 ERPM, sehingga bug ini dapat
muncul sebagai getar/commutation kasar ketika interpolation mulai aktif.

V16 menggunakan midpoint antara sector center lama dan baru sebagai posisi tepat saat
Hall edge, lalu menginterpolasi edge-to-edge. Sudut Hall yang dikoreksi juga diberi
rate-limit untuk mencegah lonjakan current akibat perubahan phase mendadak. Pada low
speed, estimator memakai calibrated sector center secara langsung.

Saat motor OFF/bridge undriven, Id/Iq/Iin dan current LPF juga di-zero-kan. Sample ADC
phase ketika bridge OFF tidak lagi ditampilkan sebagai current puluhan ampere palsu pada
Realtime Data/debug.

### 6. Hall detect

`COMM_DETECT_HALL_FOC` menggunakan power/current-control path mode 4 dengan
submode fixed electrical phase:

```text
Id detect current
-> align
-> forward sweep
-> reverse sweep
-> sample raw Hall 1..6
-> circular mean angle
-> check 6 states + sector spacing
-> apply foc_hall_table
-> EEPROM per motor
-> VESC reply: command + 8 Hall values + result
```

Proteksi phase/DC 8 A untuk mode 4 sekarang juga aktif pada fixed-phase Hall
detector walaupun `ctrlModReq` legacy bukan mode 4.

Current detect:

```text
minimum          0.5 A
maximum          40% l_current_max
absolute maximum 4.0 A
recommended start 1.0 A
```

Detector belajar mapping raw Hall terhadap electrical phase yang benar. Urutan
Hall/phase yang berbeda dapat dikalibrasi selama rangkaian 3-phase dan current
sense hardware tetap valid. State Hall 000/111 tetap invalid. Deteksi gagal/noisy
tidak mengganti tabel lama.

## VESC 6.00 protocol identity

Firmware mengiklankan:

```text
FW 6.00
motor_left  (local, ID 1)
motor_right (virtual CAN ID 2)
```

`COMM_FW_VERSION` V16 berhenti setelah field FW_NAME sesuai layout VESC 6.00;
field tambahan CRC yang sebelumnya ada di V14 sudah dihapus.

Motor/app config signature:

```text
MCCONF_SIGNATURE = 776184161
APPCONF_SIGNATURE = 486554156
```

`Src/vesc/datatypes.h` dipertahankan byte-identical dengan referensi:

```text
SHA256 4ecae1f31c12c1ab415d47dd997396d0792e94249203cbeb877ada75f76d5340
```

## VESC telemetry

`COMM_GET_VALUES`/selective menyediakan nilai aktual:

- Imotor
- Iin / DC-link current
- Id
- Iq
- duty
- ERPM
- Vin
- fault
- rotor position 0..360 deg
- VESC ID
- Vd
- Vq

## vesc_tool.py — satu CLI operasional

Jalankan terminal interaktif:

```bash
python3 tools/vesc_tool.py auto
```

Contoh command di prompt:

```text
help
target left
telemetry on 10
set pos 180
steering status
zero
tuning get left
tuning set pos 0.100 0.030 0.004 left store

target right
set rpm 2000
stop right
detect hall 4 right store

values both
diag both
config save config/leftencoder_righthall.yaml
term help @left
```

Semua frame memakai encoding VESC 6.00 dari `vesc_dual.py`; RIGHT dikirim melalui `COMM_FORWARD_CAN` ID 2. Satu-satunya frontend operasional adalah `vesc_tool.py`.

Selftest tanpa hardware:

```bash
python3 tools/vesc_tool.py --selftest
```

## Build target

```bash
pio run -e VARIANT_USART
```

Build utama menggunakan PlatformIO environment `VARIANT_USART`. Setelah build,
validasi host dijalankan dengan `tools/run_all_checks.py`; pengujian yang menggerakkan
motor tetap harus dilakukan pada hardware dengan interlock/arming yang sesuai.

## Host regression

```bash
python3 tools/run_all_checks.py
```

Lihat `VALIDATION_V15.md` dan `AUDIT_VESC600_RUNTIME_V15.md`.
