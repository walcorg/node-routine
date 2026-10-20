# node-routine

Unified Arduino Nano firmware for the **WALCORG TERRAFORMING CORP.** **v4.5 photobioreactor node**: LED control (TLC59401), 16-channel light sensing, UVC disinfection and an I2C slave protocol for Clorela-OS.

<p align="center">
  <img src="docs/img/01-node-v4.5.png" alt="Node v4.5" height="420">
</p>

## Contents

| Path | Description |
|---|---|
| [`firmware/walcorg_node_v45/walcorg_node_v45.ino`](firmware/walcorg_node_v45/walcorg_node_v45.ino) | Single sketch for the Arduino Nano (ATmega328P) on the rigid PCB |
| [`host/node_i2c_example.py`](host/node_i2c_example.py) | Python I2C master example for the Raspberry Pi Zero W (shield) |
| [`docs/img/`](docs/img) | Images of the node, flex PCB, rigid PCB, shield and kit box |

## Architecture

```
Raspberry Pi Zero W + shield (Clorela-OS)
        │  USB-B: +5 V (500 mA PTC) + I2C 100 kHz (SDA = D-, SCL = D+)
        ▼
Rigid PCB v4.5 ── Arduino Nano (I2C slave 0x10–0x13)
        │            ├─ TLC59401: 12 visible LEDs + 4 UVC LEDs
        │            ├─ CD74HC4067: 16 light sensors → A0
        │            └─ MT3608: +V_UV (9 V) for the UVC LEDs
        ▼  J1 / J2 (40-pin FPC, 0.5 mm)
Flex PCB ── LEDs, sensors and ID EEPROM on the walls of the 1 L cube
```

| | |
|---|---|
| ![Flex PCB: FPC tails](docs/img/02-flex-fpc-tails.png) | ![Rigid PCB](docs/img/03-rigid-pcb.png) |
| Flex PCB: FPC tails that plug into J1 and J2 | Rigid PCB that carries the Arduino Nano |
| ![Raspberry Pi Zero W shield](docs/img/04-pi-zero-w-shield.png) | ![Assembly kit box](docs/img/05-assembly-kit-box.png) |
| Raspberry Pi Zero W shield that drives one or more nodes | Retail box for the node assembly kit |

## Channel map

| TLC59401 | Use | CD74HC4067 | Use |
|---|---|---|---|
| 0–7 | Data LEDs (1 bit each, LSB = channel 0) | 0–7 | SENS_B0–B7, backscatter (J1) |
| 8–11 | Calibration LEDs | 8–15 | SENS_T0–T7, transmission (J2) |
| 12–15 | UVC LEDs (275 nm) | | |

## Build and upload

1. Arduino IDE, board **Arduino Nano**, processor **ATmega328P**.
2. Install the **Tlc5940** library (Library Manager, or [PaulStoffregen/Tlc5940](https://github.com/PaulStoffregen/Tlc5940)). Its default configuration matches this board's pinout.
3. If your Nano uses the old bootloader ("ATmega328P (Old Bootloader)"), set `USE_WATCHDOG 0` in the sketch.

Memory usage: ~8 KB flash (25 %) and ~1 KB RAM (51 %).

## I2C protocol

Address: `0x10 + ADDR1·2 + ADDR0` (solder jumpers SJ1/SJ2; closed = 1).

- The master writes `[command, arguments…]` (32 bytes max).
- A read after any command returns **8 status bytes**: `[state, flags, last_cmd, error, fw_version, uv_sec_L, uv_sec_H, address]`.
- `R, page` selects a **data** page (16 little-endian `uint16` values): 0 = last measurement, 1 = dark baseline, 2–13 = LED 0–11 on alone, 14 = configuration.

| Command | Action |
|---|---|
| `G` | Darkness check (sets DARK_ALERT if any sensor > threshold) |
| `H, n` | Turns on only LED n (0–11), measures, then turns it off |
| `K, byte` | Shows the byte on LEDs 0–7, measures and keeps the pattern on |
| `M, maskL, maskH` | Turns on the visible LEDs in the 12-bit mask |
| `S` / `O` | Measure with the current LEDs / turn all visible LEDs off |
| `B, vL, vH, text` | Plays text on LEDs 0–7 (v = ms per character) |
| `C` | Full calibration (pages 1–13) |
| `E [, sL, sH]` | UVC disinfection (300 s by default, 1800 s max) |
| `F` | Stops UVC disinfection |
| `D` | Safe mode: turns everything off immediately |
| `L, r` | Measurement range: 0 normal, 1 sensitive |
| `P, id, vL, vH` | Parameters: 0 settle time (ms), 1 dark threshold, 2 visible intensity, 3 UVC intensity, 4 samples, 5 default UVC duration |

Full details are in the sketch header.

## UVC safety

> ⚠️ The UVC LEDs (275 nm) are harmful to eyes and skin. Use them only with an empty node (between cultures) and the lid closed.

- Maximum duration of 30 min, hard-coded in the firmware.
- Turning on the UVC LEDs switches off the visible LEDs (USB current budget).
- While UVC is active only `F` and `D` are accepted, and both interrupt any operation in progress.
- 2 s watchdog: if the firmware hangs, the Nano resets with the UVC LEDs and boost converter off.

## Roadmap

- Per-channel dot correction (VPRG on D12; requires adjusting the Tlc5940 library).
- Firmware updates over I2C (twiboot).
- Reading the flex PCB ID EEPROM (0x50).

## License

[MIT](LICENSE) © 2026 WALCORG TERRAFORMING CORP. You may use, modify and distribute this code, including for commercial purposes, as long as you keep the copyright notice and the license text.
