# Bounce FPV

An FPV head-tracking camera system for RC trucks, written in Rust on bare-metal **nRF52840** (ARM Cortex-M4F, `no_std`) using [Embassy](https://embassy.dev) for async.
You move your head in a pair of FPV goggles; a gimbal on the truck pans and tilts the camera to match,
and the truck streams GPS speed and distance/bearing-to-home back over the same radio link.

Two boards talk over a single wired **RFM95W / SX1276 LoRa** link:

- **Goggle node** — decodes the PPM head-tracking stream from Skyzone FPV goggles (pan/tilt channels), packs pan/tilt into a protobuf `Control` message,
  and transmits it over LoRa. It also listens for the truck's telemetry reply and shows speed / link state on a small OLED, with a push-button to re-home the gimbal.
- **Truck node** (headless) — receives `Control`, drives two MG90S gimbal servos via PWM, reads an MPU-6050 IMU (to trim the gimbal to a level "home" at boot)
  and a UART GPS, and replies with a `Telemetry` message carrying ground speed and distance/bearing back to the launch point.

## Hardware

- **MCU board:** Nice!Nano v2 (nRF52840, Adafruit UF2 bootloader, SoftDevice s140 v6.1.1). One per node.
- **Radio:** RFM95W / SX1276 LoRa module (915 MHz US ISM), SPI.
- **Goggles:** Skyzone SKY04X (head tracker emits an 8-channel PPM stream on the HT-OUT 3.5 mm jack).
- **Gimbal:** two MG90S servos (pan + tilt). Power them from a **separate 4.8–6 V rail with common ground**.
- **IMU:** MPU-6050 (I2C, addr `0x68`), truck node.
- **Display:** SSD1306 128×64 OLED (I2C, addr `0x3C`), on both nodes.
- **GPS:** any NMEA-over-UART module at 9600 baud, truck node.

### Pin map (Nice!Nano v2)

The `board` crate is the single source of truth; this table mirrors it. Pins are free choices within the
freely-exposed castellated pads (reserved pins: LED `P0.15`, LFXO `P0.00/01`, battery `P0.04`, VCC control `P0.13`,
QSPI flash `P0.19/21/23`).

| Function       | Pin (pad)     | Node   | Notes                                                            |
|----------------|---------------|--------|------------------------------------------------------------------|
| PPM in         | `P0.02` (A0)  | goggle | via ~2.2 kΩ series resistor; idle-**high**, falling-edge markers |
| Re-home button | `P1.00` (D6)  | goggle | active-low + internal pull-up; GPIOTE                            |
| Servo pan      | `P0.22`       | truck  | `PWM0` ch0, 5 V rail, common GND                                 |
| Servo tilt     | `P0.24`       | truck  | `PWM0` ch1 (shares the 50 Hz frame)                              |
| LoRa SCK       | `P1.13`       | both   | `SPI3`                                                           |
| LoRa MOSI      | `P0.11` (D7)  | both   | (not the labelled MOSI pad — that is NFC2)                       |
| LoRa MISO      | `P1.11`       | both   |                                                                  |
| LoRa NSS       | `P0.29` (D20) | both   | `P0.12` is not broken out; `ExclusiveDevice`                     |
| LoRa RESET     | `P0.06`       | both   | active-low Output                                                |
| LoRa DIO0      | `P0.08`       | both   | IRQ Input via GPIOTE                                             |
| I2C SDA        | `P0.17`       | both   | shared bus: MPU-6050 + OLED on `TWISPI0`                         |
| I2C SCL        | `P0.20`       | both   | (internal pull-ups enabled)                                      |
| GPS RX         | `P1.04`       | truck  | `UARTE1`, 9600 baud, `BufferedUarteRx`                           |
| GPS TX         | `P1.06`       | truck  | wired for config only; reader is RX-only                         |

## Architecture

A virtual Cargo workspace. Functionality is split into small, focused crates under `crates/`; the two node binaries stay thin and wire the library crates together.
Each node runs four Embassy tasks that hand data off through latest-value `embassy_sync::Signal`s (lossy by design - stale packets are dropped).

**Goggle node** tasks: `ppm_task` → publishes the latest pan/tilt `Signal`; `lora_task` encodes `Control`, transmits,
and listens ~30 ms for the `Telemetry` reply; `oled_task` renders speed / link state; `button_task` classifies presses
(long-press requests a gimbal re-home).

**Truck node** tasks: `lora_task` receives `Control`, applies it, and replies with the latest `Telemetry`; `servo_task`
maps pan/tilt µs → PWM duty at 50 Hz (with the IMU home applied as a trim); `gps_task` parses NMEA into ground speed + position; `oled_task` shows status.

The LoRa link is **half-duplex bidirectional**: the goggle is the master (TX `Control` ~every 40 ms, then briefly listens),
the truck is the slave (RX `Control`, apply, immediately TX `Telemetry`). One radio per node, collision-free.
Radio config is **SF7 / BW 500 kHz / CR 4/5 @ 915 MHz, 10 dBm** with PA boost — the fastest standard LoRa setting, giving ~10–11 ms one-way latency.

Frames carry an **ExpressLRS-style binding** (`link-id`): a human-readable phrase is hashed at compile time into a
UID → link-id prefix + CRC initializer, so two pairs on the same frequency don't crosstalk. Build both nodes of a pair
with the **same** `BINDING_PHRASE` (defaults to `fabulous-default` if unset).

### Crate map

| Crate          | Kind | Purpose                                                                              |
|----------------|------|--------------------------------------------------------------------------------------|
| `proto`        | lib  | `.proto` schema + `micropb-gen` codegen; the shared `Control`/`Telemetry` wire types |
| `board`        | lib  | Nice!Nano v2 pin-map; the single source of truth for every GPIO assignment           |
| `ppm-decoder`  | lib  | reusable PPM frame decoder (edge timing → per-channel µs)                            |
| `lora-link`    | lib  | LoRa setup + TX/RX helpers, link-health state machine                                |
| `link-id`      | lib  | binding phrase → UID → link-id + CRC frame filter                                    |
| `servo`        | lib  | servo µs → PWM duty mapping                                                          |
| `imu`          | lib  | MPU-6050 boot home/center detection                                                  |
| `display`      | lib  | SSD1306 OLED status rendering                                                        |
| `gps`          | lib  | `no_std`/no-alloc NMEA parser (ground speed + position)                              |
| `geo`          | lib  | pure nav math: deg×1e7 deltas → distance + bearing to home                           |
| `nrf-adapters` | lib  | `embassy-nrf` → `embedded-hal` adapters (PWM, LoRa SPI) shared by drivers            |
| `applog`       | lib  | shared USB-CDC logging + panic/defmt handler + SoftDevice/USB-under-SD init          |
| `nrf-spike`    | bin  | Phase A de-risk spike (SoftDevice + USB-CDC + blinky)                                |
| `ppm-diag`     | bin  | Phase 0 diagnostic: prints per-channel pulse widths + channel count                  |
| `lora-ping`    | bin  | standalone LoRa link bring-up (pinger/ponger roles, OLED status)                     |
| `truck-diag`   | bin  | truck peripheral bring-up: `servo` / `imu` / `oled` / `gps` bins                     |
| `goggle-node`  | bin  | the goggle-side firmware                                                             |
| `truck-node`   | bin  | the truck-side firmware (headless)                                                   |

## Build & flash

Flashing involves copying a `.uf2` onto the Adafruit bootloader's mass-storage volume, and the serial monitor reads the app's own USB-CDC port.
The `justfile` automates all of it.

**Prerequisites:**

```bash
rustup target add thumbv7em-none-eabihf
# protoc must be on PATH (micropb codegen). MSRV >= 1.88 (micropb runtime).
# install `just`, plus `rust-objcopy` (`cargo install cargo-binutils` && `rustup component add llvm-tools`)
```

**Common recipes** (the bin name defaults to `nrf-spike`; pass another to target a node/diagnostic):

```bash
just flash goggle-node            # build -> objcopy -> uf2 -> copy onto the mounted bootloader volume
just monitor                      # open the USB-CDC serial console (newest /dev/cu.usbmodem*)
just flash-monitor truck-node     # flash, then monitor
just flash-pinger                 # LoRa link test, board A
just flash-ponger                 # LoRa link test, board B
just reboot-bootloader            # 1200-baud touch -> reboot a running app into the UF2 bootloader
```

Flashing is **button-free** once a CDC-enumerating app is running: `just flash` does a 1200-baud "touch" that the firmware answers by rebooting into DFU via `GPREGRET`.

`memory.x` (FLASH origin `0x26000`) is the single source of truth for the app flash origin; the `justfile` derives the UF2 base from it so the two can't drift.
A wrong offset writes to a valid address but the app silently never boots (the #1 "it flashed but won't run" trap — see the migration doc).

Flash two pairs that won't interfere by setting the binding phrase at build time:

```bash
BINDING_PHRASE="pair-a" just flash goggle-node
BINDING_PHRASE="pair-a" just flash truck-node     # same phrase on both halves of a pair
```

## Wire protocol

Defined once in `crates/proto/headtrack.proto` and shared by both nodes via `micropb` (`no_std`, no-alloc).
Two messages, scalar/unsigned only to keep air-time low:

- **`Control`** (goggle → truck): `pan_us`, `tilt_us` (raw PPM pulse widths, ~1000–2000 µs), `flags` (bit0 = request re-home).
- **`Telemetry`** (truck → goggle): `speed_cm_s`, `sats`, `fix_quality`, `dist_m`, `bearing_deg`, `nav_valid`.
