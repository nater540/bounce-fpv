# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

**Bounce FPV** — an FPV head-tracking camera system for RC trucks, written in Rust on bare-metal **nRF52840** (ARM
Cortex-M4F, `no_std`) using **Embassy** for async. Two nodes communicate over a wired RFM95W/SX1276 LoRa link, which
is **half-duplex bidirectional**:

- **Goggle node** — decodes a PPM stream from Skyzone FPV goggles (pan/tilt head-tracking channels), encodes pan/tilt
  into a protobuf `Control` message, transmits over LoRa, and listens for the truck's `Telemetry` reply. Drives a
  status OLED and a re-home push-button.
- **Truck node** (headless) — receives `Control`, drives two MG90S gimbal servos via PWM, reads an MPU-6050 IMU (boot
  home/center trim), reads a UART GPS, and replies with `Telemetry` (ground speed + distance/bearing to home).

> **Platform history.** The project began on the ESP32-C6 (pure `esp-hal` 1.0 path) and **migrated to the nRF52840**
> (Nice!Nano v2) for more GPIO and a future BLE path. **`docs/01-nrf52840-migration.md` is the platform source of
> truth** — board, flash offset, the validated version matrix, the USB-CDC-under-SoftDevice recipe, and the UF2
> flashing workflow. `docs/00-overview.md` is the original reference: its *application architecture* (tasks, LoRa
> link, PPM decode, control flow, the frozen driver crates) still holds, but its **toolchain, HAL, pin-map, and
> flashing sections are obsolete** (ESP-era). Read the migration doc before touching anything platform-level.

> **Status:** the migration is **code-complete (Phases A–D)** and the full workspace builds green. Head-tracking has
> been validated end-to-end on hardware (goggle PPM → LoRa → truck pan/tilt). What remains is mostly on-target
> validation (servo physical motion is blocked on an external 5 V supply; IMU + GPS telemetry are wired and pending
> validation) plus one **open bug**: the link dies intermittently (~500 s at 10 dBm) without auto-recovering — PA
> latch-up suspected; an in-task SX1276 re-init fix is implemented and pending hardware validation. See `README.md`
> for the status table.

## Toolchain & build

- Target: **`thumbv7em-none-eabihf`** (Cortex-M4F, hard-float). It is **prebuilt** — add it with `rustup target add
  thumbv7em-none-eabihf` on **stable** Rust. No `espup`/Xtensa fork, no `build-std`.
- Linker: `cortex-m-rt`'s `link.x` INCLUDEs the workspace-root `memory.x` (each crate's `build.rs` copies it onto the
  linker search path). Global rustflags: `-Tlink.x -Tdefmt.x` (the latter satisfies `lora-phy`'s unconditional
  `defmt` dependency, paired with `applog`'s `defmt-rtt` `#[global_logger]`).
- MSRV floor is driven by dependencies: **Rust ≥ 1.88** (micropb runtime). `protoc` must be on `PATH` for protobuf
  codegen.
- **There is no debug probe.** Flashing copies a `.uf2` onto the Adafruit UF2 bootloader's mass-storage volume; the
  serial monitor reads the app's own USB-CDC port. The `justfile` automates the whole loop.
- **`memory.x` is the single source of truth for the app flash origin (`0x26000`, matching the resident s140 v6.1.1).**
  The `justfile` derives the UF2 base from it so they can't drift — a wrong offset writes to a valid address but the
  app silently never boots (the #1 "flashed but won't run" trap).

Typical commands (via the `justfile`): build with `cargo build --release -p <crate>` or `just build <bin>`; flash with
`just flash <bin>`; monitor with `just monitor`; both with `just flash-monitor <bin>`. LoRa link test:
`just flash-pinger` / `just flash-ponger`. First flash (or after broken CDC) needs one manual **double-tap RESET**;
afterward `just flash` reboots a running app into DFU via a 1200-baud touch, button-free. There is no host test suite
— this is embedded firmware; validate on real hardware. Build a pair with a matching binding phrase via
`BINDING_PHRASE="…" just flash <node>`.

## Architecture

Virtual Cargo workspace. **All member crates live under `crates/`**, and functionality is **split into small, focused,
reusable crates** — drivers, pin-maps, shared protocol/control logic — with the node binaries kept thin (they wire
library crates together and own task structure).

Core crates:

- **`proto`** — owns the `.proto` schema and runs `micropb-gen` in its `build.rs`, re-exporting the generated
  `no_std`/no-alloc `Control`/`Telemetry` types. Both nodes depend on it (schema defined once).
- **`board`** — the Nice!Nano v2 pin-map; the single source of truth for every GPIO assignment, handed out once at
  boot via the `board_pins!` macro so the binaries never hard-code pin numbers.
- **`goggle-node`** (bin) — four tasks: `ppm_task` (→ pan/tilt `Signal`), `lora_task` (encode `Control`, TX, listen
  for the `Telemetry` reply), `oled_task` (status), `button_task` (long-press → re-home).
- **`truck-node`** (bin, headless) — four tasks: `lora_task` (RX `Control`, reply `Telemetry`), `servo_task` (pan/tilt
  µs → PWM duty at 50 Hz, IMU home applied as a trim), `gps_task` (NMEA → speed + position), `oled_task`. A one-shot
  MPU-6050 home detection establishes the center reference at boot.
- **`ppm-diag`** (bin) — Phase 0 diagnostic. Decodes the PPM frame and prints per-channel pulse widths + channel count
  over USB-CDC. **Build and run this first** on any new goggle/PPM setup to confirm channel count, pulse-width range,
  sync-gap length, and which indices carry pan/tilt.

Focused library/diagnostic crates beneath these: `ppm-decoder` (PPM frame decode), `lora-link` (LoRa setup + TX/RX +
link-health state machine), `link-id` (ExpressLRS-style binding phrase → UID → link-id + CRC frame filter), `servo`
(µs → duty), `imu` (MPU-6050 home), `display` (SSD1306), `gps` (NMEA parser), `geo` (distance/bearing nav math),
`nrf-adapters` (embassy-nrf → embedded-hal adapters for PWM and the LoRa SPI bus), `applog` (shared USB-CDC logging +
panic/defmt handler + SoftDevice/USB-under-SD init), `nrf-spike` (Phase A spike), `lora-ping` (standalone link
bring-up), `truck-diag` (per-peripheral bins).

Key cross-cutting patterns:

- **Control-data hand-off uses `Signal` (latest-value, lossy), not a queue** — the servo/TX loop must always act on
  the freshest orientation; stale packets are dropped. Use `Channel` only if real queuing is needed.
- **RawMutex choice:** `CriticalSectionRawMutex` for `static`/cross-executor sharing (the nodes' inter-task `Signal`s
  are `static`, so they use it), `NoopRawMutex` within a single executor. The shared I2C bus (MPU-6050 + OLED on the
  truck) uses `embassy-embedded-hal` `shared_bus` with a `StaticCell`-held `Mutex<NoopRawMutex, Twim>` and one
  `I2cDevice` per peripheral.
- **Never drive servos directly from the RX task.** A fixed-rate `servo_task` reads the latest `Signal`. Power servos
  from a dedicated 5 V rail (common ground with the nRF), not the 3.3 V pin.
- **nRF SERIAL instance aliasing:** SPIM/TWIM/UARTE controllers alias shared hardware blocks, so distinct subsystems
  MUST pick distinct instances. Truck assignment: LoRa SPI → `SPI3`, shared I2C → `TWISPI0`, GPS UART → `UARTE1`;
  servos use `PWM0` (two channels, one 50 Hz frame); PPM/button are GPIOTE.
- **SoftDevice coexistence:** the s140 SoftDevice owns RTC0 (so embassy-time drives RTC1), POWER/CLOCK, and interrupt
  priorities P0/P1/P4 — all peripheral IRQs must sit at P2/P3. `nrf-softdevice` is the workspace's sole
  `critical-section` provider.
- **Board variants:** only one board today (Nice!Nano v2). The `board` crate keeps a role-accessor macro pattern so a
  second board could later be added behind a Cargo feature selecting a different pin module — not separate crates.
- Embassy runs the thread-mode `Executor` on the nRF52840. `InterruptExecutor` is available if PPM timing ever needs
  priority over OLED updates.

## Subagents

This project defines focused subagents under `.claude/agents/`. They carry the relevant domain knowledge and own one
concern each — prefer routing work to the matching agent:

- **`protocol-agent`** — the `.proto` schema and micropb codegen in `crates/proto/` (the cross-node wire format).
- **`firmware-agent`** — the `goggle-node`/`truck-node` binaries, Embassy tasks, control loops, and the
  `Signal`/`Channel`/`Mutex` wiring.
- **`drivers-agent`** — peripheral driver crates: LoRa (lora-phy), MPU-6050, SSD1306, servo PWM, PPM decoder,
  shared I2C bus, board pin-map.
- **`tooling-agent`** — toolchain, build/flash, `.cargo/config.toml`, `memory.x`, the `[workspace.dependencies]`
  version matrix, logging.
- **`bringup-agent`** — hardware bring-up: the Phase 0 `ppm-diag` diagnostic, the standalone LoRa link prototype,
  and on-target validation.

## Version skew (the main footgun)

The Embassy family (`embassy-executor`, `embassy-time`, `embassy-sync`, `embassy-nrf`, `embassy-usb`) and
`nrf-softdevice` must stay mutually compatible — a mismatch produces `_embassy_time_*` undefined-symbol **link
errors**. The validated matrix lives in the root `Cargo.toml` `[workspace.dependencies]` (the single source of truth)
and is documented in `docs/01-nrf52840-migration.md`. The load-bearing pins:

- `embassy-executor` **0.10** with feature `platform-cortex-m` (NOT `arch-cortex-m`); a `#[task]` fn now returns
  `Result<SpawnToken, SpawnError>` — spawn as `spawner.spawn(task(args).unwrap())`.
- `embassy-nrf` **0.10** with `time-driver-rtc1` (mandatory — the SoftDevice owns RTC0), `gpiote`, `unstable-pac`.
  `embassy-time` 0.5, `embassy-sync` 0.8, `embassy-usb` 0.6.
- `nrf-softdevice` + `nrf-softdevice-s140` from **git rev `47d6121`** (the crates.io 0.1.0 release pins `embassy-sync
  ^0.5`, colliding with the workspace's 0.8). Its `critical-section-impl` feature makes it the **sole**
  `critical-section` provider — no other crate may enable a critical-section impl.
- `micropb` and `micropb-gen` must share **major.minor** (both 0.6) — a skew yields `expected Option, found Result`
  errors in the generated module. `heapless` is pinned to **0.9** (micropb's `container-heapless-0-9` feature
  implements `PbWrite` for heapless 0.9 only).

## Code style

(From `docs/00-overview.md` and `.editorconfig`.)

- **Two-space indentation for all languages**, including Rust and TOML. LF line endings, UTF-8, trim trailing
  whitespace, final newline.
- Target ~120 characters per line, and **fill lines toward that budget before wrapping** — comments included. This is
  strictly enforced.
- **Never wrap a comment onto a new line while the text still fits within ~120 chars on the current one.** Wrap only
  when the next word would push past the budget, and when you do, end the line at a natural break point (period,
  comma, clause boundary). E.g. write `// … plus the timestamp of the previous rising edge.` on one line — do not
  split it after `timestamp`.
