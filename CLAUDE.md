# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FPV head-tracking camera system for RC trucks, written in Rust on bare-metal **ESP32-C6** (RISC-V, `no_std`) using the **pure `esp-hal` 1.0 path** (no ESP-IDF) with **Embassy** for async. Two nodes communicate over a wired RFM95W/SX1276 LoRa link:

- **Goggle node** — decodes a PPM stream from Skyzone FPV goggles (pan/tilt head-tracking channels), encodes pan/tilt with protobuf, transmits over LoRa.
- **Truck node** — receives LoRa packets, drives two MG90S gimbal servos via LEDC PWM, reads an MPU-6050 IMU and an SSD1306 OLED over a shared I2C bus.

The full build-ready reference — crate versions, API entry points, pin maps, hardware specifics, and rationale — lives in **`docs/00-overview.md`**. Read it before implementing any subsystem; it is the source of truth for design decisions and the exact dependency matrix.

> **Status:** the workspace is a skeleton. `Cargo.toml` declares a virtual workspace with `members = []` and `crates/` is empty. Code does not exist yet — you are building it from the doc.

## Toolchain & build

- Target: `riscv32imac-unknown-none-elf` (C6/H2). Add it with `rustup target add riscv32imac-unknown-none-elf` on **stable** Rust — no `espup`/Xtensa fork needed.
- MSRV floor is driven by dependencies: **Rust ≥ 1.88** (micropb runtime). `protoc` must be on `PATH` for protobuf codegen.
- Flash + serial log over the C6's built-in USB Serial/JTAG (single USB-C cable, no UART bridge): `cargo-espflash`/`espflash` v4. The intended cargo runner is `espflash flash --monitor`. First flash: hold **BOOT + tap RESET** to enter download mode.
- `esp-hal` 1.0 stabilized only a core API subset. **LEDC, GPIO `wait_for_*`, I2C async, RMT, etc. are all behind the `unstable` feature** — enable `features = ["esp32c6", "unstable"]`. Pin exact versions; unstable APIs can break within 1.x minors.

Typical per-binary commands (once crates exist): build with `cargo build --release -p <crate>`, flash+monitor with `cargo run --release -p <crate>` (via the espflash runner). There is no host test suite — this is embedded firmware; validate on real hardware.

## Architecture

Virtual Cargo workspace. **All member crates live under `crates/`**, and functionality should be **split into separate crates wherever possible** — favor small, focused, reusable crates (e.g. drivers, pin-maps, shared protocol/control logic) over monolithic binaries. The node binaries should stay thin, wiring together library crates.

Core crates (add focused library crates beneath these as the code grows):

- **`crates/proto/`** — library crate that owns the `.proto` schema and runs `micropb-gen` in its `build.rs`, re-exporting generated `no_std`/no-alloc types. Both node binaries depend on it so the schema is defined **once**.
- **`crates/goggle-node/`** — binary. Tasks: PPM reader → publishes latest pan/tilt to an `embassy_sync::Signal`; LoRa TX task encodes + transmits the latest sample; optional OLED status task.
- **`crates/truck-node/`** — binary. Tasks: LoRa RX → `Signal`; servo update loop reads the `Signal` at a fixed 50–100 Hz and maps to LEDC duty; one-shot MPU-6050 home/center detection at startup before the servo loop runs.
- **`crates/ppm-diag/`** — Phase 0 diagnostic binary. Decodes the PPM frame and prints per-channel pulse widths + channel count over USB Serial/JTAG. **Build and run this first** to confirm channel count, pulse-width range, sync-gap length, and which indices carry pan/tilt before building anything else.

When extracting shared logic, pull it into its own crate under `crates/` rather than a module inside a binary — e.g. LoRa link setup, the PPM decoder, servo/LEDC mapping, and the board pin-maps are all candidates for standalone library crates depended on by the binaries.

Key cross-cutting patterns:

- **Control-data hand-off uses `Signal` (latest-value, lossy), not a queue** — the servo/TX loop must always act on the freshest orientation; stale packets are dropped. Use `Channel` only if real queuing is needed.
- **RawMutex choice:** `CriticalSectionRawMutex` for `static`/cross-executor sharing, `NoopRawMutex` within a single executor. The shared I2C bus (MPU-6050 + OLED on the truck) uses `embassy-embedded-hal` `shared_bus` with a `StaticCell`-held `Mutex<NoopRawMutex, I2c>` and one `I2cDevice` per peripheral.
- **Never drive servos directly from the RX interrupt.** Power servos from a dedicated 5 V rail (common ground with the C6), not the 3.3 V pin.
- **Board variants (XIAO vs DevKitC-1)** differ only in pin numbers — select with Cargo features (`board-xiao` / `board-devkit`) that pick a pin-map module, not separate crates. On XIAO, GPIO3/GPIO14 are reserved for the RF switch — do not use them for servos/PPM/SPI.
- Embassy is effectively single-core here: run the thread-mode `Executor` on the 160 MHz HP core. `InterruptExecutor` is available if PPM timing needs priority over OLED updates.
- esp-hal async drivers are created blocking then converted with `.into_async()`; async methods carry the `_async` suffix.

## Subagents

This project defines focused subagents under `.claude/agents/`. They carry the relevant domain knowledge from `docs/00-overview.md` and own one concern each — prefer routing work to the matching agent:

- **`protocol-agent`** — the `.proto` schema and micropb codegen in `crates/proto/` (the cross-node wire format).
- **`firmware-agent`** — the `goggle-node`/`truck-node` binaries, Embassy tasks, control loops, and `Signal`/`Channel`/`Mutex` wiring.
- **`drivers-agent`** — peripheral driver crates: LoRa (lora-phy), MPU-6050, SSD1306, LEDC servo PWM, PPM decoder, shared I2C bus, board pin-maps.
- **`tooling-agent`** — toolchain, build/flash, `.cargo/config.toml`, the `[workspace.dependencies]` version matrix, logging, persistence, CI.
- **`bringup-agent`** — hardware bring-up: the Phase 0 `ppm-diag` diagnostic, the standalone LoRa link prototype, and on-target validation.

## Version skew (the main footgun)

The Embassy-family crates (`esp-hal-embassy`, `embassy-executor`, `embassy-time`, `esp-println`, `esp-backtrace`) move together. A mismatch produces `_embassy_time_*` undefined-symbol **link errors**. To lock a known-good matrix, generate a fresh `esp-generate --chip esp32c6 <name>` project and copy its `Cargo.toml` / `.cargo/config.toml` pins, then layer `lora-phy`, `micropb`, `ssd1306` on top. See `docs/00-overview.md` for the representative pinned `Cargo.toml`.

## Code style

(From `docs/00-overview.md` and `.editorconfig`.)

- **Two-space indentation for all languages**, including Rust and TOML. LF line endings, UTF-8, trim trailing whitespace, final newline.
- Target ~120 characters per line, and **fill lines toward that budget before wrapping** — comments included. This is strictly enforced.
- **Never wrap a comment onto a new line while the text still fits within ~120 chars on the current one.** Wrap only when the next word would push past the budget, and when you do, end the line at a natural break point (period, comma, clause boundary). E.g. write `// … plus the timestamp of the previous rising edge.` on one line — do not split it after `timestamp`.
