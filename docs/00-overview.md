# Bounce FPV — Technical Reference: Rust + Embassy FPV Head-Tracking Camera System for RC Trucks

> **PLATFORM MIGRATED — read `docs/01-nrf52840-migration.md` first.** The project moved off the ESP32-C6 / esp-hal path onto the **nRF52840 (Nice!Nano v2)**. The *application architecture* below (tasks, LoRa link, PPM decode, control flow, the frozen driver crates) still holds, but every **toolchain, HAL, pin-map, SoftDevice, USB-CDC, and flashing** detail in this doc is obsolete — see `01-nrf52840-migration.md` for the validated platform, version matrix, and flashing workflow.

This is a build-ready architecture/reference document for a code-writing agent. It contains crate names, versions, API entry points, hardware specifics, and design patterns. It does **not** contain implementation code.

## TL;DR
- Build on the **pure `esp-hal` 1.0.0 bare-metal path** (no ESP-IDF), with `esp-hal-embassy` 0.9.x as the Embassy executor glue; target `riscv32imac-unknown-none-elf`, flash with `espflash`/`cargo-espflash` v4 over the C6's built-in USB Serial/JTAG.
- Recommended crate stack: `esp-hal` 1.0.0, `esp-hal-embassy` 0.9.x, `embassy-executor` 0.7, `embassy-time` 0.4, `embassy-sync` 0.8 (vendored in esp-hal), `lora-phy` 3.0.1 (SX1276/`Sx1276`), `micropb` 0.6.0 / `micropb-gen` 0.4.1, `mpu6050-dmp` (async) or `mpu6050`, `ssd1306` 0.10 with `async`, `embedded-graphics` 0.8, `embedded-hal-bus`.
- PPM from Skyzone goggles is a single-wire multi-channel pulse train; decode it with GPIO edge-await + `embassy_time::Instant` timestamping. **Build the Phase 0 diagnostic first** to confirm channel count, pulse-width range, and which channels carry pan/tilt.

## Key Findings

### Toolchain & target (decision: pure esp-hal / no_std)
- ESP32-C6 is RISC-V and does **not** require the Xtensa fork or `espup`; just `rustup target add riscv32imac-unknown-none-elf` on stable Rust. (`espup` is only needed for Xtensa chips — ESP32/S2/S3. ESP32-C2/C3 use `riscv32imc-unknown-none-elf`; C6/H2 use `riscv32imac`.)
- esp-hal reached **1.0.0, released 30 October 2025** — the first vendor-backed stable Rust embedded SDK (per Espressif's announcement by the Rust team: "Today, the Rust team at Espressif is excited to announce the official 1.0.0 release for esp-hal, the first vendor-backed Rust SDK!"). The 1.0 stabilization scope was deliberately limited to "Initializing the HAL, `esp_hal::init` and the relevant configuration associated with that" plus a core driver set; **"everything else in esp-hal is now feature-gated behind the unstable feature."** So you must enable `features = ["esp32c6", "unstable"]` to access LEDC, GPIO `wait_for_*`, RMT, and most peripheral detail.
- Flashing/logging: `espflash`/`cargo-espflash` v4. The ESP32-C6 contains a **USB Serial/JTAG controller**, so flashing and serial logging both run over a single USB-C cable with no external UART bridge. For the first flash, hold **BOOT + tap RESET** to enter download mode. `probe-rs` also supports the C6's built-in JTAG.
- `.cargo/config.toml`: `runner = "espflash flash --monitor"`, `target = "riscv32imac-unknown-none-elf"`, rustflags link-arg `-Tlinkall.x`, `[unstable] build-std = ["core"]`.
- `esp-generate` is the current project generator (it replaced the older `esp-template`); generate a skeleton with `esp-generate --chip esp32c6 <name>` and enable "unstable HAL features."

### Embassy on ESP32-C6
- Crates: `esp-hal` (chip feature `esp32c6` + `unstable`), `esp-hal-embassy` (chip feature `esp32c6`), `embassy-executor`, `embassy-time`, `embassy-sync`, `esp-backtrace`, `esp-println`. esp-hal 1.0 internally depends on `embassy-sync` 0.8 and `embassy-futures` 0.1; `esp-hal-embassy` 0.9.x pulls `embassy-executor` ^0.7.0.
- Init pattern: `let p = esp_hal::init(Config::default());` → create a `TimerGroup` and call `esp_hal_embassy::init(timg0.timer0);` (Embassy must be initialized with a timer before spawning tasks) → entry is `#[esp_hal_embassy::main] async fn main(spawner: Spawner)`.
- The ESP32-C6 has two RISC-V cores but they are highly asymmetric: a 160 MHz high-performance (HP) core and a 20 MHz low-power (LP/ULP) core. Treat it as effectively single-core for Embassy — run the thread-mode `Executor` on the HP core. `InterruptExecutor` is available for preemptive priority tasks (e.g. if PPM timing needs higher priority than OLED updates).
- Async drivers in esp-hal 1.0 are typically created in blocking mode and converted with `.into_async()`; async methods carry the `_async` suffix (e.g. `i2c.write_read_async(...)`). The HAL implements `embedded-hal` 1.0 and `embedded-hal-async` traits on its peripherals, which is what lets third-party drivers (lora-phy, ssd1306, mpu6050) bind to it.

### PPM decoding
- PPM (Pulse Position Modulation) for RC carries multiple channels on one wire as a series of pulses; the spacing/position between consecutive pulses encodes each channel value. Per-channel timing is the standard servo range (~1000–2000 µs, ~1500 µs = center), and a long "sync" gap (typically >3 ms) marks the end of a frame and the start of the next. Decoding electronics are deliberately simple — pure edge timing.
- **Skyzone goggles** emit a selectable-channel PPM stream on the HT OUT / 3.5 mm jack. On SKY01/SKY02-class goggles the head tracker defaults to **channel 5 = pan, channel 6 = tilt**, and the active PPM channel is selectable: hold the **TRACK** button while applying power to enter the head-tracker setup menu, then short-press TRACK to cycle channels (it beeps the current channel). So pan/tilt are two channels embedded in a larger PPM frame — Phase 0 must identify their exact indices for your unit and setting.
- Decode approach (Embassy): configure the PPM GPIO as `Input`, then loop `wait_for_rising_edge().await` (or `wait_for_any_edge`), capturing `embassy_time::Instant::now()` at each edge and computing deltas with `Instant::checked_duration_since`. A gap longer than the sync threshold resets the channel index to 0; subsequent inter-pulse intervals populate `channel[0..N]`.
- esp-hal 1.0 exposes `Input::wait_for_rising_edge()/wait_for_falling_edge()/wait_for_any_edge()/wait_for(Event)` — all `unstable`-gated. **Known historical caveat:** early esp-hal `wait_for_*` had reliability bugs on the C-series (issue #657); validate on real hardware, and if jitter is unacceptable, fall back to a GPIO interrupt + free-running timer capture.

### LoRa: lora-phy 3.0.1 (SX1276 / RFM95W)
Use **`lora-phy` 3.0.1** from the **lora-rs** project. The old `embassy-rs/lora-phy` repo is archived ("REPO ARCHIVED - moved to https://github.com/lora-rs/lora-rs"); the crate now lives in the lora-rs workspace and is marked "minimal maintenance" on lib.rs. It is `no_std`, edition 2021, and uses native async-fn-in-trait (stabilized in Rust 1.75), so it builds on stable Rust — **nightly is not required** (verify the exact MSRV against the crate's `Cargo.toml`, as 3.0.1 does not publish an explicit `rust-version`). It drives the SX1276 over `embedded-hal-async` SPI.

**Construction (verbatim pattern from the lora-rs Sx1276 example):**
```text
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Sx127x, Sx1276};
use lora_phy::{mod_params::*, sx127x};
use lora_phy::{LoRa, RxMode};

let config = sx127x::Config { chip: Sx1276, tcxo_used: false, rx_boost: false, tx_boost: false };
let iv = GenericSx127xInterfaceVariant::new(reset, irq, None, None).unwrap();
let mut lora = LoRa::new(Sx127x::new(spi, iv, config), false, delay).await.unwrap();
```
- `sx127x::Config` fields, in order: `chip` (set to `Sx1276` for the RFM95W), `tcxo_used: bool`, `rx_boost: bool`, `tx_boost: bool`. A bare RFM95W has **no TCXO → set `tcxo_used: false`**. PA boost (the RFM95W's PA_BOOST pin) is enabled via `tx_boost: true` if you need full output.
- `GenericSx127xInterfaceVariant::new(reset, irq, None, None)` args: RESET `Output` pin, IRQ/DIO0 `Input` pin, and two `Option` antenna-switch pins (both `None` for a plain module). Returns `Result`.
- `LoRa::new(radio_kind, enable_public_network: bool, delay)` — pass `false` (P2P, not a public LoRaWAN network). The `LoRa` delay generic bound is `DelayNs` in 3.0.1 (older v2 snippets showing `DelayUs` are outdated).

**Method signatures (verbatim, 3.0.1):**
```text
fn create_modulation_params(&mut self, spreading_factor: SpreadingFactor, bandwidth: Bandwidth,
                            coding_rate: CodingRate, frequency_in_hz: u32) -> Result<ModulationParams, RadioError>
fn create_tx_packet_params(&mut self, preamble_length: u16, implicit_header: bool, crc_on: bool,
                           iq_inverted: bool, modulation_params: &ModulationParams) -> Result<PacketParams, RadioError>
fn create_rx_packet_params(&mut self, preamble_length: u16, implicit_header: bool, max_payload_length: u8,
                           crc_on: bool, iq_inverted: bool, modulation_params: &ModulationParams) -> Result<PacketParams, RadioError>
async fn prepare_for_tx(&mut self, mdltn_params: &ModulationParams, tx_pkt_params: &mut PacketParams,
                        output_power: i32, buffer: &[u8]) -> Result<(), RadioError>
async fn tx(&mut self) -> Result<(), RadioError>
async fn prepare_for_rx(&mut self, listen_mode: RxMode, mdltn_params: &ModulationParams,
                        rx_pkt_params: &PacketParams) -> Result<(), RadioError>
async fn rx(&mut self, packet_params: &PacketParams, receiving_buffer: &mut [u8]) -> Result<(u8, PacketStatus), RadioError>
async fn sleep(&mut self, warm_start_if_possible: bool) -> Result<(), RadioError>
```
- **Low-latency modulation enums** (in `lora_phy::mod_params`, re-exported from the `lora-modulation` crate): `Bandwidth::_500KHz`, `SpreadingFactor::_7`, `CodingRate::_4_5`. (Also available: `Bandwidth::_125KHz`/`_250KHz`, `SpreadingFactor::_5`.._12, `CodingRate::_4_5`.._4_8.) SF7/BW500/CR4-5 gives the shortest air-time, which is what head tracking needs.
- **TX flow:** `create_modulation_params(SpreadingFactor::_7, Bandwidth::_500KHz, CodingRate::_4_5, 915_000_000)` → `create_tx_packet_params(preamble, false, true, false, &mdltn)` → `prepare_for_tx(&mdltn, &mut tx_pkt, output_power_i32, &buffer).await` → `tx().await`. **Note:** in 3.0.1 `prepare_for_tx` takes `output_power: i32` and the payload `&[u8]` directly — there is **no separate "boosted" argument**; PA boost is set via `Config.tx_boost`.
- **RX flow:** `create_rx_packet_params(preamble, false, buf.len() as u8, true, false, &mdltn)` → `prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt).await` → `rx(&rx_pkt, &mut buf).await` returns `(received_len: u8, PacketStatus)`. Interrupt-driven RX uses DIO0; in Embassy the `rx().await` future is driven by the DIO0 IRQ pin you passed to the interface variant.
- **Wiring RFM95W → C6:** SPI MOSI/MISO/SCK + NSS (CS, GPIO `Output`), RESET (`Output`), DIO0 (IRQ `Input`). Wrap SPI+CS with `embedded-hal-bus` `ExclusiveDevice` for the radio. 915 MHz → `frequency_in_hz = 915_000_000` (US ISM). A pan/tilt protobuf packet is only a few bytes, so use a short preamble.

### micropb (no_std protobuf)
- `micropb` 0.6.0 (runtime) + `micropb-gen` 0.4.1 (build-dependency code generator) — both by YuhanLiin, MIT OR Apache-2.0, live on crates.io. Unlike `prost` (requires `alloc`), micropb targets **no_std AND no-alloc**, generating fixed-capacity types optimized for binary size/RAM. The embassy-rs `noproto` crate is deprecated in favor of micropb ("Use https://github.com/YuhanLiin/micropb which does everything I was hoping this to do").
- Requires `protoc` installed on PATH at build time. MSRV: micropb (runtime) **1.88.0**; micropb-gen **1.83.0**.
- Workflow: keep the `.proto` in the workspace; in `build.rs` call `micropb_gen::Generator::new()`, optionally `.use_container_heapless()` (or `.use_container_arrayvec()` / `.use_container_alloc()`), then `.compile_protos(&["headtrack.proto"], std::env::var("OUT_DIR").unwrap() + "/headtrack.rs")`. Include the generated module with `include!`.
- For string/bytes/repeated/map fields you must configure a container type and max capacity (enable feature `container-heapless-0-9` and configure max sizes). For our pan/tilt message — two scalar fields (`int32`/`sint32`) — **no containers are needed**, so this complexity is avoided.
- Encode/decode: generated structs implement `MessageEncode`/`MessageDecode`. Encode into a `heapless::Vec<u8, N>` via `PbEncoder`; decode from a byte slice via `PbDecoder` / `message.decode_from_bytes(slice)`. Optional scalar fields are tracked with a compact "hazzer" bitfield instead of `Option<T>` to save space. proto3 semantics only; protobuf enums become "open" newtype structs (`pub struct X(pub i32)`).

### MPU-6050 (I2C IMU)
- Two candidate crates: **`mpu6050`** 0.2.x (blocking embedded-hal; convenient `get_acc_angles()` returns roll/pitch, `get_gyro()`, `get_acc()`, `get_temp()`) and **`mpu6050-dmp`** (async-capable, a fork of drogue-mpu-6050 using only embedded-hal 1.0 traits; supports DMP/quaternion/FIFO/motion-detection with async examples). For Embassy + a shared async I2C bus, prefer `mpu6050-dmp` (async) or `mpu6050-async`. The embassy docs themselves show `Mpu6050::new(i2c_dev)` on a shared async bus.
- I2C address **0x68** (AD0 low) or **0x69** (AD0 high). Init wakes the device from sleep (PWR_MGMT_1) and sets accel/gyro full-scale ranges.
- **Startup home/center pattern:** after power-on, settle ~1–2 s, take N samples (e.g. 100), average accel-derived roll/pitch (and/or a gyro baseline), and store as the home reference. On the truck node this defines the gimbal's auto-home/center orientation at boot.

### SSD1306 OLED (I2C, 128×64)
- `ssd1306` **0.10.0** with `features = ["async"]` provides `Ssd1306Async` built on `embedded_hal_async::i2c::I2c` (so the I2C bus can be shared). Pair with `embedded-graphics` 0.8.
- Init: `let interface = I2CDisplayInterface::new(i2c); let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode(); display.init().await.unwrap();` Use `BufferedGraphicsMode` for drawing, or `TerminalMode` for simple text. I2C address typically **0x3C** (some panels 0x3D). The yellow/blue panels are standard 128×64 SSD1306.
- Text: build a `MonoTextStyle` (`MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(BinaryColor::On).build()`), draw with `Text::with_baseline("...", Point::zero(), style, Baseline::Top).draw(&mut display)`, then `display.flush().await`.

### I2C bus sharing (MPU-6050 + OLED on the truck node)
- Use `embassy-embedded-hal` `shared_bus`: store the I2C peripheral in a `Mutex<NoopRawMutex, I2c>` inside a `static_cell::StaticCell`, then create one `embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice` per peripheral. Both the MPU-6050 driver and `Ssd1306Async` accept the shared `I2cDevice`. For single-task blocking sharing, `embedded-hal-bus` `RefCellDevice` is the simpler alternative.

### MG90S servo PWM (truck node)
- MG90S timing: **20 ms period (50 Hz)**, pulse width spanning roughly **400–2400 µs** for the full ~180° range (datasheet "Pulse Cycle: 20 ms; Pulse Width: 400–2400 µs; Rotational Range: 180°"); a conservative ~1.0–2.0 ms maps a typical 90°/center≈1.5 ms. Operating voltage 4.8–6 V, dead-band ~5 µs, speed ~0.11 s/60° at 4.8 V — drive servos from a separate 5 V supply, not the C6's 3.3 V rail.
- Generate PWM with the ESP32-C6 **LEDC** peripheral. The C6 has **no MCPWM**, LEDC supports **low-speed mode only**, and **all LEDC timers on the C6 share one clock source**. esp-hal 1.0 API (`unstable`): `let mut ledc = Ledc::new(p.LEDC); ledc.set_global_slow_clock(LSGlobalClkSource::APBClk); let mut tmr = ledc.timer::<LowSpeed>(timer::Number::Timer0);` configure `tmr` at 50 Hz, then `let mut ch = ledc.channel(channel::Number::Channel0, pin); ch.configure(channel::config::Config { timer: &tmr, duty_pct, drive_mode: DriveMode::PushPull })`.
- Two servos = two LEDC channels sharing one 50 Hz timer. **Mapping:** with a higher duty resolution at 50 Hz, `duty = (pulse_us / 20000) * max_duty`. Map incoming PPM 1000–2000 µs (or a normalized value) linearly to pulse 1.0–2.0 ms then to duty. LEDC channels implement the embedded-hal `SetDutyCycle` trait, so you can use `max_duty_cycle()` and `set_duty_cycle(u16)` for fractional positioning (the standard servo trick: avoid floats by scaling, e.g. `min_duty = 25 * max_duty / 1000` for 2.5%).
- Optional abstraction: `esp-hal-servo` 0.4 (supports `esp32c6`, optional `async` via `AsyncServo`/`DelayNs`) wraps LEDC with `set_angle()`/`step_async()`. **But it tracks fast-moving esp-hal versions** — verify it compiles against esp-hal 1.0.0 before depending on it; otherwise drive LEDC directly.

### Persistence / calibration storage (NVS / flash)
- `esp-storage` **0.8.x** (chip feature `esp32c6`) implements `embedded-storage` traits over the C6's internal flash. Build on top with either `sequential-storage` (key-value over a flash range with wear-leveling — ideal for small configs) or `esp-nvs` (ESP-IDF-NVS-compatible bare-metal library; needs esp-storage's `low-level` feature plus a `Platform` impl, and partition offsets matching your partition table). Allocate a dedicated config partition/offset.
- **Persist:** servo center offsets (pan/tilt trim), the PPM channel mapping (which indices are pan/tilt + their measured min/max), and optionally the MPU-6050 home reference (the home can instead be re-measured each boot, which is simpler).
- Caveat: an early esp-rs ecosystem table marked `esp-storage` C6 support as incomplete; current `esp-storage` (0.8.x) explicitly lists ESP32-C6 as supported, but test the read/write/erase cycle on hardware.

### Architecture, tasks & sync
- **Workspace layout** (Cargo virtual workspace):
  - `proto/` — shared library crate that owns the `.proto` and runs `micropb-gen` in its `build.rs`, re-exporting the generated types.
  - `goggle-node/` — binary.
  - `truck-node/` — binary.
  - `ppm-diag/` — Phase 0 diagnostic binary.

  Both node binaries depend on `proto`, so the `.proto` schema and generated structs are defined exactly once and shared.
- **Board variants (XIAO vs DevKitC-1)** differ only in pin numbers and antenna handling. Select them with Cargo features (e.g. `board-xiao` / `board-devkit`) that pick a pin-map module — not separate crates. On the **XIAO ESP32-C6** the RF path uses an FM8625H RF switch: **GPIO3 = RF_SWITCH_EN, GPIO14 = RF_ANT_SELECT** — per Seeed's wiki you "set GPIO3 low level to turn on this function," and GPIO14 then selects internal ceramic (low) vs external U.FL (high). This only matters if you use the C6's own WiFi/BLE radio (not the wired RFM95W), but those two pins are reserved on XIAO and should not be used for servos/PPM/SPI.
- **Goggle node tasks:** (1) PPM reader → publishes latest pan/tilt via an `embassy_sync::signal::Signal`; (2) LoRa TX task consumes the latest pan/tilt, encodes with micropb, transmits; (3) optional OLED status task. Use a `Signal` (latest-value, lossy) for head-tracking data — only the freshest sample matters.
- **Truck node tasks:** (1) LoRa RX task decodes packets → `Signal`; (2) servo update task reads the `Signal`, maps to duty, drives both LEDC channels at a fixed rate; (3) one-shot MPU-6050 home-detection at startup that establishes the center reference before the servo loop takes over.
- **embassy-sync primitives:** `Signal<RawMutex, T>` for the latest-value head-tracking hand-off (lossy, ideal for control loops); `Channel` only if you need queuing; `Mutex` for the shared I2C bus. Use `CriticalSectionRawMutex` for `static`/cross-executor sharing, `NoopRawMutex` within a single executor.
- **Latency:** SF7/BW500/CR4-5 minimizes air time; use a `Signal` (not a deep queue) so the servo always acts on the newest orientation and stale packets are dropped.

### Phase 0 diagnostic
- Standalone bin `ppm-diag`: configure the PPM input GPIO, decode the frame (edge-await + `Instant` timestamping), and print each channel's pulse width (µs) plus the detected channel count over the USB Serial/JTAG console via `esp-println` (`println!`) or `defmt`. Run with `espflash flash --monitor`. **Goal:** confirm frame structure, channel count, sync-gap length, and which indices carry pan/tilt before building the full system.

### Logging
- `esp-println` (chip feature `esp32c6` + `log`) works over the built-in USB Serial/JTAG out of the box. For `defmt`, enable `esp-println`'s defmt feature or use `esp-fast-serial` (fast concurrent defmt over the built-in USB Serial/JTAG; note its <2048-byte message limit). `esp-backtrace` supplies the panic handler (`features = ["esp32c6","panic-handler","exception-handler","println"]`).

## Details: representative Cargo.toml (truck node)
> **Stale — see the root `Cargo.toml`.** Since this table was written the C6 Embassy glue moved off `esp-hal-embassy` onto `esp-rtos`: the current matrix is esp-hal `~1.1`, `esp-rtos` 0.3 (feature `embassy`), the `esp-bootloader-esp-idf` app descriptor, embassy-executor 0.10, embassy-time 0.5, esp-println 0.17, and **no** esp-backtrace. The root workspace `Cargo.toml` `[workspace.dependencies]` is the single source of truth for all pins.

```toml
[dependencies]
esp-hal = { version = "1.0.0", features = ["esp32c6", "unstable"] }
esp-hal-embassy = { version = "0.9", features = ["esp32c6"] }
embassy-executor = { version = "0.7", features = ["task-arena-size-20480"] }
embassy-time = "0.4"
embassy-sync = "0.8"
embassy-embedded-hal = "0.5"
esp-backtrace = { version = "0.15", features = ["esp32c6","panic-handler","exception-handler","println"] }
esp-println = { version = "0.13", features = ["esp32c6","log"] }
lora-phy = "3.0.1"
micropb = { version = "0.6", features = ["container-heapless-0-9"] }
mpu6050-dmp = "*"            # confirm latest on crates.io; async-capable
ssd1306 = { version = "0.10", features = ["async"] }
embedded-graphics = "0.8"
embedded-hal-bus = "0.3"
heapless = "0.8"
static_cell = "2"

[build-dependencies]
micropb-gen = "0.4"
```
**Reconcile the Embassy-family versions** (`esp-hal-embassy`, `embassy-executor`, `embassy-time`, `esp-println`, `esp-backtrace`) against the exact compatible set documented for the `esp-hal` 1.0.0 release — these crates move together, and a mismatch produces `_embassy_time_schedule_wake` / `_embassy_time_*` undefined-symbol link errors. The reliable way to lock the matrix is to generate a fresh `esp-generate --chip esp32c6` project and copy its pins.

## Recommendations
1. **Phase 0 first.** Flash `ppm-diag` to a XIAO C6, feed it the goggles' HT OUT, and capture the real channel count, pulse-width extremes, sync-gap duration, and pan/tilt indices. This de-risks every downstream assumption. Advance only when per-channel µs readings are stable and repeatable across head motion.
2. **Lock the crate matrix** by generating a fresh `esp-generate --chip esp32c6` project and copying its `Cargo.toml` / `.cargo/config.toml` version pins; then layer lora-phy, micropb, ssd1306 on top. This avoids Embassy version-skew link errors.
3. **Prototype the LoRa link standalone** (e.g. button→LED) at SF7/BW500/CR4-5, 915 MHz, before integration, to validate NSS/RESET/DIO0 wiring and measure packet round-trip latency.
4. **Use `Signal` for control data**, a shared `Mutex` I2C bus, and run the servo task at a fixed update rate (50–100 Hz) reading the latest `Signal` value — never drive servos directly from the RX interrupt, and power servos from a dedicated 5 V rail (common ground with the C6).
5. **Persist calibration only once it's stable:** start by re-homing the MPU each boot and hard-coding servo center; add `sequential-storage`/`esp-nvs` persistence after the control loop works.
6. **Benchmarks that change the plan:** if `wait_for_*` GPIO edge timing proves jittery for PPM, switch to a dedicated interrupt + free-running timer capture; if LoRa air-time at SF7/BW500 still adds too much latency, consider raw FSK or accept a lower head-tracking update rate.

### Code-style notes for the implementing agent
- Comments should end at a natural break point (period, comma, etc.).
- Target ~120 characters per line.
- Two-space indentation for all languages, including Rust and TOML.

## Caveats
- esp-hal 1.0 stabilized only a core API subset; **LEDC, I2C async specifics, and GPIO `wait_for_*` are behind `unstable`** and may see breaking changes within 1.x minor releases. Pin exact versions.
- The historical esp-hal `wait_for_rising_edge` reliability bug on C3/C6 (issue #657) may not be fully resolved in 1.0 — verify PPM edge timing empirically.
- The `esp-hal-embassy` 0.9.x / `embassy-executor` 0.7 pairing is inferred from current crates.io metadata; confirm against the esp-hal 1.0.0 release notes, since these are the most skew-prone dependencies.
- `mpu6050-dmp`'s exact current version was not pinned here — confirm on crates.io. The plain `mpu6050` crate is blocking-only; wrap it carefully if used under async.
- `esp-hal-servo` tracks esp-hal closely and may lag the 1.0 release; verify before depending on it, otherwise drive LEDC directly.
- `lora-phy` 3.0.1 is marked "minimal maintenance," builds on stable Rust (native async-fn-in-trait, ≥1.75) but publishes no explicit MSRV — verify against its `Cargo.toml`. Its `prepare_for_tx` has **no boosted-flag argument** (boost is `Config.tx_boost`), and the `LoRa` delay bound is `DelayNs`.
- Skyzone pan/tilt default channels (5 = pan, 6 = tilt) are documented for SKY01/SKY02-class goggles and are user-reconfigurable; your specific model and PPM-channel setting must be confirmed via Phase 0.
- External tooling/MSRV requirements: `protoc` on PATH for micropb; Rust ≥1.88 for micropb runtime, ≥1.83 for micropb-gen, ≥1.75 for lora-phy. Ensure the build environment satisfies all of these.
