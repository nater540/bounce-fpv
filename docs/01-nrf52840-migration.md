# nRF52840 Migration Reference (supersedes the ESP32-C6 platform in `00-overview.md`)

The project moved off the ESP32-C6 / esp-hal path onto the **nRF52840** (a different MCU family) because the
XIAO ESP32-C6 broke out too few GPIO for the peripheral-heavy truck, and because the user wants BLE
(lap-times-to-phone) later. `docs/00-overview.md` still documents the *application architecture* (tasks,
LoRa link, PPM, control flow) which is unchanged — but its toolchain, HAL, pin-map, and flashing sections
are obsolete; this doc is the platform source of truth. The full phased plan is in
`~/.claude/plans/delegated-dancing-lake.md`.

## Hardware (validated on-target, Phase A)

- **Board: Nice!Nano v2** (`Board-ID nRF52840-nicenano`), confirmed from `INFO_UF2.TXT`. Adafruit nRF52 UF2
  bootloader 0.6.0, shipping **SoftDevice s140 v6.1.1**. No reset *button* — enter the bootloader by
  double-tapping RESET (the button if present, or bridge the RST pad to GND twice within ~½ s).
- **On-board LED: P0.15** (active-high). (The SparkFun Pro Micro nRF52840 variant uses P0.07 instead.)
- **Application flash origin: `0x26000`** — this is set by the *resident* SoftDevice (s140 v6.1.1 ends at
  0x26000). NOTE the `nrf-softdevice-s140` crate bundles **v7.0.1 bindings** (whose region ends at 0x27000),
  so the offset must track the chip's resident SD, not the crate. A wrong offset writes the app to a valid
  address (no `FAIL.TXT`) but the bootloader finds no vector table at its boot address and silently stays in
  DFU — the #1 "it flashed but won't run" trap.
- **`memory.x`** (workspace root): `FLASH ORIGIN = 0x00026000, LENGTH = 0xCE000`;
  `RAM ORIGIN = 0x20004180, LENGTH = 256K - 0x4180`. The SoftDevice reports its real required RAM base at
  enable; 0x20004180 is a safe over-reservation for s140 v6.1.1.

## Toolchain & version matrix (validated)

- Target **`thumbv7em-none-eabihf`** (Cortex-M4F, prebuilt — no `build-std`). `cortex-m` 0.7, `cortex-m-rt`
  0.7. Linker: cortex-m-rt's `link.x` INCLUDEs `memory.x` (rustflags `-C link-arg=-Tlink.x`); no `-Tlinkall.x`.
- **`embassy-nrf` 0.10.0** (features `["nrf52840", "time-driver-rtc1", "gpiote"]`). `time-driver-rtc1` is
  mandatory — the SoftDevice owns RTC0, so Embassy must use RTC1.
- **`embassy-executor` 0.10.0** with **`platform-cortex-m`** (NOT `arch-cortex-m`) + `executor-thread`. API
  delta: a `#[embassy_executor::task]` fn now returns `Result<SpawnToken, SpawnError>`; spawn as
  `spawner.spawn(task(args).unwrap())`. `embassy-time` 0.5.1, `embassy-sync` 0.8.0, `embassy-usb` 0.6.0.
- **`nrf-softdevice`** must come from **git rev `47d6121`** (the crates.io 0.1.0 release pins `embassy-sync`
  ^0.5, which collides with the workspace's 0.8). `nrf-softdevice-s140` from the same rev. Features
  `["nrf52840", "s140", "ble-peripheral", "critical-section-impl"]` — the `critical-section-impl` feature is
  the workspace's sole `critical-section` provider. `panic-halt` for now (real handler arrives in `applog`).
- Build per-package with `-p <crate>` (NOT `--bin`): while esp-* crates still coexist mid-migration, a
  workspace-wide `--bin` build collides `critical-section` restore-state features. Resolves once Phase B1
  drops the esp crates.

## SoftDevice integration (validated)

- `embassy_nrf::init(config)` with `config.gpiote_interrupt_priority` and `config.time_interrupt_priority`
  set to **P2** (P3 also OK) — the SoftDevice reserves priorities P0/P1/P4.
- `Softdevice::enable(&sd_config)` with an **external-crystal LFCLK** (`NRF_CLOCK_LF_SRC_XTAL`, 20 ppm) — the
  Nice!Nano populates the 32.768 kHz crystal. Run `sd.run_with_callback(...)` on its own task forever.
- The v7.0.1 bindings drive the resident v6.1.1 SD fine for the core enable/power/clock/GPREGRET SVCs.
  **Align the bindings to v6.1.1 (or update the board SD to v7) before relying on BLE.**

## USB-CDC serial over native USB, under the SoftDevice (the hard-won recipe)

There is no built-in USB-Serial/JTAG like the C6 — the app brings up `embassy-usb` CDC-ACM itself, and the
SoftDevice owns POWER/CLOCK, which makes three steps load-bearing (all three were needed before any byte
enumerated):

1. **Bind only `USBD`** in `bind_interrupts!` — NOT `POWER_CLOCK` (the SD owns it).
2. **After `Softdevice::enable`, enable the USB power SoC events** so the SD will report them:
   `sd_power_usbdetected_enable(1)`, `sd_power_usbpwrrdy_enable(1)`, `sd_power_usbremoved_enable(1)`
   (note `usbpwrrdy`, no `e`). Without these the SD never emits the events and `SoftwareVbusDetect` stays
   stuck. `softdevice_task` forwards the `PowerUsb*` SoC events into the `SoftwareVbusDetect`.
3. **Seed `SoftwareVbusDetect` from the live `USBREGSTATUS` at boot** via `sd_power_usbregstatus_get(&mut s)`
   (bit 0 = `VBUSDETECT`, bit 1 = `OUTPUTRDY`) — do NOT use `new(false, false)`. The board is already
   USB-powered when the firmware starts, so the edge-triggered USB events already fired before step 2
   enabled them and never re-fire; `new(false, false)` therefore leaves USB un-powered forever (the classic
   "enumerates only after a power-cycle, not after a reset" symptom). Seeding from the live register makes it
   come up regardless of plug timing.

No manual HFCLK request is needed for USB on the nRF52840 — the USBD manages its own 48 MHz clock once powered.

## Flashing & monitoring (no debug probe)

- UF2 mass-storage: double-tap RESET → the `NICENANO` volume mounts → drop a `.uf2` on it. The `.uf2` is made
  by `scripts/bin2uf2.py` (vendored; byte-identical to `uf2conv.py`) at family `0xADA52840`, base = the
  app flash origin. **`flash_base` in the `justfile` MUST equal `memory.x`'s FLASH ORIGIN** (both `0x26000`).
- `justfile` recipes: `just flash [bin]` (build → uf2 → copy), `just monitor` (auto-selects the newest
  `/dev/cu.usbmodem*`, past always-present fixtures), `just flash-monitor`, `just reboot-bootloader`.
- **Button-free re-flashing:** once a CDC-enumerating app is running, `just flash` does a **1200-baud touch**
  (`stty -f <port> 1200`) which the firmware detects and reboots into DFU via GPREGRET
  (`sd_power_gpregret_clr(0, 0xFFFF_FFFF)` then `sd_power_gpregret_set(0, 0x57)` = `DFU_MAGIC_UF2_RESET`, then
  `SCB::sys_reset()`), then copies — no button. (A firmware with broken CDC has no port to touch, so it needs
  one manual double-tap reset to break the cycle.)

## Debugging the bootloader (field notes)

- **DFU disk re-mounts = the app isn't running** (not written, wrong offset, or a lingering GPREGRET DFU
  magic). It never means "ran then crashed" — a crash hits the `loop {}` panic handler and hangs (no disk).
- **`CURRENT.UF2` (~1.4 MB) is a red herring** — it's the bootloader's full-flash read-back, not your file;
  its size is fixed by flash geometry, not your ~35 KB app.
- **`FAIL.TXT` present = the write was rejected** (wrong family id / address out of range). Absent = the write
  was accepted; if it still won't boot, suspect the offset or a stuck DFU magic (clear with a full
  power-cycle — POR clears the retained GPREGRET).

## Open follow-ups

- **Pin the `nrf-softdevice-s140` bindings to v6.1.1** (or update the board's SD to v7) before BLE — Phase B1.
- Lift the spike's USB-CDC + SoftDevice init + bootloader-touch into the shared `applog`/sd-init crate so all
  binaries reuse this exact validated recipe — Phase B5.
- The 7 frozen driver crates and the `proto` wire format are unchanged by the migration (verified).
