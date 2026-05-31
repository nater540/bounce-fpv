//! truck-diag: truck-node peripheral bring-up diagnostics (Phase C: nRF52840). ONE crate, FOUR binaries under
//! `src/bin/` — `servo`, `imu`, `oled`, `gps` — each exercising ONE peripheral in isolation.
//!
//! On the ESP32-C6 this lib carried the shared `#[panic_handler]` + esp-idf app descriptor so all four bins could
//! reuse them. On the nRF52840 that role moved to `applog`: the single workspace `#[panic_handler]` lives there and
//! each bin pulls it in with `use applog as _;`, so this lib no longer owns any runtime scaffolding. It is kept as an
//! empty `#![no_std]` crate purely so the crate layout (and `-p truck-diag` / `--bin` discovery) stays unchanged; all
//! real logic lives in the servo / imu / display / gps driver crates the bins call directly.

#![no_std]
