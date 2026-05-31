//! Shared boot scaffolding for the truck-diag binaries. Each bin under `src/bin/` is its own `#![no_std]`
//! `#![no_main]` artifact, but the `#[panic_handler]` and the esp-idf app descriptor are identical across all
//! four — defining them ONCE here (a lib crate the bins depend on) means every bin artifact links these in
//! without four copies. A `#[panic_handler]` in a linked rlib satisfies each downstream binary.
//!
//! This crate is `no_std` itself so it links cleanly into the bare-metal bins. It carries no peripheral logic;
//! that all lives in the servo / imu / display / gps driver crates the bins call directly.

#![no_std]

use esp_println::println;

// Manual panic handler: log the panic over USB Serial/JTAG then park. esp-rtos owns the runtime, so we keep the
// handler minimal rather than pulling in esp-backtrace. Defined here so every bin artifact picks it up.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  println!("PANIC: {}", info);
  loop {}
}

// App descriptor required by the esp-idf second-stage bootloader. Emitted once here; it lands in every bin that
// links this rlib, so the four diagnostic binaries each get a valid descriptor without repeating the macro.
esp_bootloader_esp_idf::esp_app_desc!();
