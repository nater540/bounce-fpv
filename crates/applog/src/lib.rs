//! Shared runtime crate for the nRF52840 binaries. Productizes the hardware-validated Phase A spike into a
//! reusable API so every binary brings up the SoftDevice + USB-CDC logging the same proven way instead of
//! re-deriving it. Three concerns: (1) `init` — enable the s140 SoftDevice (LFXO), turn on its USB power
//! events, seed the vbus detector from USBREGSTATUS, and spawn the SoftDevice + USB-CDC tasks; (2) a logging
//! facade — the `log_println!` macro and a `log`-crate backend, both writing lines over the CDC port; and
//! (3) the single workspace `#[panic_handler]`, picked up by every binary via `use applog as _;`. The
//! 1200-baud bootloader-touch detector (wired into the USB task) plus `reboot_into_uf2_bootloader` give every
//! binary button-free re-flashing. See docs/01-nrf52840-migration.md and the nrf-spike crate for the recipe.
//!
//! Typical binary `main`:
//! ```ignore
//! use applog as _; // pull in the shared panic handler.
//! #[embassy_executor::main]
//! async fn main(spawner: embassy_executor::Spawner) {
//!   let p = applog::init_embassy_nrf();              // embassy-nrf init at SD-safe priorities.
//!   applog::init(spawner, p.USBD, applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck", "d"));
//!   applog::log_println!("truck node up");
//!   log::info!("control loop starting");
//!   // ... spawn application tasks, use `p`'s remaining peripherals ...
//! }
//! ```

#![no_std]

// Image-wide defmt #[global_logger]. lora-phy depends on defmt unconditionally and needs a global logger plus
// the .defmt linker section; since every nRF binary links applog, pulling defmt-rtt in here provides that
// logger for the whole image (paired with `-Tdefmt.x` in the global rustflags), so no binary needs its own
// defmt deps / build.rs. The RTT sink is never drained (no debug probe) — real logging is USB-CDC below.
use defmt_rtt as _;

pub mod logger;
pub mod sd;
pub mod usb;

use core::cell::UnsafeCell;
use core::fmt::Write as _;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_executor::Spawner;
use embassy_nrf::{interrupt, peripherals, Peri, Peripherals};
use log::LevelFilter;
use nrf_softdevice::raw;

// Re-export the facade pieces the macro expands against, plus the leveled `log` macros for convenience so
// downstream crates can `use applog::{info, warn, ...}` without a separate `log` dependency if they prefer.
pub use logger::{init_logger, write_bytes};
pub use log::{debug, error, info, trace, warn, LevelFilter as LogLevel};

// USB device identity presented over the CDC-ACM port. Each binary supplies its own so the host can tell the
// goggle node from the truck node, etc. All strings are `'static` (string literals from the binary).
#[derive(Clone, Copy)]
pub struct UsbIdentity {
  pub vid: u16,
  pub pid: u16,
  pub manufacturer: &'static str,
  pub product: &'static str,
  pub serial_number: &'static str,
}

impl UsbIdentity {
  // Construct an identity. 0x1209 is the pid.codes community VID; 0x0001 is its "test" PID — fine for these
  // internal boards. Pick a stable product/serial per binary so the host names the CDC port consistently.
  pub fn new(
    vid: u16,
    pid: u16,
    manufacturer: &'static str,
    product: &'static str,
    serial_number: &'static str,
  ) -> Self {
    Self { vid, pid, manufacturer, product, serial_number }
  }
}

// Initialize embassy-nrf at SoftDevice-safe interrupt priorities and return the peripheral set. The SoftDevice
// reserves interrupt priorities P0/P1/P4, so embassy's GPIOTE and time-driver interrupts must sit at P2 (P3
// also works) to stay out of the SD's way. Call this FIRST in `main`, before `init`, then pass `p.USBD` into
// `init` and use the remaining peripherals (LED, I2C, SPI, ...) for the application.
pub fn init_embassy_nrf() -> Peripherals {
  let mut config = embassy_nrf::config::Config::default();
  config.gpiote_interrupt_priority = interrupt::Priority::P2;
  config.time_interrupt_priority = interrupt::Priority::P2;
  embassy_nrf::init(config)
}

// Bring up the full SoftDevice + USB-CDC logging stack and install the `log` backend at the given level. This
// enables the s140 SoftDevice (LFXO clock), turns on its USB power SoC events, seeds the vbus detector from the
// current USBREGSTATUS, installs the `log` backend, and spawns the SoftDevice event task and the USB-CDC task
// (which carries the log-pipe drain writer and the bootloader-touch watcher). After this returns, `log_println!`
// and the `log` macros emit over the CDC port and the board is re-flashable via a 1200-baud touch. Requires
// `init_embassy_nrf` (or an equivalent SD-safe `embassy_nrf::init`) to have run first.
pub fn init_with_level(spawner: Spawner, usbd: Peri<'static, peripherals::USBD>, id: UsbIdentity, level: LevelFilter) {
  init_logger(level);
  // Enable the SoftDevice and obtain the shared vbus detector; both tasks below take 'static references to it.
  let (sd, vbus) = sd::enable();
  // The SoftDevice event loop must run continuously or SD calls never complete; it also forwards USB power
  // events into the vbus detector. In embassy-executor 0.10 a #[task] fn returns Result<SpawnToken, SpawnError>;
  // unwrap the token (the pool is sized 1 per task, so this only fails on a double-spawn, which we never do).
  spawner.spawn(sd::softdevice_task(sd, vbus).unwrap());
  spawner.spawn(usb::usb_task(usbd, vbus, id).unwrap());
  // USB drain task is now spawned, so a previous-boot panic message captured in retained RAM can be reported:
  // it lands in LOG_PIPE here and flushes over CDC once the host opens the port. Reported once, then cleared.
  report_retained_panic();
}

// Convenience wrapper at the default log level (Info). See `init_with_level` for the full behavior.
pub fn init(spawner: Spawner, usbd: Peri<'static, peripherals::USBD>, id: UsbIdentity) {
  init_with_level(spawner, usbd, id, LevelFilter::Info);
}

// REUSABLE recipe lifted verbatim from the spike. Arms the Adafruit nRF52 UF2 bootloader's "stay in DFU"
// request and resets, so the board re-enumerates as the UF2 mass-storage volume instead of running the app.
// The bootloader reads GPREGRET on boot and stays in DFU when it equals DFU_MAGIC_UF2_RESET (0x57). The
// SoftDevice owns the POWER peripheral, so GPREGRET must be written through the SD's SVC API, not via the PAC.
// We clear the byte first (the *_set call only ORs bits in) so the value is exactly 0x57, then issue a
// Cortex-M system reset. Public so a binary can also trigger DFU from its own logic. Never returns.
pub fn reboot_into_uf2_bootloader() -> ! {
  // DFU_MAGIC_UF2_RESET from the Adafruit nRF52 bootloader (boards.h). GPREGRET == 0x57 -> stay in UF2 DFU.
  const DFU_MAGIC_UF2_RESET: u32 = 0x57;
  unsafe {
    // gpregret_id 0 selects GPREGRET (not GPREGRET2). Clear all bits, then set exactly the magic, so a stale
    // value can't leave extra bits set. Both are SD-routed SVCs; ignore the NRF_SUCCESS return.
    let _ = raw::sd_power_gpregret_clr(0, 0xFFFF_FFFF);
    let _ = raw::sd_power_gpregret_set(0, DFU_MAGIC_UF2_RESET);
  }
  // System reset. The SD intercepts this and performs an orderly reset; the bootloader then sees the magic.
  cortex_m::peripheral::SCB::sys_reset()
}

// Retained-RAM panic capture. A panic halts (see below), so the async usb_task never runs again and any bytes
// the handler pushes into LOG_PIPE die undrained — the message would be lost over USB-CDC every time. Instead we
// stash the message in a buffer placed in `.uninit` (cortex-m-rt's link.x marks `.uninit.*` NOLOAD and never
// zeroes/initializes it, so its contents survive a soft reset / RAM is not reset by the bootloader), guarded by
// a magic word. On the next boot `init` checks the magic and, if set, logs the previous-boot panic over CDC,
// then clears it. A cold power-on leaves `.uninit` as uninitialized garbage, so the magic almost never matches
// by chance, and a stale-but-cleared store can't re-report. Single-threaded access only (the handler runs once,
// to completion, with the rest of the system halted; init reads it before any task that could panic again).
const PANIC_MAGIC: u32 = 0x504E_4943; // "PNIC" — sentinel marking a freshly-captured panic message.
const PANIC_BUF_LEN: usize = 256; // bounded, no-alloc; longer messages are truncated to fit.

struct PanicStore {
  magic: u32,
  len: usize,
  buf: [u8; PANIC_BUF_LEN],
}

// `.uninit.applog_panic` is matched by link.x's `*(.uninit .uninit.*)` and is NOLOAD, so the linker neither
// zeroes nor loads it — exactly what lets the contents persist across a soft reset. UnsafeCell because we
// mutate it through a shared `static`; all access is serialized by the halt-on-panic / read-once-at-init
// discipline described above, so no locking is needed.
#[unsafe(link_section = ".uninit.applog_panic")]
static PANIC_STORE: PanicCell = PanicCell(UnsafeCell::new(PanicStore { magic: 0, len: 0, buf: [0; PANIC_BUF_LEN] }));

struct PanicCell(UnsafeCell<PanicStore>);
// Safe under the single-access discipline above; the cell is only touched from the panic handler (system
// halted) and once early in `init` before other tasks run.
unsafe impl Sync for PanicCell {}

// A bounded `core::fmt::Write` sink over the retained buffer: formats up to PANIC_BUF_LEN bytes and drops the
// rest, so a long panic message can never overflow the fixed store.
struct PanicBufWriter<'a> {
  store: &'a mut PanicStore,
}

impl core::fmt::Write for PanicBufWriter<'_> {
  fn write_str(&mut self, s: &str) -> core::fmt::Result {
    let remaining = PANIC_BUF_LEN - self.store.len;
    let n = remaining.min(s.len());
    self.store.buf[self.store.len..self.store.len + n].copy_from_slice(&s.as_bytes()[..n]);
    self.store.len += n;
    Ok(())
  }
}

// If a panic message from the previous boot is waiting in retained RAM, surface it over CDC and clear the magic
// so it is reported exactly once. Called early in `init` (after USB setup) so the line lands as soon as the host
// opens the port. The compiler fence keeps the magic-clear from being reordered before the read of the buffer.
fn report_retained_panic() {
  // SAFETY: single-access discipline — runs once at startup before any task that could panic again, and the
  // panic handler (the only other accessor) cannot be running concurrently.
  let store = unsafe { &mut *PANIC_STORE.0.get() };
  if store.magic == PANIC_MAGIC {
    let len = store.len.min(PANIC_BUF_LEN);
    let msg = core::str::from_utf8(&store.buf[..len]).unwrap_or("<non-utf8 panic message>");
    log_println!("PANIC (previous boot): {}", msg);
    compiler_fence(Ordering::SeqCst);
    store.magic = 0; // clear so this panic is reported only once.
  }
}

// The single workspace panic handler, defined ONCE here so every binary links it in via `use applog as _;`
// (the truck-diag lib-crate precedent). Capture the panic message into retained `.uninit` RAM and halt; the
// NEXT boot's `init` reports it over CDC (see report_retained_panic). We do NOT log over CDC here — a halted
// system never drains LOG_PIPE, so that path always loses the message — and we do NOT unwind or reset: halting
// keeps the failure observable (a reset would silently re-run, and on these boards a non-running app shows up
// as the DFU disk re-mounting, per the migration field notes), and the retained store carries it forward.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  // SAFETY: the panic handler runs once with the rest of the system halted, so this is the sole accessor.
  let store = unsafe { &mut *PANIC_STORE.0.get() };
  store.magic = 0; // mark invalid while writing, so a reset mid-format can't surface a half-written message.
  store.len = 0;
  compiler_fence(Ordering::SeqCst);
  let _ = write!(PanicBufWriter { store }, "{}", info); // truncates past PANIC_BUF_LEN; ignore the result.
  compiler_fence(Ordering::SeqCst);
  store.magic = PANIC_MAGIC; // commit: the message is fully written and valid for the next boot to report.
  loop {
    cortex_m::asm::wfe();
  }
}
