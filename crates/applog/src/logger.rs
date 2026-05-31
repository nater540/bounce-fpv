//! The logging facade: a process-global byte sink that decouples synchronous log call sites from the async
//! USB-CDC writer task. Call sites (`log_println!`, the `log` crate macros) push formatted bytes into a
//! `'static` `Pipe` via NON-BLOCKING writes — a full pipe drops bytes rather than stalling the caller, which
//! keeps logging safe to call from any context (including before USB has enumerated). The USB task drains the
//! pipe and forwards it to the CDC sender (see `usb::run_cdc`). This is the productized replacement for the
//! spike's inline `format_alive` writer.

use core::fmt::{self, Write};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use log::{LevelFilter, Metadata, Record};

// Ring-buffer capacity for queued log bytes. Modest: CDC drains continuously, and on overflow we drop the
// tail of a line rather than block. 1 KiB comfortably holds several lines of burst without back-pressure.
const PIPE_BYTES: usize = 1024;

// CriticalSectionRawMutex: the pipe is a `static` shared between arbitrary call-site contexts (any executor,
// any task) and the single USB drain task, so it needs the cross-context mutex, not NoopRawMutex.
pub(crate) type LogPipe = Pipe<CriticalSectionRawMutex, PIPE_BYTES>;

// The one global sink. `usb::run_cdc` reads from it; the facade below and the `log` backend write to it.
pub(crate) static LOG_PIPE: LogPipe = Pipe::new();

// Push bytes into the global pipe without ever blocking the caller. Partial / dropped writes are intentional:
// logging must never stall the control path, and bytes emitted before the host opens the port are discarded.
pub fn write_bytes(mut bytes: &[u8]) {
  while !bytes.is_empty() {
    match LOG_PIPE.try_write(bytes) {
      Ok(0) => break,             // pipe full — drop the rest of this line rather than block.
      Ok(n) => bytes = &bytes[n..],
      Err(_) => break,            // TryWriteError::Full — same: drop and move on.
    }
  }
}

// A `core::fmt::Write` adapter so the `write!`/`writeln!` machinery (and our macro) can target the pipe. Each
// `write_str` is a non-blocking push; this is what `log_println!` formats into.
pub struct PipeWriter;

impl Write for PipeWriter {
  fn write_str(&mut self, s: &str) -> fmt::Result {
    write_bytes(s.as_bytes());
    Ok(())
  }
}

// `println!`-style facade. Formats the arguments and appends CRLF (CDC terminals expect carriage returns), then
// pushes the line into the global pipe. Non-blocking and usable from any context. Prefer this for ad-hoc lines;
// use the `log` crate macros (info!/warn!/...) when you want levels — both land in the same pipe.
#[macro_export]
macro_rules! log_println {
  ($($arg:tt)*) => {{
    use ::core::fmt::Write as _;
    let mut w = $crate::logger::PipeWriter;
    let _ = ::core::write!(w, $($arg)*);
    $crate::logger::write_bytes(b"\r\n");
  }};
}

// `log` crate backend: routes the standard info!/warn!/error!/debug!/trace! macros into the same CDC pipe so
// downstream crates (drivers, nodes) can use idiomatic leveled logging without knowing about the transport.
struct PipeLogger;

impl log::Log for PipeLogger {
  fn enabled(&self, _metadata: &Metadata) -> bool {
    true // filtering is done globally via the LevelFilter set in init_logger.
  }

  fn log(&self, record: &Record) {
    // "[LEVEL target] message\r\n". A failed write just drops the line (PipeWriter never errors, but the
    // CRLF push can be dropped on a full pipe); that is acceptable for diagnostics.
    let mut w = PipeWriter;
    let _ = write!(w, "[{} {}] {}\r\n", record.level(), record.target(), record.args());
  }

  fn flush(&self) {} // the USB task is the real drain; nothing to flush here.
}

static LOGGER: PipeLogger = PipeLogger;

// Install the `log` backend. Idempotent in effect: `set_logger` only succeeds once, and a second call (e.g. a
// second binary path) is ignored. `init` calls this for the caller; exposed in case a binary wants a custom
// level filter before/without the full USB bring-up.
pub fn init_logger(level: LevelFilter) {
  // Ignore the error: set_logger fails only if a logger is already installed, which is harmless here.
  let _ = log::set_logger(&LOGGER);
  log::set_max_level(level);
}
