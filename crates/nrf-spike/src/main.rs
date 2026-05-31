#![no_std]
#![no_main]

// Phase A de-risk spike for the ESP32-C6 -> nRF52840 migration. It proves the full build -> flash ->
// observe loop with the s140 SoftDevice ENABLED, before any real firmware is ported:
//   * embassy-nrf init with SoftDevice-safe interrupt priorities (P2; the SD reserves P0/P1/P4),
//   * Softdevice::enable with an external-crystal LFCLK config, run on its own task,
//   * a 1 Hz blink on the on-board LED,
//   * embassy-usb CDC-ACM emitting one "alive" line/sec, with USB HFCLK obtained THROUGH the SoftDevice
//     (the SD owns POWER/CLOCK, so we feed its SoC power events into a SoftwareVbusDetect rather than
//     driving CLOCK/POWER directly),
//   * a 1200-baud USB-CDC "touch" detector that reboots into the Adafruit UF2 bootloader, so these
//     button-less Pro Micro boards can be re-flashed from the host without holding RESET (see
//     `bootloader_touch_watcher` / `reboot_into_uf2_bootloader` below).
// This is throwaway code; the real logging facade + SD-init helper land in Phase B. The two bootloader-
// touch helpers are written so Phase B5 can lift them verbatim into the shared `applog`/sd-init crate.

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, interrupt, pac, peripherals, usb, Peri};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, State};
use embassy_usb::driver::Driver as UsbDriver;
use embassy_usb::{Builder, Config as UsbConfig};
use nrf_softdevice::{raw, SocEvent, Softdevice};
use static_cell::StaticCell;

// Minimal halt-on-panic for the spike; the real log-then-halt handler lives in `applog` (Phase B5).
use panic_halt as _;

// On-board user LED. CONFIRMED Nice!Nano (INFO_UF2.TXT, Board-ID nRF52840-nicenano): the LED is on P0.15
// (active-high). The SparkFun Pro Micro nRF52840 variant routes it to P0.07 instead — swap the pin if so.
const LED_PIN_IS_P0_15: () = (); // documentation marker — the pin is taken from `p.P0_15` below.

// Bind the USBD interrupt to embassy-nrf's USB handler. CLOCK/POWER is owned by the SoftDevice, so we do
// NOT bind POWER_CLOCK here — USB power state arrives via the SD's SoC events instead (see usb_task).
bind_interrupts!(struct Irqs {
  USBD => usb::InterruptHandler<peripherals::USBD>;
});

// The SoftwareVbusDetect is shared between the SoftDevice SoC-event task (which reports USB power state)
// and the USB driver (which reads it). A 'static reference keeps both halves pointing at one instance.
static VBUS: StaticCell<SoftwareVbusDetect> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf peripherals first. The SoftDevice reserves interrupt priorities P0/P1/P4, so embassy's
  // GPIOTE and time-driver interrupts must sit at P2 (P3 also works) to stay out of the SD's way.
  let mut config = embassy_nrf::config::Config::default();
  config.gpiote_interrupt_priority = interrupt::Priority::P2;
  config.time_interrupt_priority = interrupt::Priority::P2;
  let p = embassy_nrf::init(config);

  // Enable the s140 SoftDevice with an external-crystal (LFXO) low-frequency clock. SparkFun's Pro Micro
  // populates the 32.768 kHz crystal; 20 ppm accuracy is the typical rating. CONFIRM ON-TARGET: a board
  // without the crystal would need NRF_CLOCK_LF_SRC_RC + rc_ctiv/rc_temp_ctiv instead, or it will hang here.
  let sd_config = nrf_softdevice::Config {
    clock: Some(raw::nrf_clock_lf_cfg_t {
      source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
      rc_ctiv: 0,
      rc_temp_ctiv: 0,
      accuracy: raw::NRF_CLOCK_LF_ACCURACY_20_PPM as u8,
    }),
    ..Default::default()
  };
  let sd = Softdevice::enable(&sd_config);

  // The SoftDevice OWNS the POWER peripheral, so it is the only thing that can observe USB VBUS state. By
  // default it does NOT emit the USB power SoC events, so SoftwareVbusDetect would stay (false, false) forever
  // and embassy-usb would never power up USBD / enumerate. Tell the SD to report them: usbdetected ->
  // NRF_EVT_POWER_USB_DETECTED, usbpwrrdy -> NRF_EVT_POWER_USB_POWER_READY, usbremoved ->
  // NRF_EVT_POWER_USB_REMOVED. softdevice_task forwards those into the vbus detector that drives USB power-up.
  // Pass 1 to enable; each returns NRF_SUCCESS (0). Must run AFTER Softdevice::enable (the SVCs need the SD up).
  unsafe {
    assert_eq!(raw::sd_power_usbdetected_enable(1), raw::NRF_SUCCESS, "sd_power_usbdetected_enable failed");
    assert_eq!(raw::sd_power_usbpwrrdy_enable(1), raw::NRF_SUCCESS, "sd_power_usbpwrrdy_enable failed");
    assert_eq!(raw::sd_power_usbremoved_enable(1), raw::NRF_SUCCESS, "sd_power_usbremoved_enable failed");
  }

  // Seed SoftwareVbusDetect from the ACTUAL current USB power state, not (false, false). The USBDETECTED /
  // USBPWRRDY SoC events are EDGE-triggered: on a board that is already USB-powered before the firmware runs
  // (the common case here), those edges fired before Softdevice::enable, so softdevice_task never sees them
  // and a (false, false) start would stay false forever — embassy-usb would wait for power that, as far as it
  // knows, never arrives, and USBD would never enumerate (the classic "only enumerates after a power-cycle,
  // not after reset" bug). Reading USBREGSTATUS via sd_power_usbregstatus_get captures the level at boot so an
  // already-powered board starts correct and powers USBD up immediately; the *_enable calls above still cover
  // later unplug/replug edges. USBREGSTATUS layout: bit 0 = VBUSDETECT (VBUS present), bit 1 = OUTPUTRDY (the
  // USB 3.3 V regulator output is ready). The s140 bindings expose no named masks for these, so use the bits
  // directly. Signature: `unsafe fn sd_power_usbregstatus_get(usbregstatus: *mut u32) -> u32` (NRF_SUCCESS on ok).
  const USBREGSTATUS_VBUSDETECT: u32 = 1 << 0; // VBUS present on the USB connector.
  const USBREGSTATUS_OUTPUTRDY: u32 = 1 << 1; // USB regulator 3.3 V output is ready.
  let mut usbregstatus: u32 = 0;
  let (vbus_detected, output_ready) = unsafe {
    assert_eq!(
      raw::sd_power_usbregstatus_get(&mut usbregstatus),
      raw::NRF_SUCCESS,
      "sd_power_usbregstatus_get failed",
    );
    (usbregstatus & USBREGSTATUS_VBUSDETECT != 0, usbregstatus & USBREGSTATUS_OUTPUTRDY != 0)
  };
  let vbus = VBUS.init(SoftwareVbusDetect::new(vbus_detected, output_ready));

  // Spawn the SoftDevice event loop. It must run continuously or SD calls never complete; we hook its SoC
  // events to drive the SoftwareVbusDetect so USB sees power state without touching POWER/CLOCK directly.
  // In embassy-executor 0.10 a #[task] fn returns Result<SpawnToken, SpawnError>; unwrap the token (the pool
  // is sized 1 per task here, so this only fails on a double-spawn, which we never do) and hand it to spawn.
  spawner.spawn(softdevice_task(sd, vbus).unwrap());
  spawner.spawn(blink_task(Output::new(p.P0_15, Level::Low, OutputDrive::Standard)).unwrap());
  spawner.spawn(usb_task(p.USBD, vbus).unwrap());
}

// Drives the SoftDevice and translates its USB-power SoC events into SoftwareVbusDetect updates. This is
// the SD+USB coexistence contract: the SoftDevice owns POWER, so it is the only thing allowed to observe
// USBDETECTED / USBPWRRDY / USBREMOVED, and it republishes them here for embassy-usb to consume.
#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice, vbus: &'static SoftwareVbusDetect) -> ! {
  sd.run_with_callback(|event| match event {
    SocEvent::PowerUsbDetected => vbus.detected(true),
    SocEvent::PowerUsbRemoved => vbus.detected(false),
    SocEvent::PowerUsbPowerReady => vbus.ready(),
    _ => {}
  })
  .await
}

// 1 Hz blink: the simplest end-to-end sign of life confirming the target, memory.x origin, and SD-enabled
// boot all line up. If this never blinks, the flash/RAM layout is wrong before USB is even in the picture.
#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) -> ! {
  loop {
    led.set_high();
    Timer::after(Duration::from_millis(500)).await;
    led.set_low();
    Timer::after(Duration::from_millis(500)).await;
  }
}

// Brings up embassy-usb CDC-ACM and emits one "nrf-spike: alive N" line per second. The USB device task and
// the writer run concurrently via join; the SoftDevice supplies HFCLK, so we never touch the CLOCK peripheral.
#[embassy_executor::task]
async fn usb_task(usbd: Peri<'static, peripherals::USBD>, vbus: &'static SoftwareVbusDetect) -> ! {
  let driver = Driver::new(usbd, Irqs, vbus);

  // USB descriptors. The VID/PID here are placeholders for the spike; the real device identity is decided
  // alongside the applog backend in Phase B5.
  let mut usb_config = UsbConfig::new(0x1209, 0x0001);
  usb_config.manufacturer = Some("fabulous-fpv");
  usb_config.product = Some("nrf-spike");
  usb_config.serial_number = Some("phase-a");
  usb_config.max_power = 100;
  usb_config.max_packet_size_0 = 64;

  // Descriptor + control buffers live for the whole program, so they are 'static via StaticCell.
  static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
  static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
  static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
  static CDC_STATE: StaticCell<State> = StaticCell::new();

  let mut builder = Builder::new(
    driver,
    usb_config,
    &mut CONFIG_DESC.init([0; 256])[..],
    &mut BOS_DESC.init([0; 256])[..],
    &mut [], // no Microsoft OS descriptors
    &mut CONTROL_BUF.init([0; 64])[..],
  );

  let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
  let mut usb = builder.build();

  // Split off the control half so the bootloader-touch watcher can observe line-coding / DTR changes
  // independently of the writer. `sender` keeps the normal logging path; `control` wakes the watcher and
  // `receiver` is reused only for its `line_coding()` accessor (`ControlChanged` itself can't read it).
  let (mut sender, receiver, control) = class.split_with_control();

  // The USB device event loop, the "alive" writer, and the 1200-baud bootloader-touch watcher run
  // forever side by side; none returns (the watcher only ever exits by resetting the chip).
  let usb_fut = usb.run();
  let write_fut = async {
    let mut counter: u32 = 0;
    loop {
      // Only push bytes once the host has opened the CDC port, otherwise writes would error/back up.
      sender.wait_connection().await;
      let mut buf = [0u8; 32];
      let line = format_alive(&mut buf, counter);
      // Ignore the result: a disconnect mid-write just loops back to wait_connection above.
      let _ = sender.write_packet(line).await;
      counter = counter.wrapping_add(1);
      Timer::after(Duration::from_secs(1)).await;
    }
  };
  join3(usb_fut, write_fut, bootloader_touch_watcher(&receiver, &control)).await;
  // join over diverging futures never returns, but the task signature needs `-> !`.
  loop {}
}

// REUSABLE (Phase B5 lifts this into applog/sd-init verbatim). Watches the host-controlled CDC-ACM line
// state and reboots into the Adafruit UF2 bootloader on the Arduino-style "1200-baud touch": the host
// opens the port at 1200 baud (SET_LINE_CODING) and usually closes it again (dropping DTR). We key on the
// baud rate hitting 1200 so a bare `stty -f <port> 1200` — which only issues SET_LINE_CODING, no DTR drop —
// is sufficient to trigger it. `ControlChanged::control_changed()` resolves on any line-coding/DTR/RTS
// change; the latched coding is then read back from the receiver half (`ControlChanged` can't read it
// itself). This never returns — on a 1200-baud hit it resets the MCU into the bootloader.
async fn bootloader_touch_watcher<'d, D: UsbDriver<'d>>(
  receiver: &Receiver<'d, D>,
  control: &ControlChanged<'d>,
) -> ! {
  loop {
    // Wait for the host to change any control line, then inspect the freshly-latched line coding. The
    // `data_rate()` accessor reads the value SET_LINE_CODING stored, so checking it here catches the touch.
    control.control_changed().await;
    if receiver.line_coding().data_rate() == 1200 {
      // Matches the Arduino/Adafruit convention. We do NOT also require DTR to have dropped, so that a
      // plain `stty -f <port> 1200` (no DTR toggle) still reboots the board into the bootloader.
      reboot_into_uf2_bootloader();
    }
  }
}

// REUSABLE (Phase B5 lifts this into applog/sd-init verbatim). Arms the Adafruit nRF52 UF2 bootloader's
// "stay in DFU" request and resets, so the board re-enumerates as the UF2 mass-storage volume instead of
// running the app. The bootloader reads GPREGRET on boot and stays in DFU when it equals DFU_MAGIC_UF2_RESET
// (0x57). The SoftDevice owns the POWER peripheral, so GPREGRET must be written through the SD's SVC API,
// not via the PAC. We clear the byte first (the *_set call only ORs bits in) so the value is exactly 0x57,
// then issue a Cortex-M system reset. This function never returns.
fn reboot_into_uf2_bootloader() -> ! {
  // DFU_MAGIC_UF2_RESET from the Adafruit nRF52 bootloader (boards.h). GPREGRET == 0x57 -> stay in UF2 DFU.
  const DFU_MAGIC_UF2_RESET: u32 = 0x57;
  unsafe {
    // gpregret_id 0 selects GPREGRET (not GPREGRET2). Clear all bits, then set exactly the magic, so a
    // stale value can't leave extra bits set. Both are SD-routed SVCs; ignore the NRF_SUCCESS return.
    let _ = raw::sd_power_gpregret_clr(0, 0xFFFF_FFFF);
    let _ = raw::sd_power_gpregret_set(0, DFU_MAGIC_UF2_RESET);
  }
  // System reset. The SD intercepts this and performs an orderly reset; the bootloader then sees the magic.
  cortex_m::peripheral::SCB::sys_reset()
}

// Renders "nrf-spike: alive N\r\n" into the caller's buffer without alloc or core::fmt machinery (keeps the
// spike small) and returns the filled slice. N wraps; this is only a liveness counter.
fn format_alive(buf: &mut [u8; 32], n: u32) -> &[u8] {
  const PREFIX: &[u8] = b"nrf-spike: alive ";
  let mut i = 0;
  buf[i..i + PREFIX.len()].copy_from_slice(PREFIX);
  i += PREFIX.len();

  // Decimal-encode n into a small scratch, then copy it back in order.
  let mut digits = [0u8; 10];
  let mut d = 0;
  let mut v = n;
  loop {
    digits[d] = b'0' + (v % 10) as u8;
    v /= 10;
    d += 1;
    if v == 0 {
      break;
    }
  }
  while d > 0 {
    d -= 1;
    buf[i] = digits[d];
    i += 1;
  }

  buf[i] = b'\r';
  buf[i + 1] = b'\n';
  i += 2;
  &buf[..i]
}

// Silence the unused-const warning for the LED-pin documentation marker without an attribute on the item.
const _: () = LED_PIN_IS_P0_15;

// The PAC re-export is referenced indirectly through embassy-nrf; keep the import meaningful by asserting
// the USBD register block exists at compile time (a cheap guard that we built for a USB-capable nRF).
const _USBD_PRESENT: *const () = pac::USBD.as_ptr() as *const ();
