//! SoftDevice (s140) bring-up and its event loop, lifted verbatim from the validated Phase A spike. The
//! SoftDevice owns the POWER and CLOCK peripherals, so it is the only thing that can observe USB VBUS state
//! and supply the USB HFCLK; this module enables it, turns on the USB power SoC events, seeds the
//! `SoftwareVbusDetect` from the current `USBREGSTATUS`, and runs the task that republishes power events into
//! that detector for embassy-usb to consume. Behavior is identical to the spike — only the structure changed.

use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use nrf_softdevice::{raw, SocEvent, Softdevice};
use static_cell::StaticCell;

// The SoftwareVbusDetect is shared between the SoftDevice SoC-event task (which reports USB power state) and
// the USB driver (which reads it). A 'static reference keeps both halves pointing at one instance.
static VBUS: StaticCell<SoftwareVbusDetect> = StaticCell::new();

// USBREGSTATUS layout (no named masks in the s140 bindings, so the bits are used directly): bit 0 = VBUSDETECT
// (VBUS present on the connector), bit 1 = OUTPUTRDY (the USB 3.3 V regulator output is ready).
const USBREGSTATUS_VBUSDETECT: u32 = 1 << 0;
const USBREGSTATUS_OUTPUTRDY: u32 = 1 << 1;

// Enable the s140 SoftDevice with an external-crystal (LFXO) low-frequency clock, turn on the USB power SoC
// events, and seed a `'static` SoftwareVbusDetect from the actual boot-time USB power level. Returns the
// enabled SoftDevice and the shared vbus detector; the caller spawns `softdevice_task(sd, vbus)` and hands the
// same `vbus` to the USB driver. MUST run after `embassy_nrf::init` (so the PAC is configured) and the
// returned SoftDevice handle must outlive the program (it does: `Softdevice::enable` yields a `'static`).
pub fn enable() -> (&'static Softdevice, &'static SoftwareVbusDetect) {
  // External-crystal (LFXO) low-frequency clock. SparkFun's Pro Micro / Nice!Nano populate the 32.768 kHz
  // crystal; 20 ppm is the typical rating. A board WITHOUT the crystal would need NRF_CLOCK_LF_SRC_RC +
  // rc_ctiv/rc_temp_ctiv instead, or it will hang here — confirm on-target if porting to new hardware.
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

  // The SoftDevice does not emit the USB power SoC events by default, so SoftwareVbusDetect would stay
  // (false, false) forever and embassy-usb would never power up USBD / enumerate. Enable them: usbdetected ->
  // NRF_EVT_POWER_USB_DETECTED, usbpwrrdy -> NRF_EVT_POWER_USB_POWER_READY, usbremoved -> NRF_EVT_POWER_USB_REMOVED.
  // softdevice_task forwards those into the vbus detector. Pass 1 to enable; each returns NRF_SUCCESS (0). Must
  // run AFTER Softdevice::enable (the SVCs need the SD up).
  unsafe {
    assert_eq!(raw::sd_power_usbdetected_enable(1), raw::NRF_SUCCESS, "sd_power_usbdetected_enable failed");
    assert_eq!(raw::sd_power_usbpwrrdy_enable(1), raw::NRF_SUCCESS, "sd_power_usbpwrrdy_enable failed");
    assert_eq!(raw::sd_power_usbremoved_enable(1), raw::NRF_SUCCESS, "sd_power_usbremoved_enable failed");
  }

  // Seed SoftwareVbusDetect from the ACTUAL current USB power state, not (false, false). The USBDETECTED /
  // USBPWRRDY SoC events are EDGE-triggered: on a board already USB-powered before the firmware runs (the
  // common case), those edges fired before Softdevice::enable, so softdevice_task never sees them and a
  // (false, false) start would stay false forever — embassy-usb would wait for power that never arrives and
  // USBD would never enumerate (the classic "only enumerates after a power-cycle, not after reset" bug).
  // Reading USBREGSTATUS via sd_power_usbregstatus_get captures the level at boot so an already-powered board
  // starts correct and powers USBD up immediately; the *_enable calls above still cover later unplug/replug.
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

  (sd, vbus)
}

// Drives the SoftDevice and translates its USB-power SoC events into SoftwareVbusDetect updates. This is the
// SD+USB coexistence contract: the SoftDevice owns POWER, so it is the only thing allowed to observe
// USBDETECTED / USBPWRRDY / USBREMOVED, and it republishes them here for embassy-usb to consume. Spawned by
// the binary (the `#[task]` pool is sized 1; spawning it twice would fail). Never returns.
#[embassy_executor::task]
pub async fn softdevice_task(sd: &'static Softdevice, vbus: &'static SoftwareVbusDetect) -> ! {
  sd.run_with_callback(|event| match event {
    SocEvent::PowerUsbDetected => vbus.detected(true),
    SocEvent::PowerUsbRemoved => vbus.detected(false),
    SocEvent::PowerUsbPowerReady => vbus.ready(),
    _ => {}
  })
  .await
}
