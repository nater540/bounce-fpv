//! embassy-usb CDC-ACM bring-up, the log-pipe drain writer, and the 1200-baud bootloader-touch detector,
//! lifted from the validated Phase A spike. The USB device event loop, a writer that drains the global log
//! pipe to the CDC sender, and the touch watcher run concurrently. The SoftDevice supplies the USB HFCLK and
//! the vbus state (see `sd`), so this module never touches CLOCK/POWER directly. Behavior matches the spike;
//! the only change is that the writer drains `logger::LOG_PIPE` instead of formatting an inline counter.

use embassy_futures::join::join3;
use embassy_nrf::interrupt::{self, InterruptExt};
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, peripherals, usb, Peri};
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, State};
use embassy_usb::driver::Driver as UsbDriver;
use embassy_usb::{Builder, Config as UsbConfig};
use static_cell::StaticCell;

use crate::logger::LOG_PIPE;
use crate::UsbIdentity;

// Bind the USBD interrupt to embassy-nrf's USB handler. CLOCK/POWER is owned by the SoftDevice, so we do NOT
// bind POWER_CLOCK here — USB power state arrives via the SD's SoC events (see sd::softdevice_task). This lives
// in applog so binaries never re-bind USBD themselves (a second binding for the same interrupt won't compile).
bind_interrupts!(struct Irqs {
  USBD => usb::InterruptHandler<peripherals::USBD>;
});

// CDC bulk-endpoint MTU. 64 bytes is the full-speed max packet size; writes are chunked to this.
const CDC_PACKET: usize = 64;

// Arduino/Adafruit convention: opening the CDC port at this baud rate signals "reboot into the bootloader".
const TOUCH_BAUD: u32 = 1200;

// The USB-CDC task: builds the CDC-ACM device from the passed USBD peripheral + shared vbus detector, then runs
// the device loop, the log-pipe drain writer, and the bootloader-touch watcher forever. Spawned by the binary
// (pool size 1). `id` carries the USB device identity (VID/PID/strings) so each binary can present its own.
// Never returns. The `vbus` reference MUST be the same one returned by `sd::enable` and given to softdevice_task.
#[embassy_executor::task]
pub async fn usb_task(
  usbd: Peri<'static, peripherals::USBD>,
  vbus: &'static SoftwareVbusDetect,
  id: UsbIdentity,
) -> ! {
  // Lower the USBD interrupt to P2 BEFORE Driver::new (which enables it at the NVIC default P0). The
  // SoftDevice reserves priorities P0/P1/P4, so leaving USBD at P0 would let it preempt the SD's protocol
  // timing. P2 matches the gpiote/time-driver priorities set in init_embassy_nrf and the diag bins' SPIM3/
  // TWISPI0/UARTE1. Centralizing it here means binaries can't forget to bump USBD down out of the SD's range.
  interrupt::USBD.set_priority(interrupt::Priority::P2);
  let driver = Driver::new(usbd, Irqs, vbus);

  let mut usb_config = UsbConfig::new(id.vid, id.pid);
  usb_config.manufacturer = Some(id.manufacturer);
  usb_config.product = Some(id.product);
  usb_config.serial_number = Some(id.serial_number);
  usb_config.max_power = 100;
  usb_config.max_packet_size_0 = CDC_PACKET as u8;

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

  let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), CDC_PACKET as u16);
  let mut usb = builder.build();

  // Split off the control half so the bootloader-touch watcher can observe line-coding / DTR changes
  // independently of the writer. `sender` carries the log path; `control` wakes the watcher and `receiver`
  // is reused only for its `line_coding()` accessor (`ControlChanged` itself can't read it).
  let (mut sender, receiver, control) = class.split_with_control();

  let usb_fut = usb.run();

  // Drain the global log pipe to the CDC sender. Only push once the host has opened the port, otherwise writes
  // would error / back up; while disconnected the pipe simply fills and old bytes are dropped by the writers.
  let write_fut = async {
    let mut buf = [0u8; CDC_PACKET];
    loop {
      sender.wait_connection().await;
      // Block until at least one byte is queued, then forward up to one CDC packet. A disconnect mid-write
      // just loops back to wait_connection above; ignore the write result.
      let n = LOG_PIPE.read(&mut buf).await;
      let _ = sender.write_packet(&buf[..n]).await;
    }
  };

  join3(usb_fut, write_fut, bootloader_touch_watcher(&receiver, &control)).await;
  // join over diverging futures never returns, but the task signature needs `-> !`.
  loop {}
}

// REUSABLE recipe lifted verbatim from the spike. Watches the host-controlled CDC-ACM line state and reboots
// into the Adafruit UF2 bootloader on the Arduino-style "1200-baud touch": the host opens the port at 1200 baud
// (SET_LINE_CODING) and usually closes it again (dropping DTR). We key on the baud rate hitting 1200 so a bare
// `stty -f <port> 1200` — which only issues SET_LINE_CODING, no DTR drop — is sufficient to trigger it.
// `control_changed()` resolves on any line-coding/DTR/RTS change; the latched coding is then read back from the
// receiver half (`ControlChanged` can't read it itself). Never returns — a 1200-baud hit resets into the bootloader.
async fn bootloader_touch_watcher<'d, D: UsbDriver<'d>>(
  receiver: &Receiver<'d, D>,
  control: &ControlChanged<'d>,
) -> ! {
  loop {
    // Wait for the host to change any control line, then inspect the freshly-latched line coding. The
    // `data_rate()` accessor reads the value SET_LINE_CODING stored, so checking it here catches the touch.
    control.control_changed().await;
    if receiver.line_coding().data_rate() == TOUCH_BAUD {
      // Matches the Arduino/Adafruit convention. We do NOT also require DTR to have dropped, so that a plain
      // `stty -f <port> 1200` (no DTR toggle) still reboots the board into the bootloader.
      crate::reboot_into_uf2_bootloader();
    }
  }
}
