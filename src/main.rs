#![no_std]
#![no_main]

mod protocol;

use asm::delay;
use core::{cell::RefCell, mem, panic::PanicInfo};
use cortex_m::{
    asm,
    interrupt::{free as interrupt_free, CriticalSection, Mutex},
};
use cortex_m_rt::{entry, exception};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use mem::MaybeUninit;
use pac::{Interrupt, NVIC, TIM2};
use protocol::Command;
use stm32f1xx_hal::{
    gpio::{Input, PullDown, Pxx, State},
    pac,
    pac::interrupt,
    prelude::*,
    rcc::{Clocks, APB1},
    time::Hertz,
    timer::{CountDownTimer, Event, Timer},
    usb,
};
use usb_device::bus::UsbBusAllocator;
use usb_device::{
    class_prelude::*,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

struct LogicAnalyzer {
    capture_timer: CaptureTimer,
    clocks_config: Clocks,
    apb1_bus: APB1,
    pins: [Pxx<Input<PullDown>>; 4],
    samples: [u8; 64],
    sample_offset: u8,
    lost_samples: u32,
}

impl LogicAnalyzer {
    fn start_capture(&mut self, freq: Hertz) {
        self.lost_samples = 0;
        self.sample_offset = 0;
        self.capture_timer
            .start(freq, &self.clocks_config, &mut self.apb1_bus);
    }

    fn stop_capture(&mut self) {
        self.capture_timer.stop();
    }

    unsafe fn get(cs: &CriticalSection) -> &'static mut LogicAnalyzer {
        &mut *LOGIC_ANALYZER.borrow(cs).borrow_mut().as_mut_ptr()
    }
}

/// Timer used to generate events in logic analyzer capture interval.
enum CaptureTimer {
    Uninit,
    Enabled(CountDownTimer<TIM2>),
    Disabled(TIM2),
}

impl CaptureTimer {
    /// Start generating events at given frequency.
    fn start(&mut self, freq: Hertz, clocks: &Clocks, apb1: &mut APB1) {
        let mut other = CaptureTimer::Uninit;

        mem::swap(self, &mut other);

        if let CaptureTimer::Disabled(tim) = other {
            let mut timer = Timer::tim2(tim, clocks, apb1).start_count_down(freq);
            timer.listen(Event::Update);
            *self = CaptureTimer::Enabled(timer);
        } else {
            mem::swap(self, &mut other);
        }
    }

    /// Stop generating events
    fn stop(&mut self) {
        let mut other = CaptureTimer::Uninit;

        mem::swap(self, &mut other);

        if let CaptureTimer::Enabled(mut count_down) = other {
            count_down.unlisten(Event::Update);
            *self = CaptureTimer::Disabled(count_down.release());
        } else {
            mem::swap(self, &mut other);
        }
    }
}

static mut USB_BUS: Option<UsbBusAllocator<usb::UsbBusType>> = None;
static mut USB_DEV: Option<UsbDevice<usb::UsbBusType>> = None;
static mut USB_CAPTURE: Option<Capture<usb::UsbBusType>> = None;
static LOGIC_ANALYZER: Mutex<RefCell<MaybeUninit<LogicAnalyzer>>> =
    Mutex::new(RefCell::new(MaybeUninit::uninit()));

#[entry]
fn main() -> ! {
    // Acquire peripherals using pac (peripheral access crate)
    // -------------------------------------------------------
    let peripherals = pac::Peripherals::take().unwrap();
    let core_peripherals = pac::CorePeripherals::take().unwrap();

    // Get handles to register clock control & flash controller
    // to reconfigure clock tree.
    //
    // Use external oscillator (hse) on bluepill board (8mhz).
    // Configure PLL to generate maximum sysclk of 48mhz.
    //
    // Generate 24mhz on pclk1 (requirement for using USB controller)
    //
    // The assert checks, if the configuration fulfills requirements
    // of the USB controller.
    // --------------------------------------------------------------
    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();

    let clock_cfg = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clock_cfg.usbclk_valid());

    // Get LED handle.
    // The LED will blink at a fixed interval, signaling to the user,
    // that the device is functional.
    // --------------------------------------------------------------
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc
        .pc13
        .into_push_pull_output_with_state(&mut gpioc.crh, State::Low);

    // Get necessary GPIO`s for USB
    // ----------------------------
    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    let dm = gpioa.pa11;

    // Send USB reset at microcontroller reset.
    // ----------------------------------------
    dp.set_low().unwrap();
    delay(clock_cfg.sysclk().0 / 100);

    let dp = dp.into_floating_input(&mut gpioa.crh);

    // Get digital inputs
    // downgrade() to convert per pin wrapper type into the same generic pin type.
    // ---------------------------------------------------------------------------
    let dios = [
        gpioa.pa0.into_pull_down_input(&mut gpioa.crl).downgrade(),
        gpioa.pa1.into_pull_down_input(&mut gpioa.crl).downgrade(),
        gpioa.pa2.into_pull_down_input(&mut gpioa.crl).downgrade(),
        gpioa.pa3.into_pull_down_input(&mut gpioa.crl).downgrade(),
    ];

    // Set up USB peripheral for usage with usb-device.
    // Needs access to the USB controller, and dp + dm GPIOs.
    // ------------------------------------------------------
    let usb = usb::Peripheral {
        usb: peripherals.USB,
        pin_dm: dm,
        pin_dp: dp,
    };

    let bus = usb::UsbBus::new(usb);

    unsafe {
        // Store USB bus in static context, for access in interrupt handlers.
        // Also get reference to static usb bus.
        // ------------------------------------------------------------------
        USB_BUS = Some(bus);
        let bus = USB_BUS.as_ref().unwrap();

        // Store logic analyzer capture USB class in static context for interrupt
        // handler access.
        // ----------------------------------------------------------------------
        USB_CAPTURE = Some(Capture::new(bus, 64));

        // Initialize USB device with basic descriptors for vid, pid, etc ...
        // ------------------------------------------------------------------
        let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0xdead, 0xbeef))
            .manufacturer("ruabmbua")
            .product("rlogic")
            // .serial_number("1")
            .device_class(0x03)
            .build();

        // Also store USB device handle for access in interrupt handlers.
        USB_DEV = Some(usb_dev);

        // Enable all used interrupts in NVIC interrupt controller.
        // --------------------------------------------------------
        NVIC::unmask(Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        NVIC::unmask(Interrupt::TIM2);
    }

    // Use systick as timer for blinking LED
    // -------------------------------------
    let mut timer = Timer::syst(core_peripherals.SYST, &clock_cfg).start_count_down(5.hz());
    timer.listen(Event::Update);

    let capture_timer = peripherals.TIM2;
    let apb1 = rcc.apb1;

    // Initialize logic analyzer struct in static context for access in interrupt handlers.
    // Uses Mutex from bare-metal, for synchronizing between different interrupt handlers.
    // The mutex is implemented by using "critical sections" -> sections where no interrupts are allowed.
    // --------------------------------------------------------------------------------------------------
    interrupt_free(|cs| {
        let mut logic_analyzer = LOGIC_ANALYZER.borrow(cs).borrow_mut();
        unsafe {
            logic_analyzer.as_mut_ptr().write(LogicAnalyzer {
                capture_timer: CaptureTimer::Disabled(capture_timer),
                clocks_config: clock_cfg,
                apb1_bus: apb1,
                pins: dios,
                lost_samples: 0,
                samples: [0u8; 64],
                sample_offset: 0,
            });
        }
    });

    // Toggle led at fixed interval, keep core idle otherwise
    // ------------------------------------------------------

    loop {
        match timer.wait() {
            Ok(_) => {
                led.toggle().unwrap();
            }
            Err(_) => {}
        }

        asm::wfi();
    }
}

/// USB class implementation for logic analyzer
struct Capture<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
}

impl<B: UsbBus> Capture<'_, B> {
    fn new(alloc: &UsbBusAllocator<B>, max_packet_size: u16) -> Capture<'_, B> {
        Capture {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(32, 255),
            write_ep: alloc.bulk(max_packet_size),
        }
    }

    /// Handle communication
    fn poll(&mut self) {
        let mut buf = [0; 32];

        match self.comm_ep.read(&mut buf[..5]) {
            Ok(_n) => match Command::parse(&buf[..5]) {
                Some(Command::Start(freq)) => {
                    interrupt_free(|cs| unsafe { LogicAnalyzer::get(cs) }.start_capture(freq))
                }
                Some(Command::Stop) => {
                    interrupt_free(|cs| unsafe { LogicAnalyzer::get(cs) }.stop_capture())
                }
                None => {}
            },
            Err(UsbError::WouldBlock) => {}
            Err(_) => {}
        }
    }
}

impl<B: UsbBus> UsbClass<B> for Capture<'_, B> {
    /// Provide USB descriptors for interfaces / endpoints for usb-device.
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface(self.comm_if, 0xff, 0xff, 0)?;
        writer.endpoint(&self.comm_ep)?;
        writer.endpoint(&self.write_ep)?;

        Ok(())
    }
}

#[interrupt]
fn USB_HP_CAN_TX() {
    usb_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

/// Handle USB interrupts. Calls usb stack from usb-device crate.
fn usb_interrupt() {
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };
    let capture_class = unsafe { USB_CAPTURE.as_mut().unwrap() };

    if !usb_dev.poll(&mut [capture_class]) {
        return;
    }

    capture_class.poll();
}

/// Handle capture timer interrupts. Read in logic levels, and write to buffer. Send buffer
/// over USB, when buffer reaches full capacity.
#[interrupt]
fn TIM2() {
    interrupt_free(|cs| {
        let logic_analyzer = unsafe { LogicAnalyzer::get(cs) };
        let capture_class = unsafe { USB_CAPTURE.as_mut().unwrap() };

        let mut val = 0;

        // Shift logic levels of 4 pins into one byte.
        for (i, pin) in logic_analyzer.pins.iter().enumerate() {
            val |= if pin.is_high().unwrap() { 1 } else { 0 } << i;
        }

        let buf_size = logic_analyzer.samples.len();

        if logic_analyzer.sample_offset < buf_size as u8 - 4 {
            // Free space in buffer, write sample at end of buffer.
            // ----------------------------------------------------
            logic_analyzer.samples[logic_analyzer.sample_offset as usize] = val;
            logic_analyzer.sample_offset += 1;
        } else {
            // Buffer is full, try to send via USB, if it does not work increment
            // lost sample counter.
            // ------------------------------------------------------------------
            let lost_sample_bytes = logic_analyzer.lost_samples.to_le_bytes();

            logic_analyzer.samples[buf_size - 4..].copy_from_slice(&lost_sample_bytes);

            match capture_class.write_ep.write(&logic_analyzer.samples) {
                Ok(_) => {
                    logic_analyzer.sample_offset = 0;
                    logic_analyzer.lost_samples = 0;
                }
                Err(_) => {
                    logic_analyzer.lost_samples += 1;
                }
            }
        }

        // Call .wait() on timer to clear it.
        // ----------------------------------
        if let CaptureTimer::Enabled(tim) = &mut logic_analyzer.capture_timer {
            tim.wait().unwrap();
        }
    });
}

/// Overwrite default handler for systick exception, to prevent panic at systick timer update.
#[exception]
fn SysTick() {}

/// Handle rust panics (just halt the core)
#[panic_handler]
fn handle_panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}
