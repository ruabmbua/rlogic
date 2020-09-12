#![no_std]
#![no_main]
#![feature(asm)]

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

enum CaptureTimer {
    Uninit,
    Enabled(CountDownTimer<TIM2>),
    Disabled(TIM2),
}

impl CaptureTimer {
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
    let peripherals = pac::Peripherals::take().unwrap();
    let core_peripherals = pac::CorePeripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();

    let clock_cfg = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clock_cfg.usbclk_valid());

    // Get LED
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc
        .pc13
        .into_push_pull_output_with_state(&mut gpioc.crh, State::Low);

    // Send USB reset
    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    let dm = gpioa.pa11;

    dp.set_low().unwrap();
    delay(clock_cfg.sysclk().0 / 100);

    let dp = dp.into_floating_input(&mut gpioa.crh);

    // Digital inputs
    let dios = [
        gpioa.pa0.into_pull_down_input(&mut gpioa.crl).downgrade(),
        gpioa.pa1.into_pull_down_input(&mut gpioa.crl).downgrade(),
        gpioa.pa2.into_pull_down_input(&mut gpioa.crl).downgrade(),
        gpioa.pa3.into_pull_down_input(&mut gpioa.crl).downgrade(),
    ];

    let usb = usb::Peripheral {
        usb: peripherals.USB,
        pin_dm: dm,
        pin_dp: dp,
    };

    let bus = usb::UsbBus::new(usb);

    unsafe {
        USB_BUS = Some(bus);

        let bus = USB_BUS.as_ref().unwrap();

        USB_CAPTURE = Some(Capture::new(bus, 64));

        let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0xdead, 0xbeef))
            .manufacturer("ruabmbua")
            .product("rlogic")
            // .serial_number("1")
            .device_class(0x03)
            .build();

        USB_DEV = Some(usb_dev);

        NVIC::unmask(Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        NVIC::unmask(Interrupt::TIM2);
    }

    let mut timer = Timer::syst(core_peripherals.SYST, &clock_cfg).start_count_down(5.hz());

    timer.listen(Event::Update);

    let capture_timer = peripherals.TIM2;
    let apb1 = rcc.apb1;

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

struct Capture<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointOut<'a, B>,
    data_if: InterfaceNumber,
    write_ep: EndpointIn<'a, B>,
}

impl<B: UsbBus> Capture<'_, B> {
    fn new(alloc: &UsbBusAllocator<B>, max_packet_size: u16) -> Capture<'_, B> {
        Capture {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(32, 255),
            data_if: alloc.interface(),
            // read_ep: alloc.interrupt(32, 255),
            write_ep: alloc.bulk(max_packet_size),
        }
    }

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
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface(self.comm_if, 0xff, 0xff, 0)?;
        writer.endpoint(&self.comm_ep)?;

        writer.interface(self.data_if, 0xff, 0xff, 0)?;
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

#[interrupt]
fn TIM2() {
    interrupt_free(|cs| {
        let logic_analyzer = unsafe { LogicAnalyzer::get(cs) };
        let capture_class = unsafe { USB_CAPTURE.as_mut().unwrap() };

        let mut val = 0;

        for (i, pin) in logic_analyzer.pins.iter().enumerate() {
            val |= if pin.is_high().unwrap() { 1 } else { 0 } << i;
        }

        let buf_size = logic_analyzer.samples.len();

        if logic_analyzer.sample_offset < buf_size as u8 - 4 {
            logic_analyzer.samples[logic_analyzer.sample_offset as usize] = val;
            logic_analyzer.sample_offset += 1;
        } else {
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

        if let CaptureTimer::Enabled(tim) = &mut logic_analyzer.capture_timer {
            tim.wait().unwrap();
        }
    });
}

#[exception]
fn SysTick() {}

fn usb_interrupt() {
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };
    let capture_class = unsafe { USB_CAPTURE.as_mut().unwrap() };

    if !usb_dev.poll(&mut [capture_class]) {
        return;
    }

    capture_class.poll();
}

#[panic_handler]
fn handle_panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}
