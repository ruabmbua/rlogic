#![no_std]
#![no_main]

use asm::delay;
use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use embedded_hal::digital::v2::OutputPin;
use nb::block;
use pac::{Interrupt, NVIC};
use stm32f1xx_hal::{
    gpio::State,
    pac,
    pac::interrupt,
    prelude::*,
    timer::{Event, Timer},
    usb,
};
use usb_device::bus::UsbBusAllocator;
use usb_device::{
    class_prelude::*,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

static mut USB_BUS: Option<UsbBusAllocator<usb::UsbBusType>> = None;
static mut USB_DEV: Option<UsbDevice<usb::UsbBusType>> = None;
static mut USB_CAPTURE: Option<Capture<usb::UsbBusType>> = None;

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

    let mut timer = Timer::syst(core_peripherals.SYST, &clock_cfg).start_count_down(5.hz());

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

    let usb = usb::Peripheral {
        usb: peripherals.USB,
        pin_dm: dm,
        pin_dp: dp,
    };

    let bus = usb::UsbBus::new(usb);

    unsafe {
        USB_BUS = Some(bus);

        let bus = USB_BUS.as_ref().unwrap();

        USB_CAPTURE = Some(Capture::new(bus, 16));

        let mut usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0xdead, 0xbeef))
            .manufacturer("ruabmbua")
            .product("rlogic")
            // .serial_number("1")
            .device_class(0x03)
            .build();

        USB_DEV = Some(usb_dev);

        NVIC::unmask(Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
    }

    timer.listen(Event::Update);

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
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    write_ep: EndpointIn<'a, B>,
}

impl<B: UsbBus> Capture<'_, B> {
    fn new(alloc: &UsbBusAllocator<B>, max_packet_size: u16) -> Capture<'_, B> {
        Capture {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(8, 255),
            data_if: alloc.interface(),
            write_ep: alloc.bulk(max_packet_size),
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

#[exception]
fn SysTick() {

}

fn usb_interrupt() {
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };
    let capture_class = unsafe { USB_CAPTURE.as_mut().unwrap() };

    if !usb_dev.poll(&mut [capture_class]) {
        return;
    }
}

#[panic_handler]
fn handle_panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}
