#![no_std]
#![no_main]

use asm::delay;
use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use nb::block;
use stm32f1xx_hal::{
    gpio::State,
    pac,
    prelude::*,
    timer::{Event, Timer},
    usb,
};
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};

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
    let mut usb_dev = UsbDeviceBuilder::new(&bus, UsbVidPid(0xdead, 0xbeef))
        .manufacturer("Fake company")
        .product("Logic analyzer")
        .serial_number("1")
        .device_class(0x03)
        .build();

    loop {
        usb_dev.poll(&mut []);

        match timer.wait() {
            Ok(_) => {
                led.toggle().unwrap();
            }
            Err(_) => {}
        }
    }
}

#[panic_handler]
fn handle_panic(_info: &PanicInfo) -> ! {
    loop {
        asm::wfi();
    }
}
