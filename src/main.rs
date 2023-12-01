#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::debug_assert_ne;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::spi::MODE_0;
use embedded_time::duration::Milliseconds;
use embedded_time::fixed_point::FixedPoint;
// use embedded_time::rate::Extensions;
// use embedded_time::rate::Megahertz;
use panic_probe as _;
use rp_pico::hal;

use rp_pico::hal::fugit::RateExtU32;
use rp_pico::hal::gpio;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use rp_pico::hal::{sio::Sio, spi::Spi};
use smart_leds::{SmartLedsWrite, White, RGBW};
use ws2812_spi::Ws2812;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
// #[link_section = ".boot_loader"]
// #[used]
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const SYS_HZ: u32 = 125_000_000_u32;

#[entry]
fn main() -> ! {
    info!("Program start");

    const DELAY: Milliseconds<u32> = Milliseconds::<u32>(1_000);
    const NUM_LEDS: usize = 8;
    debug_assert_ne!(NUM_LEDS, 0);

    let mut peripherals = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(peripherals.WATCHDOG);

    // Configure the clocks
    //
    // Our default is 12 MHz crystal input, 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = Sio::new(peripherals.SIO);

    let pins = rp_pico::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio2.reconfigure();
    let spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio3.reconfigure();
    let spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio4.reconfigure();

    let spi = Spi::<_, _, _, 8>::new(peripherals.SPI0, (spi_mosi, spi_miso, spi_sclk));

    let spi = spi.init(
        &mut peripherals.RESETS,
        SYS_HZ.Hz(),
        3_000_000u32.Hz(),
        MODE_0,
    );

    let mut ws = Ws2812::new_sk6812w(spi);

    let mut data: [RGBW<u8>; NUM_LEDS] = [RGBW::default(); NUM_LEDS];
    let empty: [RGBW<u8>; NUM_LEDS] = [RGBW::default(); NUM_LEDS];

    // Blink the LED's in a blue-green-red-white pattern.
    for led in data.iter_mut().step_by(4) {
        led.b = 0x10;
    }

    if NUM_LEDS > 1 {
        for led in data.iter_mut().skip(1).step_by(4) {
            led.g = 0x10;
        }
    }

    if NUM_LEDS > 2 {
        for led in data.iter_mut().skip(2).step_by(4) {
            led.r = 0x10;
        }
    }

    if NUM_LEDS > 3 {
        for led in data.iter_mut().skip(3).step_by(4) {
            led.a = White(0x10);
        }
    }

    loop {
        ws.write(data.iter().cloned()).unwrap();
        delay.delay_ms(DELAY.integer());
        ws.write(empty.iter().cloned()).unwrap();
        delay.delay_ms(DELAY.integer());
    }
}
