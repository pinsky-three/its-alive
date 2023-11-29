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
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use rp_pico::hal::{gpio::FunctionSpi, sio::Sio, spi::Spi};
use smart_leds::{SmartLedsWrite, White, RGBW};
use ws2812_spi::Ws2812;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot_loader"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// const SYS_HZ: u32 = 125_000_000_u32;

#[entry]
fn main() -> ! {
    info!("Program start");

    const DELAY: Milliseconds<u32> = Milliseconds::<u32>(1_000);
    const NUM_LEDS: usize = 8;
    debug_assert_ne!(NUM_LEDS, 0);

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // Our default is 12 MHz crystal input, 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // let frequency = Rate::MHz(3u32);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_function::<FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_function::<FunctionSpi>();
    let _spi_miso = pins.gpio4.into_function::<FunctionSpi>();
    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, (_spi_mosi, _spi_miso, _spi_sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        3u32.MHz(),
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
