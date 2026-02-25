//! # Auto-Sync Compile-Time RTC Test
#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;
use embedded_hal::i2c::I2c;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use core::fmt::Write;
use heapless::String;
use embedded_hal::digital::OutputPin;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, FunctionSioOutput, Pin, PullUp};
use rp235x_hal_examples::drivers::ds3231::{DS3231, DateTime};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(12_000_000u32, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog).unwrap();
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // USB Setup
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(pac.USB, pac.USB_DPRAM, clocks.usb_clock, true, &mut pac.RESETS));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default().manufacturer("Argus").product("RTC Sync")])
        .unwrap().device_class(2).build();

    // Hardware Power
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut pwr: Pin<_, FunctionSioOutput, PullUp> = pins.gpio42.reconfigure();
    let _ = pwr.set_high();

    // I2C Setup
    let mut i2c = hal::I2C::i2c1(pac.I2C1, pins.gpio46.reconfigure(), pins.gpio47.reconfigure(), 100.kHz(), &mut pac.RESETS, &clocks.system_clock);
    let rtc = DS3231::new(0x68);

    let mut last_print = 0;
    let mut has_checked_startup = false;

    loop {
        usb_dev.poll(&mut [&mut serial]);
        let now = timer.get_counter().ticks();

        // 1. STARTUP & AUTO-SYNC (Using BUILD_EPOCH)
        if !has_checked_startup && now > 2_000_000 {
            if let Ok(dt) = rtc.datetime(&mut i2c) {
                if dt.year < 2025 {
                    // This value is baked in at the exact second you ran 'cargo run'
                    let compile_time: i64 = env!("BUILD_EPOCH").parse().unwrap_or(1771891200);
                    let _ = rtc.set_unix_time(&mut i2c, compile_time);
                    let _ = serial.write(b"\r\n[AUTO-SYNC] Clock calibrated to build time.\r\n");
                } else {
                    let _ = serial.write(b"\r\n[SYSTEM] Battery OK - Resuming Time.\r\n");
                }
            }
            has_checked_startup = true;
        }

        // 2. Status Print
        if now - last_print > 1_000_000 {
            last_print = now;
            if let Ok(dt) = rtc.datetime(&mut i2c) {
                let mut out: String<128> = String::new();
                let _ = writeln!(out, "{:02}/{:02}/{:04} | {:02}:{:02}:{:02} UTC\r", 
                    dt.day, dt.month, dt.year, dt.hour, dt.minute, dt.second);
                let _ = serial.write(out.as_bytes());
            }
        }
    }
}

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"RTC Auto-Sync"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
