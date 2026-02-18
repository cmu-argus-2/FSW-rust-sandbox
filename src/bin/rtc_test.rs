//! # RTC Test Example (USB Serial Logging + I2C Scan)
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

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog
    ).unwrap();
    
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // USB Setup
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Argus")
            .product("RTC Test USB")
            .serial_number("1")])
        .unwrap()
        .device_class(2)
        .build();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS
    );

    // Power gate setup
    let mut periph_pwr_en: Pin<_, FunctionSioOutput, PullUp> = pins.gpio42.reconfigure();
    let _ = periph_pwr_en.set_high();

    // 200ms delay for power stability
    let start_wait = timer.get_counter().ticks();
    while timer.get_counter().ticks() - start_wait < 200_000 {
        usb_dev.poll(&mut [&mut serial]);
    }

    // Use 100kHz for initial testing and internal PullUp just in case
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        pins.gpio46.reconfigure::<FunctionI2C, PullUp>(),
        pins.gpio47.reconfigure::<FunctionI2C, PullUp>(),
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock
    );
    
    let rtc = DS3231::new(0x68);

    let mut last_print = 0;
    let mut has_scanned = false;
    let mut has_checked_power = false;
    
    loop {
        usb_dev.poll(&mut [&mut serial]);
        
        let now = timer.get_counter().ticks();
        
        // Initial I2C Bus Scan
        if !has_scanned && now > 2_000_000 {
            let _ = serial.write(b"\r\n--- Starting I2C Scan (I2C1) ---\r\n");
            for addr in 0..128u8 {
                let mut buf = [0u8; 1];
                // Try reading 1 byte to see if address ACKs
                if i2c.read(addr, &mut buf).is_ok() {
                    let mut out: String<32> = String::new();
                    let _ = writeln!(out, "Found device at 0x{:02x}\r", addr);
                    let _ = serial.write(out.as_bytes());
                }
                usb_dev.poll(&mut [&mut serial]);
            }
            let _ = serial.write(b"--- Scan Complete ---\r\n");
            has_scanned = true;
        }

        if now - last_print > 1_000_000 { // Every 1 second
            last_print = now;
            
            // Check lost power and set if needed (one-time check)
            if !has_checked_power && has_scanned {
                 match rtc.lost_power(&mut i2c) {
                    Ok(true) => {
                        let _ = serial.write(b"RTC Lost Power - Resetting to 2026-02-16 12:00:00\r\n");
                        let test_time = DateTime { year: 2026, month: 2, day: 16, hour: 12, minute: 0, second: 0 };
                        if rtc.set_datetime(&mut i2c, &test_time).is_ok() {
                            let _ = serial.write(b"Time set successfully\r\n");
                        } else {
                            let _ = serial.write(b"Failed to set time\r\n");
                        }
                        has_checked_power = true;
                    }
                    Ok(false) => {
                        let _ = serial.write(b"RTC Maintained Power - Clock is ticking\r\n");
                        has_checked_power = true;
                    }
                    Err(_) => {
                        // Don't set has_checked_power so we try again until I2C is ready
                    }
                 }
            }

            if has_checked_power {
                match rtc.datetime(&mut i2c) {
                    Ok(dt) => {
                        let mut out: String<128> = String::new();
                        let _ = writeln!(out, "Time: {:04}-{:02}-{:02} {:02}:{:02}:{:02}\r", 
                            dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
                        let _ = serial.write(out.as_bytes());
                    }
                    Err(_) => {
                        let _ = serial.write(b"I2C Read Error\r\n");
                    }
                }
            }
        }
    }
}
