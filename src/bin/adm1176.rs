//! adm1176 driver

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::i2c::I2c;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};  
use hal::i2c::ValidAddress;
use embedded_hal::digital::OutputPin;

 use cortex_m::asm::delay;
// USB stuff
use core::fmt::Write;
use heapless::String;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

//drivers
extern crate rp235x_hal_examples;
use rp235x_hal_examples::drivers::adm1176::ADM1176 as ADM1176;

const CONTROL_REG_ADDR: u8 = 0x83; //10000011, MSB 1 indicates it's for extended register, 7 bits for address of CONTROL extended register address
const CONTROL_SWOFF: u8 = !(0x1 << 0);


#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // power up peripherals
    let mut peri_pin = pins.gpio42.into_push_pull_output();
    peri_pin.set_high().unwrap();

    // Configure two pins as being I²C, not GPIO
    // I2C0 only connects to IMU, and it looks like it's not on for the board I'm testing on
    // So use I2C1 instead
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio46.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio47.reconfigure();

    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let sensor = ADM1176::new(0x40);


    let mut on: [u8;2] = [CONTROL_REG_ADDR, CONTROL_SWOFF];


    let mut config: [u8; 1] = [0b00000101];

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")]) //change this to change the name needed for minicom
        .unwrap()
        .max_packet_size_0(64)
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    
    let mut said_hello = false;
    let mut printed = false;
    loop {
        //timer condition needed to wait for peripherals to power up
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {time}\r\n").unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());
        }
        //timer condition not necessary, just for having some time between messages
        if said_hello && !printed && timer.get_counter().ticks() >= 3_000_000 {
            let _ = serial.write(b"Starting I2C scan...\r\n");
            let mut text: String<64> = String::new();
            sensor.config(&mut i2c, &["V_CONT", "I_CONT"]);
            delay(100_000);
            let mut text: String<64> = String::new();
            let (voltage, current) = sensor.read_voltage_current(&mut i2c).unwrap();
            writeln!(&mut text, "Voltage: {voltage}, Current: {current}\r\n").unwrap();
            // for val in &buf {
            //     writeln!(&mut text, "{:#010b}\r\n", val).unwrap();
            // }
            // writeln!(&mut text, "{:?}\r\n", buf).unwrap();
            // for b in &buf {
            //     writeln!(&mut text, "{:#010b}\r", b).unwrap();
            // }
            write!(&mut text, "End Scan\r\n").unwrap();
            let _ = serial.write(text.as_bytes());
            printed = true;
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}



/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"USB Serial Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
