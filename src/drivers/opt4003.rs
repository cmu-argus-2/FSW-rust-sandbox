use rp235x_hal as hal;
use embedded_hal::i2c::I2c as I2C;
use core::result::Result;
use hal::i2c::Error as I2cError;
use embedded_hal_0_2::blocking::i2c::WriteRead;
use embedded_hal_0_2::blocking::i2c::Write;
use embedded_hal_0_2::blocking::i2c::Read;
use crate::drivers::opt4003::hal::i2c::Error;
use rp235x_hal::pac;
use rp235x_hal::timer::Timer;
use embedded_hal::delay::DelayNs;
use heapless::Vec;
use fugit::ExtU64;

// Registers as descirbed in page 25 of datasheet
const RESULT_MSB_CH0: u8 = 0x00;
const RESULT_LSB_CH0: u8 = 0x01;
// RESULT_MSB_CH1 = 0x02
// RESULT_LSB_CH1 = 0x03
// FIFO_MSB_CH0 = 0x04
// FIFO_LSB_CH0 = 0x05
// FIFO_MSB_CH1 = 0x06
// FIFO_LSB_CH1 = 0x07
// THRESHOLD_L = 0x08
// THRESHOLD_H = 0x09
const CONFIGURATION: u8 = 0x0A;
const FLAGS: u8 = 0x0C;
const DEVICE_ID: u8 = 0x11;

// Errors based on flags
pub enum Errors {
    LightSensorHigherThanThreshold,
    LightSensorLowerThanThreshold,
    LightSensorOverflow,
}

pub struct OPT4003 <I2C> {
    // buffer
    buf: [u8; 3],

    // i2c bus
    i2c_device: I2C,

    // address of the device
    address: u8,

    // configuration settings
    quick_wakeup: bool,
    lux_range: u8,
    conversion_time: u8,
    operating_mode: u8,
    latch: bool,
    int_pol: bool,
    fault_count: u8,

    // flags
    overload_flag: bool,
    conversion_ready_flag: bool,
    flag_h: bool,
    flag_l: bool,
}

// Our default conversion time is 0b1000, which corresponds to 100ms.
// Our default lux range is automatic
// TODO: Operating mode should be 0b00??
impl<I2C, E> OPT4003<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>
{
    pub fn new(mut i2c: I2C, addr: u8) -> Self {

        // Read flags from device.
        let mut reg = [0u8; 1];
        i2c.write_read(addr, &[FLAGS], &mut reg);
        let overload_flag = ((reg[0] >> 3) & 1) == 1;
        let conversion_ready_flag = ((reg[0] >> 2) & 1) == 1;
        let flag_h = ((reg[0] >> 1) & 1) == 1;
        let flag_l = (reg[0] & 1) == 1;

        // Send config to device.
        let mut config: u16 = 0;
        
        // Assemble configuration. 
        // Refer to configuration register on datasheet for specifics.
        config |= 1u16 << 3;    // latch bit
        config |= 0b1100 << 10; // lux range
        config |= 0b1000 << 6;  // conversion time
        config |= 0b11 << 4;    // operating mode
        config |= 0b00 as u16;  // fault count

        let bytes = config.to_be_bytes();
        i2c.write(addr, &[CONFIGURATION, bytes[0], bytes[1]]);

        Self {
            buf: [0u8; 3],
            i2c_device: i2c,
            address: addr,
            quick_wakeup: false,
            lux_range: 0b1100,
            conversion_time: 0b1000,
            operating_mode: 0b11,
            latch: true,
            int_pol: false,
            fault_count: 0b00,
            overload_flag: overload_flag,
            conversion_ready_flag: conversion_ready_flag,
            flag_h: flag_h,
            flag_l: flag_l,
        }
    }

    pub fn read_u16(&mut self, addr: u8) -> Result<(), E>
    {   
        // Write to addr register, then read to buf[0] and buf[1]
        self.i2c_device.write_read(self.address, &[addr], &mut self.buf)?;

        return Ok(());
    }

    pub fn check_id(&mut self) -> Result<bool, E>
    {
        // read device id register. Automatically propogate error if one occurs. 
        self.read_u16(DEVICE_ID)?;
        let didl: u8 = (self.buf[0] >> 4) & ((1 << 2) - 1); // bits 13-12
        if didl != 0 { return Ok(false); }
        
        let mut didh: u16 = (self.buf[0] & ((1 << 4) - 1)).into(); // bits 11-8
        didh = (didh << 8) + self.buf[1] as u16; // adding bits 7-0
        if didh != 0x221 { return Ok(false); }

        return Ok(true);
    }

    pub fn get_exp_msb(&mut self, register: u8) -> Result<(u8, u16), E>
    {
        // read register. Automatically propogate error if one occurs. 
        self.read_u16(register)?;

        // separate components
        let exponent: u8 = (self.buf[0] >> 4) & ((1 << 4) - 1);  // bits 15-12

        let mut result_msb: u16 = (self.buf[0] & ((1 << 4) - 1)).into();  // 11-8
        result_msb = result_msb << 8;  // pad
        result_msb += self.buf[1] as u16;  // add bits 7-0

        return Ok((exponent, result_msb));
    }

    pub fn get_lsb_counter_crc(&mut self, register: u8) -> Result<(u8, u8, u8), E>
    {
        // read register. Automatically propogate error if one occurs. 
        self.read_u16(register)?;

        // separate components
        let result_lsb: u8 = self.buf[0]; // bits 15-8
        let counter = (self.buf[1] >> 4) & ((1 << 4) - 1); // bits 7-4
        let crc = self.buf[1] & ((1 << 4) - 1); // bits 3-0

        return Ok((result_lsb, counter, crc));
    }

    pub fn result_of_addr(&mut self) -> Result<f32, E>
    {
        // Note: the just_lux input from the previous version has been removed.
        // We do not support returning the crc or counter.

        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = rp235x_hal::Watchdog::new(pac.WATCHDOG);
        let clocks = rp235x_hal::clocks::init_clocks_and_plls(
            12_000_000u32, // depends on board
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        ).ok().unwrap();

        let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
        let start_time = timer.get_counter() + 1_100.micros();
        while timer.get_counter() < start_time {
            if self.conversion_ready_flag { break; }
            timer.delay_ns(1000000);
        }

        let (exponent, mut result_msb) = self.get_exp_msb(RESULT_MSB_CH0)?;

        let (mut result_lsb, _, _) = self.get_lsb_counter_crc(RESULT_LSB_CH0)?;

        let mantissa: u16 = ((result_msb as u16) << 8) + result_lsb as u16;
        let adc_codes: u32 = (mantissa as u32) << exponent;
        let lux: f32 = (adc_codes as f32) * 0.000535;

        return Ok(lux);

    }

    pub fn lux(&mut self) -> Result<f32, E>
    {
        return self.result_of_addr();
    }

    pub fn device_errors(&mut self) -> Vec<Errors, 8>
    {
        let mut results = Vec::new();

        if self.flag_h {
            results.push(Errors::LightSensorHigherThanThreshold);
        }
        if self.flag_l {
            results.push(Errors::LightSensorLowerThanThreshold);
        }
        if self.overload_flag {
            results.push(Errors::LightSensorOverflow);
        }

        return results;
    }
}