use rp235x_hal as hal;
use embedded_hal::i2c::I2c;
use core::result::Result;
use hal::i2c::Error as I2cError;
use embedded_hal_0_2::blocking::i2c:: {Write, Read};



const DATA_V_MASK: u8 = 0xF0;
const DATA_I_MASK: u8 = 0x0F;

// Status register
const STATUS_READ: u8 = 0x1 << 6;
// _STATUS_ADC_OC = const(0x1 << 0)
const STATUS_ADC_ALERT: u8 = 0x1 << 1;
// _STATUS_HS_OC = const(0x1 << 2)
// STATUS_HS_ALERT = const(0x1 << 3)
const STATUS_OFF_STATUS: u8 = 0x1 << 4;
// STATUS_OFF_ALERT = const(0x1 << 5)

// Extended registers
const ALERT_EN_EXT_REG_ADDR: u8 = 0x81;
const ALERT_EN_EN_ADC_OC4: u8 = 0x1 << 1;
const ALERT_EN_CLEAR: u8 = 0x1 << 4;

const ALERT_TH_EN_REG_ADDR: u8 = 0x82;

const CONTROL_REG_ADDR: u8 = 0x83;
const CONTROL_SWOFF: u8 = 0x1 << 0;

pub struct ADM1176 {
    pub addr: u8,
    sense_resistor: f32,
    // on: bool,
    overcurrent_level: u8,
    v_fs_over_res: f32,
    i_fs_over_res: f32,
}

impl ADM1176 {
    pub fn new(addr: u8) -> Self {
        Self {
            addr: addr,
            sense_resistor: 0.01,
            // on: true,
            overcurrent_level: 0xFF,
            v_fs_over_res: 26.35 / 4096.0,
            i_fs_over_res: 0.10584 / 4096.0,
        }
    }

    pub fn config<I2C>(&self, i2c: &mut I2C, values: &[&str]) -> Result<(), I2cError>
    where
        I2C: Write<Error = I2cError>
    {
        const V_CONT_BIT: u8 = 0x1 << 0;
        const V_ONCE_BIT: u8 = 0x1 << 1;
        const I_CONT_BIT: u8 = 0x1 << 2;
        const I_ONCE_BIT: u8 = 0x1 << 3;
        const V_RANGE_BIT: u8 = 0x1 << 4;
        
        let mut config:u8 = 0;
        for value in values.iter() {
            match *value {
                "V_CONT" => config |= V_CONT_BIT,
                "V_ONCE" => config |= V_ONCE_BIT,
                "I_CONT" => config |= I_CONT_BIT,
                "I_ONCE" => config |= I_ONCE_BIT,
                "V_RANGE" => config |= V_RANGE_BIT,
                _ => {}
            }
        }
        i2c.write(self.addr, &[config])
    }

    pub fn read_voltage_current<I2C>(&self, i2c: &mut I2C) -> Result<(f32, f32), I2cError>
    where
        I2C: Read<Error = I2cError>
    {
        let mut buf = [0u8; 3];
        match i2c.read(self.addr, &mut buf) {
            Ok(_) => {
                let raw_voltage = (((buf[0] as u16) << 8) | ((buf[2] & 0xF0) as u16)) >> 4;
                let raw_current = ((buf[1] << 4) | (buf[2] & 0x0F)) as u16;
                let voltage = (self.v_fs_over_res) * raw_voltage as f32;  // volts
                let current = ((self.i_fs_over_res) * raw_current as f32) / self.sense_resistor;  // amperes
                Ok((voltage, current))
            },
            Err(e) => Err(e)
        }
    }

    fn turn_off<I2C>(&self, i2c: &mut I2C) -> Result<(), I2cError>
    where
        I2C: Write<Error = I2cError>
    {
        let mut off: [u8;2] = [CONTROL_REG_ADDR, 0x04 | CONTROL_SWOFF];
        i2c.write(self.addr, &mut off)
    }

    fn turn_on<I2C>(&self, i2c: &mut I2C) -> Result<(), I2cError>
    where
        I2C: Write<Error = I2cError>
    {
        let mut on: [u8;2] = [CONTROL_REG_ADDR, 0x04 & !CONTROL_SWOFF];
        i2c.write(self.addr, &mut on);
        self.config(i2c, &["V_CONT", "I_CONT"])
    }

    pub fn set_device_on<I2C>(&self, i2c: &mut I2C, on: bool) -> Result<(), I2cError>
    where
        I2C: Write<Error = I2cError>
    {
        if on {
            self.turn_on(i2c)
        } else {
            self.turn_off(i2c)
        }
    }

    pub fn device_on<I2C>(&self, i2c: &mut I2C) -> Result<bool, I2cError>
    where
        I2C: Read<Error = I2cError> + Write<Error = I2cError>
    {
        match self.status(i2c) {
            Ok(status) => Ok((status & STATUS_OFF_STATUS) != STATUS_OFF_STATUS),
            Err(e) => Err(e)
        }
    }

    pub fn overcurrent_level(&self) -> u8 {
        self.overcurrent_level
    }

    pub fn set_overcurrent_level<I2C>(&mut self, i2c: &mut I2C, value: u8) -> Result<(), I2cError>
    where
        I2C: Write<Error = I2cError>
    {
        let mut cmd: [u8;2] = [ALERT_EN_EXT_REG_ADDR, 0x04 | ALERT_EN_EN_ADC_OC4];
        i2c.write(self.addr, &mut cmd);
        cmd = [ALERT_TH_EN_REG_ADDR, value];
        let res = i2c.write(self.addr, &mut cmd);
        self.overcurrent_level = value;
        res
    }

    pub fn clear<I2C>(&self, i2c: &mut I2C) -> Result<(), I2cError>
    where
        I2C: Write<Error = I2cError>
    {
        let mut cmd: [u8;2] = [ALERT_EN_EXT_REG_ADDR, 0x04 | ALERT_EN_CLEAR];
        i2c.write(self.addr, &mut cmd)
    }

    pub fn status<I2C>(&self, i2c: &mut I2C) -> Result<u8, I2cError>
    where
        I2C: Read<Error = I2cError> + Write<Error = I2cError>
    {
        i2c.write(self.addr, &[STATUS_READ]);
        let mut status_buf = [0u8; 1];
        i2c.read(self.addr, &mut status_buf);
        i2c.write(self.addr, &[0x00 & !STATUS_READ]);
        Ok(status_buf[0])
    }

}