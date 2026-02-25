use rp235x_hal as hal;
use embedded_hal::i2c::I2c;
use hal::i2c::Error as I2cError;
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};

pub struct DS3231 {
    pub addr: u8,
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    I2c(I2cError),
    LostPower,
    InvalidTimestamp,
}

impl From<I2cError> for Error {
    fn from(err: I2cError) -> Self {
        Error::I2c(err)
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct DateTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl DS3231 {
    pub fn new(addr: u8) -> Self {
        Self { addr }
    }

    fn bcd_to_dec(bcd: u8) -> u8 {
        ((bcd & 0xF0) >> 4) * 10 + (bcd & 0x0F)
    }

    fn dec_to_bcd(dec: u8) -> u8 {
        ((dec / 10) << 4) | (dec % 10)
    }

    pub fn set_unix_time<I2C>(&self, i2c: &mut I2C, timestamp: i64) -> Result<(), Error>
    where
        I2C: I2c<Error = I2cError>,
    {
        let dt = NaiveDateTime::from_timestamp_opt(timestamp, 0)
            .ok_or(Error::InvalidTimestamp)?;

        let datetime = DateTime {
            year: dt.year() as u16,
            month: dt.month() as u8,
            day: dt.day() as u8,
            hour: dt.hour() as u8,
            minute: dt.minute() as u8,
            second: dt.second() as u8,
        };

        self.set_datetime(i2c, &datetime)
    }

    pub fn unix_time<I2C>(&self, i2c: &mut I2C) -> Result<i64, Error>
    where
        I2C: I2c<Error = I2cError>,
    {
        let dt = self.datetime(i2c)?;
        let c_date = NaiveDate::from_ymd_opt(dt.year as i32, dt.month as u32, dt.day as u32)
            .ok_or(Error::InvalidTimestamp)?;
        let c_time = NaiveTime::from_hms_opt(dt.hour as u32, dt.minute as u32, dt.second as u32)
            .ok_or(Error::InvalidTimestamp)?;
        
        Ok(NaiveDateTime::new(c_date, c_time).and_utc().timestamp())
    }

    pub fn set_datetime<I2C>(&self, i2c: &mut I2C, dt: &DateTime) -> Result<(), Error>
    where
        I2C: I2c<Error = I2cError>,
    {
        let data = [
            0x00,
            Self::dec_to_bcd(dt.second),
            Self::dec_to_bcd(dt.minute),
            Self::dec_to_bcd(dt.hour) & 0x3F, 
            0,
            Self::dec_to_bcd(dt.day),
            Self::dec_to_bcd(dt.month),
            Self::dec_to_bcd((dt.year % 100) as u8),
        ];

        i2c.write(self.addr, &data)?;
        i2c.write(self.addr, &[0x0E, 0x00])?; 
        i2c.write(self.addr, &[0x0F, 0x00])?; 
        Ok(())
    }

    pub fn datetime<I2C>(&self, i2c: &mut I2C) -> Result<DateTime, Error>
    where
        I2C: I2c<Error = I2cError>,
    {
        let mut buf = [0u8; 7];
        i2c.write_read(self.addr, &[0x00], &mut buf)?;

        Ok(DateTime {
            second: Self::bcd_to_dec(buf[0] & 0x7F),
            minute: Self::bcd_to_dec(buf[1]),
            hour: Self::bcd_to_dec(buf[2] & 0x3F),
            day: Self::bcd_to_dec(buf[4]),
            month: Self::bcd_to_dec(buf[5] & 0x1F),
            year: 2000 + Self::bcd_to_dec(buf[6]) as u16,
        })
    }

    pub fn lost_power<I2C>(&self, i2c: &mut I2C) -> Result<bool, Error>
    where
        I2C: I2c<Error = I2cError>,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.addr, &[0x0F], &mut buf)?;
        Ok((buf[0] & 0x80) != 0)
    }
}
