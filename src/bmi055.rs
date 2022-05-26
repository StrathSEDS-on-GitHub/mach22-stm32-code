use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::i2c;

const GYRO_ADDR: u8 = 0x68;
const ACC_ADDR: u8 = 0x18;

pub struct BMI055<I2C: i2c::WriteRead> {
    com: I2C,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Register {
    ChipId = 0x00,
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidId,
}

impl<I2C: i2c::WriteRead> BMI055<I2C> {
    pub fn new<E: core::fmt::Debug>(i2c: I2C) -> Result<Self, Error<E>>
    where
        I2C: i2c::WriteRead<Error = E>,
    {
        let mut chip = BMI055 { com: i2c };

        if chip.id().map_err(|e| Error::I2c(e))? != 0x0F {
            return Err(Error::InvalidId);
        }
        Ok(chip)
    }

    pub fn id(&mut self) -> Result<u8, I2C::Error> {
        self.read_byte(Register::ChipId)
    }

    fn read_byte(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data: [u8; 1] = [0];
        match self.com.write_read(GYRO_ADDR, &[reg as u8], &mut data) {
            Ok(_) => Ok(data[0]),
            Err(e) => Err(e)
        }
    }
}
