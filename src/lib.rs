#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;

pub mod mpl115a2 {
    use hal::blocking::i2c::{Write, WriteRead};

    #[allow(dead_code)]
    pub const MPL115A2_I2C_ADDR: u16 = 0x60; // i2c device address

    #[allow(dead_code)]
    const REGISTER_ADDR_PADC: u8 = 0x00;
    #[allow(dead_code)]
    const REGISTER_ADDR_TADC: u8 = 0x02;
    #[allow(dead_code)]
    const REGISTER_ADDR_A0: u8 = 0x04; // Start address for coefficients
    #[allow(dead_code)]
    const REGISTER_ADDR_START_CONVERSION: u8 = 0x12;

    pub struct MPL115A2<I2C> {
        pub i2c: I2C,
    }

    impl <I2C, E> MPL115A2 <I2C> 
        where I2C : WriteRead<Error = E> + Write<Error = E>,
    {
        /// Creates a new driver from a I2C peripheral
        pub fn new(i2c: I2C) -> Result<Self, E> {
            let mpl115a2 = MPL115A2 { i2c };
            Ok(mpl115a2)
        }

    }

    /// Trait for sensors that provide access to temperature readings
    pub trait Thermometer {
        type Error;
        /// Get a temperature from the sensor in degrees celsius
        ///
        /// Returns `Ok(temperature)` if available, otherwise returns
        /// `Err(Self::Error)`
        fn temperature_celsius(&mut self) -> Result<f32, Self::Error>;
    }

    /// Trait for sensors that provide access to pressure readings
    pub trait Barometer {
        type Error;
        /// Get a pressure reading from the sensor in kPa
        ///
        /// Returns `Ok(pressure)` if avialable, otherwise returns
        /// `Err(Self::Error)`
        fn pressure_kpa(&mut self) -> Result<f32, Self::Error>;
    }
}