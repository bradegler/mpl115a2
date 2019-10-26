#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;

pub mod mpl115a2 {
    use hal::blocking::i2c::{Write, WriteRead, Read};
    use byteorder::{BigEndian, ByteOrder};

    pub const MPL115A2_I2C_ADDR: u8 = 0x60; // i2c device address

    #[allow(dead_code)]
    const REGISTER_ADDR_PADC: u8 = 0x00;
    #[allow(dead_code)]
    const REGISTER_ADDR_TADC: u8 = 0x02;
    const REGISTER_ADDR_A0: u8 = 0x04; // Start address for coefficients
    #[allow(dead_code)]
    const REGISTER_ADDR_START_CONVERSION: u8 = 0x12;

    /// Calculates a coefficient value for a 
    fn calc_coefficient(
        msb: u8,
        lsb: u8,
        integer_bits: i32,
        fractional_bits: i32,
        dec_pt_zero_pad: i32,
    ) -> f32 {
        // If values are less than 16 bytes, need to adjust
        let extrabits = 16 - integer_bits - fractional_bits - 1;
        let rawval: i16 = BigEndian::read_i16(&[msb, lsb]);
        let adj = (rawval as f32 / 2_f32.powi(fractional_bits + extrabits))
            / 10_f32.powi(dec_pt_zero_pad);
        adj
    }

    pub struct MPL115A2<I2C> {
        pub i2c: I2C,
        pub coefficients: MPL115A2Coefficients,
    }

    impl <I2C, E> MPL115A2 <I2C> 
        where I2C : WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
    {
        /// Creates a new driver from a I2C peripheral
        pub fn new(mut i2c: I2C) -> Result<Self, E> {
            let mut buf: [u8; 8] = [0; 8];
            // This should be built from a read of registers 0x04-0x0B in
            // order.  This gets the raw, unconverted value of each coefficient.
            i2c.write_read(MPL115A2_I2C_ADDR, &[REGISTER_ADDR_A0], &mut buf)?;
            let a0 = calc_coefficient(buf[0], buf[1], 12, 3, 0);
            let b1 = calc_coefficient(buf[2], buf[3], 2, 13, 0);
            let b2 = calc_coefficient(buf[4], buf[5], 1, 14, 0);
            let c12 = calc_coefficient(buf[6], buf[7], 0, 13, 9);
            let coefficients = MPL115A2Coefficients::new(a0, b1, b2, c12);
            let mpl115a2 = MPL115A2 { 
                i2c: i2c,
                coefficients: coefficients,
            };
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

        /// The sensors has several coefficients that must be used in order
    /// to calculate a correct value for pressure/temperature.
    ///
    /// This structure provides access to those.  It is usually only
    /// necessary to read these coefficients once per interaction
    /// with the acclerometer.  It does not need to be read again
    /// on each sample.
    #[derive(Debug)]
    pub struct MPL115A2Coefficients {
        a0: f32,  // 16 bits, 1 sign, 12 int, 3 fractional, 0 dec pt 0 pad
        b1: f32,  // 16 bits, 1 sign, 2 int, 13 fractional, 0 dec pt 0 pad
        b2: f32,  // 16 bits, 1 sign, 1 int, 14 fractional, 0 dec pt 0 pad
        c12: f32, // 16 bits, 1 sign, 0 int, 13 fractional, 9 dec pt 0 pad
    }

    impl MPL115A2Coefficients 
        {
        pub fn new(a0: f32, b1: f32, b2: f32, c12: f32
        ) -> MPL115A2Coefficients {
            MPL115A2Coefficients {
                a0: a0,
                b1: b1,
                b2: b2,
                c12: c12,
            }
        }
    }
}