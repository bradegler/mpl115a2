//! # MPL115A2 Barometric Pressure Sensor Library
//!
//! The MPL115A2 sensor utilizes the I2C interface.
//! This crate utilizes the embedded_hal constructs to provide a device
//! neutral implementation.
//!
//! Creation of the sensor retrieves the internal calibration coefficients
//! and stores them for future use in calculating pressure and temperature.
//!
//! See the main datasheet for this sensor [Data Sheet](https://www.nxp.com/docs/en/data-sheet/MPL115A2.pdf)
//!
#![no_std]

extern crate byteorder;
extern crate cortex_m_semihosting as sh;
extern crate embedded_hal as hal;
extern crate libm;

pub mod mpl115a2 {
    use byteorder::{BigEndian, ByteOrder};
    use core::marker::PhantomData;
    use hal::blocking::i2c::{Write, WriteRead};
    use sh::hprintln;

    pub const MPL115A2_I2C_ADDR: u8 = 0x60; // i2c device address
    const REGISTER_ADDR_PADC: u8 = 0x00; // Start address for PADC and TADC
    const REGISTER_ADDR_A0: u8 = 0x04; // Start address for coefficients
    const REGISTER_ADDR_START_CONVERSION: u8 = 0x12;

    /// Calculates a coefficient value for a msb + lsb pair
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
        let adj = (rawval as f32 / libm::powf(2_f32, (fractional_bits + extrabits) as f32))
            / libm::powf(10_f32, dec_pt_zero_pad as f32);
        adj
    }

    pub struct MPL115A2<I2C, Delay> {
        pub i2c: PhantomData<I2C>,
        pub delay: PhantomData<Delay>,
        pub coefficients: MPL115A2Coefficients,
    }

    impl<I2C, Delay, E> MPL115A2<I2C, Delay>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        Delay: embedded_hal::blocking::delay::DelayMs<u8>,
    {
        /// Creates a new driver from a I2C peripheral
        /// Reads coefficent data from the device from registers 0x04-0x0B
        /// This data is stored in memory in the sensor to be used during pressure
        /// reading.
        pub fn new(i2c: &mut I2C) -> Result<Self, E> {
            let mut buf: [u8; 8] = [0; 8];
            hprintln!("MPL115A2: Reading coefficients").unwrap();
            // This should be built from a read of registers 0x04-0x0B in
            // order. This gets the raw, unconverted value of each coefficient.
            i2c.write_read(MPL115A2_I2C_ADDR, &[REGISTER_ADDR_A0], &mut buf)?;
            hprintln!("MPL115A2: Raw coefficients -> {:?}", buf).unwrap();
            let a0 = calc_coefficient(buf[0], buf[1], 12, 3, 0);
            let b1 = calc_coefficient(buf[2], buf[3], 2, 13, 0);
            let b2 = calc_coefficient(buf[4], buf[5], 1, 14, 0);
            let c12 = calc_coefficient(buf[6], buf[7], 0, 13, 9);
            let coefficients = MPL115A2Coefficients::new(a0, b1, b2, c12);
            let mpl115a2 = MPL115A2 {
                i2c: PhantomData,
                delay: PhantomData,
                coefficients: coefficients,
            };
            Ok(mpl115a2)
        }

        /// Intiate a reading from the sensor. There is a max a 3ms time for conversion
        /// within the device.
        pub fn pressure_kpa(&mut self, i2c: &mut I2C, delay: &mut Delay) -> Result<f32, E> {
            hprintln!("MPL115A2: Reading pressure in kPa").unwrap();
            i2c.write(MPL115A2_I2C_ADDR, &[REGISTER_ADDR_START_CONVERSION])?;
            delay.delay_ms(50);
            let mut buf = [0_u8; 4];
            i2c.write_read(MPL115A2_I2C_ADDR, &[REGISTER_ADDR_PADC], &mut buf)?;
            hprintln!("MPL115A2: Read pressure buffer {:?}", buf).unwrap();
            let padc: u16 = BigEndian::read_u16(&buf) >> 6;
            let tadc: u16 = BigEndian::read_u16(&buf[2..]) >> 6;
            let reading = MPL115A2RawReading {
                padc: padc,
                tadc: tadc,
            };
            let pressure = reading.pressure_kpa(&self.coefficients);
            hprintln!("MPL115A2: Pressure value {:?}", pressure).unwrap();
            Ok(pressure)
        }
    }

    /// The sensor has several coefficients that must be used in order
    /// to calculate a correct value for pressure/temperature.
    ///
    /// This structure provides access to those. It is usually only
    /// necessary to read these coefficients once per interaction
    /// with the acclerometer.  It does not need to be read again
    /// on each sample.
    #[derive(Debug, Copy, Clone)]
    pub struct MPL115A2Coefficients {
        pub a0: f32,  // 16 bits, 1 sign, 12 int, 3 fractional, 0 dec pt 0 pad
        pub b1: f32,  // 16 bits, 1 sign, 2 int, 13 fractional, 0 dec pt 0 pad
        pub b2: f32,  // 16 bits, 1 sign, 1 int, 14 fractional, 0 dec pt 0 pad
        pub c12: f32, // 16 bits, 1 sign, 0 int, 13 fractional, 9 dec pt 0 pad
    }

    impl MPL115A2Coefficients {
        pub fn new(a0: f32, b1: f32, b2: f32, c12: f32) -> MPL115A2Coefficients {
            MPL115A2Coefficients {
                a0: a0,
                b1: b1,
                b2: b2,
                c12: c12,
            }
        }
    }

    /// In order to get either the temperature or pressure it is
    /// necessary to read several different values from the chip.
    ///
    /// These are not generally useful in and of themselves but
    /// are used for calculating temperature/pressure.  The structure
    /// is exposed externally as they *could* be useful for some
    /// unknown use case.  Generally, you shouldn't ever need
    /// to use this directly.
    #[derive(Debug, Copy, Clone)]
    pub struct MPL115A2RawReading {
        pub padc: u16, // 10-bit pressure ADC output value
        pub tadc: u16, // 10-bit pressure ADC output value
    }
    impl MPL115A2RawReading {
        /// Calculate the temperature in centrigrade for this reading
        pub fn temperature_celsius(&self) -> f32 {
            (self.tadc as f32 - 498.0) / -5.35 + 25.0
        }

        /// Calculate the pressure in pascals for this reading
        pub fn pressure_kpa(&self, coeff: &MPL115A2Coefficients) -> f32 {
            // Pcomp = a0 + (b1 + c12 * Tadc) * Padc + b2 * Tadc
            // Pkpa = Pcomp * ((115 - 50) / 1023) + 50
            let pcomp: f32 = coeff.a0
                + (coeff.b1 + coeff.c12 * self.tadc as f32) * self.padc as f32
                + (coeff.b2 * self.tadc as f32);

            // scale has 1023 bits of range from 50 kPa to 115 kPa
            let pkpa: f32 = pcomp * ((115.0 - 50.0) / 1023.0) + 50.0;
            pkpa
        }
    }
}
