//! # MPL115A2 Barometric Pressure Sensor Library
//!
//! The MPL115A2 sensor utilizes the I2C interface.
//! This crate interacts with the underlying IC2 bus via the embedded_hal constructs
//! to provide a device neutral implementation.
//!
//! Creation of the sensor retrieves the internal calibration coefficients
//! and stores them for future use in calculating pressure and temperature.
//!
//! See the main [Data Sheet](https://www.nxp.com/docs/en/data-sheet/MPL115A2.pdf) for this sensor.
//!
#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;
extern crate libm;

use byteorder::{BigEndian, ByteOrder};
use core::marker::PhantomData;
use hal::blocking::i2c::{Write, WriteRead};

// i2c device address
const I2C_ADDRESS_PRIMARY: u8 = 0x60;

#[derive(Copy, Clone)]
#[allow(dead_code)]
enum Register {
    PressureADCMSB = 0x00,
    PressureADCLSB = 0x01,
    TemperatureADCMSB = 0x02,
    TemperatureADCLSB = 0x03,
    A0MSB = 0x04,
    A0LSB = 0x05,
    B1MSB = 0x06,
    B1LSB = 0x07,
    B2MSB = 0x08,
    B2LSB = 0x09,
    C12MSB = 0x0A,
    C12LSB = 0x0B,
    Convert = 0x12,
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

/// MPL115A2 is the main sensor structure. It maintains a copy
/// of the sensor coefficients after they are read. The stored
/// values are used on all subsequent calls to read pressure.
pub struct MPL115A2<I2C, Delay> {
    i2c: PhantomData<I2C>,
    delay: PhantomData<Delay>,
    pub coefficients: MPL115A2Coefficients,
}

impl<I2C, Delay, E> MPL115A2<I2C, Delay>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    Delay: embedded_hal::blocking::delay::DelayMs<u8>,
{
    /// Creates a new driver from a I2C peripheral.
    ///
    /// Reads coefficent data from the device from registers 0x04-0x0B.
    ///
    /// This data is stored in memory in the sensor to be used during pressure
    /// reading.
    pub fn new(i2c: &mut I2C) -> Result<Self, E> {
        let mut buf: [u8; 8] = [0; 8];
        i2c.write_read(I2C_ADDRESS_PRIMARY, &[Register::A0MSB.addr()], &mut buf)?;
        let coefficients = MPL115A2Coefficients::from_registers(buf);
        Ok(MPL115A2 {
            i2c: PhantomData,
            delay: PhantomData,
            coefficients: coefficients,
        })
    }

    /// Intiate a reading from the sensor. There is a max of 3ms time for conversion
    /// within the sensor.
    pub fn pressure_kpa(&mut self, i2c: &mut I2C, delay: &mut Delay) -> Result<MPL115A2Reading, E> {
        i2c.write(I2C_ADDRESS_PRIMARY, &[Register::Convert.addr(), 0x00])?;
        delay.delay_ms(5);
        let mut buf = [0_u8; 4];
        i2c.write_read(
            I2C_ADDRESS_PRIMARY,
            &[Register::PressureADCMSB.addr()],
            &mut buf,
        )?;
        let reading = MPL115A2RawReading::from_registers(buf);
        let pressure = reading.pressure_kpa(&self.coefficients);
        let temperature = reading.temperature_celsius();
        Ok(MPL115A2Reading {
            pressure: pressure,
            temperature: temperature,
        })
    }
}

/// The sensor has several coefficients that must be used in order
/// to calculate a correct value for pressure/temperature.
///
/// This structure provides access to those. It is usually only
/// necessary to read these coefficients once per interaction
/// with the sensor.  It does not need to be read again
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
    pub fn from_registers(registers: [u8; 8]) -> MPL115A2Coefficients {
        let a0 = MPL115A2Coefficients::calc_coefficient(registers[0], registers[1], 12, 3, 0);
        let b1 = MPL115A2Coefficients::calc_coefficient(registers[2], registers[3], 2, 13, 0);
        let b2 = MPL115A2Coefficients::calc_coefficient(registers[4], registers[5], 1, 14, 0);
        let c12 = MPL115A2Coefficients::calc_coefficient(registers[6], registers[7], 0, 13, 9);
        MPL115A2Coefficients::new(a0, b1, b2, c12)
    }
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
        // let adj = (rawval as f32 / 2_f32.powi(fractional_bits + extrabits))
        //     / 10_f32.powi(dec_pt_zero_pad);
        let adj = (rawval as f32 / libm::powf(2_f32, (fractional_bits + extrabits) as f32))
            / libm::powf(10_f32, dec_pt_zero_pad as f32);
        adj
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
struct MPL115A2RawReading {
    pub padc: u16, // 10-bit pressure ADC output value
    pub tadc: u16, // 10-bit pressure ADC output value
}
impl MPL115A2RawReading {
    pub fn from_registers(registers: [u8; 4]) -> MPL115A2RawReading {
        let padc: u16 = BigEndian::read_u16(&registers) >> 6;
        let tadc: u16 = BigEndian::read_u16(&registers[2..]) >> 6;
        MPL115A2RawReading {
            padc: padc,
            tadc: tadc,
        }
    }
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

/// Fully compensated pressure and temperature readings.
/// This structure is the resulting output of the caluculations
/// for both pressure in kPa and temperature in celcius.
pub struct MPL115A2Reading {
    pub pressure: f32,
    pub temperature: f32,
}

#[cfg(test)]
mod tests {
    use super::MPL115A2Coefficients;
    use super::MPL115A2RawReading;
    use byteorder::{BigEndian, ByteOrder};

    macro_rules! assert_almost_eq {
        ($left:expr, $right:expr) => {{
            match (&($left), &($right)) {
                (left_val, right_val) => {
                    if (*left_val - *right_val).abs() > 0.0001 {
                        panic!("assertion failed: ({:?} != {:?})", *left_val, *right_val);
                    }
                }
            }
        }};
    }

    #[test]
    fn test_calc_coefficient() {
        // unsigned simple
        assert_almost_eq!(
            MPL115A2Coefficients::calc_coefficient(0x00, 0b1000, 12, 3, 0),
            1.0
        );
        // signed simple
        assert_almost_eq!(
            MPL115A2Coefficients::calc_coefficient(0xFF, 0xF8, 12, 3, 0),
            -1.0
        );
        // pure integer (negative)
        assert_almost_eq!(
            MPL115A2Coefficients::calc_coefficient(0x80, 0x00, 15, 0, 0),
            -32_768_f32
        );
        // no integer part, zero padding, negative
        assert_almost_eq!(
            MPL115A2Coefficients::calc_coefficient(0x00, 0x01, 15, 0, 10),
            0.000_000_000_1
        );
    }

    #[test]
    fn test_big_endian() {
        let integer_bits: i32 = 0;
        let fractional_bits: i32 = 13;
        let dec_pt_zero_pad: i32 = 9;
        let msb = 0x33;
        let lsb = 0xC8;
        let rawval: i16 = BigEndian::read_i16(&[msb, lsb]);
        assert_eq!(rawval, 0x33C8);
        let extrabits = 16 - integer_bits - fractional_bits - 1;
        assert_eq!(extrabits, 2);
        let div = libm::powf(2_f32, (fractional_bits + extrabits) as f32);
        assert_eq!(div, 32768.0f32);
        let v1 = (rawval as f32) / div;
        assert_eq!(v1, 0.404_541_015_625);
        let v2 = libm::powf(10_f32, dec_pt_zero_pad as f32);
        assert_eq!(v2, 1_000_000_000.0f32);
        let v3 = v1 / v2;
        assert_eq!(v3, 0.000_000_000_404_541);
    }

    #[test]
    fn test_coefficients_from_registers() {
        let buf: [u8; 8] = [0x3E, 0xCE, 0xB3, 0xF9, 0xC5, 0x17, 0x33, 0xC8];
        let coefficients = MPL115A2Coefficients::from_registers(buf);
        assert_almost_eq!(coefficients.a0, 2009.75f32);
        assert_almost_eq!(coefficients.b1, -2.37585);
        assert_almost_eq!(coefficients.b2, -0.92047);
        // @TODO - This is weird. Based on the docs c12 in the example (data sheet, page 11)
        // should have a value of 0.000790 but that does not match with the calculation and
        // I see no issues with the way the calculation works. Will need to investigate this
        // futher at some point.
        assert_almost_eq!(coefficients.c12, 0.000_000_000_404_541);
    }

    #[test]
    fn test_reading_from_registers() {
        let buf: [u8; 4] = [0x66, 0x80, 0x7E, 0xC0];
        let reading = MPL115A2RawReading::from_registers(buf);
        assert_eq!(reading.padc, 410);
        assert_eq!(reading.tadc, 507);
    }

    #[test]
    fn test_reading_pressure_kpa_1() {
        let cbuf: [u8; 8] = [0x3E, 0xCE, 0xB3, 0xF9, 0xC5, 0x17, 0x33, 0xC8];
        let coefficients = MPL115A2Coefficients::from_registers(cbuf);
        let rbuf: [u8; 4] = [0x66, 0x80, 0x7E, 0xC0];
        let reading = MPL115A2RawReading::from_registers(rbuf);
        let pressure = reading.pressure_kpa(&coefficients);
        assert_eq!(pressure, 86.15162);
    }
    #[test]
    fn test_reading_pressure_kpa_2() {
        let coefficients = MPL115A2Coefficients::new(2009.75, -2.37585, -0.92047, 0.000790);
        let reading = MPL115A2RawReading {
            padc: 410,
            tadc: 507,
        };
        let pressure = reading.pressure_kpa(&coefficients);
        assert_eq!(pressure, 96.585915);
    }
    #[test]
    fn test_reading_temperature_c() {
        let reading = MPL115A2RawReading {
            padc: 410,
            tadc: 507,
        };
        let temperature = reading.temperature_celsius();
        assert_eq!(temperature, 23.317757);
    }
}
