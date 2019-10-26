# [mpl115a2](https://crates.io/crates/mpl115a2)

A platform agnostic driver to interface with the MPL115A2 Barometer via I2C

This driver was built using [`embedded-hal`] traits.

[`embedded-hal`]: https://docs.rs/embedded-hal/~0.1

## Documentation

 Read the detailed documentation [here](https://docs.rs/mpl115a2/)

## What works

- [x] Read device coefficients
- [x] Initiate barometric pressure and temperature sampling
- [x] Read barometric pressure value
- [x] Read temperature value

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  [APACHE](http://www.apache.org/licenses/LICENSE-2.0))

- MIT license ([LICENSE-MIT](LICENSE-MIT) or [MIT](http://opensource.org/licenses/MIT))

at your option.

## Resources

MPL115A2 - [Data Sheet](https://www.nxp.com/docs/en/data-sheet/MPL115A2.pdf)

The following resources were consulted when making this driver:

- [an-introduction-to-writing-embedded-hal-based-drivers-in-rust](http://pramode.in/2018/02/24/an-introduction-to-writing-embedded-hal-based-drivers-in-rust/)

- [mma7600fc driver](https://github.com/rahul-thakoor/mma7660fc)

- [rust i2cdev example](https://github.com/rust-embedded/rust-i2cdev)
