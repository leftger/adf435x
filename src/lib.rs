#![cfg_attr(not(feature = "std"), no_std)]

//! Device driver abstractions for the Analog Devices ADF435x wideband PLL/VCO family.
//!
//! This crate provides type-safe register access to the ADF435x series of RF PLL synthesizers
//! using the [`device-driver`](https://crates.io/crates/device-driver) crate for compile-time
//! code generation from YAML register definitions.
//!
//! # Features
//!
//! - **Type-safe register access** - Generated API from YAML manifest prevents invalid operations
//! - **Automatic LE pulsing** - High-level drivers handle Load Enable pin timing automatically
//! - **One-shot initialization** - [`InitConfig`] and [`initialize()`] methods for datasheet-compliant R5→R0 programming
//! - **Frequency configuration helpers** - [`Fpfd`] and [`FracN`] for fractional-N mode
//! - **No_std support** - Works in embedded environments (with optional `std` feature)
//! - **Flexible interface** - Bring your own SPI implementation via [`RegisterInterface`]
//! - **YAML-driven** - All register definitions in `manifests/adf435x.yaml`
//!
//! # Supported Devices
//!
//! - ADF4350
//! - ADF4351
//! - Other compatible variants in the ADF435x family
//!
//! # Hardware Notes
//!
//! ## CE (Chip Enable) Pin
//!
//! The CE pin controls device power state. **This driver assumes CE is always high (device powered).**
//! If you need to power down the device, control CE externally or use the power-down bit in R2.
//!
//! ## LE (Load Enable) Pin
//!
//! When LE goes high, data from the 32-bit shift register is loaded into the register selected by
//! the 3 control bits (bits \[2:0\] of the SPI word). The LE pin must be pulsed after each SPI write.
//!
//! This crate provides two usage patterns:
//! - **Manual LE control**: Use the raw [`Device`] type and pulse LE yourself

//! - **Automatic LE control**: Use [`Adf435xDriver`] which handles LE pulsing automatically
//!
//! # Usage Patterns
//!
//! ## Pattern 1: Manual LE Control (Maximum Flexibility)
//!
//! Use the raw [`Device`] type when you need complete control:
//!
//! ```
//! use {new_device, RegisterInterface};
//! use core::convert::Infallible;
//!
//! struct SpiInterface;
//! impl RegisterInterface for SpiInterface {
//!     type Error = Infallible;
//!     type AddressType = u8;
//!     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Self::Error> { Ok(()) }
//!     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
//! }
//!
//! let mut device = new_device(SpiInterface);
//!
//! // Write register
//! device.r_0_frequency_control()
//!     .write(|reg| {
//!         reg.set_integer_value(96);
//!         reg.set_fractional_value(0);
//!     })
//!     .unwrap();
//!
//! // YOU must pulse LE here:
//! // le_pin.set_high();
//! // delay.delay_us(10);
//! // le_pin.set_low();
//! ```
//!
//! ## Pattern 2: Automatic LE Control (Recommended for CE always high)
//!
//! Use [`Adf435xDriver`] for automatic LE pulsing when CE is tied high:
//!
//! ```no_run
//! use {Adf435xDriver, RegisterInterface, OutputPin, DelayUs};
//! # use core::convert::Infallible;
//! # struct SpiInterface;
//! # impl RegisterInterface for SpiInterface {
//! #     type Error = Infallible;
//! #     type AddressType = u8;
//! #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
//! #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
//! # }
//! # struct LePin;
//! # impl OutputPin for LePin {
//! #     type Error = Infallible;
//! #     fn set_high(&mut self) -> Result<(), Infallible> { Ok(()) }
//! #     fn set_low(&mut self) -> Result<(), Infallible> { Ok(()) }
//! # }
//! # struct Delay;
//! # impl DelayUs for Delay {
//! #     fn delay_us(&mut self, _: u32) {}
//! # }
//!
//! let spi = SpiInterface;
//! let le_pin = LePin;
//! let mut delay = Delay;
//!
//! let mut driver = Adf435xDriver::new(spi, le_pin);
//!
//! // LE pulsing is automatic!
//! driver.device_mut().r_0_frequency_control()
//!     .write(|reg| {
//!         reg.set_integer_value(96);
//!         reg.set_fractional_value(0);
//!     })
//!     .unwrap();
//! driver.latch(&mut delay).unwrap();
//! ```
//!
//! **See `examples/basic_usage.rs` for a complete working example.**
//!
//! ## Pattern 3: Full Pin Control (CE and LE)
//!
//! Use [`Adf435xDriverWithCE`] for CE control and frequency helpers:
//!
//! ```no_run
//! use {Adf435xDriverWithCE, RegisterInterface, OutputPin, DelayUs16, Fpfd, FracN};
//! # use core::convert::Infallible;
//! # struct SpiInterface;
//! # impl RegisterInterface for SpiInterface {
//! #     type Error = Infallible;
//! #     type AddressType = u8;
//! #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
//! #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
//! # }
//! # struct Pin;
//! # impl OutputPin for Pin {
//! #     type Error = Infallible;
//! #     fn set_high(&mut self) -> Result<(), Infallible> { Ok(()) }
//! #     fn set_low(&mut self) -> Result<(), Infallible> { Ok(()) }
//! # }
//! # struct Delay;
//! # impl DelayUs16 for Delay {
//! #     fn delay_us(&mut self, _: u16) {}
//! # }
//!
//! let mut driver = Adf435xDriverWithCE::new(SpiInterface, Pin, Pin);
//! let mut delay = Delay;
//!
//! // Power up
//! driver.enable().unwrap();
//!
//! // Use frequency helpers
//! let fpfd = Fpfd::from_device(25_000_000, driver.device_mut()).unwrap();
//! let frac_n = FracN::new(fpfd);
//! frac_n.set_frequency(driver.device_mut(), 2_400_000_000).unwrap();
//! driver.latch(&mut delay).unwrap();
//!
//! // Power down
//! driver.disable().unwrap();
//! ```
//!
//! **See `examples/full_featured.rs` for a complete working example.**
//!
//! # Register Map
//!
//! The ADF435x has six 32-bit registers (R0-R5):
//!
//! - **R0**: Frequency Control (INT, FRAC values)
//! - **R1**: Phase and Modulus (MOD, phase, prescaler)
//! - **R2**: Reference and Charge Pump
//! - **R3**: Function Control
//! - **R4**: Output Stage (RF divider, output power)
//! - **R5**: Lock Detect and Readback
//!
//! # Examples
//!
//! - **`examples/basic_usage.rs`** - Simple driver with automatic LE pulsing (CE always high)
//! - **`examples/full_featured.rs`** - Extended driver with CE control and frequency helpers

pub use device_driver::RegisterInterface;

device_driver::create_device!(
    device_name: Adf435x,
    manifest: "manifests/adf435x.yaml"
);

/// Convenience alias for the generated device driver type.
///
/// The generic parameter `I` must implement [`RegisterInterface`] with
/// `AddressType = u8`, matching the 8-bit address tags used by the ADF435x
/// register map.
pub type Device<I> = Adf435x<I>;

/// Instantiate a new [`Device`] backed by the provided register interface.
///
/// # Example
///
/// ```
/// use core::convert::Infallible;
/// use {new_device, RegisterInterface};
///
/// struct MockInterface;
///
/// impl RegisterInterface for MockInterface {
///     type Error = Infallible;
///     type AddressType = u8;
///
///     fn write_register(
///         &mut self,
///         _address: Self::AddressType,
///         _size_bits: u32,
///         _data: &[u8],
///     ) -> Result<(), Self::Error> {
///         Ok(())
///     }
///
///     fn read_register(
///         &mut self,
///         _address: Self::AddressType,
///         _size_bits: u32,
///         _data: &mut [u8],
///     ) -> Result<(), Self::Error> {
///         Ok(())
///     }
/// }
///
/// let driver = new_device(MockInterface);
/// # let _ = driver;
/// ```
pub fn new_device<I>(interface: I) -> Device<I>
where
    I: RegisterInterface<AddressType = u8>,
{
    Adf435x::new(interface)
}

// Frequency configuration module
mod frequency;
pub use frequency::{Fpfd, FracN};

/// Alias for `OutputPin` trait for backward compatibility.
///
/// This trait is identical to [`OutputPin`] and is provided for compatibility
/// with code that expects a `PinControl` trait.
pub trait PinControl: OutputPin {}

// Blanket implementation: anything that implements OutputPin also implements PinControl
impl<T: OutputPin> PinControl for T {}

/// Trait for GPIO output pins.
///
/// This abstracts over platform-specific GPIO implementations.
/// Compatible with `embedded-hal` v1.0+ `OutputPin` trait.
pub trait OutputPin {
    /// Error type for pin operations
    type Error;

    /// Set the pin high
    fn set_high(&mut self) -> Result<(), Self::Error>;

    /// Set the pin low
    fn set_low(&mut self) -> Result<(), Self::Error>;
}

/// Trait for delay providers.
///
/// This abstracts over platform-specific delay implementations.
/// Compatible with embedded-hal delay traits.
pub trait DelayUs {
    /// Delay for the specified number of microseconds
    fn delay_us(&mut self, us: u32);
}

/// Trait for delay providers (u16 variant).
///
/// This is an alternative to [`DelayUs`] that uses u16 for delay values.
/// Some platforms prefer this for smaller delay values.
pub trait DelayUs16 {
    /// Delay for the specified number of microseconds
    fn delay_us(&mut self, us: u16);
}

// Automatic conversion: anything implementing DelayUs also implements DelayUs16
impl<T: DelayUs> DelayUs16 for T {
    fn delay_us(&mut self, us: u16) {
        DelayUs::delay_us(self, us as u32)
    }
}

/// Error type for driver operations
#[derive(Debug)]
pub enum DriverError<SpiError, CeError, LeError> {
    /// Error from the SPI interface
    Spi(SpiError),

    /// Error from the CE pin
    CePin(CeError),

    /// Error from the LE pin
    LePin(LeError),
}

// Simplified error type for drivers without CE pin
impl<SpiError, PinError> From<DriverError<SpiError, PinError, PinError>>
    for Result<(), DriverError<SpiError, PinError, PinError>>
{
    fn from(err: DriverError<SpiError, PinError, PinError>) -> Self {
        Err(err)
    }
}

/// High-level driver with automatic LE pulsing.
///
/// This wrapper owns the LE pin and automatically pulses it with proper timing
/// after register writes. This is the recommended way to use the driver.
///
/// # Hardware Assumption
///
/// CE (Chip Enable) is assumed to be always high (device always powered).
///
/// # Example
///
/// ```no_run
/// # use {Adf435xDriver, RegisterInterface, OutputPin, DelayUs};
/// # use core::convert::Infallible;
/// # struct SpiInterface;
/// # impl RegisterInterface for SpiInterface {
/// #     type Error = Infallible;
/// #     type AddressType = u8;
/// #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
/// #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
/// # }
/// # struct LePin;
/// # impl OutputPin for LePin {
/// #     type Error = Infallible;
/// #     fn set_high(&mut self) -> Result<(), Infallible> { Ok(()) }
/// #     fn set_low(&mut self) -> Result<(), Infallible> { Ok(()) }
/// # }
/// # struct Delay;
/// # impl DelayUs for Delay {
/// #     fn delay_us(&mut self, _: u32) {}
/// # }
/// let mut driver = Adf435xDriver::new(SpiInterface, LePin);
/// let mut delay = Delay;
///
/// // Configure frequency
/// driver.device_mut().r_0_frequency_control()
///     .write(|reg| {
///         reg.set_integer_value(96);
///         reg.set_fractional_value(0);
///     })
///     .unwrap();
///
/// // Latch with automatic timing
/// driver.latch(&mut delay).unwrap();
/// ```
pub struct Adf435xDriver<I, P> {
    device: Device<I>,
    le_pin: P,
}

impl<I, P> Adf435xDriver<I, P>
where
    I: RegisterInterface<AddressType = u8>,
    P: OutputPin,
{
    /// Create a new driver with the given SPI interface and LE pin.
    ///
    /// # Arguments
    ///
    /// * `interface` - SPI interface implementing [`RegisterInterface`]
    /// * `le_pin` - LE (Load Enable) GPIO pin implementing [`OutputPin`]
    ///
    /// # Note
    ///
    /// CE (Chip Enable) is assumed to be always high. If you need to control CE,
    /// manage it externally before creating the driver.
    pub fn new(interface: I, le_pin: P) -> Self {
        Self {
            device: Adf435x::new(interface),
            le_pin,
        }
    }

    /// Get a reference to the underlying device.
    ///
    /// Use this to access the type-safe register API.
    pub fn device(&self) -> &Device<I> {
        &self.device
    }

    /// Get a mutable reference to the underlying device.
    ///
    /// Use this to write/modify registers. Remember to call [`latch()`](Self::latch)
    /// after making changes to load the data into the device.
    pub fn device_mut(&mut self) -> &mut Device<I> {
        &mut self.device
    }

    /// Pulse the LE pin to latch register data.
    ///
    /// This uses the timing recommended by the datasheet:
    /// - 5µs delay before LE high
    /// - LE high for 10µs (minimum pulse width)
    /// - 5µs delay after LE low
    ///
    /// Call this after writing to any register(s) via [`device_mut()`](Self::device_mut).
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider implementing [`DelayUs`]
    ///
    /// # Errors
    ///
    /// Returns [`DriverError::LePin`] if the LE pin operations fail.
    pub fn latch<D>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), DriverError<I::Error, P::Error, P::Error>>
    where
        D: DelayUs,
    {
        // Pre-delay
        delay.delay_us(5);

        // Pulse LE high
        self.le_pin.set_high().map_err(DriverError::LePin)?;

        // Pulse width
        delay.delay_us(10);

        // Pulse LE low
        self.le_pin.set_low().map_err(DriverError::LePin)?;

        // Post-delay
        delay.delay_us(5);

        Ok(())
    }

    /// Consume the driver and return the SPI interface and LE pin.
    pub fn into_parts(self) -> (I, P) {
        // Extract the interface from the device
        // Since we own the device, we can destructure it
        let Adf435x { interface, .. } = self.device;
        (interface, self.le_pin)
    }
}

/// Extended driver that also owns CE (Chip Enable) pin.
///
/// This version of the driver owns both CE and LE pins and provides
/// `enable()` and `disable()` methods for CE control.
///
/// # Example
///
/// ```no_run
/// # use {Adf435xDriverWithCE, RegisterInterface, OutputPin, DelayUs16};
/// # use core::convert::Infallible;
/// # struct SpiInterface;
/// # impl RegisterInterface for SpiInterface {
/// #     type Error = Infallible;
/// #     type AddressType = u8;
/// #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
/// #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
/// # }
/// # struct Pin;
/// # impl OutputPin for Pin {
/// #     type Error = Infallible;
/// #     fn set_high(&mut self) -> Result<(), Infallible> { Ok(()) }
/// #     fn set_low(&mut self) -> Result<(), Infallible> { Ok(()) }
/// # }
/// # struct Delay;
/// # impl DelayUs16 for Delay {
/// #     fn delay_us(&mut self, _: u16) {}
/// # }
/// let mut driver = Adf435xDriverWithCE::new(SpiInterface, Pin, Pin);
/// let mut delay = Delay;
///
/// // Power up
/// driver.enable().unwrap();
///
/// // Configure using type-safe API
/// driver.device_mut().r_0_frequency_control()
///     .write(|reg| {
///         reg.set_integer_value(96);
///         reg.set_fractional_value(0);
///     })
///     .unwrap();
///
/// // Latch with automatic LE pulsing
/// driver.latch(&mut delay).unwrap();
///
/// // Power down
/// driver.disable().unwrap();
/// ```
pub struct Adf435xDriverWithCE<I, CE, LE> {
    device: Device<I>,
    ce_pin: CE,
    le_pin: LE,
}

impl<I, CE, LE> Adf435xDriverWithCE<I, CE, LE>
where
    I: RegisterInterface<AddressType = u8>,
    CE: OutputPin,
    LE: OutputPin,
{
    /// Create a new driver with CE and LE pins.
    ///
    /// # Arguments
    ///
    /// * `interface` - SPI interface implementing [`RegisterInterface`]
    /// * `ce_pin` - CE (Chip Enable) GPIO pin implementing [`OutputPin`]
    /// * `le_pin` - LE (Load Enable) GPIO pin implementing [`OutputPin`]
    pub fn new(interface: I, ce_pin: CE, le_pin: LE) -> Self {
        Self {
            device: Adf435x::new(interface),
            ce_pin,
            le_pin,
        }
    }

    /// Get a reference to the underlying device.
    pub fn device(&self) -> &Device<I> {
        &self.device
    }

    /// Get a mutable reference to the underlying device.
    pub fn device_mut(&mut self) -> &mut Device<I> {
        &mut self.device
    }

    /// Enable the device (set CE pin high).
    ///
    /// # Errors
    ///
    /// Returns [`DriverError::CePin`] if the CE pin operation fails.
    pub fn enable(&mut self) -> Result<(), DriverError<I::Error, CE::Error, LE::Error>> {
        self.ce_pin.set_high().map_err(DriverError::CePin)
    }

    /// Disable the device (set CE pin low).
    ///
    /// # Errors
    ///
    /// Returns [`DriverError::CePin`] if the CE pin operation fails.
    pub fn disable(&mut self) -> Result<(), DriverError<I::Error, CE::Error, LE::Error>> {
        self.ce_pin.set_low().map_err(DriverError::CePin)
    }

    /// Pulse the LE pin to latch register data.
    ///
    /// This uses the timing recommended by the datasheet:
    /// - 5µs delay before LE high
    /// - LE high for 10µs (minimum pulse width)
    /// - 5µs delay after LE low
    ///
    /// Call this after writing to any register(s) via [`device_mut()`](Self::device_mut).
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider implementing [`DelayUs16`]
    ///
    /// # Errors
    ///
    /// Returns [`DriverError::LePin`] if the LE pin operations fail.
    pub fn latch<D>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), DriverError<I::Error, CE::Error, LE::Error>>
    where
        D: DelayUs16,
    {
        // Pre-delay
        delay.delay_us(5);

        // Pulse LE high
        self.le_pin.set_high().map_err(DriverError::LePin)?;

        // Pulse width
        delay.delay_us(10);

        // Pulse LE low
        self.le_pin.set_low().map_err(DriverError::LePin)?;

        // Post-delay
        delay.delay_us(5);

        Ok(())
    }

    /// Consume the driver and return the SPI interface, CE pin, and LE pin.
    pub fn into_parts(self) -> (I, CE, LE) {
        let Adf435x { interface, .. } = self.device;
        (interface, self.ce_pin, self.le_pin)
    }
}

/// Noise/spur optimization mode setting helper.
///
/// Maps to the datasheet's "Low Noise" (dither disabled) and "Low Spur" (dither enabled) modes.
#[derive(Debug, Copy, Clone)]
pub enum NoiseModeSetting {
    /// Low noise mode (dither disabled)
    LowNoise,
    /// Low spur mode (dither enabled)
    LowSpur,
}

/// Initialization configuration for programming registers R5 → R0.
///
/// This structure captures commonly used fields across the register map and is used by
/// [`Adf435xDriver::initialize`] and [`Adf435xDriverWithCE::initialize`] to program the device
/// in the datasheet-specified sequence: R5, R4, R3, R2, R1, R0.
///
/// For convenience, you can use [`InitConfig::default()`] for sensible defaults,
/// or create from raw values using [`InitConfig::from_raw()`].
#[derive(Debug, Copy, Clone)]
pub struct InitConfig {
    // R5 - Latch and Status
    pub lock_detect_pin_operation: LockDetectPinOperation,

    // R4 - Output Stage
    pub output_power: OutputPowerLevel,
    pub rf_out_enable: bool,
    pub aux_output_power: AuxOutputPowerLevel,
    pub aux_out_enable: bool,
    pub aux_output_select: AuxOutputSelect,
    pub mute_till_lock_detect: bool,
    pub vco_power_down: bool,
    pub band_select_clock_divider: u8, // 0..=255
    pub rf_divider_select: RfDividerSelect,

    // R3 - Function Control (muxout, noise mode, lock detect filters)
    pub muxout_control: MuxoutControl,
    pub noise_mode: NoiseMode,
    pub ldp: Ldp,
    pub ldf: Ldf,
    pub phase_detector_polarity: PdPolarity,
    pub power_down: bool,
    pub charge_pump_three_state: bool,
    pub counter_reset: bool,

    // R2 - Reference and Charge Pump
    pub charge_pump_current: u8, // 0..=15
    pub reference_counter: u16,  // 1..=1023
    pub ref_divide_by_2: bool,
    pub ref_doubler: bool,

    // R1 - Phase and Modulus
    pub modulus: u16,         // 1..=4095
    pub phase_value: u16,     // < modulus
    pub prescaler_89: bool,   // true => 8/9, false => 4/5
    pub phase_adjust: bool,   // enable phase adjust in R1

    // R0 - Frequency Control
    pub integer_value: u16,
    pub fractional_value: u16,
}

impl InitConfig {
    /// Create an InitConfig from raw u8/u16 values.
    ///
    /// This is a convenience method for users who have raw register values
    /// and want to convert them to the type-safe enum-based InitConfig.
    ///
    /// # Panics
    ///
    /// Panics if any raw value is out of range for its corresponding enum.
    pub fn from_raw(
        lock_detect_pin_operation: u8,
        output_power: u8,
        rf_out_enable: bool,
        aux_output_power: u8,
        aux_out_enable: bool,
        aux_output_select_fundamental: bool,
        mute_till_lock_detect: bool,
        vco_power_down: bool,
        band_select_clock_divider: u8,
        rf_divider_select: u8,
        muxout_control: u8,
        noise_mode: NoiseModeSetting,
        ldp_6ns: bool,
        ldf_integer_mode: bool,
        phase_detector_polarity_positive: bool,
        power_down: bool,
        charge_pump_three_state: bool,
        counter_reset: bool,
        charge_pump_current: u8,
        reference_counter: u16,
        ref_divide_by_2: bool,
        ref_doubler: bool,
        modulus: u16,
        phase_value: u16,
        prescaler_89: bool,
        phase_adjust: bool,
        integer_value: u16,
        fractional_value: u16,
    ) -> Self {
        InitConfig {
            lock_detect_pin_operation: match lock_detect_pin_operation {
                0b00 => LockDetectPinOperation::Low,
                0b01 => LockDetectPinOperation::DigitalLockDetect,
                0b10 => LockDetectPinOperation::LowAlt,
                0b11 => LockDetectPinOperation::High,
                _ => LockDetectPinOperation::DigitalLockDetect,
            },
            output_power: match output_power {
                0b00 => OutputPowerLevel::Minus4DBm,
                0b01 => OutputPowerLevel::Minus1DBm,
                0b10 => OutputPowerLevel::Plus2DBm,
                0b11 => OutputPowerLevel::Plus5DBm,
                _ => OutputPowerLevel::Plus5DBm,
            },
            rf_out_enable,
            aux_output_power: match aux_output_power {
                0b00 => AuxOutputPowerLevel::Minus4DBm,
                0b01 => AuxOutputPowerLevel::Minus1DBm,
                0b10 => AuxOutputPowerLevel::Plus2DBm,
                0b11 => AuxOutputPowerLevel::Plus5DBm,
                _ => AuxOutputPowerLevel::Minus4DBm,
            },
            aux_out_enable,
            aux_output_select: if aux_output_select_fundamental {
                AuxOutputSelect::FundamentalOutput
            } else {
                AuxOutputSelect::DividedOutput
            },
            mute_till_lock_detect,
            vco_power_down,
            band_select_clock_divider,
            rf_divider_select: match rf_divider_select {
                0b000 => RfDividerSelect::DivBy1,
                0b001 => RfDividerSelect::DivBy2,
                0b010 => RfDividerSelect::DivBy4,
                0b011 => RfDividerSelect::DivBy8,
                0b100 => RfDividerSelect::DivBy16,
                0b101 => RfDividerSelect::DivBy32,
                0b110 => RfDividerSelect::DivBy64,
                0b111 => RfDividerSelect::Reserved,
                _ => RfDividerSelect::DivBy1,
            },
            muxout_control: match muxout_control {
                0b000 => MuxoutControl::ThreeState,
                0b001 => MuxoutControl::DVdd,
                0b010 => MuxoutControl::DGnd,
                0b011 => MuxoutControl::RCounterOutput,
                0b100 => MuxoutControl::NDividerOutput,
                0b101 => MuxoutControl::AnalogLockDetect,
                0b110 => MuxoutControl::DigitalLockDetect,
                0b111 => MuxoutControl::Reserved,
                _ => MuxoutControl::DigitalLockDetect,
            },
            noise_mode: match noise_mode {
                NoiseModeSetting::LowNoise => NoiseMode::LowNoiseMode,
                NoiseModeSetting::LowSpur => NoiseMode::LowSpurMode,
            },
            ldp: if ldp_6ns { Ldp::Nanosecs6 } else { Ldp::Nanosecs10 },
            ldf: if ldf_integer_mode { Ldf::IntN } else { Ldf::FracN },
            phase_detector_polarity: if phase_detector_polarity_positive {
                PdPolarity::Positive
            } else {
                PdPolarity::Negative
            },
            power_down,
            charge_pump_three_state,
            counter_reset,
            charge_pump_current,
            reference_counter,
            ref_divide_by_2,
            ref_doubler,
            modulus,
            phase_value,
            prescaler_89,
            phase_adjust,
            integer_value,
            fractional_value,
        }
    }
}

impl Default for InitConfig {
    fn default() -> Self {
        InitConfig {
            // R5
            lock_detect_pin_operation: LockDetectPinOperation::DigitalLockDetect,
            // R4
            output_power: OutputPowerLevel::Plus5DBm,
            rf_out_enable: true,
            aux_output_power: AuxOutputPowerLevel::Minus4DBm,
            aux_out_enable: false,
            aux_output_select: AuxOutputSelect::FundamentalOutput,
            mute_till_lock_detect: false,
            vco_power_down: false,
            band_select_clock_divider: 200,
            rf_divider_select: RfDividerSelect::DivBy1,
            // R3
            muxout_control: MuxoutControl::DigitalLockDetect,
            noise_mode: NoiseMode::LowNoiseMode,
            ldp: Ldp::Nanosecs10, // 10 ns recommended for frac-N
            ldf: Ldf::FracN,       // fractional-N default
            phase_detector_polarity: PdPolarity::Positive,
            power_down: false,
            charge_pump_three_state: false,
            counter_reset: false,
            // R2
            charge_pump_current: 7,
            reference_counter: 1,
            ref_divide_by_2: false,
            ref_doubler: false,
            // R1
            modulus: 4095,
            phase_value: 1,
            prescaler_89: false,
            phase_adjust: false,
            // R0
            integer_value: 96,
            fractional_value: 0,
        }
    }
}

impl<I, P> Adf435xDriver<I, P>
where
    I: RegisterInterface<AddressType = u8>,
    P: OutputPin,
{
    /// Program the device using the datasheet's initialization sequence (R5 → R0).
    ///
    /// This method writes the registers in the order recommended by the ADF4351 datasheet:
    /// 1. R5 (Lock Detect and Readback)
    /// 2. R4 (Output Stage)
    /// 3. R3 (Function Control)
    /// 4. R2 (Reference and Charge Pump)
    /// 5. R1 (Phase and Modulus)
    /// 6. R0 (Frequency Control)
    ///
    /// An LE pulse is issued after each register write using [`latch()`](Self::latch).
    ///
    /// The configuration values are taken from `cfg`. See [`InitConfig::default()`] for sensible defaults.
    pub fn initialize<D>(
        &mut self,
        cfg: &InitConfig,
        delay: &mut D,
    ) -> Result<(), DriverError<I::Error, P::Error, P::Error>>
    where
        D: DelayUs,
    {
        // R5
        self.device
            .r_5_latch_and_status()
            .write(|r| {
                r.set_lock_detect_pin_operation(cfg.lock_detect_pin_operation);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R4
        self.device
            .r_4_output_stage()
            .write(|r| {
                r.set_output_power(cfg.output_power);
                r.set_rf_out(cfg.rf_out_enable);
                r.set_aux_output_power(cfg.aux_output_power);
                r.set_aux_out(cfg.aux_out_enable);
                r.set_aux_output_select(cfg.aux_output_select);
                r.set_mute_till_lock_detect(cfg.mute_till_lock_detect);
                r.set_vco_power_down(cfg.vco_power_down);
                r.set_band_select_clock_divider(cfg.band_select_clock_divider);
                r.set_rf_divider_select(cfg.rf_divider_select);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R3
        self.device
            .r_3_function_control()
            .write(|r| {
                r.set_ldp(cfg.ldp);
                r.set_ldf(cfg.ldf);
                r.set_pd_polarity(cfg.phase_detector_polarity);
                r.set_power_down(cfg.power_down);
                r.set_cp_three_state(cfg.charge_pump_three_state);
                r.set_counter_reset(cfg.counter_reset);
                r.set_muxout_control(cfg.muxout_control);
                r.set_noise_mode(cfg.noise_mode);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R2
        self.device
            .r_2_reference_and_charge_pump()
            .write(|r| {
                r.set_charge_pump_current(cfg.charge_pump_current);
                r.set_reference_counter(cfg.reference_counter);
                r.set_ref_divide_by_2(cfg.ref_divide_by_2);
                r.set_ref_doubler(cfg.ref_doubler);
                // Lock detect function and precision recommended per mode
                // For fractional-N: LDF=0, LDP=0; For integer-N: LDF=1, LDP=1
                r.set_lock_detect_function(matches!(cfg.ldf, Ldf::IntN));
                r.set_lock_detect_precision(matches!(cfg.ldf, Ldf::IntN));
                r.set_phase_detector_polarity(matches!(cfg.phase_detector_polarity, PdPolarity::Positive));
                r.set_power_down(cfg.power_down);
                r.set_charge_pump_three_state(cfg.charge_pump_three_state);
                r.set_counter_reset(cfg.counter_reset);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R1
        self.device
            .r_1_phase_and_modulus()
            .write(|r| {
                r.set_modulus(cfg.modulus);
                r.set_phase_value(cfg.phase_value);
                r.set_prescaler_89(cfg.prescaler_89);
                r.set_phase_adjust(cfg.phase_adjust);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R0
        self.device
            .r_0_frequency_control()
            .write(|r| {
                r.set_integer_value(cfg.integer_value);
                r.set_fractional_value(cfg.fractional_value);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        Ok(())
    }

    /// Convenience helper to set the Noise Mode (Low Noise vs Low Spur).
    ///
    /// This modifies R3 noise mode field and issues no LE pulse by itself.
    /// Call [`latch()`](Self::latch) after invoking this method.
    pub fn set_noise_mode(
        &mut self,
        mode: NoiseMode,
    ) -> Result<(), DriverError<I::Error, P::Error, P::Error>> {
        self.device
            .r_3_function_control()
            .modify(|r| {
                r.set_noise_mode(mode);
            })
            .map_err(DriverError::Spi)
    }
}

impl<I, CE, LE> Adf435xDriverWithCE<I, CE, LE>
where
    I: RegisterInterface<AddressType = u8>,
    CE: OutputPin,
    LE: OutputPin,
{
    /// Program the device using the datasheet's initialization sequence (R5 → R0).
    ///
    /// Same behavior as [`Adf435xDriver::initialize`], but for the CE-owning driver.
    pub fn initialize<D>(
        &mut self,
        cfg: &InitConfig,
        delay: &mut D,
    ) -> Result<(), DriverError<I::Error, CE::Error, LE::Error>>
    where
        D: DelayUs16,
    {
        // R5
        self.device
            .r_5_latch_and_status()
            .write(|r| {
                r.set_lock_detect_pin_operation(cfg.lock_detect_pin_operation);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R4
        self.device
            .r_4_output_stage()
            .write(|r| {
                r.set_output_power(cfg.output_power);
                r.set_rf_out(cfg.rf_out_enable);
                r.set_aux_output_power(cfg.aux_output_power);
                r.set_aux_out(cfg.aux_out_enable);
                r.set_aux_output_select(cfg.aux_output_select);
                r.set_mute_till_lock_detect(cfg.mute_till_lock_detect);
                r.set_vco_power_down(cfg.vco_power_down);
                r.set_band_select_clock_divider(cfg.band_select_clock_divider);
                r.set_rf_divider_select(cfg.rf_divider_select);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R3
        self.device
            .r_3_function_control()
            .write(|r| {
                r.set_ldp(cfg.ldp);
                r.set_ldf(cfg.ldf);
                r.set_pd_polarity(cfg.phase_detector_polarity);
                r.set_power_down(cfg.power_down);
                r.set_cp_three_state(cfg.charge_pump_three_state);
                r.set_counter_reset(cfg.counter_reset);
                r.set_muxout_control(cfg.muxout_control);
                r.set_noise_mode(cfg.noise_mode);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R2
        self.device
            .r_2_reference_and_charge_pump()
            .write(|r| {
                r.set_charge_pump_current(cfg.charge_pump_current as u8);
                r.set_reference_counter(cfg.reference_counter);
                r.set_ref_divide_by_2(cfg.ref_divide_by_2);
                r.set_ref_doubler(cfg.ref_doubler);
                r.set_lock_detect_function(matches!(cfg.ldf, Ldf::IntN));
                r.set_lock_detect_precision(matches!(cfg.ldf, Ldf::IntN));
                r.set_phase_detector_polarity(matches!(cfg.phase_detector_polarity, PdPolarity::Positive));
                r.set_power_down(cfg.power_down);
                r.set_charge_pump_three_state(cfg.charge_pump_three_state);
                r.set_counter_reset(cfg.counter_reset);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R1
        self.device
            .r_1_phase_and_modulus()
            .write(|r| {
                r.set_modulus(cfg.modulus);
                r.set_phase_value(cfg.phase_value);
                r.set_prescaler_89(cfg.prescaler_89);
                r.set_phase_adjust(cfg.phase_adjust);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        // R0
        self.device
            .r_0_frequency_control()
            .write(|r| {
                r.set_integer_value(cfg.integer_value);
                r.set_fractional_value(cfg.fractional_value);
            })
            .map_err(DriverError::Spi)?;
        self.latch(delay)?;

        Ok(())
    }

    /// Convenience helper to set the Noise Mode (Low Noise vs Low Spur).
    ///
    /// This modifies R3 noise mode field and issues no LE pulse by itself.
    /// Call [`latch()`](Self::latch) after invoking this method.
    pub fn set_noise_mode(
        &mut self,
        mode: NoiseMode,
    ) -> Result<(), DriverError<I::Error, CE::Error, LE::Error>> {
        self.device
            .r_3_function_control()
            .modify(|r| {
                r.set_noise_mode(mode);
            })
            .map_err(DriverError::Spi)
    }
}
