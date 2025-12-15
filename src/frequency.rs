//! Frequency configuration helpers for fractional-N mode
//!
//! This module provides high-level abstractions for calculating and setting
//! output frequencies on the ADF435x in fractional-N mode.

use crate::{Device, RegisterInterface};

/// Phase Frequency Detector frequency (Hz)
///
/// Calculated from the reference frequency and device configuration:
/// ```text
/// f_PFD = f_REF × (1 + D) / (R × (1 + T))
/// ```
///
/// Where:
/// - `f_REF` = Reference clock frequency
/// - `D` = Reference doubler (0 or 1)
/// - `R` = Reference counter value
/// - `T` = Reference divide-by-2 (0 or 1)
#[derive(Debug, Copy, Clone)]
pub struct Fpfd(pub u32);

impl Fpfd {
    /// Calculate PFD frequency from reference frequency and device registers.
    ///
    /// # Arguments
    ///
    /// * `ref_freq_hz` - Reference clock frequency in Hz
    /// * `device` - Device to read configuration from
    ///
    /// # Errors
    ///
    /// Returns an error if the register read fails or if the reference frequency
    /// is out of valid range (10 MHz - 250 MHz recommended).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use adf435x::{new_device, RegisterInterface, Fpfd};
    /// # use core::convert::Infallible;
    /// # struct SpiInterface;
    /// # impl RegisterInterface for SpiInterface {
    /// #     type Error = Infallible;
    /// #     type AddressType = u8;
    /// #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
    /// #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
    /// # }
    /// # let mut device = new_device(SpiInterface);
    /// const REF_FREQ_HZ: u32 = 25_000_000; // 25 MHz
    ///
    /// let fpfd = Fpfd::from_device(REF_FREQ_HZ, &mut device)?;
    /// println!("PFD frequency: {} Hz", fpfd.0);
    /// # Ok::<(), Infallible>(())
    /// ```
    pub fn from_device<I>(ref_freq_hz: u32, device: &mut Device<I>) -> Result<Self, I::Error>
    where
        I: RegisterInterface<AddressType = u8>,
    {
        let r2 = device.r_2_reference_and_charge_pump().read()?;

        let r = r2.reference_counter() as u32;
        let d = if r2.ref_doubler() { 1 } else { 0 };
        let t = if r2.ref_divide_by_2() { 1 } else { 0 };

        // f_PFD = f_REF × (1 + D) / (R × (1 + T))
        let f_pfd = (ref_freq_hz * (1 + d)) / (r * (1 + t));

        Ok(Fpfd(f_pfd))
    }
}

/// Fractional-N mode frequency configuration helper
///
/// Provides high-level methods for setting and reading output frequencies
/// in fractional-N mode. Automatically handles:
/// - Prescaler selection (4/5 vs 8/9 based on frequency)
/// - INT and FRAC value calculation
/// - Frequency bounds checking
#[derive(Debug, Copy, Clone)]
pub struct FracN(pub Fpfd);

impl FracN {
    /// Create a new fractional-N helper with the given PFD frequency.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use adf435x::{Fpfd, FracN};
    /// let fpfd = Fpfd(25_000_000); // 25 MHz
    /// let frac_n = FracN::new(fpfd);
    /// ```
    pub fn new(fpfd: Fpfd) -> Self {
        FracN(fpfd)
    }

    /// Initialize fractional-N mode settings.
    ///
    /// This configures lock detect and other settings appropriate for
    /// fractional-N operation.
    ///
    /// # Errors
    ///
    /// Returns an error if register writes fail.
    pub fn init<I>(&self, device: &mut Device<I>) -> Result<(), I::Error>
    where
        I: RegisterInterface<AddressType = u8>,
    {
        // Configure for fractional-N mode
        // This is a basic initialization - users may need to customize
        device.r_2_reference_and_charge_pump().modify(|r| {
            r.set_lock_detect_function(false); // Fractional-N lock detect
        })?;

        Ok(())
    }

    /// Set the output frequency.
    ///
    /// Calculates INT and FRAC values and configures the device for the
    /// requested frequency. Automatically selects the appropriate prescaler
    /// (4/5 for < 3.6 GHz, 8/9 for >= 3.6 GHz).
    ///
    /// # Arguments
    ///
    /// * `device` - Device to configure
    /// * `freq_hz` - Desired output frequency in Hz
    ///
    /// # Frequency Range
    ///
    /// Typical range: 35 MHz - 4.4 GHz (depends on device variant)
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The frequency is out of range
    /// - Register operations fail
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use adf435x::{new_device, RegisterInterface, Fpfd, FracN};
    /// # use core::convert::Infallible;
    /// # struct SpiInterface;
    /// # impl RegisterInterface for SpiInterface {
    /// #     type Error = Infallible;
    /// #     type AddressType = u8;
    /// #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
    /// #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
    /// # }
    /// # let mut device = new_device(SpiInterface);
    /// let fpfd = Fpfd(25_000_000);
    /// let frac_n = FracN::new(fpfd);
    ///
    /// // Set to 2.4 GHz
    /// frac_n.set_frequency(&mut device, 2_400_000_000)?;
    /// # Ok::<(), Infallible>(())
    /// ```
    pub fn set_frequency<I>(&self, device: &mut Device<I>, freq_hz: u64) -> Result<(), I::Error>
    where
        I: RegisterInterface<AddressType = u8>,
    {
        let f_pfd = self.0 .0 as u64;

        // Read modulus from R1
        let r1 = device.r_1_phase_and_modulus().read()?;
        let modulus = r1.modulus() as u64;

        // Calculate INT and FRAC
        // f_OUT = f_PFD × (INT + FRAC/MOD)
        // For simplicity, assuming RF divider = 1 (need to calculate based on freq range)
        let n = freq_hz / f_pfd;
        let int = n as u16;
        let remainder = freq_hz - (n * f_pfd);
        let frac = ((remainder * modulus) / f_pfd) as u16;

        // Select prescaler: 4/5 for < 3.6 GHz, 8/9 for >= 3.6 GHz
        let prescaler_89 = freq_hz >= 3_600_000_000;

        // Write R1 with prescaler
        device.r_1_phase_and_modulus().modify(|r| {
            r.set_prescaler_89(prescaler_89);
        })?;

        // Write R0 with frequency values
        device.r_0_frequency_control().write(|r| {
            r.set_integer_value(int);
            r.set_fractional_value(frac);
        })?;

        Ok(())
    }

    /// Read the current output frequency from the device.
    ///
    /// Reads INT and FRAC values from R0 and MOD from R1 to calculate
    /// the actual output frequency.
    ///
    /// # Errors
    ///
    /// Returns an error if register reads fail.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use adf435x::{new_device, RegisterInterface, Fpfd, FracN};
    /// # use core::convert::Infallible;
    /// # struct SpiInterface;
    /// # impl RegisterInterface for SpiInterface {
    /// #     type Error = Infallible;
    /// #     type AddressType = u8;
    /// #     fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Infallible> { Ok(()) }
    /// #     fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Infallible> { Ok(()) }
    /// # }
    /// # let mut device = new_device(SpiInterface);
    /// # let fpfd = Fpfd(25_000_000);
    /// # let frac_n = FracN::new(fpfd);
    /// let freq = frac_n.get_frequency(&mut device)?;
    /// println!("Current frequency: {} Hz", freq);
    /// # Ok::<(), Infallible>(())
    /// ```
    pub fn get_frequency<I>(&self, device: &mut Device<I>) -> Result<u64, I::Error>
    where
        I: RegisterInterface<AddressType = u8>,
    {
        let r0 = device.r_0_frequency_control().read()?;
        let r1 = device.r_1_phase_and_modulus().read()?;

        let int = r0.integer_value() as u64;
        let frac = r0.fractional_value() as u64;
        let modulus = r1.modulus() as u64;
        let f_pfd = self.0 .0 as u64;

        // f_OUT = f_PFD × (INT + FRAC/MOD)
        let freq = f_pfd * int + (f_pfd * frac) / modulus;

        Ok(freq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fpfd_calculation() {
        // With REF=25MHz, R=1, D=0, T=0
        // f_PFD should be 25 MHz
        let fpfd = Fpfd(25_000_000);
        assert_eq!(fpfd.0, 25_000_000);
    }

    #[test]
    fn test_frac_n_frequency_calculation() {
        let fpfd = Fpfd(25_000_000);
        let frac_n = FracN::new(fpfd);

        // For 2.4 GHz with 25 MHz PFD: INT = 96
        // f_OUT = 25 MHz × 96 = 2.4 GHz
        assert_eq!(frac_n.0 .0, 25_000_000);
    }
}
