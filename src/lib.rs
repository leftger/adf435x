#![cfg_attr(not(feature = "std"), no_std)]

//! Device driver abstractions for the Analog Devices ADF435x wideband PLL/VCO family.
//!
//! The register map is defined in `manifests/adf435x.yaml`, and the high-level
//! Rust API is generated at compile time by the `device-driver` crate. You only
//! need to supply an implementation of [`RegisterInterface`] that can move data
//! between your MCU (or test harness) and the ADF435x part.

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
/// use adf435x::{new_device, RegisterInterface};
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
