use core::convert::Infallible;

use adf435x::{RegisterInterface, new_device};

#[derive(Default)]
struct MockInterface {
    memory: [u8; 24],
}

impl MockInterface {
    fn write_bytes(&mut self, offset: usize, data: &[u8]) {
        self.memory[offset..offset + data.len()].copy_from_slice(data);
    }

    fn read_bytes(&self, offset: usize, data: &mut [u8]) {
        data.copy_from_slice(&self.memory[offset..offset + data.len()]);
    }
}

impl RegisterInterface for MockInterface {
    type Error = Infallible;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        assert_eq!(size_bits, 32);
        let offset = address as usize * 4;
        self.write_bytes(offset, data);
        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        assert_eq!(size_bits, 32);
        let offset = address as usize * 4;
        self.read_bytes(offset, data);
        Ok(())
    }
}

fn main() {
    let mut device = new_device(MockInterface::default());

    device
        .r_0_frequency_control()
        .write(|reg| {
            reg.set_integer_value(320u16);
            reg.set_fractional_value(512u16);
        })
        .unwrap();

    let reg = device.r_0_frequency_control().read().unwrap();

    println!(
        "R0 -> INT: {}, FRAC: {}",
        reg.integer_value(),
        reg.fractional_value()
    );
}
