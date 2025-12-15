//! Full-featured example - Adf435xDriverWithCE with frequency helpers
//!
//! This example demonstrates the `Adf435xDriverWithCE` wrapper which:
//! - Owns both CE (Chip Enable) and LE (Load Enable) pins
//! - Provides `enable()` / `disable()` methods for power control via CE
//! - Automatically handles LE pulsing with proper timing (5µs → 10µs → 5µs)
//! - Uses high-level frequency helpers (`Fpfd` and `FracN`)
//! - Demonstrates automatic prescaler selection
//! - Shows frequency calculation and verification
//!
//! This is the recommended pattern when you need software control of device power
//! and want high-level frequency configuration.
//!
//! Run with: `cargo run --example full_featured`

use adf435x::{Adf435xDriverWithCE, DelayUs16, Fpfd, FracN, RegisterInterface};
use core::convert::Infallible;

// Mock SPI interface
struct MockSpi {
    memory: [u8; 24],
}

impl MockSpi {
    fn new() -> Self {
        Self { memory: [0; 24] }
    }
}

impl RegisterInterface for MockSpi {
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
        self.memory[offset..offset + data.len()].copy_from_slice(data);
        println!(
            "  → SPI write to R{}: {:02X} {:02X} {:02X} {:02X}",
            address, data[0], data[1], data[2], data[3]
        );
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
        data.copy_from_slice(&self.memory[offset..offset + data.len()]);
        Ok(())
    }
}

// Mock GPIO pin
struct MockPin {
    name: &'static str,
    state: bool,
}

impl MockPin {
    fn new(name: &'static str) -> Self {
        Self { name, state: false }
    }
}

impl adf435x::OutputPin for MockPin {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        println!("  {} -> HIGH", self.name);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        println!("  {} -> LOW", self.name);
        Ok(())
    }
}

// Mock delay provider
struct MockDelay;

impl DelayUs16 for MockDelay {
    fn delay_us(&mut self, us: u16) {
        println!("  delay {}µs", us);
    }
}

fn main() {
    println!("=== Full Featured Example - Adf435xDriverWithCE ===\n");

    // Create driver with owned CE and LE pins
    let spi = MockSpi::new();
    let ce_pin = MockPin::new("CE");
    let le_pin = MockPin::new("LE");
    let mut delay = MockDelay;

    let mut driver = Adf435xDriverWithCE::new(spi, ce_pin, le_pin);

    // Power up the device
    println!("1. Powering up device (CE pin):");
    driver.enable().unwrap();
    println!();

    // NEW: One-shot initialization using InitConfig
    println!("2. One-shot initialization using InitConfig:");
    println!("   Programming R5→R0 in datasheet order with automatic LE pulsing");
    let mut cfg = adf435x::InitConfig::default();
    cfg.reference_counter = 1;     // R = 1 for 25 MHz reference
    cfg.modulus = 4095;            // Maximum resolution
    cfg.integer_value = 96;        // INT = 96 for 2.4 GHz with 25 MHz PFD
    cfg.fractional_value = 0;      // FRAC = 0 (integer mode)
    cfg.charge_pump_current = 7;   // Mid-range
    cfg.output_power = adf435x::OutputPowerLevel::Plus5DBm; // +5 dBm
    cfg.rf_out_enable = true;      // Enable primary output
    cfg.noise_mode = adf435x::NoiseMode::LowNoiseMode; // Start with low noise

    driver.initialize(&cfg, &mut delay).unwrap();
    println!("   All registers programmed and latched!\n");

    // Alternative: Manual register-by-register approach (commented out)
    /*
    // Configure reference path using the device-driver API
    println!("2. Configuring reference path (R2):");
    driver
        .device_mut()
        .r_2_reference_and_charge_pump()
        .write(|r| {
            r.set_reference_counter(1); // R = 1
            r.set_ref_doubler(false); // D = 0
            r.set_ref_divide_by_2(false); // T = 0
            r.set_charge_pump_current(7); // Mid-range
            r.set_power_down(false); // Ensure powered up
        })
        .unwrap();
    driver.latch(&mut delay).unwrap();
    println!();

    // Configure modulus
    println!("3. Configuring modulus (R1):");
    driver
        .device_mut()
        .r_1_phase_and_modulus()
        .write(|r| {
            r.set_modulus(4095); // Maximum resolution
            r.set_phase_value(1); // Default phase
        })
        .unwrap();
    driver.latch(&mut delay).unwrap();
    println!();
    */

    // Use high-level frequency configuration
    println!("4. Setting output frequency using FracN:");
    const REF_FREQ_HZ: u32 = 25_000_000;
    const TARGET_FREQ_HZ: u64 = 2_400_000_000;

    let fpfd = Fpfd::from_device(REF_FREQ_HZ, driver.device_mut()).unwrap();
    println!("  PFD frequency: {} MHz", fpfd.0 / 1_000_000);

    let frac_n = FracN::new(fpfd);
    frac_n.init(driver.device_mut()).unwrap();
    frac_n
        .set_frequency(driver.device_mut(), TARGET_FREQ_HZ)
        .unwrap();

    // Latch the frequency configuration
    driver.latch(&mut delay).unwrap();
    println!();

    // Read back and verify
    println!("5. Reading back configuration:");
    let r0 = driver.device_mut().r_0_frequency_control().read().unwrap();
    let int_val = r0.integer_value();
    let frac_val = r0.fractional_value();
    println!("  R0: INT={}, FRAC={}", int_val, frac_val);

    // Verify frequency
    println!("\n6. Verifying output frequency:");
    let actual_freq = frac_n.get_frequency(driver.device_mut()).unwrap();
    println!(
        "  Target:  {} Hz ({:.4} GHz)",
        TARGET_FREQ_HZ,
        TARGET_FREQ_HZ as f64 / 1e9
    );
    println!(
        "  Actual:  {} Hz ({:.4} GHz)",
        actual_freq,
        actual_freq as f64 / 1e9
    );
    let error = (actual_freq as i64 - TARGET_FREQ_HZ as i64).abs();
    println!("  Error:   {} Hz", error);
    println!();

    // Test a few more frequencies
    println!("7. Testing multiple frequencies:");
    let test_frequencies = [
        (500_000_000, "500 MHz"),
        (1_000_000_000, "1.0 GHz"),
        (2_000_000_000, "2.0 GHz"),
        (3_500_000_000, "3.5 GHz"),
        (4_000_000_000, "4.0 GHz"),
    ];

    for (freq, name) in test_frequencies {
        match frac_n.set_frequency(driver.device_mut(), freq) {
            Ok(()) => {
                driver.latch(&mut delay).unwrap();
                let actual = frac_n.get_frequency(driver.device_mut()).unwrap();
                let error = (actual as i64 - freq as i64).abs();
                println!("  {} -> {} Hz (error: {} Hz)", name, actual, error);
            }
            Err(_) => println!("  {} -> Error setting frequency", name),
        }
    }
    println!();

    // Demonstrate noise mode switching
    println!("7.5. Switching to Low Spur mode for better spurious performance:");
    driver.set_noise_mode(adf435x::NoiseMode::LowSpurMode).unwrap();
    driver.latch(&mut delay).unwrap();
    println!("    Switched to Low Spur mode\n");

    // Power down
    println!("8. Powering down device (CE pin):");
    driver.disable().unwrap();
    println!();

    println!("=== Example complete ===\n");
    println!("Key features demonstrated:");
    println!("✓ Adf435xDriverWithCE owns both CE and LE pins");
    println!("✓ enable() / disable() methods for CE control");
    println!("✓ Automatic LE pulsing with latch() method");
    println!("✓ Type-safe register access via device_mut()");
    println!("✓ High-level frequency configuration with Fpfd and FracN");
    println!("✓ Automatic prescaler selection based on frequency");
    println!("\nFor real hardware:");
    println!("• Implement RegisterInterface for your SPI peripheral");
    println!("• Implement OutputPin for your GPIO (CE and LE pins)");
    println!("• Implement DelayUs16 for timing (e.g., embedded-hal delay)");
    println!("\nSee also:");
    println!("• examples/basic_usage.rs - For simpler usage when CE is always high");
}
