//! Basic usage example - Adf435xDriver with automatic LE pulsing
//!
//! This example demonstrates the `Adf435xDriver` wrapper which:
//! - Owns the LE (Load Enable) pin
//! - Automatically handles LE pulsing with proper timing (5µs → 10µs → 5µs)
//! - Assumes CE (Chip Enable) is always HIGH (device always powered)
//! - Provides type-safe register access via device-driver API
//!
//! This is the recommended pattern when CE is tied high or controlled externally.
//!
//! Run with: `cargo run --example basic_usage`

use adf435x::{Adf435xDriver, DelayUs, OutputPin, RegisterInterface};
use core::convert::Infallible;

/// Mock SPI interface for demonstration
///
/// In a real application, this would be your platform's SPI peripheral
#[derive(Default)]
struct MockInterface {
    memory: [u8; 24], // 6 registers × 4 bytes each
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
        assert_eq!(size_bits, 32, "ADF435x uses 32-bit registers");
        let offset = address as usize * 4;
        self.write_bytes(offset, data);
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
        assert_eq!(size_bits, 32, "ADF435x uses 32-bit registers");
        let offset = address as usize * 4;
        self.read_bytes(offset, data);
        Ok(())
    }
}

/// Mock LE pin for demonstration
///
/// In a real application, use your platform's GPIO implementation
struct MockLePin {
    state: bool,
}

impl MockLePin {
    fn new() -> Self {
        Self { state: false }
    }
}

impl OutputPin for MockLePin {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        println!("  LE -> HIGH");
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        println!("  LE -> LOW");
        Ok(())
    }
}

/// Mock delay provider for demonstration
///
/// In a real application, use embedded-hal delay traits
struct MockDelay;

impl DelayUs for MockDelay {
    fn delay_us(&mut self, us: u32) {
        println!("  delay {}µs", us);
    }
}

fn main() {
    println!("=== ADF435x Driver Demo (with automatic LE pulsing) ===");
    println!("CE (Chip Enable) is assumed to be always HIGH\n");

    let spi = MockInterface::default();
    let le_pin = MockLePin::new();
    let mut delay = MockDelay;

    let mut driver = Adf435xDriver::new(spi, le_pin);

    // NEW: One-shot initialization using InitConfig
    println!("1. One-shot initialization using InitConfig:");
    println!("   Programming R5→R0 in datasheet order with automatic LE pulsing");
    let mut cfg = adf435x::InitConfig::default();
    cfg.reference_counter = 1;     // R = 1 for 25 MHz reference
    cfg.modulus = 4095;            // Maximum resolution
    cfg.prescaler_89 = true;       // 8/9 prescaler for higher frequencies
    cfg.integer_value = 96;        // INT = 96 for 2.4 GHz with 25 MHz PFD
    cfg.fractional_value = 0;      // FRAC = 0 (integer mode)
    cfg.charge_pump_current = 7;   // Mid-range
    cfg.output_power = adf435x::OutputPowerLevel::Plus5DBm; // +5 dBm
    cfg.rf_out_enable = true;      // Enable primary output

    driver.initialize(&cfg, &mut delay).unwrap();
    println!("   All registers programmed and latched!\n");

    // Alternative: Manual register-by-register approach (commented out)
    /*
    // Step 1: Configure reference path (R2)
    println!("1. Configure reference path (R2):");
    println!("   Setting: R=1, D=0, T=0 for 25 MHz reference");
    driver
        .device_mut()
        .r_2_reference_and_charge_pump()
        .write(|reg| {
            reg.set_reference_counter(1); // R = 1
            reg.set_ref_doubler(false); // D = 0
            reg.set_ref_divide_by_2(false); // T = 0
            reg.set_charge_pump_current(7); // Mid-range charge pump
            reg.set_phase_detector_polarity(true); // Positive polarity
            reg.set_power_down(false); // Ensure not powered down
        })
        .unwrap();

    // Automatic LE pulsing!
    println!("   Latching register (automatic LE pulse with timing):");
    driver.latch(&mut delay).unwrap();
    println!();

    // Step 2: Set modulus for fractional-N resolution (R1)
    println!("2. Configure modulus (R1):");
    println!("   Setting: MOD=4095 (maximum resolution), prescaler=8/9");
    driver
        .device_mut()
        .r_1_phase_and_modulus()
        .write(|reg| {
            reg.set_modulus(4095); // Maximum resolution
            reg.set_phase_value(1); // Default phase
            reg.set_prescaler_89(true); // 8/9 prescaler for higher frequencies
        })
        .unwrap();

    println!("   Latching register (automatic LE pulse with timing):");
    driver.latch(&mut delay).unwrap();
    println!();

    // Step 3: Set output frequency (R0)
    // For 2.4 GHz with 25 MHz PFD: INT = 96, FRAC = 0
    // f_OUT = f_PFD × (INT + FRAC/MOD)
    // f_OUT = 25 MHz × (96 + 0/4095) = 2.4 GHz
    println!("3. Set output frequency to 2.4 GHz (R0):");
    println!("   Formula: f_OUT = f_PFD × (INT + FRAC/MOD)");
    println!("   f_OUT = 25 MHz × (96 + 0/4095) = 2.4 GHz");
    driver
        .device_mut()
        .r_0_frequency_control()
        .write(|reg| {
            reg.set_integer_value(96); // INT = 96
            reg.set_fractional_value(0); // FRAC = 0 (integer mode for this example)
        })
        .unwrap();

    println!("   Latching register (automatic LE pulse with timing):");
    driver.latch(&mut delay).unwrap();
    println!();
    */

    // Step 4: Read back and verify configuration
    println!("4. Read back configuration:");

    let r0 = driver.device_mut().r_0_frequency_control().read().unwrap();
    println!(
        "   R0: INT={}, FRAC={}",
        r0.integer_value(),
        r0.fractional_value()
    );

    let r1 = driver.device_mut().r_1_phase_and_modulus().read().unwrap();
    println!(
        "   R1: MOD={}, Phase={}, Prescaler={}",
        r1.modulus(),
        r1.phase_value(),
        if r1.prescaler_89() { "8/9" } else { "4/5" }
    );

    let r2 = driver
        .device_mut()
        .r_2_reference_and_charge_pump()
        .read()
        .unwrap();
    println!(
        "   R2: R={}, D={}, T={}, CP_current={}",
        r2.reference_counter(),
        if r2.ref_doubler() { 1 } else { 0 },
        if r2.ref_divide_by_2() { 1 } else { 0 },
        r2.charge_pump_current()
    );
    println!();

    // Step 5: Demonstrate modifying a register (read-modify-write)
    println!("5. Modify charge pump current using .modify():");
    driver
        .device_mut()
        .r_2_reference_and_charge_pump()
        .modify(|reg| {
            println!("   Current value: {}", reg.charge_pump_current());
            reg.set_charge_pump_current(10); // Increase charge pump current
            println!("   New value: 10");
        })
        .unwrap();

    println!("   Latching register (automatic LE pulse with timing):");
    driver.latch(&mut delay).unwrap();
    println!();

    // Step 6: Calculate actual frequency
    println!("6. Calculate actual output frequency:");
    let r0 = driver.device_mut().r_0_frequency_control().read().unwrap();
    let r1 = driver.device_mut().r_1_phase_and_modulus().read().unwrap();
    let r2 = driver
        .device_mut()
        .r_2_reference_and_charge_pump()
        .read()
        .unwrap();

    let ref_freq_hz = 25_000_000_u64; // 25 MHz
    let r = r2.reference_counter() as u64;
    let d = if r2.ref_doubler() { 1 } else { 0 };
    let t = if r2.ref_divide_by_2() { 1 } else { 0 };

    let f_pfd = ref_freq_hz * (1 + d) / (r * (1 + t));
    println!(
        "   f_PFD = {} MHz × (1 + {}) / ({} × (1 + {})) = {} MHz",
        ref_freq_hz / 1_000_000,
        d,
        r,
        t,
        f_pfd / 1_000_000
    );

    let int_val = r0.integer_value() as u64;
    let frac_val = r0.fractional_value() as u64;
    let mod_val = r1.modulus() as u64;

    let f_out = f_pfd * int_val + (f_pfd * frac_val) / mod_val;
    println!(
        "   f_OUT = {} MHz × ({} + {}/{}) = {} MHz",
        f_pfd / 1_000_000,
        int_val,
        frac_val,
        mod_val,
        f_out / 1_000_000
    );
    println!("   f_OUT = {} GHz", f_out as f64 / 1_000_000_000.0);
    println!();

    // Step 7: Demonstrate multiple frequency settings
    println!("7. Testing different frequencies:");
    let test_frequencies = [
        (100, 0, "2.5 GHz"),      // INT=100, FRAC=0
        (80, 2048, "2.0125 GHz"), // INT=80, FRAC=2048 (half of MOD)
        (120, 0, "3.0 GHz"),      // INT=120, FRAC=0
    ];

    for (int, frac, desc) in test_frequencies {
        driver
            .device_mut()
            .r_0_frequency_control()
            .write(|reg| {
                reg.set_integer_value(int);
                reg.set_fractional_value(frac);
            })
            .unwrap();
        driver.latch(&mut delay).unwrap();

        let calculated = f_pfd * int as u64 + (f_pfd * frac as u64) / mod_val;
        println!(
            "   INT={:3}, FRAC={:4} → {} ({:.4} GHz)",
            int,
            frac,
            desc,
            calculated as f64 / 1_000_000_000.0
        );
    }
    println!();

    println!("=== Demo complete ===\n");

    println!("Key takeaways:");
    println!("✓ Adf435xDriver automatically handles LE pulsing with proper timing");
    println!("✓ CE (Chip Enable) is assumed to be always HIGH (device always powered)");
    println!("✓ Use initialize() for one-shot R5→R0 programming with InitConfig");
    println!("✓ Use device_mut() to access type-safe register API for fine-grained control");
    println!("✓ Call latch() after register modifications to load data into the chip");
    println!("✓ Register access is compile-time checked for type safety");
    println!("\nFor real hardware:");
    println!("• Implement RegisterInterface for your SPI peripheral");
    println!("• Implement OutputPin for your GPIO (LE pin)");
    println!("• Implement DelayUs for timing (e.g., embedded-hal delay)");
    println!("• Ensure CE pin is held HIGH externally");
    println!("\nSee also:");
    println!("• examples/full_featured.rs - For CE pin control and frequency helpers");
}
