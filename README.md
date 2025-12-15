# adf435x

Rust driver for the ADF435X series of Wideband RF PLL Synthesizers using the `device-driver` crate for compile-time code generation from YAML register definitions.

## Features

- **Type-safe register access** - Generated API from YAML manifest prevents invalid operations at compile time
- **Automatic LE pulsing** - Optional high-level driver handles Load Enable pin timing automatically
- **No dependencies on std** - Works in `no_std` environments (with optional `std` support)
- **Flexible interface** - Bring your own SPI implementation via the `RegisterInterface` trait
- **YAML-driven** - All register definitions in `manifests/adf435x.yaml` for easy customization

## Supported Devices

This driver supports the ADF435x family of wideband RF PLL synthesizers:
- ADF4350
- ADF4351
- Other compatible variants

## Hardware Assumptions

### CE (Chip Enable) Pin

**This driver assumes CE is always HIGH (device always powered).** The CE pin should be tied high or controlled externally. If you need to power down the device, use the power-down bit in register R2.

### LE (Load Enable) Pin

When LE goes high, the data stored in the 32-bit shift register is loaded into the register selected by the 3 control bits (bits [2:0] of the SPI word). The LE pin must be pulsed after each SPI write.

This crate provides two usage patterns:
- **Manual LE control**: Use the raw `Device` type and pulse LE yourself
- **Automatic LE control** (recommended): Use `Adf435xDriver` which handles LE pulsing automatically

## Usage

### Pattern 1: Automatic LE Control (Recommended)

Use `Adf435xDriver` for automatic LE pulsing with proper timing:

```rust
use adf435x::{Adf435xDriver, RegisterInterface, OutputPin, DelayUs};
use core::convert::Infallible;

// Your SPI implementation
struct SpiInterface {
    // Your SPI peripheral handle
}

impl RegisterInterface for SpiInterface {
    type Error = Infallible;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // Write 32-bit register value to ADF435x over SPI
        // The address is encoded in bits [2:0] of the 32-bit word
        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Read 32-bit register from ADF435x
        Ok(())
    }
}

// Your GPIO pin implementation
struct LePin;

impl OutputPin for LePin {
    type Error = Infallible;
    
    fn set_high(&mut self) -> Result<(), Self::Error> {
        // Set LE pin high
        Ok(())
    }
    
    fn set_low(&mut self) -> Result<(), Self::Error> {
        // Set LE pin low
        Ok(())
    }
}

// Your delay implementation
struct Delay;

impl DelayUs for Delay {
    fn delay_us(&mut self, us: u32) {
        // Delay for specified microseconds
    }
}

// Create the driver
let spi = SpiInterface { /* ... */ };
let le_pin = LePin;
let mut delay = Delay;

let mut driver = Adf435xDriver::new(spi, le_pin);

// Configure registers using type-safe API
driver.device_mut().r_0_frequency_control()
    .write(|reg| {
        reg.set_integer_value(96);
        reg.set_fractional_value(0);
    })
    .unwrap();

// Latch with automatic LE pulsing (5µs → LE high → 10µs → LE low → 5µs)
driver.latch(&mut delay).unwrap();
```

### Pattern 2: Manual LE Control (Maximum Flexibility)

Use the raw `Device` type when you need full control:

```rust
use adf435x::{new_device, RegisterInterface};
use core::convert::Infallible;

struct SpiInterface;
impl RegisterInterface for SpiInterface {
    type Error = Infallible;
    type AddressType = u8;
    fn write_register(&mut self, _: u8, _: u32, _: &[u8]) -> Result<(), Self::Error> { Ok(()) }
    fn read_register(&mut self, _: u8, _: u32, _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
}

let mut device = new_device(SpiInterface);

// Write register
device.r_0_frequency_control()
    .write(|reg| {
        reg.set_integer_value(96);
        reg.set_fractional_value(0);
    })
    .unwrap();

// YOU must pulse LE manually:
// le_pin.set_high();
// delay.delay_us(10);
// le_pin.set_low();
```

### Type-Safe Register Access

The device-driver crate generates type-safe methods for all registers and fields:

```rust
// Write to frequency control register (R0)
driver.device_mut().r_0_frequency_control()
    .write(|reg| {
        reg.set_integer_value(100);      // INT value
        reg.set_fractional_value(512);   // FRAC value
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// Read and modify registers
driver.device_mut().r_2_reference_and_charge_pump()
    .modify(|reg| {
        reg.set_reference_counter(10);
        reg.set_charge_pump_current(5);
        reg.set_ref_doubler(false);
        reg.set_ref_divide_by_2(false);
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// Read current register values
let r0 = driver.device_mut().r_0_frequency_control().read().unwrap();
println!("INT: {}, FRAC: {}", r0.integer_value(), r0.fractional_value());
```

## Register Map

The ADF435x has six 32-bit registers (R0-R5) defined in `manifests/adf435x.yaml`:

- **R0** - Frequency Control
  - Integer value (INT)
  - Fractional value (FRAC)
  
- **R1** - Phase and Modulus
  - Modulus value (MOD)
  - Phase value
  - Prescaler select (4/5 or 8/9)

- **R2** - Reference and Charge Pump
  - Reference counter (R)
  - Reference doubler (D)
  - Reference divide-by-2 (T)
  - Charge pump current
  - Phase detector polarity
  - Power-down bit

- **R3** - Function Control
  - Clock divider value
  - Clock divider mode
  - Charge pump mode

- **R4** - Output Stage
  - RF divider select
  - Output power level
  - RF output enable
  - Auxiliary output settings

- **R5** - Lock Detect and Readback
  - Lock detect pin configuration
  - Digital lock detect

## Frequency Calculation (Fractional-N Mode)

The output frequency is calculated as:

```
f_OUT = f_PFD × (INT + FRAC/MOD) × output_divider
```

Where:
- `f_PFD` = Phase Frequency Detector frequency
- `f_PFD` = `f_REF × (1 + D) / (R × (1 + T))`
- `f_REF` = Reference clock frequency
- `D` = Reference doubler (0 or 1)
- `R` = Reference counter value
- `T` = Reference divide-by-2 (0 or 1)

### Example: Setting 2.4 GHz output with 25 MHz reference

```rust
let mut driver = Adf435xDriver::new(spi, le_pin);
let mut delay = Delay;

// Step 1: Configure reference path (R2)
driver.device_mut().r_2_reference_and_charge_pump()
    .write(|reg| {
        reg.set_reference_counter(1);      // R = 1
        reg.set_ref_doubler(false);         // D = 0
        reg.set_ref_divide_by_2(false);     // T = 0
        reg.set_charge_pump_current(7);     // Mid-range
        reg.set_power_down(false);          // Ensure powered up
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// Step 2: Set modulus for resolution (R1)
// f_PFD = 25 MHz × (1 + 0) / (1 × (1 + 0)) = 25 MHz
driver.device_mut().r_1_phase_and_modulus()
    .write(|reg| {
        reg.set_modulus(4095);        // Maximum resolution
        reg.set_prescaler_89(false);   // Use 4/5 prescaler for < 3.6 GHz
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// Step 3: Set frequency (R0)
// For 2.4 GHz: INT = 96, FRAC = 0
// f_OUT = 25 MHz × (96 + 0/4095) = 2.4 GHz
driver.device_mut().r_0_frequency_control()
    .write(|reg| {
        reg.set_integer_value(96);
        reg.set_fractional_value(0);
    })
    .unwrap();
driver.latch(&mut delay).unwrap();
```

## Initialization Sequence

Typical initialization sequence from datasheet:

```rust
let mut driver = Adf435xDriver::new(spi, le_pin);
let mut delay = Delay;

// Write registers in reverse order (R5 → R4 → R3 → R2 → R1 → R0)
// Each write automatically includes the register address in bits [2:0]

// 1. R5: Lock detect configuration
driver.device_mut().r_5_latch_and_status()
    .write(|reg| {
        // Configure lock detect settings via payload
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// 2. R4: Output stage
driver.device_mut().r_4_output_stage()
    .write(|reg| {
        // Configure output power, RF divider via payload
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// 3. R3: Function control
driver.device_mut().r_3_function_control()
    .write(|reg| {
        // Configure clock divider via payload
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// 4. R2: Reference path
driver.device_mut().r_2_reference_and_charge_pump()
    .write(|reg| {
        reg.set_reference_counter(1);
        reg.set_charge_pump_current(7);
        reg.set_power_down(false);
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// 5. R1: Modulus
driver.device_mut().r_1_phase_and_modulus()
    .write(|reg| {
        reg.set_modulus(4095);
    })
    .unwrap();
driver.latch(&mut delay).unwrap();

// 6. R0: Frequency
driver.device_mut().r_0_frequency_control()
    .write(|reg| {
        reg.set_integer_value(96);
        reg.set_fractional_value(0);
    })
    .unwrap();
driver.latch(&mut delay).unwrap();
```

## Platform Integration

This driver is platform-agnostic. For embedded systems, implement the required traits using your platform's HAL:

```rust
use embedded_hal::spi::SpiDevice;
use embedded_hal::digital::OutputPin as EmbeddedOutputPin;
use embedded_hal::delay::DelayUs as EmbeddedDelayUs;

// Implement adf435x::OutputPin for your GPIO pin type
impl<T: EmbeddedOutputPin> adf435x::OutputPin for T {
    type Error = T::Error;
    
    fn set_high(&mut self) -> Result<(), Self::Error> {
        EmbeddedOutputPin::set_high(self)
    }
    
    fn set_low(&mut self) -> Result<(), Self::Error> {
        EmbeddedOutputPin::set_low(self)
    }
}

// Use with your platform's types
let spi = /* your SPI peripheral */;
let le_pin = /* your GPIO pin */;
let mut delay = /* your delay provider */;

let mut driver = adf435x::Adf435xDriver::new(spi, le_pin);
// Use driver...
```

## Building and Extending

The register definitions are generated at compile time from `manifests/adf435x.yaml`. To customize the driver:

1. Edit `manifests/adf435x.yaml` to add/modify register definitions
2. Rebuild the project - the device-driver crate regenerates the API automatically
3. New fields and registers become available as type-safe Rust methods

## Examples

The crate includes two comprehensive examples demonstrating different usage patterns:

### Basic Usage (`examples/basic_usage.rs`)

Demonstrates `Adf435xDriver` - the simple driver for when CE is always high:
- Automatic LE pulsing with proper timing
- Type-safe register access via device-driver API
- Manual frequency calculations
- Read-modify-write operations
- Multiple frequency settings

```bash
cargo run --example basic_usage
```

### Full Featured (`examples/full_featured.rs`)

Demonstrates `Adf435xDriverWithCE` - the extended driver with CE control:
- CE pin control (enable/disable methods)
- Automatic LE pulsing with proper timing
- High-level frequency configuration with `Fpfd` and `FracN`
- Automatic prescaler selection (4/5 vs 8/9)
- Frequency verification and error calculation
- Testing multiple frequencies

```bash
cargo run --example full_featured
```

Both examples use mock hardware interfaces and compile without requiring actual hardware.

## License

Licensed under either of:

- Apache License, Version 2.0
- MIT license

at your option.