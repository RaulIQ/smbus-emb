//! SMBus smart-battery slave stub for ArduPilot.
//!
//! # Wiring (STM32F103 Blue Pill)
//!
//! ```text
//!   PB6 ── SCL ── autopilot I2C
//!   PB7 ── SDA
//!   4.7k pull-ups on SCL/SDA
//! ```
//!
//! # ArduPilot
//!
//! - `BATT_MONITOR` = **7** (SMBusGeneric)
//! - `BATT_I2C_ADDR` = **11** (0x0B)
//! - `BATT_I2C_BUS` = bus where the MCU is connected
//!
//! Stub values: 12.6 V, 0 A, 25 °C, 5000 mAh full / 2500 mAh remaining.

#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{
    self, Config as I2cConfig, I2c, SlaveAddrConfig, SlaveCommandKind, Smbus, SmbusConfig,
};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use panic_probe as _;

/// 7-bit SMBus battery address (ArduPilot default).
const SLAVE_ADDR: u8 = 0x0B;

/// Block payload for register 0x20 (manufacturer name).
const MANUFACTURER_NAME: &[u8] = b"Stub";

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!("SMBus slave stub @ 0x{:02X} on I2C1 (PB6/PB7)", SLAVE_ADDR);

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000);

    let i2c_slave = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, i2c_cfg)
        .into_slave_multimaster(SlaveAddrConfig::basic(SLAVE_ADDR));
    let smbus_slave = Smbus::new(i2c_slave, SmbusConfig::default());

    spawner.spawn(smbus_slave_task(smbus_slave).unwrap());

    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(blinker(led).unwrap());
}

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>) {
    let mut on = false;
    loop {
        if on {
            led.set_high();
        } else {
            led.set_low();
        }
        on = !on;
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn smbus_slave_task(
    mut slave: Smbus<'static, embassy_stm32::mode::Blocking, i2c::mode::MultiMaster>,
) {
    let mut last_reg = 0u8;
    // After a block length byte, the next read returns length + data.
    let mut block_data_pending = false;

    loop {
        match slave.blocking_listen() {
            Ok(cmd) => match cmd.kind {
                SlaveCommandKind::Write => {
                    let mut buf = [0u8; 4];
                    match slave.blocking_respond_to_write(&mut buf) {
                        Ok(n) if n > 0 => {
                            last_reg = buf[0];
                            block_data_pending = false;
                            info!("reg write: 0x{:02X}", last_reg);
                        }
                        Ok(_) => {}
                        Err(e) => error!("write: {}", i2c_err(e)),
                    }
                }
                SlaveCommandKind::Read => {
                    let mut payload = [0u8; 32];
                    let len = fill_read_payload(last_reg, &mut block_data_pending, &mut payload);
                    match slave.blocking_respond_to_read(&payload[..len]) {
                        Ok(n) => info!("reg 0x{:02X} read: {} bytes", last_reg, n),
                        Err(e) => error!("read: {}", i2c_err(e)),
                    }
                }
            },
            Err(e) => {
                error!("listen: {}", i2c_err(e));
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
}

/// Fill `out` with the SMBus read response; returns payload length.
fn fill_read_payload(reg: u8, block_data_pending: &mut bool, out: &mut [u8]) -> usize {
    if is_block_register(reg) {
        if *block_data_pending {
            *block_data_pending = false;
            let n = MANUFACTURER_NAME.len();
            out[0] = n as u8;
            out[1..=n].copy_from_slice(MANUFACTURER_NAME);
            n + 1
        } else {
            *block_data_pending = true;
            out[0] = MANUFACTURER_NAME.len() as u8;
            1
        }
    } else {
        *block_data_pending = false;
        let v = register_word(reg);
        out[0] = v as u8;
        out[1] = (v >> 8) as u8;
        2
    }
}

fn is_block_register(reg: u8) -> bool {
    matches!(reg, 0x20 | 0x23)
}

/// Smart Battery word values (little-endian on the wire).
fn register_word(reg: u8) -> u16 {
    match reg {
        // ArduPilot BATTMONITOR_SMBUS_* registers
        0x08 => 2981,   // temperature, 0.1 K → ~25 °C
        0x09 => 12_600, // voltage, mV → 12.6 V
        0x0A => 0,      // current, mA (signed)
        0x0F => 2_500,  // remaining capacity, mAh
        0x10 => 5_000,  // full charge capacity, mAh
        0x17 => 42,     // cycle count
        0x1A => 0x0010, // specification info: version 1 → no PEC
        0x1C => 0x1234, // serial number
        // Per-cell voltages (optional, mV)
        0x3f | 0x3e | 0x3d | 0x3c => 4_200,
        _ => 0,
    }
}

fn i2c_err(e: embassy_stm32::i2c::Error) -> &'static str {
    match e {
        embassy_stm32::i2c::Error::Bus => "bus",
        embassy_stm32::i2c::Error::Arbitration => "arbitration",
        embassy_stm32::i2c::Error::Nack => "nack",
        embassy_stm32::i2c::Error::Timeout => "timeout",
        embassy_stm32::i2c::Error::Crc => "crc",
        embassy_stm32::i2c::Error::Overrun => "overrun",
        embassy_stm32::i2c::Error::ZeroLengthTransfer => "zero-length",
    }
}
