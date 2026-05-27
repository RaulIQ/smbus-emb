//! SMBus smart-battery slave stub for ArduPilot.
//!
//! # Wiring (STM32F103 Blue Pill)
//!
//! ```text
//!   PB10 ── SCL ── autopilot I2C
//!   PB11 ── SDA
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

use defmt::{debug, error, info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c, SlaveAddrConfig, SlaveCommandKind};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use panic_probe as _;

/// 7-bit SMBus battery address (ArduPilot default).
const SLAVE_ADDR: u8 = 0x0B;

// Keep values aligned with the ESP32-C3 stub you provided.
const TEXT_MANUFACTURER: &str = "ESP32-C3";
const TEXT_DEVICE_NAME: &str = "SmartBat-Stub";

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!(
        "SMBus slave stub @ 0x{:02X} on I2C2 (PB10/PB11)",
        SLAVE_ADDR
    );

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000);

    // Plain I2C slave mode (ESP `on_receive`/`on_request` style):
    // master writes 1-byte command, then does separate read transaction.
    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, i2c_cfg);
    let i2c_slave = i2c.into_slave_multimaster(SlaveAddrConfig::basic(SLAVE_ADDR));

    spawner.spawn(i2c_slave_task(i2c_slave).unwrap());

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
async fn i2c_slave_task(
    mut slave: I2c<'static, embassy_stm32::mode::Blocking, i2c::mode::MultiMaster>,
) {
    let mut pending_cmd: u8 = 0;
    let mut cmd_valid = false;
    let mut last_cmd: u8 = 0;

    loop {
        match slave.blocking_listen() {
            Ok(cmd) => match cmd.kind {
                SlaveCommandKind::Write => {
                    let mut buf = [0u8; 4];
                    match slave.blocking_respond_to_write(&mut buf) {
                        Ok(n) if n > 0 => {
                            pending_cmd = buf[0];
                            cmd_valid = true;
                            last_cmd = pending_cmd;
                            info!("RX cmd=0x{:02X}", pending_cmd);
                        }
                        Ok(_) => {}
                        Err(e) => error!("write: {}", i2c_err(e)),
                    }
                }
                SlaveCommandKind::Read => {
                    let cmd = if cmd_valid { pending_cmd } else { 0x00 };
                    cmd_valid = false;

                    let mut payload = [0u8; 36];
                    let len = smbus_battery_fill_response(cmd, &mut payload);
                    match slave.blocking_respond_to_read(&payload[..len]) {
                        Ok(0) => {
                            warn!("TX cmd=0x{:02X} aborted by master (len=0)", cmd);
                        }
                        Ok(n) => info!("TX cmd=0x{:02X} len={}", cmd, n),
                        Err(e) => error!("read: {}", i2c_err(e)),
                    }
                }
            },
            Err(e) => {
                match e {
                    embassy_stm32::i2c::Error::Timeout => {
                        // Idle bus is expected when master is not polling.
                        debug!("listen timeout (last_cmd=0x{:02X})", last_cmd);
                    }
                    _ => error!("listen: {}", i2c_err(e)),
                }
                Timer::after(Duration::from_millis(20)).await;
            }
        }
    }
}

fn append_word(out: &mut [u8], value: u16) -> usize {
    if out.len() < 2 {
        return 0;
    }
    out[0] = (value & 0xFF) as u8;
    out[1] = (value >> 8) as u8;
    2
}

fn append_block(out: &mut [u8], text: &str) -> usize {
    let bytes = text.as_bytes();
    if bytes.len() > 31 || out.len() < 1 + bytes.len() {
        return 0;
    }
    out[0] = bytes.len() as u8;
    out[1..1 + bytes.len()].copy_from_slice(bytes);
    1 + bytes.len()
}

/// Build SMBus read response for `cmd` into `out`.
/// Word: LSB first. Block: [length][payload...].
/// Unknown command: returns a single 0x00 byte.
fn smbus_battery_fill_response(cmd: u8, out: &mut [u8]) -> usize {
    match cmd {
        // Smart Battery Specification 1.1 command codes.
        0x03 => append_word(out, 0x0000), // ManufacturerAccess()
        0x08 => append_word(out, 2982),   // Temperature() 25.0°C in 0.1K
        0x09 => append_word(out, 12_600), // Voltage() mV
        0x0A => append_word(out, 0),      // Current() mA (signed)
        0x0D => append_word(out, 75),     // RelativeStateOfCharge() %
        0x0F => append_word(out, 3000),   // RemainingCapacity() mAh
        0x10 => append_word(out, 4000),   // FullChargeCapacity() mAh
        0x16 => append_word(out, 0x0000), // BatteryStatus() OK
        0x18 => append_word(out, 4000),   // DesignCapacity()
        0x19 => append_word(out, 12_600), // DesignVoltage()
        0x1C => append_word(out, 0x0001), // SerialNumber()

        0x1D => append_block(out, TEXT_MANUFACTURER), // ManufacturerName()
        0x1E => append_block(out, TEXT_DEVICE_NAME),  // DeviceName()
        _ => {
            if out.is_empty() {
                0
            } else {
                out[0] = 0x0000;
                1
            }
        }
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
