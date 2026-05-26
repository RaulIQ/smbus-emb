//! SMBus bridge on STM32F103 (I2C v1).
//!
//! - **I2C1** (PB6 SCL, PB7 SDA): SMBus master, periodically reads a sensor.
//! - **I2C2** (PB10 SCL, PB11 SDA): SMBus slave at `0x42`, returns the last
//!   sensor sample to an external master.
//!
//! # Wiring
//!
//! ```text
//! I2C1 (sensor bus)          I2C2 (export bus)
//!   PB6 ── SCL ── sensor       PB10 ── SCL ── external master
//!   PB7 ── SDA                 PB11 ── SDA
//!   4.7k pull-ups on each bus
//! ```
//!
//! Default sensor address `0x48` (e.g. LM75). Register `0x00`, 2 data bytes.
//! Change `SENSOR_ADDR` / pins if your board differs.

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{
    self, Config as I2cConfig, I2c, SlaveAddrConfig, SlaveCommandKind, Smbus, SmbusConfig,
};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use panic_probe as _;

/// 7-bit address of the temperature sensor on I2C1.
const SENSOR_ADDR: u8 = 0x0B;
/// 7-bit slave address on I2C2 (visible to external master).
const EXPORT_SLAVE_ADDR: u8 = 0x0B;
/// Number of payload bytes forwarded to the external master.
const PAYLOAD_LEN: usize = 2;

static SENSOR_DATA: Mutex<ThreadModeRawMutex, [u8; PAYLOAD_LEN]> = Mutex::new([0x00, 0x00]);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!(
        "SMBus bridge: I2C1=master (sensor), I2C2=slave (export 0x{:02X})",
        EXPORT_SLAVE_ADDR
    );

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000);

    let smbus_cfg = SmbusConfig::default();

    // --- I2C2: slave (export bus) ---
    let i2c_slave = I2c::new_blocking(
        p.I2C2, p.PB10, // SCL
        p.PB11, // SDA
        i2c_cfg,
    )
    .into_slave_multimaster(SlaveAddrConfig::basic(EXPORT_SLAVE_ADDR));
    let smbus_slave = Smbus::new(i2c_slave, smbus_cfg);

    spawner.spawn(export_slave_task(smbus_slave).unwrap());

    // --- I2C1: master (sensor bus) ---
    let smbus_master = Smbus::new(
        I2c::new_blocking(
            p.I2C1, p.PB6, // SCL
            p.PB7, // SDA
            i2c_cfg,
        ),
        smbus_cfg,
    );

    spawner.spawn(sensor_master_task(smbus_master).unwrap());

    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(blinker(led).unwrap());
}

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>) {
    info!("Run blinker!");
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn sensor_master_task(
    mut master: Smbus<'static, embassy_stm32::mode::Blocking, i2c::mode::Master>,
) {
    let reg = [0u8];
    let mut sample = [0u8; PAYLOAD_LEN];

    loop {
        match master.blocking_write_read(SENSOR_ADDR, &reg, &mut sample) {
            Ok(()) => {
                *SENSOR_DATA.lock().await = sample;
                info!("sensor: {:02X}{:02X}", sample[0], sample[1]);
            }
            Err(e) => {
                warn!("sensor read failed: {}", i2c_err(e));
            }
        }

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn export_slave_task(
    mut slave: Smbus<'static, embassy_stm32::mode::Blocking, i2c::mode::MultiMaster>,
) {
    loop {
        match slave.blocking_listen() {
            Ok(cmd) => match cmd.kind {
                SlaveCommandKind::Read => {
                    let payload = *SENSOR_DATA.lock().await;
                    match slave.blocking_respond_to_read(&payload) {
                        Ok(n) => info!(
                            "export read: sent {} bytes {:02X}{:02X}",
                            n, payload[0], payload[1]
                        ),
                        Err(e) => error!("export read failed: {}", i2c_err(e)),
                    }
                }
                SlaveCommandKind::Write => {
                    let mut discard = [0u8; 16];
                    match slave.blocking_respond_to_write(&mut discard) {
                        Ok(n) => info!("export write: ignored {} bytes", n),
                        Err(e) => error!("export write failed: {}", i2c_err(e)),
                    }
                }
            },
            Err(e) => {
                error!("export listen failed: {}", i2c_err(e));
                Timer::after(Duration::from_millis(100)).await;
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
