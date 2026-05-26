//! SMBus master: read smart-battery data from ESP32-C3 slave (`smbus-esp32c3`).
//!
//! # Wiring (STM32F103 Blue Pill → ESP32-C3)
//!
//! ```text
//!   PB6 ── SCL ── ESP GPIO21
//!   PB7 ── SDA ── ESP GPIO20
//!   GND ── GND
//!   4.7k pull-ups on SCL/SDA (ESP internal pull-ups alone are often too weak)
//! ```
//!
//! ESP slave: `SMBUS_BATTERY_ADDR` = **0x0B**.
//!
//! Protocol matches ESP firmware: master **writes** command byte, then **reads**
//! response (separate transactions with STOP between — same as ArduPilot
//! `read_word`, and what ESP-IDF slave expects via `on_receive` + `on_request`).

#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Smbus, SmbusConfig, SmbusRole};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use panic_probe as _;

/// 7-bit smart-battery address (`smbus_battery.h`).
const BATTERY_ADDR: u8 = 0x0B;

// `smbus_battery.c` command codes
const CMD_TEMP: u8 = 0x08;
const CMD_VOLTAGE: u8 = 0x09;
const CMD_CURRENT: u8 = 0x0A;
const CMD_SOC: u8 = 0x0D;
const CMD_REMAINING: u8 = 0x0F;
const CMD_FULL: u8 = 0x10;
const CMD_SERIAL: u8 = 0x1C;

/// Pause between register write and read so ESP slave task can arm TX.
const T_WRITE_READ_US: u64 = 500;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!(
        "SMBus master → ESP battery @ 0x{:02X}, I2C1 PB6/PB7",
        BATTERY_ADDR
    );

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000);
    i2c_cfg.timeout = Duration::from_millis(100);

    let i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, i2c_cfg);
    let mut smbus_cfg = SmbusConfig::default();
    smbus_cfg.role = SmbusRole::Host;
    let smbus = Smbus::new(i2c, smbus_cfg);

    spawner.spawn(battery_reader_task(smbus).unwrap());

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
async fn battery_reader_task(
    mut master: Smbus<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::mode::Master>,
) {
    // Let ESP finish boot and release the bus.
    Timer::after(Duration::from_millis(200)).await;

    loop {
        match read_battery(&mut master).await {
            Ok(b) => {
                info!(
                    "battery: {}mV {}mA {}.{:02}C soc={}% {}/{}mAh sn=0x{:04X}",
                    b.voltage_mv,
                    b.current_ma,
                    b.temp_c / 10,
                    (b.temp_c.unsigned_abs() % 10) as u8,
                    b.soc_percent,
                    b.remaining_mah,
                    b.full_mah,
                    b.serial,
                );
            }
            Err(e) => warn!("battery read failed: {}", i2c_err(e)),
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}

struct BatterySample {
    voltage_mv: u16,
    current_ma: i16,
    temp_c: i16,
    soc_percent: u8,
    remaining_mah: u16,
    full_mah: u16,
    serial: u16,
}

async fn read_battery(
    master: &mut Smbus<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::mode::Master>,
) -> Result<BatterySample, embassy_stm32::i2c::Error> {
    let voltage_mv = read_word(master, CMD_VOLTAGE).await?;
    let current_raw = read_word(master, CMD_CURRENT).await?;
    let temp_raw = read_word(master, CMD_TEMP).await?;
    let soc_percent = read_word(master, CMD_SOC).await? as u8;
    let remaining_mah = read_word(master, CMD_REMAINING).await?;
    let full_mah = read_word(master, CMD_FULL).await?;
    let serial = read_word(master, CMD_SERIAL).await?;

    let current_ma = current_raw as i16;
    let temp_c = (temp_raw as i32 - 2731) as i16;

    Ok(BatterySample {
        voltage_mv,
        current_ma,
        temp_c,
        soc_percent,
        remaining_mah,
        full_mah,
        serial,
    })
}

async fn read_word(
    master: &mut Smbus<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::mode::Master>,
    reg: u8,
) -> Result<u16, embassy_stm32::i2c::Error> {
    let cmd = [reg];
    let mut buf = [0u8; 2];

    // ESP slave: on_receive(cmd) on write, on_request → TX on read.
    // Use write + STOP, then read so ESP slave gets RX then TX phases.
    master.blocking_write(BATTERY_ADDR, &cmd)?;
    Timer::after(Duration::from_micros(T_WRITE_READ_US)).await;
    master.blocking_read(BATTERY_ADDR, &mut buf)?;

    Ok(u16::from_le_bytes(buf))
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
