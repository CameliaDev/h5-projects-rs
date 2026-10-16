//! cerberus-3 SHT30 peripherals — issue #48
//!
//! Uses `sht3x-ner` for single-shot measure + CRC.
//! Heater on/off uses datasheet 16-bit commands (crate has no public heater API).

// Module is not wired into main.rs yet; remove this once Sht30_init is called from main.
#![allow(dead_code)]

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use sht3x_ner::{Address, ClockStretch, Error, Repeatability, Sht3x};
use static_cell::StaticCell;

/// Temperature (°C), humidity (%RH), and I2C address.
pub struct Sht30Data {
    pub temperature: f32,
    pub humidity: f32,
    pub address: Address,
}

/// Owns the I2C bus + cached reading. Wrapped in a shared Mutex for multi-task use.
pub struct Sht30Sensor<I2C> {
    i2c: I2C,
    data: Sht30Data,
}

const HEATER_ENABLE: u16 = 0x306D;
const HEATER_DISABLE: u16 = 0x3066;
const COMMAND_WAIT_MS: u64 = 1;

// The I2C instance (I2C1..I2C4) is chosen by the caller in main.rs; this module is not tied to a specific instance.
type I2cBus =
    embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>;
type Shared = Mutex<CriticalSectionRawMutex, Sht30Sensor<I2cBus>>;
type Sht3xError = Error<embassy_stm32::i2c::Error>;

static SHT30: StaticCell<Shared> = StaticCell::new();

/// Sht30_init — initialize over I2C at `address`, store behind shared Mutex.
/// Returns a `'static` handle for the caller to pass into the `tempsensor_*` functions.
#[allow(non_snake_case)] // issue #48 specifies this name
pub async fn Sht30_init(i2c: I2cBus, address: Address) -> &'static Shared {
    let mut sensor = Sht30Sensor {
        i2c,
        data: Sht30Data {
            temperature: 0.0,
            humidity: 0.0,
            address,
        },
    };
    // Soft reset via driver
    {
        let mut delay = Delay;
        let mut drv = Sht3x::new(&mut sensor.i2c, address);
        if drv.reset(&mut delay).await.is_err() {
            defmt::error!("SHT30 reset failed");
        }
    }
    SHT30.init(Mutex::new(sensor))
}

/// tempsensor_toggleHeater(enable)
#[allow(non_snake_case)] // issue #48 specifies this name
pub async fn tempsensor_toggleHeater(
    shared: &'static Shared,
    enable: bool,
) -> Result<(), Sht3xError> {
    let mut g = shared.lock().await;
    let addr = g.data.address as u8;
    let cmd = if enable {
        HEATER_ENABLE
    } else {
        HEATER_DISABLE
    };
    g.i2c
        .write(addr, &cmd.to_be_bytes())
        .await
        .map_err(Error::I2c)?;
    Timer::after_millis(COMMAND_WAIT_MS).await;
    Ok(())
}

/// tempsensor_getTemperatureAndHumidity() -> (f32, f32)
#[allow(non_snake_case)] // issue #48 specifies this name
pub async fn tempsensor_getTemperatureAndHumidity(
    shared: &'static Shared,
) -> Result<(f32, f32), Sht3xError> {
    let mut g = shared.lock().await;
    let address = g.data.address;
    let mut delay = Delay;
    // Temporarily wrap bus for the driver call
    let mut drv = Sht3x::new(&mut g.i2c, address);
    let m = drv
        .measure(ClockStretch::Disabled, Repeatability::High, &mut delay)
        .await?;
    // sht3x-ner: temp in 0.01°C, humidity in 0.01%RH
    let t = m.temperature as f32 / 100.0;
    let h = m.humidity as f32 / 100.0;
    g.data.temperature = t;
    g.data.humidity = h;
    Ok((t, h))
}

/// Example task — spawn from main.rs:
/// `let sht30 = peripherals::Sht30_init(i2c, addr).await;`
/// `spawner.spawn(peripherals::tempsensor_task(sht30))`
#[embassy_executor::task]
pub async fn tempsensor_task(shared: &'static Shared) {
    loop {
        match tempsensor_getTemperatureAndHumidity(shared).await {
            Ok((t, h)) => defmt::info!("SHT30: {} C, {} %RH", t, h),
            Err(e) => defmt::error!("SHT30 read failed: {:?}", defmt::Debug2Format(&e)),
        }
        Timer::after_millis(1000).await;
    }
}
