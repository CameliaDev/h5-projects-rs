//! SHT30 temperature/humidity sensor on the shared I2C bus.
//!
//! The bus is set up in [`crate::i2c`]; [`init`] takes a handle from it.

// TODO(sht30): remove once the module is wired into `main.rs`.
#![allow(dead_code)]

use defmt::{error, info};
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_stm32::i2c::Error as I2cError;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use sht3x_ner::{ClockStretch, Error as ShtError, Repeatability, Sht3x};

use crate::i2c::{self, Device};

/// Re-exported so callers do not need to import `sht3x_ner` directly.
pub use sht3x_ner::Address;

/// Error reported by the shared bus, on top of the driver's own errors.
type BusError = I2cDeviceError<I2cError>;

/// Latest temperature (°C) and relative humidity (%RH) with timestamp.
#[derive(Clone, Copy, defmt::Format)]
pub struct Sht30Data {
    pub temperature: f32,
    pub humidity: f32,
    pub at: Instant,
}

/// Driver plus the last sample.
struct Sht30Sensor {
    drv: Sht3x<Device>,
    latest: Option<Sht30Data>,
}

/// Populated by [`init`], waited on by the other functions.
static SHT30: OnceLock<Mutex<ThreadModeRawMutex, Sht30Sensor>> = OnceLock::new();

/// Error from the sensor driver or the shared I2C bus.
#[derive(Debug)]
pub enum Sht30Error {
    Sensor(ShtError<BusError>),
}

impl From<ShtError<BusError>> for Sht30Error {
    fn from(value: ShtError<BusError>) -> Self {
        Self::Sensor(value)
    }
}

impl defmt::Format for Sht30Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Sensor(ShtError::Crc) => defmt::write!(f, "SHT30 CRC mismatch"),
            Self::Sensor(ShtError::I2c(e)) => defmt::write!(f, "SHT30 I2C error: {}", e),
        }
    }
}

/// Initialize the sensor on the shared I2C bus.
///
/// [`crate::i2c::init`] must have run first. Soft-resets the sensor (2 ms,
/// datasheet Table 4) and publishes it; panics if called twice.
pub async fn init(address: Address) -> Result<(), Sht30Error> {
    let mut drv = Sht3x::new(i2c::device(), address);
    drv.reset(&mut Delay).await?;

    SHT30
        .init(Mutex::new(Sht30Sensor { drv, latest: None }))
        .ok()
        .expect("SHT30 already initialized");

    Ok(())
}

/// Latest cached sample; waits for [`init`] and returns `None` until the first
/// measurement completes.
pub async fn data() -> Option<Sht30Data> {
    SHT30.get().await.lock().await.latest
}

/// Single-shot high-repeatability measurement (~15 ms), cached for [`data`].
pub async fn measure() -> Result<Sht30Data, Sht30Error> {
    let mut guard = SHT30.get().await.lock().await;
    let mut delay = Delay;

    let measurement = guard
        .drv
        .measure(ClockStretch::Disabled, Repeatability::High, &mut delay)
        .await?;

    // sht3x-ner reports temperature and humidity in hundredths
    let reading = Sht30Data {
        temperature: measurement.temperature as f32 / 100.0,
        humidity: measurement.humidity as f32 / 100.0,
        at: Instant::now(),
    };

    guard.latest = Some(reading);
    Ok(reading)
}

/// Turn the heater on or off.
///
/// Intended for short pulses only (~1 s, datasheet §4.10); long use biases the
/// readings. Prefer [`heater_pulse`].
pub async fn toggle_heater(enable: bool) -> Result<(), Sht30Error> {
    let mut guard = SHT30.get().await.lock().await;
    let mut delay = Delay;
    guard.drv.set_heater(enable, &mut delay).await?;
    Ok(())
}

/// Run the typical 1 s heater pulse; holds the sensor lock for its duration.
pub async fn heater_pulse() -> Result<(), Sht30Error> {
    let mut guard = SHT30.get().await.lock().await;
    let mut delay = Delay;

    guard.drv.set_heater(true, &mut delay).await?;
    Timer::after_millis(1000).await;
    guard.drv.set_heater(false, &mut delay).await?;

    Ok(())
}

/// Read and log a sample once a second.
#[embassy_executor::task]
pub async fn sht30_task() -> ! {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        match measure().await {
            Ok(reading) => {
                info!(
                    "SHT30: {} °C, {} %RH",
                    reading.temperature, reading.humidity
                );
            }
            Err(err) => error!("SHT30 read failed: {}", err),
        }
    }
}
