//! SHT30 temperature and humidity.
//!
//! One `Sht3x` is created in `sht30_init` and kept in the static mutex. Later
//! calls reuse that driver. The latest sample is copied into the same mutex
//! (`*SHT30.lock().await = Some(sensor)`), matching the state-machine mutex.
//!
//! Board I2C pins are not fixed yet, so `main` does not call this.

// Nothing spawns the task until the I2C pins are known.
#![allow(dead_code)]

use defmt::{Debug2Format, error, info};
use embassy_stm32::Peri;
use embassy_stm32::i2c::{
    self, Config, ErrorInterruptHandler, EventInterruptHandler, I2c, Instance, SclPin, SdaPin,
};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use sht3x_ner::{Address, ClockStretch, Commands, Error as ShtError, Repeatability, Sht3x};

/// Latest temperature (°C) and relative humidity (%RH).
#[derive(Clone, Copy, Debug)]
pub struct Sht30Data {
    pub temperature: f32,
    pub humidity: f32,
}

impl Sht30Data {
    const INIT: Self = Self {
        temperature: 0.0,
        humidity: 0.0,
    };
}

type I2cBus = I2c<'static, Async, i2c::Master>;

/// Driver plus the last sample copied out for other tasks.
struct Sht30Sensor {
    drv: Sht3x<I2cBus>,
    data: Sht30Data,
}

/// Shared sensor. `None` until `sht30_init` publishes one.
static SHT30: Mutex<ThreadModeRawMutex, Option<Sht30Sensor>> = Mutex::new(None);

/// Peripherals the SHT30 is wired to.
///
/// The I2C instance and pins come from the caller, same way `CanPins` and
/// `EfusePins` hand `Peri`s into the module that constructs the driver.
pub struct Sht30Pins<T, SCL, SDA>
where
    T: Instance,
    SCL: SclPin<T>,
    SDA: SdaPin<T>,
{
    pub i2c: Peri<'static, T>,
    pub scl: Peri<'static, SCL>,
    pub sda: Peri<'static, SDA>,
}

#[derive(Debug)]
pub enum Sht30Error {
    /// `sht30_init` has not stored a sensor yet.
    NotInitialized,
    Sensor(ShtError<i2c::Error>),
}

impl From<ShtError<i2c::Error>> for Sht30Error {
    fn from(value: ShtError<i2c::Error>) -> Self {
        Self::Sensor(value)
    }
}

/// Soft-reset the sensor and store it in the shared mutex.
///
/// Uses interrupt-driven I2C without DMA. A measurement is only a few bytes.
pub async fn sht30_init<T, SCL, SDA, IRQ>(
    pins: Sht30Pins<T, SCL, SDA>,
    irq: IRQ,
    address: Address,
) -> Result<(), Sht30Error>
where
    T: Instance,
    SCL: SclPin<T>,
    SDA: SdaPin<T>,
    IRQ: Binding<T::EventInterrupt, EventInterruptHandler<T>>
        + Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
        + 'static,
{
    let mut sensor = Sht30Sensor {
        drv: Sht3x::new(
            I2c::new_no_dma(pins.i2c, pins.scl, pins.sda, irq, Config::default()),
            address,
        ),
        data: Sht30Data::INIT,
    };

    let mut delay = Delay;
    sensor.drv.reset(&mut delay).await?;
    *SHT30.lock().await = Some(sensor);
    Ok(())
}

/// Turn the heater on or off through `Sht3x::send_command`.
pub async fn sht30_toggle_heater(enable: bool) -> Result<(), Sht30Error> {
    let mut guard = SHT30.lock().await;
    let sensor = guard.as_mut().ok_or(Sht30Error::NotInitialized)?;
    let mut delay = Delay;
    let command = if enable {
        Commands::HeaterEnable
    } else {
        Commands::HeaterDisable
    };
    sensor.drv.send_command(command, &mut delay).await?;
    Ok(())
}

/// Single-shot high-repeatability measurement, copied back into the mutex.
pub async fn sht30_temperature_and_humidity() -> Result<(f32, f32), Sht30Error> {
    let mut guard = SHT30.lock().await;
    let sensor = guard.as_mut().ok_or(Sht30Error::NotInitialized)?;
    let mut delay = Delay;
    let measurement = sensor
        .drv
        .measure(ClockStretch::Disabled, Repeatability::High, &mut delay)
        .await?;

    // sht3x-ner reports temperature and humidity in hundredths.
    let data = Sht30Data {
        temperature: measurement.temperature as f32 / 100.0_f32,
        humidity: measurement.humidity as f32 / 100.0_f32,
    };
    sensor.data = data;
    Ok((data.temperature, data.humidity))
}

/// Copy of the latest sample. Does not start a measurement.
pub async fn sht30_reading() -> Option<Sht30Data> {
    SHT30.lock().await.as_ref().map(|sensor| sensor.data)
}

/// Log a sample once a second. Spawn after `sht30_init`.
#[embassy_executor::task]
pub async fn sht30_task() {
    loop {
        match sht30_temperature_and_humidity().await {
            Ok((temperature, humidity)) => {
                info!("SHT30: {} C, {} %RH", temperature, humidity);
            }
            Err(err) => error!("SHT30 read failed: {:?}", Debug2Format(&err)),
        }
        Timer::after_millis(1000).await;
    }
}
