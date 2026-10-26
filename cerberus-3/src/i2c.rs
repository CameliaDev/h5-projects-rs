//! Shared I2C1 bus.

// TODO: remove once wired into `main.rs`.
#![allow(dead_code)]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::i2c::{self, Config, ErrorInterruptHandler, EventInterruptHandler, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;

bind_interrupts!(struct Irqs {
    I2C1_EV => EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => ErrorInterruptHandler<peripherals::I2C1>;
});

/// Peripherals the I2C1 bus is wired to.
pub struct I2cPins {
    pub i2c: Peri<'static, peripherals::I2C1>,
    pub scl: Peri<'static, peripherals::PB8>,
    pub sda: Peri<'static, peripherals::PB9>,
}

/// The I2C1 bus type.
pub type I2cBus = i2c::I2c<'static, Async, i2c::Master>;
/// Handle a device driver uses to talk on the shared bus.
pub type Device = I2cDevice<'static, ThreadModeRawMutex, I2cBus>;

/// One lock for every device on the bus.
static I2C: OnceLock<Mutex<ThreadModeRawMutex, I2cBus>> = OnceLock::new();

/// Build the bus. Call once, before any device.
pub fn init(pins: I2cPins) {
    let bus = I2c::new_no_dma(pins.i2c, pins.scl, pins.sda, Irqs, Config::default());
    I2C.init(Mutex::new(bus))
        .ok()
        .expect("I2C bus already initialized");
}

/// Handle for one device on the shared bus.
pub fn device() -> Device {
    I2cDevice::new(I2C.try_get().expect("I2C bus used before i2c::init()"))
}
