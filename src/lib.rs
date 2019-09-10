//! This crate is a Rust implementation for the HDC2080 temperature and humidity sensor from Texas instruments (http://www.ti.com/product/HDC2080).
//!
//! Communication with this sensor is implemented over the I2C bus.
//! It used the embedded-hal crate to provide abstraction to the hardware I2C implementation/API.
//! A demo implementation for the Raspberry PI can be found in the examples.
//!
//! ```rust,no_run
//! use hdc2080_rust::*;
//! use linux_embedded_hal::I2cdev;
//!
//! fn main() {
//!     // Initialize I2C
//!     let i2c_address = I2CAddress::AddrPinToVDD;
//!     let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap();
//!     // Initialize the hdc2080 sensor communication
//!     let mut hdc2080 = Hdc2080::new(i2c_dev, i2c_address);
//!     // Create a default setting
//!     let mut settings = DeviceSettings::new();
//!
//!     // In the default mode a measurement needs to be triggered explicitly
//!     hdc2080.trigger_measurement(&settings).unwrap();
//!
//!     let manufacturer_id = hdc2080.read_manufacturer_id().unwrap();
//!     println!("Manufacturer id: 0x{:x}", manufacturer_id);
//!
//!     let temperature = hdc2080.read_temperature().unwrap();
//!     println!("Temperature: {:.*} °C", 2, temperature);
//!
//!     let humidity = hdc2080.read_humidity().unwrap();
//!     println!("Humidity: {:.*} %", 1, humidity);
//! }
//! ```

#![no_std]
#![allow(dead_code)]

//extern crate embedded_hal;
use embedded_hal::blocking::i2c::{Write, WriteRead};
//use embedded_hal::prelude::*;

mod registers;
use registers::*;

/// HDC2080 measurement resolution settings.
/// Used for temperature and humidity configuration.
pub enum MeasurementResolution {
    Resolution14Bit,
    Resolution11Bit,
    Resolution9Bit,
}

/// HDC2080 measurement mode settings.
///
/// Defines the trigger interval for the automatic measurements.
///
/// Default: Disabled
pub enum AutoMeasurementMode {
    Disabled,
    IntervalEvery2Minutes,
    IntervalEveryMinute,
    IntervalEvery10Seconds,
    IntervalEvery5Seconds,
    Interval1PerSecond,
    Interval2PerSecond,
    Interval5PerSecond,
}

/// HDC2080 interrupt pin configuration.
/// 
pub enum InterruptPinConfig {
    HighZ,
    Enable,
}

pub enum InterruptPolarity {
    ActiveLow,
    ActiveHigh,
}

pub enum InterruptMode {
    LevelSensitive,
    ComparatorMode,
}

#[derive(Debug)]
pub enum Error<R, W> {
    //TODO: change to I2C_Error<E> or I2C_WriteError(W), I2C_ReadError(R)
    //I2cError(E),
    I2CWrite(W),
    I2CRead(R),
    DeviceNotFound,
    InvalidRange,
}

pub enum I2CAddress {
    AddrPinToGND,
    AddrPinToVDD,
}

impl I2CAddress {
    pub fn addr(&self) -> u8 {
        match &self {
            I2CAddress::AddrPinToGND => HDC2080_I2C_ADDR_GND,
            I2CAddress::AddrPinToVDD => HDC2080_I2C_ADDR_VDD,
        }
    }
}

pub struct DeviceSettings {
    // Measurement Configuration Register 0x0F
    temperature_resolution: MeasurementResolution,
    humidity_resolution: MeasurementResolution,
    temperature_enabled: bool,
    humidity_enabled: bool,
    trigger_measurement: bool,

    // Interrupt Configuration Register 0x07
    drdy_int_enable: bool,
    th_enable: bool,
    tl_enable: bool,
    hh_enable: bool,
    hl_enable: bool,

    // Reset and DRDY/INT Configuration Register 0x0E
    auto_measurement_mode: AutoMeasurementMode,
    heater_on: bool,
    dataready_interrupt_pin_config: InterruptPinConfig,
    interrupt_polarity: InterruptPolarity,
    interrupt_mode: InterruptMode,

    // Temperature Max Register 0x05
    //temperature_max: f32,

    // Humidity Max Register 0x06
    //humidity_max: f32,

    // Temperature Offset Adjustment Register 0x08

    // Humidity Offset Adjustment Register 0x09

    // Temperature Threshold Low Register 0x0A
    temperature_threshold_low: f32,

    // Temperature Threshold High Register 0x0B
    temperature_threshold_high: f32,

    // Humidity Threshold Low Register 0x0C
    humidity_threshold_low: f32,

    // Humidity Threshold High Register 0x0D
    humidity_threshold_high: f32,

    // Register values for configuration are stored here after calling
    // create_register_values function
    pub interrupt_config_register_value: u8,
    pub reset_drdy_config_register_value: u8,
    pub measurement_config_register_value: u8,
    pub temperature_threshold_low_register_value: u8,
    pub temperature_threshold_high_register_value: u8,
    pub humidity_threshold_low_register_value: u8,
    pub humidity_threshold_high_register_value: u8,
    pub temperature_offset_adjust_register_value: u8,
    pub humidity_offset_adjust_register_value: u8,
}

impl DeviceSettings {
    /// Constructs a new 'DeviceSettings' builder instance
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use hdc2080_rust::*;
    /// use linux_embedded_hal::I2cdev;
    /// 
    /// let settings = DeviceSettings::new();
    /// ```
    pub fn new() -> Self {
        Self {
            // Measurement Configuration Register 0x0F
            temperature_resolution: MeasurementResolution::Resolution14Bit,
            humidity_resolution: MeasurementResolution::Resolution14Bit,
            temperature_enabled: true,
            humidity_enabled: true,
            trigger_measurement: false,

            // Interrupt Configuration Register 0x07
            drdy_int_enable: false,
            th_enable: false,
            tl_enable: false,
            hh_enable: false,
            hl_enable: false,

            // Reset and DRDY/INT Configuration Register 0x0E
            heater_on: false,
            auto_measurement_mode: AutoMeasurementMode::Disabled,
            dataready_interrupt_pin_config: InterruptPinConfig::HighZ,
            interrupt_polarity: InterruptPolarity::ActiveLow,
            interrupt_mode: InterruptMode::LevelSensitive,

            // Temperature Max Register 0x05
            //temperature_max: -40.0, // default register value after reset: 0x00

            // Humidity Max Register 0x06
            //humidity_max: 0.0, // default register value after reset: 0x00

            // Temperature Threshold Low Register 0x0A
            temperature_threshold_low: -40.0, // default register value after reset: 0x00

            /// Temperature Threshold High Register 0x0B
            temperature_threshold_high: -40.0, // default register value after reset: 0x00

            /// Humidity Threshold Low Register 0x0C
            humidity_threshold_low: 0.0, // default register value after reset: 0x00

            /// Humidity Threshold High Register 0x0D
            humidity_threshold_high: 0.0, // default register value after reset: 0x00

            interrupt_config_register_value: 0x00,
            reset_drdy_config_register_value: 0x00,
            measurement_config_register_value: 0x00,
            temperature_threshold_low_register_value: 0x00,
            temperature_threshold_high_register_value: 0xFF,
            humidity_threshold_low_register_value: 0x00,
            humidity_threshold_high_register_value: 0xFF,
            temperature_offset_adjust_register_value: 0x00,
            humidity_offset_adjust_register_value: 0x00,
        }
    }

    /// Sets the resolution for temperature measurement.
    /// Higher resolution leads to longer conversion time.
    /// # Conversion times
    /// * 9 bit:  225 us
    /// * 11 bit: 350 us
    /// * 14 bit: 610 us (default)
    pub fn set_temperature_resolution(
        mut self,
        resolution: MeasurementResolution,
    ) -> DeviceSettings {
        self.temperature_resolution = resolution;
        self
    }

    /// Sets the resolution for humidity measurement.
    /// Higher resolution leads to longer conversion time.
    /// # Conversion times
    /// * 9 bit:  275 us
    /// * 11 bit: 400 us
    /// * 14 bit: 660 us (default)
    pub fn set_humidity_resolution(&mut self, resolution: MeasurementResolution) -> &mut Self {
        self.humidity_resolution = resolution;
        self
    }

    /// Defines that combined temperature and humidity measurement will be triggered.
    pub fn set_measure_humidity_and_temperature(&mut self) -> &mut Self {
        self.temperature_enabled = true;
        self.humidity_enabled = true;
        self
    }

    /// Defines that temperature measurement only will be triggered.
    pub fn set_measurement_temperature_only(&mut self) -> &mut Self {
        self.temperature_enabled = true;
        self.humidity_enabled = false;
        self
    }

    /// Enables the DRDY/INT interrupt output (Pin 4)
    pub fn set_dataready_interrupt_enabled(&mut self, enabled: bool) -> &mut Self {
        self.drdy_int_enable = enabled;
        self
    }

    /// Enables the temperature threshold high interrupt
    pub fn set_temperature_high_interrupt_enabled(&mut self, enabled: bool) -> &mut Self {
        self.th_enable = enabled;
        self
    }

    /// Enables the temperature threshold low interrupt
    pub fn set_temperature_low_interrupt_enabled(&mut self, enabled: bool) -> &mut Self {
        self.tl_enable = enabled;
        self
    }

    /// Enables the humidity threshold high interrupt
    pub fn set_humidity_high_interrupt_enabled(&mut self, enabled: bool) -> &mut Self {
        self.hh_enable = enabled;
        self
    }

    /// Enables the humidity threshold low interrupt
    pub fn set_humidity_low_interrupt_enabled(&mut self, enabled: bool) -> &mut Self {
        self.hl_enable = enabled;
        self
    }

    /// Sets the auto measurement mode
    pub fn set_auto_measurement_mode(&mut self, mode: AutoMeasurementMode) -> &mut Self {
        self.auto_measurement_mode = mode;
        self
    }

    /// Enables or disabled the heater for humidity measurement
    pub fn set_heater_enabled(&mut self, state: bool) -> &mut Self {
        self.heater_on = state;
        self
    }

    /// Sets the DRDY/INT_EN pin configuration.
    /// # config
    /// * High Z
    /// * Enable
    pub fn set_dataready_interrupt_mode(&mut self, config: InterruptPinConfig) -> &mut Self {
        self.dataready_interrupt_pin_config = config;
        self
    }

    /// Sets the interrupt polarity.
    /// # polarity
    /// * Active Low
    /// * Active High
    pub fn set_interrupt_polarity(&mut self, polarity: InterruptPolarity) -> &mut Self {
        self.interrupt_polarity = polarity;
        self
    }

    /// Sets interrupt mode.
    /// # mode
    /// * Level sensitive
    /// * Comparator mode
    pub fn set_interrupt_mode(&mut self, mode: InterruptMode) -> &mut Self {
        self.interrupt_mode = mode;
        self
    }

    /// Sets temperature threshold low.
    /// # Arguments
    /// * `value` - temperature in °C
    pub fn set_temperature_threshold_low(&mut self, value: f32) -> &mut Self {
        //TODO: check range of value
        self.temperature_threshold_low = value;
        self
    }

    // Sets temperature threshold high.
    /// # Arguments
    /// * `value` - temperature in °C
    pub fn set_temperature_threshold_high(&mut self, value: f32) -> &mut Self {
        //TODO: check range of value
        self.temperature_threshold_high = value;
        self
    }

    /// Sets humidity threshold low.
    /// # Arguments
    /// * `value` - humidity in %
    pub fn set_humidity_threshold_low(&mut self, value: f32) -> &mut Self {
        //TODO: check range of value
        self.humidity_threshold_low = value;
        self
    }

    /// Sets humidity threshold high.
    /// # Arguments
    /// * `value` - humidity in %
    pub fn set_humidity_threshold_high(&mut self, value: f32) -> &mut Self {
        //TODO: check range of value
        self.humidity_threshold_high = value;
        self
    }

    /// Sets temperature offset value.
    /// See datasheet for more information. Will be extended/improved in future version of this implementation.
    pub fn set_temperature_offset(&mut self, value: u8) -> &mut Self {
        self.temperature_offset_adjust_register_value = value;
        self
    }

    /// Sets humidity offset value.
    /// See datasheet for more information. Will be extended/improved in future version of this implementation.
    pub fn set_humidity_offset(&mut self, value: u8) -> &mut Self {
        self.humidity_offset_adjust_register_value = value;
        self
    }

    //TODO: implement functions to set temperature and humidity offset correction

    /// Creates register values for the HDC2080 based on the settings.
    pub fn create_register_values(&mut self) -> &mut Self {
        self.interrupt_config_register_value = 0x00;
        self.reset_drdy_config_register_value = 0x00;
        self.measurement_config_register_value = 0x00;

        // self.reset_drdy_config_register_value
        self.reset_drdy_config_register_value |= match self.auto_measurement_mode {
            AutoMeasurementMode::Disabled => 0x00,
            AutoMeasurementMode::IntervalEvery2Minutes => HDC2080_FIELD_INTCONF_AMM_1_120,
            AutoMeasurementMode::IntervalEveryMinute => HDC2080_FIELD_INTCONF_AMM_1_60,
            AutoMeasurementMode::IntervalEvery10Seconds => HDC2080_FIELD_INTCONF_AMM_1_10,
            AutoMeasurementMode::IntervalEvery5Seconds => HDC2080_FIELD_INTCONF_AMM_1_5,
            AutoMeasurementMode::Interval1PerSecond => HDC2080_FIELD_INTCONF_AMM_1,
            AutoMeasurementMode::Interval2PerSecond => HDC2080_FIELD_INTCONF_AMM_2,
            AutoMeasurementMode::Interval5PerSecond => HDC2080_FIELD_INTCONF_AMM_5,
        };

        self.reset_drdy_config_register_value |= match self.heater_on {
            true => HDC2080_FIELD_INTCONF_HEATER_ON,
            false => HDC2080_FIELD_INTCONF_HEATER_OFF,
        };

        self.reset_drdy_config_register_value |= match self.dataready_interrupt_pin_config {
            InterruptPinConfig::Enable => HDC2080_FIELD_INTCONF_DRDYPIN_EN,
            InterruptPinConfig::HighZ => HDC2080_FIELD_INTCONF_DRDYPIN_HIGHZ,
        };

        self.reset_drdy_config_register_value |= match self.interrupt_polarity {
            InterruptPolarity::ActiveHigh => HDC2080_FIELD_INTCONF_INTPOL_HIGH,
            InterruptPolarity::ActiveLow => HDC2080_FIELD_INTCONF_INTPOL_LOW,
        };

        self.reset_drdy_config_register_value |= match self.interrupt_mode {
            InterruptMode::ComparatorMode => HDC2080_FIELD_INTCONF_INTMODE_COMP,
            InterruptMode::LevelSensitive => HDC2080_FIELD_INTCONF_INTMODE_LEVEL,
        };

        // self.measurement_config_register_value
        self.measurement_config_register_value |= match self.temperature_resolution {
            MeasurementResolution::Resolution14Bit => HDC2080_FIELD_MEASCONF_TRES_14BIT,
            MeasurementResolution::Resolution11Bit => HDC2080_FIELD_MEASCONF_TRES_11BIT,
            MeasurementResolution::Resolution9Bit => HDC2080_FIELD_MEASCONF_TRES_9BIT,
        };

        self.measurement_config_register_value |= match self.humidity_resolution {
            MeasurementResolution::Resolution14Bit => HDC2080_FIELD_MEASCONF_HRES_14BIT,
            MeasurementResolution::Resolution11Bit => HDC2080_FIELD_MEASCONF_HRES_11BIT,
            MeasurementResolution::Resolution9Bit => HDC2080_FIELD_MEASCONF_HRES_9BIT,
        };

        self.measurement_config_register_value |=
            match (self.humidity_enabled, self.temperature_enabled) {
                (true, true) => HDC2080_FIELD_MEASCONF_HUM_AND_TEMP,
                (false, true) => HDC2080_FIELD_MEASCONF_TEMP_ONLY,
                _ => panic!("invalid configuration"),
            };

        // self.interrupt_config_register_value
        self.interrupt_config_register_value |= match self.drdy_int_enable {
            true => HDC2080_MASK_INTEN_DRDY_ENABLE,
            false => 0,
        };

        self.interrupt_config_register_value |= match self.th_enable {
            true => HDC2080_MASK_INTEN_TH_ENABLE,
            false => 0,
        };

        self.interrupt_config_register_value |= match self.tl_enable {
            true => HDC2080_MASK_INTEN_TL_ENABLE,
            false => 0,
        };

        self.interrupt_config_register_value |= match self.hh_enable {
            true => HDC2080_MASK_INTEN_HH_ENABLE,
            false => 0,
        };

        self.interrupt_config_register_value |= match self.hl_enable {
            true => HDC2080_MASK_INTEN_HL_ENABLE,
            false => 0,
        };

        self
    }

    pub fn get_interrupt_config_register_value(&self) -> u8 {
        self.interrupt_config_register_value
    }

    pub fn get_reset_drdy_config_register_value(&self) -> u8 {
        self.reset_drdy_config_register_value
    }

    pub fn get_measurement_config_register_value(&self) -> u8 {
        self.measurement_config_register_value
    }

    pub fn get_temperature_offset_adjust_register_value(&self) -> u8 {
        self.temperature_offset_adjust_register_value
    }

    pub fn get_humidity_offset_adjust_register_value(&self) -> u8 {
        self.humidity_offset_adjust_register_value
    }

    pub fn get_temperature_high_threshold_register_value(&self) -> u8 {
        self.temperature_threshold_high_register_value
    }

    pub fn get_temperature_low_threshold_register_value(&self) -> u8 {
        self.temperature_threshold_low_register_value
    }

    pub fn get_humidity_high_threshold_register_value(&self) -> u8 {
        self.humidity_threshold_high_register_value
    }

    pub fn get_humidity_low_threshold_register_value(&self) -> u8 {
        self.humidity_threshold_low_register_value
    }
}

struct I2CHelper<I2C> {
    i2c_dev: I2C,
    i2c_addr: I2CAddress,
}

impl<I2C> I2CHelper<I2C>
where
    I2C: WriteRead + Write,
{
    fn new(i2c_dev: I2C, i2c_addr: I2CAddress) -> Self {
        I2CHelper { i2c_dev, i2c_addr }
    }

    fn read_reg8(&mut self, reg_addr: u8) -> Result<u8, <I2C as WriteRead>::Error>
    where
        I2C: WriteRead,
    {
        let mut buf: [u8; 1] = [0; 1];
        self.i2c_dev
            .write_read(self.i2c_addr.addr(), &mut [reg_addr], &mut buf)?;
        Ok(buf[0])
    }

    fn read_reg16(&mut self, reg_addr: u8) -> Result<u16, <I2C as WriteRead>::Error>
    where
        I2C: WriteRead,
    {
        let mut buf: [u8; 2] = [0; 2];
        self.i2c_dev
            .write_read(self.i2c_addr.addr(), &mut [reg_addr], &mut buf)?;
        //TODO: use system function for byte order
        let r: u16 = ((buf[1] as u16) << 8) | buf[0] as u16;
        Ok(r)
    }

    fn write_reg8(&mut self, reg_addr: u8, value: u8) -> Result<(), <I2C as Write>::Error>
    where
        I2C: Write,
    {
        let mut bytes: [u8; 2] = [0; 2];
        bytes[0] = reg_addr;
        self.i2c_dev
            .write(self.i2c_addr.addr(), &[reg_addr, value])?;
        Ok(())
    }
}

pub struct Hdc2080<I2C> {
    i2c_helper: I2CHelper<I2C>,
    measurement_settings_register: DeviceSettings,
}

impl<I2C> Hdc2080<I2C>
where
    I2C: WriteRead + Write,
{
    pub fn new(i2c_dev: I2C, i2c_addr: I2CAddress) -> Self {
        Hdc2080 {
            i2c_helper: I2CHelper::new(i2c_dev, i2c_addr),
            measurement_settings_register: DeviceSettings::new(),
        }
    }

    /// Read manufacturer id registers (16 bit).
    pub fn read_manufacturer_id(&mut self) -> Result<u16, <I2C as WriteRead>::Error> {
        self.i2c_helper
            .read_reg16(HDC2080_REGADDR_MANUFACTURER_ID_LOW)
    }

    pub fn read_device_id(&mut self) -> Result<u16, <I2C as WriteRead>::Error> {
        self.i2c_helper
            .read_reg16(HDC2080_REGADDR_DEVICE_ID_LOW)
    }

    //TODO: Extract function to convert register value to °C/%

    /// Read temperature register and convert to float °C.
    pub fn read_temperature(&mut self) -> Result<f32, <I2C as WriteRead>::Error> {
        match self.i2c_helper.read_reg16(HDC2080_REGADDR_TEMPERATURE_LOW) {
            Ok(v) => {
                let temp_c = (((v as u32) * 165) as f32) / 65536.0 - 40.0;
                Ok(temp_c)
            }
            Err(e) => Err(e),
        }
    }

    /// Read temperature max register and convert to float °C.
    ///
    /// This register implements temperature peak detector function.
    /// It stores the highest temperature value converted after the power up.
    /// Value is reset at power up and/or with soft reset procedure.
    /// **Not supported in Auto Measurement Mode**
    pub fn read_temperature_max(&mut self) -> Result<f32, <I2C as WriteRead>::Error> {
        match self.i2c_helper.read_reg16(HDC2080_REGADDR_TEMPERATURE_MAX) {
            Ok(v) => {
                let temp_c = (((v as u32) * 165) as f32) / 65536.0 - 40.0;
                Ok(temp_c)
            }
            Err(e) => Err(e),
        }
    }

    /// Read humidity register and convert to float %.
    pub fn read_humidity(&mut self) -> Result<f32, <I2C as WriteRead>::Error> {
        match self.i2c_helper.read_reg16(HDC2080_REGADDR_HUMIDITY_LOW) {
            Ok(v) => {
                let humidity = (((v as u32) * 100) as f32) / 65536.0;
                Ok(humidity)
            }
            Err(e) => Err(e),
        }
    }

    /// Read humidity max register and convert to float %.
    ///
    /// This register implements temperature peak detector function.
    /// It stores the highest temperature value converted after the power up.
    /// Value is reset at power up and/or with soft reset procedure.
    /// **Not supported in Auto Measurement Mode**
    pub fn read_humidity_max(&mut self) -> Result<f32, <I2C as WriteRead>::Error> {
        match self.i2c_helper.read_reg16(HDC2080_REGADDR_HUMIDITY_MAX) {
            Ok(v) => {
                let temp_c = (((v as u32) * 100) as f32) / 65536.0;
                Ok(temp_c)
            }
            Err(e) => Err(e),
        }
    }

    pub fn write_config(&mut self, config: DeviceSettings) -> Result<(), <I2C as Write>::Error> {
        self.i2c_helper.write_reg8(
            HDC2080_REGADDR_MEASUREMENT_CONFIG,
            config.get_measurement_config_register_value(),
        )?;
        Ok(())
    }

    /// Triggers a soft reset of the HDC2080.
    pub fn soft_reset(&mut self) -> Result<(), <I2C as Write>::Error> {
        self.i2c_helper
            .write_reg8(HDC2080_REGADDR_RESET_INT_CONF, 0x80)?;
        Ok(())
    }

    /// Triggers a conversion when auto measurement mode is set to disabled.
    pub fn trigger_measurement(
        &mut self,
        config: &DeviceSettings,
    ) -> Result<(), <I2C as Write>::Error> {
        self.i2c_helper.write_reg8(
            HDC2080_REGADDR_MEASUREMENT_CONFIG,
            config.get_measurement_config_register_value() | HDC2080_FIELD_MEASCONF_MEAS_TRIGGER,
        )?;
        Ok(())
    }

    /// Applies the configuration to the HDC2080 sensor.
    pub fn apply_configuration(
        &mut self,
        config: &mut DeviceSettings,
    ) -> Result<(), Error<<I2C as WriteRead>::Error, <I2C as Write>::Error>> {
        //TODO: use .map_err
        //TODO: change result type to WriteRead only

        config.create_register_values();

        // Address 0x07 Interrupt Configuration Register      
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_INTERRUPT_ENABLE,
            config.get_interrupt_config_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x08 Temperature Offset Adjustment Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_TEMP_OFFSET_ADJUST,
            config.get_temperature_offset_adjust_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x09 Humidity Offset Adjustment Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_HUM_OFFSET_ADJUST,
            config.get_humidity_offset_adjust_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x0A Temperature Threshold LOW Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_TEMP_THR_LOW,
            config.get_temperature_low_threshold_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x0B Temperature Threshold HIGH Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_TEMP_THR_HIGH,
            config.get_temperature_high_threshold_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x0C Humidity Threshold LOW Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_RH_THR_LOW,
            config.get_humidity_low_threshold_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x0D Humidity Threshold HIGH Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_RH_THR_HIGH,
            config.get_humidity_high_threshold_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x0E Configuration Register
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_RESET_INT_CONF,
            config.get_reset_drdy_config_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        // Address 0x0F Measurement Configuration Field Descriptions
        match self.i2c_helper.write_reg8(
            HDC2080_REGADDR_MEASUREMENT_CONFIG,
            config.get_measurement_config_register_value(),
        ) {
            Ok(_) => (),
            Err(e) => return Err(Error::I2CWrite(e)),
        }

        Ok(())
    }

    pub fn read_interrupt_drdy_register(&mut self) -> Result<u8, <I2C as WriteRead>::Error> {
        self.i2c_helper.read_reg8(HDC2080_REGADDR_INTERRUPT_DRDY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use linux_embedded_hal::I2cdev;

    #[test]
    fn config_builder_interrupt_config() {
        let interrupt_config_register_value = DeviceSettings::new()
            .set_dataready_interrupt_enabled(true)
            .create_register_values()
            .get_interrupt_config_register_value();
        assert_eq!(interrupt_config_register_value, 0b1000_0000);

        let interrupt_config_register_value = DeviceSettings::new()
            .set_dataready_interrupt_enabled(true)
            .set_temperature_low_interrupt_enabled(true)
            .create_register_values()
            .get_interrupt_config_register_value();
        assert_eq!(interrupt_config_register_value, 0b1010_0000);
    }

    #[test]
    fn config_builder_reset_drdy_config() {
        let reset_drdy_register = DeviceSettings::new().get_reset_drdy_config_register_value();
        assert_eq!(reset_drdy_register, 0x00);

        let reset_drdy_register = DeviceSettings::new()
            .set_heater_enabled(true)
            .create_register_values()
            .get_reset_drdy_config_register_value();
        assert_eq!(reset_drdy_register, 0b0000_1000);

        let reset_drdy_register = DeviceSettings::new()
            .set_auto_measurement_mode(AutoMeasurementMode::Interval1PerSecond)
            .create_register_values()
            .get_reset_drdy_config_register_value();
        assert_eq!(reset_drdy_register, 0b0101_0000);

        let reset_drdy_register = DeviceSettings::new()
            .set_auto_measurement_mode(AutoMeasurementMode::Disabled)
            .set_heater_enabled(false)
            .set_interrupt_mode(InterruptMode::ComparatorMode)
            .create_register_values()
            .get_reset_drdy_config_register_value();
        assert_eq!(reset_drdy_register, 0b0000_0001);
    }

    #[test]
    fn builder_ownership() {
        let settings1_regval = DeviceSettings::new()
            .set_heater_enabled(true)
            .set_humidity_resolution(MeasurementResolution::Resolution9Bit)
            .set_interrupt_polarity(InterruptPolarity::ActiveHigh)
            .set_auto_measurement_mode(AutoMeasurementMode::IntervalEveryMinute)
            .create_register_values()
            .get_measurement_config_register_value();

        let mut settings2 = DeviceSettings::new();
        settings2.set_heater_enabled(true);
        settings2.set_humidity_resolution(MeasurementResolution::Resolution9Bit);
        settings2.set_interrupt_polarity(InterruptPolarity::ActiveHigh);
        settings2.set_auto_measurement_mode(AutoMeasurementMode::IntervalEveryMinute);
        settings2.create_register_values();
        let settings2_regval = settings2.get_measurement_config_register_value();

        assert_eq!(settings1_regval, settings2_regval);
    }

    #[test]
    fn test_settings_write() {

        // Initialize I2C
        let i2c_address = I2CAddress::AddrPinToGND;
        let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap();
        // Initialize the hdc2080 sensor communication
        let mut hdc2080 = Hdc2080::new(i2c_dev, i2c_address);

        let manufacturer_id = hdc2080.read_manufacturer_id().unwrap();
        println!("Manufacturer id: 0x{:x}", manufacturer_id);
        assert_eq!(manufacturer_id, 0x5449);

        // Create default setting
        let mut settings = DeviceSettings::new()
            .set_temperature_resolution(MeasurementResolution::Resolution9Bit);
        hdc2080.apply_configuration(&mut settings);

        let reg_value = hdc2080.i2c_helper.read_reg8(HDC2080_REGADDR_MEASUREMENT_CONFIG).unwrap();
        println!("register value read: {:x}", reg_value);
        assert_eq!(reg_value, HDC2080_FIELD_MEASCONF_TRES_9BIT);
    }
}
