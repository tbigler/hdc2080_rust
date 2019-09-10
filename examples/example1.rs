//extern crate hdc2080_rust;

use hdc2080_rust::*;
use linux_embedded_hal::I2cdev;

fn main() {
    // Initialize I2C
    let i2c_address = I2CAddress::AddrPinToGND;
    let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap();
    // Initialize the hdc2080 sensor communication
    let mut hdc2080 = Hdc2080::new(i2c_dev, i2c_address);
    // Create default setting
    let mut settings = DeviceSettings::new();

    // In the default mode a measurement needs to be triggered explicitly
    hdc2080.trigger_measurement(&settings).unwrap();

    let manufacturer_id = hdc2080.read_manufacturer_id().unwrap();
    println!("Manufacturer id: 0x{:x}", manufacturer_id);

    let device_id = hdc2080.read_device_id().unwrap();
    println!("Device id: 0x{:x}", device_id);

    let temperature = hdc2080.read_temperature().unwrap();
    println!("Temperature: {:.*} °C", 2, temperature);

    let humidity = hdc2080.read_humidity().unwrap();
    println!("Humidity: {:.*} %", 1, humidity);
}
