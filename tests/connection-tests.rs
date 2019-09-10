use hdc2080_rust::*;
use linux_embedded_hal::I2cdev;

const I2C_ADDRESS: I2CAddress = I2CAddress::AddrPinToGND; // Address 0x40

#[test]
fn read_manufacturer_id() {
    let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap();
    // Initialize the hdc2080 sensor communication
    let mut hdc2080 = Hdc2080::new(i2c_dev, I2C_ADDRESS);

    // Create a default setting
    let settings = DeviceSettings::new();

    // In the default mode a measurement needs to be triggered explicitly
    hdc2080.trigger_measurement(&settings).unwrap();

    //hdc2080.soft_reset().unwrap();
    let manufacturer_id = hdc2080.read_manufacturer_id().unwrap();
    assert_eq!(manufacturer_id, 0x5449);
}