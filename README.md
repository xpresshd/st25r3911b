# Rust ST25R3911B driver
This is a no_std driver for the ST25R3911B

Currently support only reading UID from PICCs

Has custom `SpiWithCustomCS` trait to give you control over ChipSelect and ability to implement SPI Lock
