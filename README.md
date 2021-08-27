# PiicoDev® Pressure Sensor MS5637 MicroPython Module

This is the firmware repo for the [Core Electronics PiicoDev® Pressure Sensor MS5637](https://core-electronics.com.au/catalog/product/view/sku/CE07832)

This module depends on the [PiicoDev Unified Library](https://github.com/CoreElectronics/CE-PiicoDev-Unified), include `PiicoDev_Unified.py` in the project directory on your MicroPython device.

See the Quickstart Guides:
- [Micro:bit v2](https://core-electronics.com.au/tutorials/micro-bit/piicodev-pressure-sensor-ms5637-micro-bit-guide.html)
- [Raspberry Pi Pico](https://core-electronics.com.au/tutorials/piicodev-pressure-sensor-ms5637-raspberry-pi-pico-guide.html)
- [Raspberry Pi](https://core-electronics.com.au/tutorials/piicodev-pressure-sensor-ms5637-raspberry-pi-guide.html)

# Usage
## Simple Example
[main.py](https://github.com/CoreElectronics/CE-PiicoDev-MS5637-MicroPython-Module/blob/main/main.py) is a simple example to get started.
```
# PiicoDev MS5637 minimal example code
# This program temperature and pressure data from the PiicoDev MS5637 pressure sensor
# and displays the result

from PiicoDev_MS5637 import PiicoDev_MS5637
from PiicoDev_Unified import sleep_ms

pressure = PiicoDev_MS5637()

while True:
    press_hPa = pressure.read_pressure()
    altitude_m = pressure.read_altitude()

    # Print Pressure
    print(str(press_hPa) + " hPa")

    # Print Altitude (metres)
    print(str(altitude_m) + " m")
    sleep_ms(100)
```
## Details
### PiicoDev_MS5637(bus=, freq=, sda=, scl=, addr=0x76)

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
bus | int | 0,1 | Raspberry Pi Pico: 0, Raspberry Pi: 1 | I2C Bus.  Ignored on Micro:bit
freq | int | 100-1000000 | Device dependent | I2C Bus frequency (Hz).  Ignored on Raspberry Pi
sda | Pin | Device Dependent | Device dependent | I2C SDA Pin. Implemented on Raspberry Pi Pico only
scl | Pin | Device Dependent | Device dependent | I2C SCL Pin. Implemented on Raspberry Pi Pico only
addr | int | 0x76 | 0x76 | The address cannot be changed

### PiicoDev_MS5637.read_pressure(res=5)
Parameter | Type | Range | Default | Description | Unit
--- | --- | --- | --- | --- | ---
res | int | 0-5 | 5 | Sets the resolution of the ADC (0=256, 1=512, 2=1024, 3=2048, 4=4096, 5 = 8192) | #
returned | float | | | Pressure reading|hPa

### PiicoDev_MS5637.read_altitude(pressure_sea_level=1013.25)
Parameter | Type | Range | Default | Description | Unit
--- | --- | --- | --- | --- | ---
pressure_sea_level | float | any | 1013.25 | Enter the current sea level pressure.  This value is available from your favourite weather service. | hPa
returned | float | | | Altitude | m

# License
This project is open source - please review the LICENSE.md file for further licensing information.

If you have any technical questions, or concerns about licensing, please contact technical support on the [Core Electronics forums](https://forum.core-electronics.com.au/).

*\"PiicoDev\" and the PiicoDev logo are trademarks of Core Electronics Pty Ltd.*
