# PiicoDev MS5637 minimal example code
# This program temperature and pressure data from the PiicoDev MS5637 pressure sensor
# and displays the result

from PiicoDev_MS5637 import PiicoDev_MS5637

from utime import sleep_ms

pressure = PiicoDev_MS5637()

while True:
    barometric_pressure = pressure.read_temperature_and_pressure(pressure._RESOLUTION_OSR_8192)
    print("{:2.2f} {:4.2f}".format(barometric_pressure[0],barometric_pressure[1]))
    sleep_ms(100)
    
    

       