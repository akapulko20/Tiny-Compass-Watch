# Tiny-Compass-Watch
*An electronic watch that makes hard to get lost*

The Tiny Compass Watch is an extended version of the [Mega Tiny Time Watch](https://github.com/technoblogy/mega-tiny-time-watch/) project created by [David Johnson-Davies](https://github.com/technoblogy). In order to obtain a compass functionality, just three additional components have been used: MPU-9250 (TDK InvenSense), 0.1µF capacitor and a push button. A minimal modifications of the original circuit have been made: push button connected to the PA1 line, MPU-9250 chip connected to the ATtiny814 via Two Wire Interface (INT output of the MPU-9250 connected to PA7 pin just for a possible future improvements).

![Circuit of the Tiny Compass Watch](figures/circuit.png)
<p align="center">Figure 1. Circuit diagram of the Tiny Compass Watch

  The [MPU-9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) is a System in Package (SiP) that combines two chips: the MPU-6500, which contains a 3-axis gyroscope, a 3-axis accelerometer, and the AK8963 3-axis digital compass. A 3-axis accelerometer have been used for tilt compensation (some theory can be found [here](https://ieeexplore.ieee.org/abstract/document/838300)). The AK8963 has resolution of 16-bits (0.15 µT per LSB) which is quite enough to detect Earth's magnetic field (50-60 µT). Gyroscope remain always not active. The total power consumption in sleep mode is ~9µA, an estimated battery life from a single CR2016 battery is more than a year (from experience: more than 6 months of intensive usage from a single CR2032 battery).
  
  **Code details**
  
  A single line of code should be changed according to your location (visit [noaa.gov](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination) website to calculate the declination angle):
  ```C++
                       #define Declination     8        // For Kyiv, Ukraine ~ +8 deg (2022 year)
```
  The [TinyMegaI2C Library](https://github.com/technoblogy/tiny-mega-i2c) by David Johnson-Davies has been chosen for communication with MPU-9250, however *TinyMegaI2C.read()* function replaced, as was discussed [here](https://github.com/technoblogy/tiny-mega-i2c/issues/3). The corresponding function proposed by [buckket](https://gist.github.com/buckket/09619e6cdc5dee056d41bfb57065db81) has been used. Also, clock rate has been decreased down to 20kHz for a proper work with internal pull-up resistors.
