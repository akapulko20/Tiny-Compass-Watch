# Tiny-Compass-Watch
An electronic watch that makes hard to get lost

The Tiny Compass Watch is an extended version of the [Mega Tiny Time Watch](https://github.com/technoblogy/mega-tiny-time-watch/) project created by [David Johnson-Davies](https://github.com/technoblogy). In order to obtain a compass functionality, just three additional components have been used: MPU-9250 (TDK InvenSense), 0.1uF capacitor and a push button. A minimal modifications of the original circuit have been made: push button connected to the PA1 line, MPU-9250 chip connected to the ATtiny814 via Two Wire Interface (INT output of the MPU-9250 connected to PA7 pin just for a possible future improvements).

![Circuit of the Tiny Compass Watch](figures/circuit.png)
<p align="center">Figure 1. Circuit diagram of the Tiny Compass Watch

  Communication with MPU-9250 performed using [TinyMegaI2C Library](https://github.com/technoblogy/tiny-mega-i2c) by David Johnson-Davies, however *TinyMegaI2C.read()* function has been replaced, as was discussed [here](https://github.com/technoblogy/tiny-mega-i2c/issues/3). The corresponding function proposed by [buckket](https://gist.github.com/buckket/09619e6cdc5dee056d41bfb57065db81) has been used.
