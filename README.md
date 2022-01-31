# Tiny-Compass-Watch
*An electronic watch that makes hard to get lost*

![Front view of the Tiny Compass Watch](figures/front_side.png)

The Tiny Compass Watch is an extended version of the [Mega Tiny Time Watch](https://github.com/technoblogy/mega-tiny-time-watch/) project created by [David Johnson-Davies](https://github.com/technoblogy). In order to obtain a compass functionality, just three additional components have been used: MPU-9250 (TDK InvenSense), 0.1µF capacitor and a push button. A minimal modifications of the original circuit have been made: push button connected to the PA1 line, MPU-9250 chip connected to the ATtiny814 via Two Wire Interface (INT output of the MPU-9250 connected to PA7 pin just for a possible future improvements).

![Circuit of the Tiny Compass Watch](figures/circuit.png)
<p align="center">Circuit diagram of the Tiny Compass Watch

  The [MPU-9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) is a System in Package (SiP) that combines two chips: the MPU-6500 (which contains a 3-axis gyroscope and a 3-axis accelerometer), and the AK8963 (3-axis digital compass). A 3-axis accelerometer have been used for tilt compensation (some theory can be found [here](https://ieeexplore.ieee.org/abstract/document/838300)). The AK8963 has resolution of 16-bits (0.15 µT per LSB) which is quite enough to detect Earth's magnetic field (50-60 µT). Gyroscope remain always not active. The total power consumption in sleep mode is ~9µA, an estimated battery life from a single CR2016 battery is more than a year (from experience: more than 6 months of intensive usage from a single CR2032 battery).
  
  **Code Details**
  
  A single line of code should be changed according to your location (visit [noaa.gov](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination) website to calculate the declination angle):
  ```C++
  #define Declination     8                       // For Kyiv, Ukraine ~ +8 deg (2022 year)
```
  The [TinyMegaI2C Library](https://github.com/technoblogy/tiny-mega-i2c) by David Johnson-Davies has been chosen for communication with MPU-9250. However, *TinyMegaI2C.read()* function replaced, as was discussed [here](https://github.com/technoblogy/tiny-mega-i2c/issues/3). The corresponding function proposed by [buckket](https://gist.github.com/buckket/09619e6cdc5dee056d41bfb57065db81) has been used. Also, I2C clock frequency decreased down to 20kHz for a proper work with an internal pull-up resistors.
  
  The program interact with minimally sufficient set of the MPU-9250 registers. In order to separately operate the AK8963 and MPU-6500 devices bypass mode should be selected. Magnetometer factory calibrations of the axial sensitivity (ASAX,ASAY,ASAZ) have been read and applied for both the measurements
and calibration.
  ```C++
  void SetMPU9250() {
    ...
    I2CSetRegister(ACEL_GIRO, INT_PIN_CFG, 0x02); // Set bypass enable bit
    ...
    I2CSetRegister(MAG, AK8963_CNTL1, 0x0F);      // Fuse ROM access mode
    TinyMegaI2C.start(MAG, 0);
    TinyMegaI2C.write(AK8963_ASAX);
    TinyMegaI2C.restart(MAG, -1);
    uint8_t asax = TinyMegaI2C.read(0x01);        // Read x-axis sensitivity adjustment value
    uint8_t asay = TinyMegaI2C.read(0x01);        // Read y-axis sensitivity adjustment value
    uint8_t asaz = TinyMegaI2C.read(0x00);        // Read z-axis sensitivity adjustment value
    TinyMegaI2C.stop();
    ASAX = (asax / 256) + 0.5F;
    ASAY = (asay / 256) + 0.5F;
    ASAZ = (asaz / 256) + 0.5F;
    ...
  }
```
  When not in power-down mode, the AK8963 operates in Continuous measurement mode 2 - a compromise between performance and power consumption:
   ```C++
  void WakeAK8963() {
    I2CSetRegister(MAG, AK8963_CNTL1, 0x16);      // 16-bit output, Continuous measurement mode 2
  }
```
  
  Two buttons on a board means more flexible device control. Press and hold the *Show North* button, then press the *Show Time* button to launch the compass calibration procedure (each LED blink one-by-one clockwise starting from 12 - the *DisplayCircle()* function indicate start/done of the compass calibration procedure). During the calibration process, slowly rotate watch so that each side (front, back, left, right, top and bottom) points down towards the earth for a few seconds in turn. A concise algorithm proposed by [kriswiner](https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration) has been used in order to determine magnetometer calibration parameters:
  ```C++
  void MagCalibration(uint16_t points) {
    ...
    /* Hard-iron offsets */
    OFFX = (MAXX + MINX) >> 1;                           
    OFFY = (MAXY + MINY) >> 1;
    OFFZ = (MAXZ + MINZ) >> 1;
    /* Soft-iron scale factors */
    SCAX = 0.33F * (1 + (MAXY + MAXZ - MINY - MINZ) / (MAXX - MINX));
    SCAY = 0.33F * (1 + (MAXX + MAXZ - MINX - MINZ) / (MAXY - MINY));
    SCAZ = 0.33F * (1 + (MAXX + MAXY - MINX - MINY) / (MAXZ - MINZ));
    ...
  }
```
  May not be the best approach, but probably the simplest one. It was found that an accelerometer calibration is not mandatory (checked for five MPU-9250 chips), but the same algorithm could be used if needed. It should be noted that the electromagnetic field of a CR20XX battery (which is placed very close to the MPU-9250) depends on it's charge (time-dependent value), temperature and even spatial orientation, so whenever you need a precise direction or the device has not been used for a long time - just launch the calibration procedure.
  
  Press the *Show Time* button, then press the *Show North* button to launch the time correction procedure (LEDs from 6 to 12 blink one-by-one anti-clockwise - the *DisplaySemicircle()* function indicate start/done of the time correction procedure) - simply subtract 30 seconds from current time.
  
  The earth's horizontal magnetic field components have been calculated without explicit sine or cosine functions:
  ```C
  AVEC = sqrt((AX * AX) + (AY * AY) + (AZ * AZ));
  HXh = AVEC * HX * sqrt((AX * AX) + (AZ * AZ)) - HY * AX * AY + HZ * AY * sqrt((AY * AY) + (AZ * AZ));
  HYh = AVEC * (HY * sqrt((AY * AY) + (AZ * AZ)) + HZ * AX);
  Heading = atan2(HYh, HXh) * RAD_TO_DEG;         // Magnetic North
```

  **Displaying North Direction**
  
  You can either press and release (for single short flash of one or two LEDs) or press and hold (for continuous short flashes of one or two LEDs) the *Show North* button whenever you want to find out North direction (short video is available on [YouTube](https://www.youtube.com/watch?v=BqxkJCPgP0E)). Original 12 LEDs display has been conditionally divided into 24 sectors (15 degree each one). Two possible cases are presented in the figure below: North direction lies in the *pink* (short flashes of one LED) or in the *grey* (short flashes of two LEDs) sector.
  
![Circuit of the Tiny Compass Watch](figures/north_show_example.png)
<p align="center">Examples of how the Tiny Compass Watch displays North direction
  
  **Uploading Program**
  
  All instructions can be found on [Mega Tiny Time Watch](http://www.technoblogy.com/show?2OKF) page. The only difference is the code size. ATtiny814 and ATtiny1614 are suitable for the [Tiny Compass Watch](https://github.com/akapulko20/Tiny-Compass-Watch/blob/main/CompassWatch.ino) program. [Gerber files](https://github.com/akapulko20/Tiny-Compass-Watch/raw/main/Gerber_Tiny_Compass_Watch%20.zip) for PCB fabrication attached.
