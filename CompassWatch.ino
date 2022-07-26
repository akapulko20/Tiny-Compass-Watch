/* 
   Tiny Compass Watch
   An extended version of the original project:
       Mega Tiny Time Watch v2
       David Johnson-Davies - www.technoblogy.com - 8th November 2020
   Alex Slobodian - https://github.com/akapulko20/Tiny-Compass-Watch - January 2022
   ATtiny814 @ 5 MHz (internal oscillator; BOD disabled)
 
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "TinyMegaI2CMaster.h"

#define MIN(a,b)            (((a)<(b))?(a):(b))
#define MAX(a,b)            (((a)>(b))?(a):(b))

#define ACEL_GIRO           0x68                        // MPU-6500 address
#define MAG                 0x0C                        // AK8963 address

/* AK8963 register map */
#define AK8963_ST1          0x02
#define AK8963_XOUT_L       0x03
#define AK8963_CNTL1        0x0A
#define AK8963_ASTC         0x0C
#define AK8963_ASAX         0x10
/* MPU-6500 register map */
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define INT_PIN_CFG         0x37
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C

#define CAL_COUNT       3000                            // Calibration points
#define Declination     12                              // For Kyiv, Ukraine ~ +8 deg (2022 year) - https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
#define DISOFF          7                               // Compass display offset, deg

int16_t HX, HY, HZ, Heading;                            // Magnetic field data variables
float ASAX, ASAY, ASAZ;                                 // Magnetic sensitivity adjustment data
float SCAX, SCAY, SCAZ;                                 // Magnetometer scale values
int16_t OFFX, OFFY, OFFZ;                               // Magnetometer offset values
int32_t AX, AY, AZ, AVEC;                               // Accelerometer data variables
int32_t BUFX, BUFY, BUFZ;                               // Buffer variables for average calculations
float HXh, HYh;                                         // Horizontal magnetic field components

const int Tickspersec = 250;                            // Units of MyDelay()

volatile int Timeout;
volatile uint8_t Cycle = 0;

volatile unsigned int Secs = 0;
volatile int Hours = 0;                                 // From 0 to 11, or 12 = off.
volatile int Fivemins = 0;                              // From 0 to 11, or 12 = off.

volatile int ShowTime = false;
volatile int ShowNorth = false;
volatile int StartCalibration = false;
volatile int TimeCorrection = false;

void I2CSetRegister(uint8_t add, uint8_t reg, uint8_t bits) {
  TinyMegaI2C.start(add, 0);
  TinyMegaI2C.write(reg);
  TinyMegaI2C.write(bits);
  TinyMegaI2C.stop();
}

void SleepAccel() {
  I2CSetRegister(ACEL_GIRO, PWR_MGMT_2, 0x3F);          // Disable accelerometer and giro
  I2CSetRegister(ACEL_GIRO, PWR_MGMT_1, 0x48);          // Set sleep mode, Power down internal PTAT voltage generator and PTAT ADC
}

void WakeAccel() {
  I2CSetRegister(ACEL_GIRO, PWR_MGMT_1, 0x08);          // Internal 20MHz oscillator clock source, Power down internal PTAT voltage generator and PTAT ADC
  I2CSetRegister(ACEL_GIRO, PWR_MGMT_2, 0x07);          // Enable accelerometer, keep giro off
  _delay_ms(10);                                        // !!!Wait 10ms for oscillations to stabilize
}

void SleepAK8963() {
  I2CSetRegister(MAG, AK8963_CNTL1, 0x00);              // Set ak8963 power-down mode
}

void WakeAK8963() {
  I2CSetRegister(MAG, AK8963_CNTL1, 0x16);              // Set 16-bit output, Continuous measurement mode 2 (100Hz rate)
}

void SetMPU9250() {
  NorthButtonEnable();
  I2CSetRegister(ACEL_GIRO, PWR_MGMT_1, 0x80);          // Reset chip
  I2CSetRegister(ACEL_GIRO, ACCEL_CONFIG, 0x00);        // Bytes 3 and 4 (00) for +-2G
  I2CSetRegister(ACEL_GIRO, ACCEL_CONFIG2, 0x02);       // DLPF config: rate 1 kHz, delay 2.88 ms
  I2CSetRegister(ACEL_GIRO, INT_PIN_CFG, 0x02);         // Set bypass enable bit
  WakeAccel(); AccelRead(); SleepAccel();
  I2CSetRegister(MAG, AK8963_CNTL1, 0x0F);              // Fuse ROM access mode
  TinyMegaI2C.start(MAG, 0);
  TinyMegaI2C.write(AK8963_ASAX);
  TinyMegaI2C.restart(MAG, -1);
  uint8_t asax = TinyMegaI2C.read(0x01);                // Read x-axis sensitivity adjustment value
  uint8_t asay = TinyMegaI2C.read(0x01);                // Read y-axis sensitivity adjustment value
  uint8_t asaz = TinyMegaI2C.read(0x00);                // Read z-axis sensitivity adjustment value
  TinyMegaI2C.stop();
  ASAX = (float) (asax / 256.) + .5;
  ASAY = (float) (asay / 256.) + .5;
  ASAZ = (float) (asaz / 256.) + .5;
  WakeAK8963(); MagRead(); SleepAK8963();
}

uint8_t MagRaw() {
   uint8_t DRDY_OFL = 0x00;                             // Variable temporary keeps DRDY (STATUS1 REG) or OFL (STATUS2 REG) bits
   do {
     TinyMegaI2C.start(MAG, 0);
     TinyMegaI2C.write(AK8963_ST1);
     TinyMegaI2C.restart(MAG, -1);
     DRDY_OFL = TinyMegaI2C.read(0x00);                 // Read STATUS1 register. If DRDY == 1, auto increments address by 1 (0x03)
     TinyMegaI2C.stop();
   } while (!(DRDY_OFL & 0x01));                        // Mask for DRDY bit of STATUS1 register
   TinyMegaI2C.start(MAG, 0);
   TinyMegaI2C.write(AK8963_XOUT_L);
   TinyMegaI2C.restart(MAG, -1);
   HX = TinyMegaI2C.read(0x01);                         // Read HXL register & auto increments address by 1 (0x04)
   HX |= (TinyMegaI2C.read(0x01) << 8);                 // Read HXH register & auto increments address by 1 (0x05)
   HY = TinyMegaI2C.read(0x01);                         // Read HYL ... (0x06)
   HY |= (TinyMegaI2C.read(0x01) << 8);                 // Read HYH ... (0x07)
   HZ = TinyMegaI2C.read(0x01);                         // Read HZL ... (0x08)
   HZ |= (TinyMegaI2C.read(0x01) << 8);                 // Read HZH register & auto increments address 0x09 - STATUS2 register
   DRDY_OFL = TinyMegaI2C.read(0x00);                   // Read STATUS2 register (required!)
   TinyMegaI2C.stop();
   if (DRDY_OFL & 0x08) {                               // Mask for HOFL bit of STATUS2 register, case data corrupted
     HX = 0; HY = 0; HZ = 0;                            // Mamagnetic sensor overlow occurs - data are not correct
     return(0);
   }
   else {
     return(1);
   }
}

void MagCalibration(uint16_t points) {
   int16_t MINX, MAXX, MINY, MAXY, MINZ, MAXZ;
   MINX = MINY = MINZ = 32767;
   MAXX = MAXY = MAXZ = -32768;
   DisplayCircle();
   WakeAK8963();
   for (uint16_t i=0; i<points; i++) {
     uint8_t data = MagRaw();
     if (data) {
       HX *= ASAX; HY *= ASAY; HZ *= ASAZ;              // Adjusted measurement data: Hadj = H * ( (ASA-128) / 256 + 1)
       MINX = MIN(HX, MINX); MAXX = MAX(HX, MAXX);      // Max and Min Hx, Hy, Hz values
       MINY = MIN(HY, MINY); MAXY = MAX(HY, MAXY);
       MINZ = MIN(HZ, MINZ); MAXZ = MAX(HZ, MAXZ);
     }
   }
                                                        // Calculate Hard-iron offsets
   OFFX = (MAXX + MINX) >> 1;                           
   OFFY = (MAXY + MINY) >> 1;
   OFFZ = (MAXZ + MINZ) >> 1;
                                                        // Calculate Soft-iron scale factors
   SCAX = (float) (1 + (MAXY + MAXZ - MINY - MINZ) / (MAXX - MINX)) * .33;
   SCAY = (float) (1 + (MAXX + MAXZ - MINX - MINZ) / (MAXY - MINY)) * .33;
   SCAZ = (float) (1 + (MAXX + MAXY - MINX - MINY) / (MAXZ - MINZ)) * .33;
   SleepAK8963();
   DisplayCircle();
}

// Signal for magnetometer calibration start/stop

void DisplayCircle() {
  Fivemins = 12;
  for (uint8_t i=0; i<12; i++) {
    Hours = i;
    DisplayOn(); MyDelay(Tickspersec >> 3); DisplayOff();
  }
}

// Signal for time correction

void DisplaySemicircle() {
  Fivemins = 12;
  for (uint8_t i=6; i>0; i--) {
    Hours = i;
    DisplayOn(); MyDelay(Tickspersec >> 3); DisplayOff();
  }
}

void MagRead() {                                        // Take avarage of 16 samples
  BUFX = BUFY = BUFZ = 0;
  for (uint8_t i=0; i<16; i++) {
     uint8_t data = MagRaw();
     if (data) {
       HX *= ASAX; HY *= ASAY; HZ *= ASAZ;              // Adjusted measurement data: Hadj = H * ( (ASA-128) / 256 + 1)
       HX -= OFFX; HY -= OFFY; HZ -= OFFZ;              // Apply Hard-iron offsets
       HX *= SCAX; HY *= SCAY; HZ *= SCAZ;              // Apply Soft-iron correction
       BUFX += HX; BUFY += HY; BUFZ += HZ;
     }
  }
  HX = BUFX >> 4;  HY = BUFY >> 4; HZ = BUFZ >> 4;      // Calculate avarage values
}

void AccelRead() {                                      // Take avarage of 16 samples
  BUFX = BUFY = BUFZ = 0;
  uint8_t i = 16;
  do {
      uint8_t RAW_RDY = 0x00;                           // Variable temporary keeps RAW_RDY_EN (INT_STATUS REG) bit
      do {
        TinyMegaI2C.start(ACEL_GIRO, 0);
        TinyMegaI2C.write(INT_STATUS);
        TinyMegaI2C.restart(ACEL_GIRO, -1);
        RAW_RDY = TinyMegaI2C.read(0x00);               // Read INT_STATUS register
        TinyMegaI2C.stop();
      } while (!(RAW_RDY & 0x01));                      // Mask for DRDY bit of STATUS1 register
      TinyMegaI2C.start(ACEL_GIRO, 0);
      TinyMegaI2C.write(ACCEL_XOUT_H);
      TinyMegaI2C.restart(ACEL_GIRO, -1);
      AX = (TinyMegaI2C.read(0x01) << 8);               // Read ACCEL_XOUT_H register & auto increments address by 1
      AX |= TinyMegaI2C.read(0x01);
      AY = (TinyMegaI2C.read(0x01) << 8);               // Read ACCEL_YOUT_H register & auto increments address by 1
      AY |= TinyMegaI2C.read(0x01);
      AZ = (TinyMegaI2C.read(0x01) << 8);               // Read ACCEL_XOUT_H register & auto increments address by 1
      AZ |= TinyMegaI2C.read(0x00);
      TinyMegaI2C.stop();
      BUFX += AX; BUFY += AY; BUFZ += AZ;
      i--;
  } while (i>0);
  AX = BUFX >> 4;  AY = BUFY >> 4; AZ = BUFZ >> 4;
}

// Pin assignments

int Pins[4][4] = {{ -1,  8,  6,  4 },   
                  {  7, -1, 11,  9 },
                  {  0, 10, -1,  2 },
                  {  5,  3,  1, -1 }};

// Display multiplexer **********************************************

void DisplaySetup () {
  // Set up Timer/Counter TCB to multiplex the display
  TCB0.CCMP = 19999;                                    // Divide 5MHz by 20000 = 250Hz
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;   // Enable timer, divide by 1
  TCB0.CTRLB = 0;                                       // Periodic Interrupt Mode
  TCB0.INTCTRL = TCB_CAPT_bm;                           // Enable interrupt
}

// Timer/Counter TCB interrupt - multiplexes the display and counts ticks
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                          // Clear the interrupt flag
  DisplayNextRow();
  Timeout--;
}

void DisplayOn () {
  // Turn off all the pullups
  PORTA.PIN0CTRL = 0;
  PORTA.PIN3CTRL = 0;
  PORTA.PIN4CTRL = 0;
  PORTA.PIN5CTRL = 0;
  PORTA.PIN6CTRL = 0;
  TCB0.INTCTRL = TCB_CAPT_bm;                           // Enable interrupt
}

void DisplayOff () {
  TCB0.INTCTRL = 0;                                     // Disable interrupt
  PORTA.DIR = 0;                                        // All PORTA pins inputs
  // Turn on all the pullups for minimal power in sleep
  PORTA.PIN0CTRL = PORT_PULLUPEN_bm;                    // UPDI
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN5CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN6CTRL = PORT_PULLUPEN_bm;
}
  
void DisplayNextRow() {
  Cycle++;
  uint8_t row = Cycle & 0x03;
  uint8_t bits = 0;
  for (int i=0; i<4; i++) {
    if (Hours == Pins[row][i]) bits = bits | 1<<(i+3);
    if (Fivemins == Pins[row][i]) bits = bits | 1<<(i+3);
  }
  PORTA.DIR = 1<<(row+3) | bits;                        // Make outputs for lit LED
  PORTA.OUT = bits;                                     // Set outputs high
}

// Delay in 1/250 of a second
void MyDelay (int count) {
  Timeout = count;
  while (Timeout);
}

// Real-Time Clock **********************************************

void RTCSetup () {
  uint8_t temp;
  // Initialize 32.768kHz Oscillator:

  // Disable oscillator:
  temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_ENABLE_bm;

  // Enable writing to protected register
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);     // Wait until XOSC32KS is 0
  
  temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_SEL_bm;        // Use External Crystal
  
  // Enable writing to protected register
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;
  
  temp = CLKCTRL.XOSC32KCTRLA | CLKCTRL_ENABLE_bm;      // Enable oscillator
  
  // Enable writing to protected register
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;
  
  // Initialize RTC
  while (RTC.STATUS > 0);                               // Wait until registers synchronized

  // 32.768kHz External Crystal Oscillator (XOSC32K)
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  
  RTC.DBGCTRL = RTC_DBGRUN_bm;                          // Run in debug: enabled

  RTC.PITINTCTRL = RTC_PI_bm;                           // Periodic Interrupt: enabled
  
  // RTC Clock Cycles 32768, enabled ie 1Hz interrupt
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
}

// Interrupt Service Routine called every second
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;                          // Clear interrupt flag
  Secs = (Secs + 1) % 43200;                            // Wrap around after 12 hours
}

// Show time button **********************************************

void TimeButtonEnable () {
  PORTA.PIN2CTRL = PORT_PULLUPEN_bm;                    // Pullup
  PORTA.PIN2CTRL = PORTA.PIN2CTRL | PORT_ISC_LEVEL_gc;  // Trigger low level
}

void NorthButtonEnable () {
  PORTA.PIN1CTRL = PORT_PULLUPEN_bm;                    // Pullup
  PORTA.PIN1CTRL = PORTA.PIN1CTRL | PORT_ISC_LEVEL_gc;  // Trigger low level
}

ISR(PORTA_PORT_vect) {
  if (PORTA.INTFLAGS & 0x04){
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;                  // Disable button
    PORTA.INTFLAGS = PORT_INT2_bm;                      // Clear PA2 interrupt flag
    ShowTime = true;
    ShowNorth = false;                                  // Two buttons pressed exception
  }
  if (PORTA.INTFLAGS & 0x02){
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm;                  // Disable button
    PORTA.INTFLAGS = PORT_INT1_bm;                      // Clear PA1 interrupt flag
    ShowTime = false;                                   // Two buttons pressed exception
    ShowNorth = true;
  }   
}

// Setup **********************************************

void SleepSetup () {
  SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
  SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
}

void SetTime () {
  unsigned int secs = 0, offset = 0;
  TimeButtonEnable();
  while (!ShowTime) {
    Fivemins = (secs/300)%12;
    Hours = ((secs+1800)/3600)%12;
    // Write time to global Secs
    Secs = secs + offset;
    DisplayOn();
    MyDelay(Tickspersec);
    DisplayOff();
    offset++;
    secs = secs + 300;
  }
}

void setup() {
  DisplaySetup();
  SleepSetup();
  // Set time on power-on
  SetTime();
  RTCSetup();
  TinyMegaI2C.init();
  // Set mpu9250: read samples and go to sleep
  SetMPU9250();
  ShowTime = true;
}

void loop() {
  unsigned int secs;
  if (ShowTime) {
    cli(); secs = Secs; sei();
    Hours = ((secs+1800)/3600)%12;
    Fivemins = 12;
    int Mins = (secs/60)%60;
    int From = Mins/5;
    int Count = Mins%5;
    DisplayOn();
    for (int i=0; i<5-Count; i++) {
      Fivemins = From; MyDelay(Tickspersec/5);
      Fivemins = 12; MyDelay(Tickspersec/5);
    }
    for (int i=0; i<Count; i++) {
      Fivemins = (1+From)%12; MyDelay(Tickspersec/5);
      Fivemins = 12; MyDelay(Tickspersec/5);
    }
    DisplayOff();
    ShowTime = false;
    if (ShowNorth) {                                    // Two buttons pressed - Time Correction ON
      ShowNorth = false;
      TimeCorrection = true;
    }
  }
  if (ShowNorth) {
    WakeAccel(); AccelRead(); SleepAccel();
    WakeAK8963(); MagRead(); SleepAK8963();
    AVEC = sqrt((AX * AX) + (AY * AY) + (AZ * AZ));
    HXh = AVEC * HX * sqrt((AX * AX) + (AZ * AZ)) - HY * AX * AY + HZ * AY * sqrt((AY * AY) + (AZ * AZ));
    HYh = AVEC * (HY * sqrt((AY * AY) + (AZ * AZ)) + HZ * AX);
    Heading = atan2(HYh, HXh) * RAD_TO_DEG;             // Magnetic North
    Heading += Declination;                             // Geographic North
    Heading += DISOFF;                                  // Display shift
    if (Heading < 0) Heading += 360;                    // Allow for under/overflow
    if (Heading >= 360) Heading -= 360;
    Heading /= 15;                                      // Course in range 0..23                           
    if (Heading % 2) {                                  // North direction lies between LEDs pair
      Hours = Heading / 2;                          
      Fivemins = (Hours + 1) % 12;
    }
    else {                                              // North direction lies through one of the LEDs
      Hours = Heading / 2;
      Fivemins = 12;                                    // One LED off
    }
    DisplayOn();
    MyDelay(Tickspersec >> 5);                          // Tickspersec / 32
    DisplayOff();
    ShowNorth = false;
    if (ShowTime) {                                     // Calibration ON
      ShowTime = false;
      StartCalibration = true;
    }
  }
  if (StartCalibration) {
    MagCalibration(CAL_COUNT);
    StartCalibration = false;
    ShowNorth = false;
    ShowTime = false;
  }
  if (TimeCorrection) {
    DisplaySemicircle();
    if (Secs > 30) cli(); Secs -= 30; sei();            // Time correction (-30 sec)
    TimeCorrection = false;
    ShowNorth = false;
    ShowTime = false;
  }
  TimeButtonEnable();
  NorthButtonEnable();
  sleep_cpu();
}
