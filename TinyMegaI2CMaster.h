/* TinyMegaI2C
   David Johnson-Davies - www.technoblogy.com - 17th September 2019
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/

   !Corrections: 
   1. TinyMegaI2C.read() function replaced, the corresponding function proposed by buckket has been used - https://gist.github.com/buckket/09619e6cdc5dee056d41bfb57065db81
   2. I2C clock frequency decreased down to 20kHz for a proper work with an internal pull-up resistors
*/

#ifndef TinyMegaI2CMaster_h
#define TinyMegaI2CMaster_h

#include <stdint.h>
#include <avr/io.h>


// 20kHz clock
uint32_t const FREQUENCY = 20000L;  // Hardware I2C clock in Hz
uint32_t const T_RISE = 6000L;      // Rise time

// Choose these for 1MHz clock
//uint32_t const FREQUENCY = 1000000L; // Hardware I2C clock in Hz
//uint32_t const T_RISE = 120L;        // Rise time

class TinyMegaI2CMaster {

    public:
  TinyMegaI2CMaster();
  void init(void);
  uint8_t read(uint8_t);
  bool write(uint8_t data);
  bool start(uint8_t address, int readcount);
  bool restart(uint8_t address, int readcount);
  void stop(void);

    private:
  int I2Ccount;
};

extern TinyMegaI2CMaster TinyMegaI2C;

#endif
