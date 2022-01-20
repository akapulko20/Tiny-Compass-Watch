/* TinyMegaI2C
   David Johnson-Davies - www.technoblogy.com - 17th September 2019
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/

   !Corrections: 
   1. TinyMegaI2C.read() function replaced, the corresponding function proposed by buckket has been used - https://gist.github.com/buckket/09619e6cdc5dee056d41bfb57065db81
   2. I2C clock frequency decreased down to 20kHz for a proper work with an internal pull-up resistors
*/

#include "TinyMegaI2CMaster.h"

TinyMegaI2CMaster::TinyMegaI2CMaster() 
{
}

// Minimal Tiny I2C Routines **********************************************

void TinyMegaI2CMaster::init () {
  PORTB.PIN0CTRL = PORT_PULLUPEN_bm;
  PORTB.PIN1CTRL = PORT_PULLUPEN_bm;
  uint32_t baud = ((F_CPU/FREQUENCY) - (((F_CPU*T_RISE)/1000)/1000)/1000 - 10)/2;
  TWI0.MBAUD = (uint8_t)baud;
  TWI0.MCTRLA = TWI_ENABLE_bm;                                        // Enable as master, no interrupts
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

uint8_t TinyMegaI2CMaster::read (uint8_t ack) {
  while (!(TWI0.MSTATUS & TWI_RIF_bm));                               // Wait for incoming data
  uint8_t data = TWI0.MDATA; 
  if (ack) TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;                       // Send ACK and receive more data
  else TWI0.MCTRLB = TWI_ACKACT_NACK_gc;                              // Prepare to send NACK to indicate no more data is needed; NACK gets send be either issuing a stop condition or sending a new address packet
  return data;
}

bool TinyMegaI2CMaster::write (uint8_t data) {
  while (!(TWI0.MSTATUS & TWI_WIF_bm));                               // Wait for write interrupt flag
  TWI0.MDATA = data;
  TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;                                // Do nothing
  return !(TWI0.MSTATUS & TWI_RXACK_bm);                              // Returns true if slave gave an ACK
}

// Start transmission by sending address
bool TinyMegaI2CMaster::start (uint8_t address, int readcount) {
  bool read;
  if (readcount == 0) read = 0;                                       // Write
  else { I2Ccount = readcount; read = 1; }                            // Read
  TWI0.MADDR = address<<1 | read;                                     // Send START condition
  while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));                // Wait for write or read interrupt flag
  if ((TWI0.MSTATUS & TWI_ARBLOST_bm)) return false;                  // Return false if arbitration lost or bus error
  return !(TWI0.MSTATUS & TWI_RXACK_bm);                              // Return true if slave gave an ACK
}

bool TinyMegaI2CMaster::restart(uint8_t address, int readcount) {
  return TinyMegaI2CMaster::start(address, readcount);
}

void TinyMegaI2CMaster::stop (void) {
  TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;                     // Send STOP
}

TinyMegaI2CMaster TinyMegaI2C = TinyMegaI2CMaster();                  // Instantiate a TinyMegaI2C object
