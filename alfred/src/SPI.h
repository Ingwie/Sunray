/*
  SPI Master library for Linux.
*/

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdio.h>
#include <linux/spi/spidev.h>
#include "Arduino.h"

class SPISettings {
public:
  SPISettings():freq(4000000),mode(SPI_MODE_3 | SPI_NO_CS){}
  SPISettings(uint32_t clockFreq, uint8_t bitOrder, uint8_t dataMode) {
    freq = clockFreq;
    mode = dataMode;
  }
private:
  uint32_t freq;
  uint8_t mode;
  friend class SPIClass;
};


class SPIClass {
public:
  static void begin();
  static void end();
  static void setDataMode(uint16_t);
  static void setClockDivider(uint32_t);
  static void setBitsPerWord(uint8_t);
  static void setClock(uint32_t);
  static void beginTransaction(SPISettings settings);
  static uint8_t transfer(uint8_t data);
  inline static void endTransaction(void){}
  //no bit order on the pi
  inline static void setBitOrder(uint32_t){}
protected:
  static int fd;
  static uint16_t mode;
  static uint32_t speed;
  static uint16_t delay;
  static uint8_t bits;               //  8 bits per word
  static uint8_t busAddressMajor;
  static uint8_t busAddressMinor;
};

extern SPIClass SPI;

#endif
