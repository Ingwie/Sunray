/*
  SPI Master library for Linux.
*/

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdio.h>
#include "Arduino.h"

#define SPI_HAS_TRANSACTION

//AVR Arduino compatibility
#define SPI_CLOCK_DIV2 SPI0F2DIV(8000000)
#define SPI_CLOCK_DIV4 SPI0F2DIV(4000000)
#define SPI_CLOCK_DIV8 SPI0F2DIV(1000000)
#define SPI_CLOCK_DIV16 SPI0F2DIV(500000)
#define SPI_CLOCK_DIV32 SPI0F2DIV(250000)
#define SPI_CLOCK_DIV64 SPI0F2DIV(175000)
#define SPI_CLOCK_DIV128 SPI0F2DIV(87500)

#define SPI_CPHA      0x01
#define SPI_CPOL      0x02
#define SPI_MODE_0    (0|0)
#define SPI_MODE_1    (0|SPI_CPHA)
#define SPI_MODE_2    (SPI_CPOL|0)
#define SPI_MODE_3    (SPI_CPOL|SPI_CPHA)
#define SPI_CS_HIGH   0x04
#define SPI_LSB_FIRST 0x08
#define SPI_3WIRE     0x10
#define SPI_LOOP      0x20
#define SPI_NO_CS     0x40
#define SPI_READY     0x80
#define SPI_TX_DUAL   0x100
#define SPI_TX_QUAD   0x200
#define SPI_RX_DUAL   0x400
#define SPI_RX_QUAD   0x800

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
