/*
  SPI Master library for Linux
*/

#include "SPI.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/types.h>

int SPIClass::fd = 0;
uint16_t SPIClass::mode = SPI_MODE_3 | SPI_NO_CS;
uint32_t SPIClass::speed = 4000000;
uint16_t SPIClass::delay = 0;
uint8_t SPIClass::bits = 8;
uint8_t SPIClass::busAddressMajor = 0;
uint8_t SPIClass::busAddressMinor = 0;

void SPIClass::begin()
{
  char filename[50];
  sprintf(filename, "/dev/spidev%d.%d", busAddressMajor, busAddressMinor);
  //  Open SPI device.
  fd = open(filename, O_RDWR);
  if (fd < 0)
    {
      perror("error: can't open SPI bus");
      return;
    }
  setDataMode(mode);
  setClock(speed);
  setBitsPerWord(bits);
  printf("spi mode: %d\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
}

void SPIClass::end()
{
}

void SPIClass::setBitsPerWord(uint8_t abits)
{
  abits = abits;
  //  Set SPI read and write bits per word
  int ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1)
    {
      perror("can't set bits per word");
    }
}

void SPIClass::setDataMode(uint16_t amode)
{
  mode = amode;
  //  Set SPI mode to read and write.
  int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1)
    {
      perror("can't set spi mode");
    }
}

void SPIClass::setClockDivider(uint32_t rate)
{
}

void SPIClass::setClock(uint32_t rate)
{
  speed = rate;
  //  Set SPI read and write max speed.
  int ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1)
    {
      perror("can't set max speed hz");
    }
}

uint8_t SPIClass::transfer(uint8_t data)
{
  uint8_t res = 0;

  struct spi_ioc_transfer spi;
  memset (&spi, 0, sizeof(spi));

  spi.tx_buf = (uintptr_t) &data;
  spi.rx_buf = (uintptr_t) &res;
  spi.len = 1;
  spi.delay_usecs = delay;
  spi.speed_hz = speed;
  spi.bits_per_word = bits;

  int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
  //  Check SPI result.
  if (ret < 0)
    {
      perror("spi_transmit failed");
    }

  return res;
}


void SPIClass::beginTransaction(SPISettings settings)
{
  //setDataMode(settings.mode);
  //setClock(settings.freq);
}

SPIClass SPI;

