/*
.-------.        ,-----.     _______       ,-----.  ,---------. ,---.    ,---.    .-''-.    ___    _ .---.  .---.
|  _ _   \     .'  .-,  '.  \  ____  \   .'  .-,  '.\          \|    \  /    |  .'_ _   \ .'   |  | ||   |  |_ _|
| ( ' )  |    / ,-.|  \ _ \ | |    \ |  / ,-.|  \ _ \`--.  ,---'|  ,  \/  ,  | / ( ` )   '|   .'  | ||   |  ( ' )
|(_ o _) /   ;  \  '_ /  | :| |____/ / ;  \  '_ /  | :  |   \   |  |\_   /|  |. (_ o _)  |.'  '_  | ||   '-(_{;}_)
| (_,_).' __ |  _`,/ \ _/  ||   _ _ '. |  _`,/ \ _/  |  :_ _:   |  _( )_/ |  ||  (_,_)___|'   ( \.-.||      (_,_)
|  |\ \  |  |: (  '\_/ \   ;|  ( ' )  \: (  '\_/ \   ;  (_I_)   | (_ o _) |  |'  \   .---.' (`. _` /|| _ _--.   |
|  | \ `'   / \ `"/  \  ) / | (_{;}_) | \ `"/  \  ) /  (_(=)_)  |  (_,_)  |  | \  `-'    /| (_ (_) _)|( ' ) |   |
|  |  \    /   '. \_/``".'  |  (_,_)  /  '. \_/``".'    (_I_)   |  |      |  |  \       /  \ /  . \ /(_{;}_)|   |
''-'   `'-'      '-----'    /_______.'     '-----'      '---'   '--'      '--'   `'-..-'    ``-'`-'' '(_,_) '---'
*/
/*          Copyright 2023 by Ingwie (Bracame)         */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*    Ardumower Alfred mod to drive my autoclip 325    */

#include "CPeripheryInterface.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// SPI

void SPIC::begin()
{
  spi = spi_new();
}

void SPIC::beginTransaction(SPISettings settings)
{
  if (spi_open(spi, "/dev/spidev0.0", 3, 4e6) < 0) // overide settings (mode3 and 4MHz)
    {
      fprintf(stderr, "spi_open(): %s\n", spi_errmsg(spi));
      exit(1);
    }
}

uint8_t SPIC::transfer(uint8_t data)
{
  uint8_t ret = 0;
  if (spi_transfer(spi, &data, &ret, 1) < 0)
    {
      fprintf(stderr, "spi_transfer(): %s\n", spi_errmsg(spi));
      exit(1);
      return 0;
    }
  else return ret;
}

void SPIC::transfertTmcFrame(tmcFrame * frame)
{
  if (spi_transfer(spi, (uint8_t*)frame, (uint8_t*)frame, sizeof(tmcFrame)) < 0)
    {
      fprintf(stderr, "spi_transfer5(): %s\n", spi_errmsg(spi));
      exit(1);
    }
}

void SPIC::endTransaction()
{

}

//  I2C

void I2CC::begin()
{
  i2cc = i2c_new();
  if (i2c_open(i2cc, "/dev/i2c-1") < 0)
    {
      fprintf(stderr, "i2c_open(): %s\n", i2c_errmsg(i2cc));
      exit(1);
    }
}

void I2CC::transfer(struct i2c_msg* msgs, size_t msgsnum)
{
  if (i2c_transfer(i2cc, msgs, msgsnum) < 0)
    {
      fprintf(stderr, "i2c_transfer(): %s\n", i2c_errmsg(i2cc));
      exit(1);
    }
}
