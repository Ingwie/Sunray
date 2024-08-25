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
#include <string.h>
#include "../../robot.h"

//  GPIO

void GpioPinWrite(gpio_t * name, bool value)
{
  if (gpio_write(name, value) < 0)
    {
      fprintf(stderr, "gpio_write(): %s\n", gpio_errmsg(name));
      robotDriver.exitApp(1);
    }
}

bool GpioPinRead(gpio_t * name)
{
  bool tmp;
  if (gpio_read(name, &tmp) < 0) \
    {fprintf(stderr, "gpio_read(): %s\n", gpio_errmsg(name)); robotDriver.exitApp(1);}
  return(tmp);
}

void GpioSetEdge(gpio_t * name, gpio_edge edge)
{
  if (gpio_set_edge(name, edge) < 0) \
    {fprintf(stderr, "gpio_set_edge(): %s\n", gpio_errmsg(name)); robotDriver.exitApp(1);}
}

void GpioReadEvent(gpio_t * name, gpio_edge edge, uint64_t * timestamp)
{
  if (gpio_read_event(name, &edge, timestamp) < 0) \
    {fprintf(stderr, "gpio_read_event(): %s\n", gpio_errmsg(name)); robotDriver.exitApp(1);}
}


//  PWM

void PwmEnable(pwm_t * pwm)
{
  if (pwm_enable(pwm) < 0)
    {
      fprintf(stderr, "pwm_enable(): %s\n", pwm_errmsg(pwm));
      robotDriver.exitApp(1);
    }
}

void PwmDisable(pwm_t * pwm)
{
  if (pwm_disable(pwm) < 0)
    {
      fprintf(stderr, "pwm_disable(): %s\n", pwm_errmsg(pwm));
      robotDriver.exitApp(1);
    }
}

void PwmSetFrequency(pwm_t * pwm, double frequency)
{
  if (pwm_set_frequency(pwm, frequency) < 0)
    {
      fprintf(stderr, "pwm_set_frequency(): %s\n", pwm_errmsg(pwm));
      robotDriver.exitApp(1);
    }
}

void PwmSetDutyCycle(pwm_t * pwm, double dutycycle)
{
  if (pwm_set_duty_cycle(pwm, dutycycle) < 0)
    {
      fprintf(stderr, "pwm_set_duty_cycle(): %s\n", pwm_errmsg(pwm));
      robotDriver.exitApp(1);
    }
}

void PwmSetPolarity(pwm_t * pwm, pwm_polarity_t polarity)
{
  if (pwm_set_polarity(pwm, polarity) < 0)
    {
      fprintf(stderr, "pwm_set_polarity(): %s\n", pwm_errmsg(pwm));
      robotDriver.exitApp(1);
    }
}


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
      robotDriver.exitApp(1);
    }
}

uint8_t SPIC::transfer(uint8_t data)
{
  uint8_t ret = 0;
  if (spi_transfer(spi, &data, &ret, 1) < 0)
    {
      fprintf(stderr, "spi_transfer(): %s\n", spi_errmsg(spi));
      robotDriver.exitApp(1);
      return 0;
    }
  else return ret;
}

void SPIC::transfertTmcFrame(tmcFrame * frame)
{
  tmcFrame ret = {0};
  if (spi_transfer(spi, (uint8_t*)frame, (uint8_t*)&ret, sizeof(struct tmcFrame)) < 0)
    {
      fprintf(stderr, "spi_transfer5(): %s\n", spi_errmsg(spi));
      robotDriver.exitApp(1);
    }
  memcpy((uint8_t*)frame, &ret, sizeof(struct tmcFrame));
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
      robotDriver.exitApp(1);
    }
}

bool I2CC::transfer(struct i2c_msg* msgs, size_t msgsnum)
{
  bool ret = 0;
  if (i2c_transfer(i2cc, msgs, msgsnum) < 0)
    {
      fprintf(stderr, "i2c_transfer(): %s\n", i2c_errmsg(i2cc));
      ret = 1;
      robotDriver.exitApp(1);
    }
  return ret;
}
