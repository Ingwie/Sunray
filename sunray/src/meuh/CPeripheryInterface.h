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

#ifndef C_PERIPHERY_INTERFACE_H
#define C_PERIPHERY_INTERFACE_H

#include "./c-periphery/gpio.h"
#include "./c-periphery/spi.h"
#include <linux/spi/spidev.h>
#include "./c-periphery/pwm.h"
#include "./c-periphery/i2c.h"


//  GPIO

#define SetGpioPin(name, direction) \
 name = gpio_new(); \
 if (gpio_open(name, "/dev/gpiochip0", name##_Number , direction) < 0) \
  {fprintf(stderr, "gpio_open(): %s\n", gpio_errmsg(name));exit(1);}

void GpioPinWrite(gpio_t * name, bool value);
void GpioPinRead(gpio_t * name, bool value);
void GpioSetEdge(gpio_t * name, gpio_edge edge);
void GpioReadEvent(gpio_t * name, gpio_edge edge, uint64_t * timestamp);


//  PWM

#define SetNewPwm(pwm, num) \
pwm = pwm_new(); \
if (pwm_open(pwm, 0, num) < 0) \
   {fprintf(stderr, "pwm_open(): %s\n", pwm_errmsg(pwm)); exit(1);}

void PwmDisable(pwm_t * pwm);
void PwmSetFrequency(pwm_t * pwm, double frequency);
void PwmSetDutyCycle(pwm_t * pwm, double dutycycle);
void PwmSetPolarity(pwm_t * pwm, pwm_polarity_t polarity);


//  SPI

struct tmcFrame { // Specialy to manage TMC comminication transfertTmcFrame()
uint32_t datas;
uint8_t firstByte;
};

class SPISettings {
public:
  SPISettings():freq(4000000),mode(SPI_MODE_3){}
  SPISettings(uint32_t clockFreq, uint8_t bitOrder, uint8_t dataMode) {
    freq = clockFreq;
    mode = dataMode;
  }
private:
  uint32_t freq;
  uint8_t mode;
  friend class SPIC;
};

class SPIC
{
public:
  void begin();
  void beginTransaction(SPISettings settings);
  void endTransaction();
  uint8_t transfer(uint8_t data);
  void transfertTmcFrame(tmcFrame * frame);
protected:
  spi_t *spi;
};

extern SPIC SPI;

//  I2C

class I2CC
{
public:
  void begin();
  bool transfer(struct i2c_msg* msgs, size_t msgsnum);
protected:
  i2c_t *i2cc;
};

extern I2CC I2C;



#endif
