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
/*          Copyright 2023-2024 by Ingwie (Bracame)    */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*    Ardumower Alfred mod to drive my autoclip 325    */

// I2Chelper to use Arduino code

#include "I2chelper.h"
#include <string.h>

bool i2c_readRegByte(uint8_t address, uint8_t reg) // Return 0 if success
{
  uint8_t ret = 0;
  struct i2c_msg msgs[2] =
  {
    /* Write 8-bit address */
    { .addr = address, .flags = 0, .len = 1, .buf = &reg },
    /* Read 8-bit data */
    { .addr = address, .flags = I2C_M_RD, .len = 1, .buf = &ret},
  };

  /* Transfer a transaction with two I2C messages */
  return I2C.transfer(msgs, 2);
}

bool i2c_readReg(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t len) // Return 0 if success
{
  struct i2c_msg msgs[2] =
  {
    /* Write 8-bit address */
    { .addr = address, .flags = 0, .len = 1, .buf = &reg },
    /* Read len 8-bit data */
    { .addr = address, .flags = I2C_M_RD, .len = len, .buf = buffer},
  };

  /* Transfer a transaction with two I2C messages */
  return I2C.transfer(msgs, 2);
}

bool i2c_writeRegByte(uint8_t address, uint8_t reg, uint8_t value) // Return 0 if success
{
  uint8_t buf[2] = {reg, value};
  struct i2c_msg msgs[1] =
  {
    /* Write 8-bit address + reg + value */
    { .addr = address, .flags = 0, .len = 2, .buf = buf },
  };

  /* Transfer a transaction with two I2C messages */
  return I2C.transfer(msgs, 1);
}

bool i2c_writeReg(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t len) // Return 0 if success
{
  uint8_t buf[len + 1];
  buf[0] = reg;
  memcpy(&buf[1], buffer, len);
  struct i2c_msg msgs[1] =
  {
    /* Write 8-bit address + reg + values */
    { .addr = address, .flags = 0, .len = (++len), .buf = buf },
  };

  /* Transfer a transaction with two I2C messages */
  return I2C.transfer(msgs, 1);
}
