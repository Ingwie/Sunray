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

// I2Chelper to use Arduino code

#include "I2chelper.h"

uint8_t i2c_readRegByte(uint8_t address, uint8_t reg) // Return 0 if success
{
  /*Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  return Wire.read();*/

uint8_t ret = 0;
struct i2c_msg msgs[2] =
        {
            /* Write 8-bit address */
            { .addr = address, .flags = 0, .len = 1, .buf = &reg },
            /* Read 8-bit data */
            { .addr = address, .flags = I2C_M_RD, .len = 1, .buf = &ret},
        };

    /* Transfer a transaction with two I2C messages */
    I2C.transfer(msgs, 2);
    return ret;
}

uint8_t i2c_readReg(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t len) // Return 0 if fail
{
  /*Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(false);
  uint8_t ret = Wire.requestFrom(address, len);
  for (uint8_t i = 0; i < len; i++)
    {
      buffer[i] = Wire.read();
    }
  return ret;*/
 struct i2c_msg msgs[2] =
        {
            /* Write 8-bit address */
            { .addr = address, .flags = 0, .len = 1, .buf = &reg },
            /* Read len 8-bit data */
            { .addr = address, .flags = I2C_M_RD, .len = len, .buf = buffer},
        };

    /* Transfer a transaction with two I2C messages */
    I2C.transfer(msgs, 2);
    return 0;
}

uint8_t i2c_writeRegByte(uint8_t address, uint8_t reg, uint8_t value) // Return 0 if success
{
  /*Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission();*/
struct i2c_msg msgs[2] =
        {
            /* Write 8-bit address */
            { .addr = address, .flags = 0, .len = 1, .buf = &reg },
            /* Write 8-bit data */
            { .addr = address, .flags = 0, .len = 1, .buf = &value},
        };

    /* Transfer a transaction with two I2C messages */
    I2C.transfer(msgs, 2);
    return 0;
}

uint8_t i2c_writeReg(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t len) // Return 0 if success
{
  /*Wire.beginTransmission(address);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++)
    {
      Wire.write(buffer[i]);
    }
  return Wire.endTransmission();*/
 struct i2c_msg msgs[2] =
        {
            /* Write 8-bit address */
            { .addr = address, .flags = 0, .len = 1, .buf = &reg },
            /* Write len 8-bit data */
            { .addr = address, .flags = 0, .len = len, .buf = buffer},
        };

    /* Transfer a transaction with two I2C messages */
    I2C.transfer(msgs, 2);
    return 0;
}
