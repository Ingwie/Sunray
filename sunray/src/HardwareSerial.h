/*
  HardwareSerial.h - Hardware serial library for Wiring
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include "Stream.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"

class HardwareSerial : public Stream
{
protected:

public:
  inline HardwareSerial() {}
  virtual void begin(uint32_t baud) {}
  virtual void end() {}
  virtual int16_t available(void)
  {
    return 0;
  }
  virtual int16_t peek(void)
  {
    return 0;
  }
  virtual int16_t read(void)
  {
    return 0;
  }
  virtual void flush(void) {}
  virtual size_t write(uint8_t)
  {
    return 0;
  }
  inline size_t write(uint32_t n)
  {
    return write((uint8_t)n);
  }
  inline size_t write(int32_t n)
  {
    return write((uint8_t)n);
  }
  inline size_t write(uint16_t n)
  {
    return write((uint8_t)n);
  }
  inline size_t write(int16_t n)
  {
    return write((uint8_t)n);
  }
  using Print::write; // pull in write(str) and write(buf, size) from Print
  operator bool()
  {
    return true;
  }
};
#pragma GCC diagnostic pop
#ifdef SERIAL_TO_CONSOLE
//extern HardwareSerial Serial1;
#else
//extern HardwareSerial Serial;
#endif

#endif
