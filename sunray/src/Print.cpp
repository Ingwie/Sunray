/*
 Print.cpp - Base class that provides print() and println()
 Copyright (c) 2008 David A. Mellis.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 Modified 23 November 2006 by David A. Mellis
 Modified 03 August 2015 by Chuck Todd
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <math.h>

#include "Print.h"

size_t Print::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--)
    {
      n += write(*buffer++);
    }
  return n;
}

size_t Print::printf(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char temp[4096];
  size_t len = vsnprintf(temp, 4096, format, arg);
  len = print(temp);
  va_end(arg);
  return len;
}

size_t Print::print(const String &s)
{
  return write(s.c_str(), s.length());
}

size_t Print::print(const char *str)
{
  return write(str);
}

size_t Print::print(char c)
{
  return write(c);
}

size_t Print::print(uint8_t b, int16_t base)
{
  return print((uint32_t) b, base);
}

size_t Print::print(int16_t n, int16_t base)
{
  return print((int32_t) n, base);
}

size_t Print::print(uint16_t n, int16_t base)
{
  return print((uint32_t) n, base);
}

size_t Print::print(int32_t n, int16_t base)
{
  if (base == 0)
    {
      return write(n);
    }
  else if (base == 10)
    {
      if (n < 0)
        {
          int16_t t = print('-');
          n = -n;
          return printNumber(n, 10) + t;
        }
      return printNumber(n, 10);
    }
  else
    {
      return printNumber(n, base);
    }
}

size_t Print::print(uint32_t n, int16_t base)
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

size_t Print::print(double n, int16_t digits)
{
  return printFloat(n, digits);
}

size_t Print::print(const Printable& x)
{
  return x.printTo(*this);
}

size_t Print::println(void)
{
  size_t n = print('\r');
  n += print('\n');
  return n;
}

size_t Print::println(const String &s)
{
  size_t n = print(s);
  n += println();
  return n;
}

size_t Print::println(const char *s)
{
  size_t n = print(s);
  n += println();
  return n;
}

size_t Print::println(char c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(uint8_t b, int16_t base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t Print::println(int16_t num, int16_t base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(uint16_t num, int16_t base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(int32_t num, int16_t base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(uint32_t num, int16_t base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(double num, int16_t digits)
{
  size_t n = print(num, digits);
  n += println();
  return n;
}

size_t Print::println(const Printable& x)
{
  size_t n = print(x);
  n += println();
  return n;
}

// Private Methods /////////////////////////////////////////////////////////////

size_t Print::printNumber(uint32_t n, uint8_t base)
{
  char buf[8 * sizeof(int32_t) + 1]; // Assumes 8-bit chars plus zero uint8_t.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do
    {
      uint32_t m = n;
      n /= base;
      char c = m - base * n;
      *--str = c < 10 ? c + '0' : c + 'A' - 10;
    }
  while(n);

  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits)
{
  size_t n = 0;

  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically

  // Handle negative numbers
  if (number < 0.0)
    {
      n += print('-');
      number = -number;
    }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  uint32_t int_part = (uint32_t)number;
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    {
      n += print(".");
    }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
    {
      remainder *= 10.0;
      int16_t toPrint = int16_t(remainder);
      n += print(toPrint);
      remainder -= toPrint;
    }

  return n;
}
