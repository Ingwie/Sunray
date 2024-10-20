/*
  WString.cpp - String library for Wiring & Arduino
  ...mostly rewritten by Paul Stoffregen...
  Copyright (c) 2009-10 Hernando Barragan.  All rights reserved.
  Copyright 2011, Paul Stoffregen, paul@pjrc.com

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
 */

#include "WString.h"
#include <stdio.h>
#include <stdlib.h>
#include "stdlib_noniso.h"

/*********************************************/
/*  Constructors                             */
/*********************************************/

String::String(const char *cstr)
{
  init();
  if (cstr) copy(cstr, strlen(cstr));
}

String::String(const String &value)
{
  init();
  *this = value;
}

String::String(char c)
{
  init();
  char buf[2];
  buf[0] = c;
  buf[1] = 0;
  *this = buf;
}

String::String(uint8_t value, uint8_t base)
{
  init();
  char buf[1 + 8 * sizeof(uint8_t)];
  utoa(value, buf, base);
  *this = buf;
}

String::String(int16_t value, uint8_t base)
{
  init();
  char buf[2 + 8 * sizeof(int16_t)];
  itoa(value, buf, base);
  *this = buf;
}

String::String(uint16_t value, uint8_t base)
{
  init();
  char buf[1 + 8 * sizeof(uint16_t)];
  utoa(value, buf, base);
  *this = buf;
}

String::String(int32_t value, uint8_t base)
{
  init();
  char buf[2 + 8 * sizeof(int32_t)];
  ltoa(value, buf, base);
  *this = buf;
}

String::String(uint32_t value, uint8_t base)
{
  init();
  char buf[1 + 8 * sizeof(uint32_t)];
  ultoa(value, buf, base);
  *this = buf;
}

String::String(float value, uint8_t decimalPlaces)
{
  init();
  char buf[33];
  *this = dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf);
}

String::String(double value, uint8_t decimalPlaces)
{
  init();
  char buf[33];
  *this = dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf);
}

String::~String()
{
  if(buffer)
    {
      free(buffer);
    }
  init();
}

/*********************************************/
/*  Memory Management                        */
/*********************************************/

inline void String::init(void)
{
  buffer = NULL;
  capacity = 0;
  len = 0;
}

void String::invalidate(void)
{
  if (buffer) free(buffer);
  buffer = NULL;
  capacity = len = 0;
}

uint8_t String::reserve(uint16_t size)
{
  if (buffer && capacity >= size) return 1;
  if (changeBuffer(size))
    {
      if (len == 0) buffer[0] = 0;
      return 1;
    }
  return 0;
}

uint8_t String::changeBuffer(uint16_t maxStrLen)
{
  char *newbuffer = (char *)realloc(buffer, maxStrLen + 1);
  if (newbuffer)
    {
      buffer = newbuffer;
      capacity = maxStrLen;
      return 1;
    }
  return 0;
}

/*********************************************/
/*  Copy and Move                            */
/*********************************************/

String & String::copy(const char *cstr, uint16_t length)
{
  if (!reserve(length))
    {
      invalidate();
      return *this;
    }
  len = length;
  strcpy(buffer, cstr);
  return *this;
}

String & String::operator = (const String &rhs)
{
  if (this == &rhs) return *this;

  if (rhs.buffer) copy(rhs.buffer, rhs.len);
  else invalidate();

  return *this;
}

String & String::operator = (const char *cstr)
{
  if (cstr) copy(cstr, strlen(cstr));
  else invalidate();

  return *this;
}

/*********************************************/
/*  concat                                   */
/*********************************************/

uint8_t String::concat(const String &s)
{
  return concat(s.buffer, s.len);
}

uint8_t String::concat(const char *cstr, uint16_t length)
{
  uint16_t newlen = len + length;
  if (!cstr) return 0;
  if (length == 0) return 1;
  if (!reserve(newlen)) return 0;
  strcpy(buffer + len, cstr);
  len = strlen(buffer);
  return 1;
}

uint8_t String::concat(const char *cstr)
{
  if (!cstr) return 0;
  return concat(cstr, strlen(cstr));
}

uint8_t String::concat(char c)
{
  char buf[2];
  buf[0] = c;
  buf[1] = 0;
  return concat(buf, 1);
}

uint8_t String::concat(uint8_t num)
{
  char buf[1 + 3 * sizeof(uint8_t)];
  itoa(num, buf, 10);
  return concat(buf, strlen(buf));
}

uint8_t String::concat(int16_t num)
{
  char buf[2 + 3 * sizeof(int16_t)];
  itoa(num, buf, 10);
  return concat(buf, strlen(buf));
}

uint8_t String::concat(uint16_t num)
{
  char buf[1 + 3 * sizeof(uint16_t)];
  utoa(num, buf, 10);
  return concat(buf, strlen(buf));
}

uint8_t String::concat(int32_t num)
{
  char buf[2 + 3 * sizeof(int32_t)];
  ltoa(num, buf, 10);
  return concat(buf, strlen(buf));
}

uint8_t String::concat(uint32_t num)
{
  char buf[1 + 3 * sizeof(uint32_t)];
  ultoa(num, buf, 10);
  return concat(buf, strlen(buf));
}

uint8_t String::concat(float num)
{
  char buf[20];
  char* string = dtostrf(num, 4, 2, buf);
  return concat(string, strlen(string));
}

uint8_t String::concat(double num)
{
  char buf[20];
  char* string = dtostrf(num, 4, 2, buf);
  return concat(string, strlen(string));
}

/*********************************************/
/*  Concatenate                              */
/*********************************************/

StringSumHelper & operator + (const StringSumHelper &lhs, const String &rhs)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if (!a.concat(rhs.buffer, rhs.len)) a.invalidate();
  return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, const char *cstr)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if (!cstr || !a.concat(cstr, strlen(cstr))) a.invalidate();
  return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, char c)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if (!a.concat(c)) a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, uint8_t num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, int16_t num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, uint16_t num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, int32_t num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, uint32_t num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, float num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

StringSumHelper & operator +(const StringSumHelper &lhs, double num)
{
  StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
  if(!a.concat(num))
    a.invalidate();
  return a;
}

/*********************************************/
/*  Comparison                               */
/*********************************************/

int16_t String::compareTo(const String &s) const
{
  if (!buffer || !s.buffer)
    {
      if (s.buffer && s.len > 0) return 0 - *(uint8_t *)s.buffer;
      if (buffer && len > 0) return *(uint8_t *)buffer;
      return 0;
    }
  return strcmp(buffer, s.buffer);
}

uint8_t String::equals(const String &s2) const
{
  return (len == s2.len && compareTo(s2) == 0);
}

uint8_t String::equals(const char *cstr) const
{
  if (len == 0) return (cstr == NULL || *cstr == 0);
  if (cstr == NULL) return buffer[0] == 0;
  return strcmp(buffer, cstr) == 0;
}

uint8_t String::operator<(const String &rhs) const
{
  return compareTo(rhs) < 0;
}

uint8_t String::operator>(const String &rhs) const
{
  return compareTo(rhs) > 0;
}

uint8_t String::operator<=(const String &rhs) const
{
  return compareTo(rhs) <= 0;
}

uint8_t String::operator>=(const String &rhs) const
{
  return compareTo(rhs) >= 0;
}

uint8_t String::equalsIgnoreCase( const String &s2 ) const
{
  if (this == &s2) return 1;
  if (len != s2.len) return 0;
  if (len == 0) return 1;
  const char *p1 = buffer;
  const char *p2 = s2.buffer;
  while (*p1)
    {
      if (tolower(*p1++) != tolower(*p2++)) return 0;
    }
  return 1;
}

uint8_t String::startsWith( const String &s2 ) const
{
  if (len < s2.len) return 0;
  return startsWith(s2, 0);
}

uint8_t String::startsWith( const String &s2, uint16_t offset ) const
{
  if (offset > len - s2.len || !buffer || !s2.buffer) return 0;
  return strncmp( &buffer[offset], s2.buffer, s2.len ) == 0;
}

uint8_t String::endsWith( const String &s2 ) const
{
  if ( len < s2.len || !buffer || !s2.buffer) return 0;
  return strcmp(&buffer[len - s2.len], s2.buffer) == 0;
}

/*********************************************/
/*  Character Access                         */
/*********************************************/

char String::charAt(uint16_t loc) const
{
  return operator[](loc);
}

void String::setCharAt(uint16_t loc, char c)
{
  if (loc < len) buffer[loc] = c;
}

char & String::operator[](uint16_t index)
{
  static char dummy_writable_char;
  if (index >= len || !buffer)
    {
      dummy_writable_char = 0;
      return dummy_writable_char;
    }
  return buffer[index];
}

char String::operator[]( uint16_t index ) const
{
  if (index >= len || !buffer) return 0;
  return buffer[index];
}

void String::getBytes(uint8_t *buf, uint16_t bufsize, uint16_t index) const
{
  if (!bufsize || !buf) return;
  if (index >= len)
    {
      buf[0] = 0;
      return;
    }
  uint16_t n = bufsize - 1;
  if (n > len - index) n = len - index;
  strncpy((char *)buf, buffer + index, n);
  buf[n] = 0;
}

/*********************************************/
/*  Search                                   */
/*********************************************/

int16_t String::indexOf(char c) const
{
  return indexOf(c, 0);
}

int16_t String::indexOf( char ch, uint16_t fromIndex ) const
{
  if (fromIndex >= len) return -1;
  const char* temp = strchr(buffer + fromIndex, ch);
  if (temp == NULL) return -1;
  return temp - buffer;
}

int16_t String::indexOf(const String &s2) const
{
  return indexOf(s2, 0);
}

int16_t String::indexOf(const String &s2, uint16_t fromIndex) const
{
  if (fromIndex >= len) return -1;
  const char *found = strstr(buffer + fromIndex, s2.buffer);
  if (found == NULL) return -1;
  return found - buffer;
}

int16_t String::lastIndexOf( char theChar ) const
{
  return lastIndexOf(theChar, len - 1);
}

int16_t String::lastIndexOf(char ch, uint16_t fromIndex) const
{
  if (fromIndex >= len) return -1;
  char tempchar = buffer[fromIndex + 1];
  buffer[fromIndex + 1] = '\0';
  char* temp = strrchr( buffer, ch );
  buffer[fromIndex + 1] = tempchar;
  if (temp == NULL) return -1;
  return temp - buffer;
}

int16_t String::lastIndexOf(const String &s2) const
{
  return lastIndexOf(s2, len - s2.len);
}

int16_t String::lastIndexOf(const String &s2, uint16_t fromIndex) const
{
  if (s2.len == 0 || len == 0 || s2.len > len) return -1;
  if (fromIndex >= len) fromIndex = len - 1;
  int16_t found = -1;
  for (char *p = buffer; p <= buffer + fromIndex; p++)
    {
      p = strstr(p, s2.buffer);
      if (!p) break;
      if ((uint16_t)(p - buffer) <= fromIndex) found = p - buffer;
    }
  return found;
}

String String::substring(uint16_t left, uint16_t right) const
{
  if (left > right)
    {
      uint16_t temp = right;
      right = left;
      left = temp;
    }
  String out;
  if (left > len) return out;
  if (right > len) right = len;
  char temp = buffer[right];  // save the replaced character
  buffer[right] = '\0';
  out = buffer + left;  // pointer arithmetic
  buffer[right] = temp;  //restore character
  return out;
}

/*********************************************/
/*  Modification                             */
/*********************************************/

void String::replace(char find, char replace)
{
  if (!buffer) return;
  for (char *p = buffer; *p; p++)
    {
      if (*p == find) *p = replace;
    }
}

void String::replace(const String& find, const String& replace)
{
  if (len == 0 || find.len == 0) return;
  int16_t diff = replace.len - find.len;
  char *readFrom = buffer;
  char *foundAt;
  if (diff == 0)
    {
      while ((foundAt = strstr(readFrom, find.buffer)) != NULL)
        {
          memcpy(foundAt, replace.buffer, replace.len);
          readFrom = foundAt + replace.len;
        }
    }
  else if (diff < 0)
    {
      char *writeTo = buffer;
      while ((foundAt = strstr(readFrom, find.buffer)) != NULL)
        {
          uint16_t n = foundAt - readFrom;
          memcpy(writeTo, readFrom, n);
          writeTo += n;
          memcpy(writeTo, replace.buffer, replace.len);
          writeTo += replace.len;
          readFrom = foundAt + find.len;
          len += diff;
        }
      strcpy(writeTo, readFrom);
    }
  else
    {
      uint16_t size = len; // compute size needed for result
      while ((foundAt = strstr(readFrom, find.buffer)) != NULL)
        {
          readFrom = foundAt + find.len;
          size += diff;
        }
      if (size == len) return;
      if (size > capacity && !changeBuffer(size)) return; // XXX: tell user!
      int16_t index = len - 1;
      while (index >= 0 && (index = lastIndexOf(find, index)) >= 0)
        {
          readFrom = buffer + index + find.len;
          memmove(readFrom + diff, readFrom, len - (readFrom - buffer));
          len += diff;
          buffer[len] = 0;
          memcpy(buffer + index, replace.buffer, replace.len);
          index--;
        }
    }
}

void String::remove(uint16_t index)
{
  if (index >= len)
    {
      return;
    }
  int16_t count = len - index;
  remove(index, count);
}

void String::remove(uint16_t index, uint16_t count)
{
  if (index >= len)
    {
      return;
    }
  if (count <= 0)
    {
      return;
    }
  if (index + count > len)
    {
      count = len - index;
    }
  char *writeTo = buffer + index;
  len = len - count;
  strncpy(writeTo, buffer + index + count,len - index);
  buffer[len] = 0;
}

void String::toLowerCase(void)
{
  if (!buffer) return;
  for (char *p = buffer; *p; p++)
    {
      *p = tolower(*p);
    }
}

void String::toUpperCase(void)
{
  if (!buffer) return;
  for (char *p = buffer; *p; p++)
    {
      *p = toupper(*p);
    }
}

void String::trim(void)
{
  if (!buffer || len == 0) return;
  char *begin = buffer;
  while (isspace(*begin)) begin++;
  char *end = buffer + len - 1;
  while (isspace(*end) && end >= begin) end--;
  len = end + 1 - begin;
  if (begin > buffer) memcpy(buffer, begin, len);
  buffer[len] = 0;
}

/*********************************************/
/*  Parsing / Conversion                     */
/*********************************************/

int32_t String::toInt(void) const
{
  if (buffer) return atol(buffer);
  return 0;
}

float String::toFloat(void) const
{
  if (buffer) return float(atof(buffer));
  return 0;
}

double String::toDouble(void) const
{
  if (buffer) return double(atof(buffer));
  return 0;
}
