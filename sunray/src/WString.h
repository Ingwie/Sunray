/*
  WString.h - String library for Wiring & Arduino
*/

#ifndef String_class_h
#define String_class_h
#ifdef __cplusplus

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include "stdlib_noniso.h"

// An inherited class for holding the result of a concatenation.  These
// result objects are assumed to be writable by subsequent concatenations.
class StringSumHelper;

// The string class
class String
{
  // use a function pointer to allow for "if (s)" without the
  // complications of an operator bool(). for more information, see:
  // http://www.artima.com/cppsource/safebool.html
  typedef void (String::*StringIfHelperType)() const;
  void StringIfHelper() const
  {
  }

public:
  // Constructors
  // creates a copy of the initial value.
  // if the initial value is null or invalid, or if memory allocation
  // fails, the string will be marked as invalid (i.e. "if (s)" will
  // be false).
  String(const char *cstr = "");
  String(const String &str);

  explicit String(char c);
  explicit String(uint8_t, uint8_t base = 10);
  explicit String(int16_t, uint8_t base = 10);
  explicit String(uint16_t, uint8_t base = 10);
  explicit String(int32_t, uint8_t base = 10);
  explicit String(uint32_t, uint8_t base = 10);
  explicit String(float, uint8_t decimalPlaces = 2);
  explicit String(double, uint8_t decimalPlaces = 2);
  ~String(void);

  uint8_t reserve(uint16_t size);
  inline uint16_t length(void) const
  {
    if(buffer)
      {
        return len;
      }
    else
      {
        return 0;
      }
  }

  // creates a copy of the assigned value.  if the value is null or
  // invalid, or if the memory allocation fails, the string will be
  // marked as invalid ("if (s)" will be false).
  String & operator = (const String &rhs);
  String & operator = (const char *cstr);

  // concatenate (works w/ built-in types)

  // returns true on success, false on failure (in which case, the string
  // is left unchanged).  if the argument is null or invalid, the
  // concatenation is considered unsucessful.
  uint8_t concat(const String &str);
  uint8_t concat(const char *cstr);
  uint8_t concat(char c);
  uint8_t concat(uint8_t c);
  uint8_t concat(int16_t num);
  uint8_t concat(uint16_t num);
  uint8_t concat(int32_t num);
  uint8_t concat(uint32_t num);
  uint8_t concat(float num);
  uint8_t concat(double num);

  // if there's not enough memory for the concatenated value, the string
  // will be left unchanged (but this isn't signalled in any way)
  String & operator += (const String &rhs)
  {
    concat(rhs);
    return (*this);
  }
  String & operator += (const char *cstr)
  {
    concat(cstr);
    return (*this);
  }
  String & operator += (char c)
  {
    concat(c);
    return (*this);
  }
  String & operator += (uint8_t num)
  {
    concat(num);
    return (*this);
  }
  String & operator += (int16_t num)
  {
    concat(num);
    return (*this);
  }
  String & operator += (uint16_t num)
  {
    concat(num);
    return (*this);
  }
  String & operator += (int32_t num)
  {
    concat(num);
    return (*this);
  }
  String & operator += (uint32_t num)
  {
    concat(num);
    return (*this);
  }
  String & operator += (float num)
  {
    concat(num);
    return (*this);
  }
  String & operator += (double num)
  {
    concat(num);
    return (*this);
  }

  friend StringSumHelper & operator + (const StringSumHelper &lhs, const String &rhs);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, const char *cstr);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, char c);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, uint8_t num);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, int16_t num);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, uint16_t num);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, int32_t num);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, uint32_t num);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, float num);
  friend StringSumHelper & operator + (const StringSumHelper &lhs, double num);

  // comparison (only works w/ Strings and "strings")
  operator StringIfHelperType() const
  {
    return buffer ? &String::StringIfHelper : 0;
  }
  int16_t compareTo(const String &s) const;
  uint8_t equals(const String &s) const;
  uint8_t equals(const char *cstr) const;
  uint8_t operator == (const String &rhs) const
  {
    return equals(rhs);
  }
  uint8_t operator == (const char *cstr)  const
  {
    return equals(cstr);
  }
  uint8_t operator != (const String &rhs) const
  {
    return !equals(rhs);
  }
  uint8_t operator != (const char *cstr)  const
  {
    return !equals(cstr);
  }
  uint8_t operator <  (const String &rhs) const;
  uint8_t operator >  (const String &rhs) const;
  uint8_t operator <= (const String &rhs) const;
  uint8_t operator >= (const String &rhs) const;
  uint8_t equalsIgnoreCase(const String &s) const;
  uint8_t startsWith( const String &prefix) const;
  uint8_t startsWith(const String &prefix, uint16_t offset) const;
  uint8_t endsWith(const String &suffix) const;

  // character acccess
  char charAt(uint16_t index) const;
  void setCharAt(uint16_t index, char c);
  char operator [] (uint16_t index) const;
  char& operator [] (uint16_t index);
  void getBytes(uint8_t *buf, uint16_t bufsize, uint16_t index=0) const;
  void toCharArray(char *buf, uint16_t bufsize, uint16_t index=0) const
  {
    getBytes((uint8_t *)buf, bufsize, index);
  }
  const char * c_str() const
  {
    return buffer;
  }
  char* begin()
  {
    return buffer;
  }
  char* end()
  {
    return buffer + length();
  }
  const char* begin() const
  {
    return c_str();
  }
  const char* end() const
  {
    return c_str() + length();
  }

  // search
  int16_t indexOf( char ch ) const;
  int16_t indexOf( char ch, uint16_t fromIndex ) const;
  int16_t indexOf( const String &str ) const;
  int16_t indexOf( const String &str, uint16_t fromIndex ) const;
  int16_t lastIndexOf( char ch ) const;
  int16_t lastIndexOf( char ch, uint16_t fromIndex ) const;
  int16_t lastIndexOf( const String &str ) const;
  int16_t lastIndexOf( const String &str, uint16_t fromIndex ) const;
  String substring( uint16_t beginIndex ) const
  {
    return substring(beginIndex, len);
  };
  String substring( uint16_t beginIndex, uint16_t endIndex ) const;

  // modification
  void replace(char find, char replace);
  void replace(const String& find, const String& replace);
  void remove(uint16_t index);
  void remove(uint16_t index, uint16_t count);
  void toLowerCase(void);
  void toUpperCase(void);
  void trim(void);

  // parsing/conversion
  int32_t toInt(void) const;
  float toFloat(void) const;
  double toDouble(void) const;

protected:
  char *buffer;          // the actual char array
  uint16_t capacity;  // the array length minus one (for the '\0')
  uint16_t len;       // the String length (not counting the '\0')
protected:
  void init(void);
  void invalidate(void);
  uint8_t changeBuffer(uint16_t maxStrLen);
  uint8_t concat(const char *cstr, uint16_t length);

  // copy and move
  String & copy(const char *cstr, uint16_t length);
};

class StringSumHelper : public String
{
public:
  StringSumHelper(const String &s) 	 : String(s) {}
  StringSumHelper(const char *p) 	 : String(p) {}
  StringSumHelper(char c) 			 : String(c) {}
  StringSumHelper(uint8_t num) : String(num) {}
  StringSumHelper(int16_t num) 			 : String(num) {}
  StringSumHelper(uint16_t num)  : String(num) {}
  StringSumHelper(int32_t num) 		 : String(num) {}
  StringSumHelper(uint32_t num) : String(num) {}
  StringSumHelper(float num) 		 : String(num) {}
  StringSumHelper(double num) 		 : String(num) {}
};

#endif  // __cplusplus
#endif  // String_class_h
