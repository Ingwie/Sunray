/*
  stdlib_noniso.h - nonstandard (but usefull) conversion functions
*/

#ifndef STDLIB_NONISO_H
#define STDLIB_NONISO_H

#ifdef __cplusplus
extern "C" {
#endif

char* itoa (int16_t val, char *s, int16_t radix);

char* ltoa (int32_t val, char *s, int16_t radix);

char* utoa (uint16_t val, char *s, int16_t radix);

char* ultoa (uint32_t val, char *s, int16_t radix);

char* dtostrf (double val, signed char width, uint8_t prec, char *s);

void reverse(char* begin, char* end);

#ifdef __cplusplus
} // extern "C"
#endif


#endif

