/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
   Concepts and parts of this file have been contributed by Fabio Utzig,
   chvprintf() added by Brent Roman.
 */

/**
 * @file    oop_chprintf.c
 * @brief   Mini printf-like functionality.
 *
 * @addtogroup OOP_CHPRINTF
 * @details Mini printf-like functionality.
 * @{
 */

#include "oop_chprintf.h"
#include "oop_memstreams.h"

#define MAX_FILLER 11
#define FLOAT_PRECISION 9

static char *long_to_string_with_divisor(char *p,
                                         long num,
                                         unsigned radix,
                                         long divisor) {
  int i;
  char *q;
  long l, ll;

  l = num;
  if (divisor == 0) {
    ll = num;
  } else {
    ll = divisor;
  }

  q = p + MAX_FILLER;
  do {
    i = (int)(l % radix);
    i += '0';
    if (i > '9') {
      i += 'A' - '0' - 10;
    }
    *--q = i;
    l /= radix;
  } while ((ll /= radix) != 0);

  i = (int)(p + MAX_FILLER - q);
  do
    *p++ = *q++;
  while (--i);

  return p;
}

static char *ch_ltoa(char *p, long num, unsigned radix) {

  return long_to_string_with_divisor(p, num, radix, 0);
}

#if CHPRINTF_USE_FLOAT
static char *ftoa(char *p, double num, unsigned long precision) {
  static const long chpow10[FLOAT_PRECISION] = {
    10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
  };
  long l;

  if ((precision == 0) || (precision > FLOAT_PRECISION)) {
    precision = FLOAT_PRECISION;
  }
  precision = chpow10[precision - 1];

  l = (long)num;
  p = long_to_string_with_divisor(p, l, 10, 0);
  *p++ = '.';
  l = (long)((num - l) * precision);

  return long_to_string_with_divisor(p, l, 10, precision / 10);
}
#endif

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vprintf()-like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] stmp      pointer to a @p sequential_stream_i interface
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 *
 * @api
 */
int chvprintf(sequential_stream_i *stmp, const char *fmt, va_list ap) {
  char *p, *s, c, filler;
  int i, precision, width;
  int n = 0;
  bool is_long, left_align, do_sign;
  long l;
#if CHPRINTF_USE_FLOAT
  float f;
  char tmpbuf[2*MAX_FILLER + 1];
#else
  char tmpbuf[MAX_FILLER + 1];
#endif

  while (true) {
    c = *fmt++;
    if (c == 0) {
      return n;
    }
    
    if (c != '%') {
      stmPut(stmp, (uint8_t)c);
      n++;
      continue;
    }
    
    p = tmpbuf;
    s = tmpbuf;

    /* Alignment mode.*/
    left_align = false;
    if (*fmt == '-') {
      fmt++;
      left_align = true;
    }

    /* Sign mode.*/
    do_sign = false;
    if (*fmt == '+') {
      fmt++;
      do_sign = true;
    }

    /* Filler mode.*/
    filler = ' ';
    if (*fmt == '0') {
      fmt++;
      filler = '0';
    }
    
    /* Width modifier.*/
    if ( *fmt == '*') {
      width = va_arg(ap, int);
      ++fmt;
      c = *fmt++;
    }
    else {
      width = 0;
      while (true) {
        c = *fmt++;
        if (c == 0) {
          return n;
        }
        if (c >= '0' && c <= '9') {
          c -= '0';
          width = width * 10 + c;
        }
        else {
          break;
        }
      }
    }
    
    /* Precision modifier.*/
    precision = 0;
    if (c == '.') {
      c = *fmt++;
      if (c == 0) {
        return n;
      }
      if (c == '*') {
        precision = va_arg(ap, int);
        c = *fmt++;
      }
      else {
        while (c >= '0' && c <= '9') {
          c -= '0';
          precision = precision * 10 + c;
          c = *fmt++;
          if (c == 0) {
            return n;
          }
        }
      }
    }
    
    /* Long modifier.*/
    if (c == 'l' || c == 'L') {
      is_long = true;
      c = *fmt++;
      if (c == 0) {
        return n;
      }
    }
    else {
      is_long = (c >= 'A') && (c <= 'Z');
    }

    /* Command decoding.*/
    switch (c) {
    case 'c':
      filler = ' ';
      *p++ = va_arg(ap, int);
      break;
    case 's':
      filler = ' ';
      if ((s = va_arg(ap, char *)) == 0) {
        s = "(null)";
      }
      if (precision == 0) {
        precision = 32767;
      }
      for (p = s; *p && (--precision >= 0); p++)
        ;
      break;
    case 'D':
    case 'd':
    case 'I':
    case 'i':
      if (is_long) {
        l = va_arg(ap, long);
      }
      else {
        l = va_arg(ap, int);
      }
      if (l < 0) {
        *p++ = '-';
        l = -l;
      }
      else
        if (do_sign) {
          *p++ = '+';
        }
      p = ch_ltoa(p, l, 10);
      break;
#if CHPRINTF_USE_FLOAT
    case 'f':
      f = (float) va_arg(ap, double);
      if (f < 0) {
        *p++ = '-';
        f = -f;
      }
      else {
        if (do_sign) {
          *p++ = '+';
        }
      }
      p = ftoa(p, f, precision);
      break;
#endif
    case 'X':
    case 'x':
    case 'P':
    case 'p':
      c = 16;
      goto unsigned_common;
    case 'U':
    case 'u':
      c = 10;
      goto unsigned_common;
    case 'O':
    case 'o':
      c = 8;
unsigned_common:
      if (is_long) {
        l = va_arg(ap, unsigned long);
      }
      else {
        l = va_arg(ap, unsigned int);
      }
      p = ch_ltoa(p, l, c);
      break;
    default:
      *p++ = c;
      break;
    }
    i = (int)(p - s);
    if ((width -= i) < 0) {
      width = 0;
    }
    if (left_align == false) {
      width = -width;
    }
    if (width < 0) {
      if ((*s == '-' || *s == '+') && filler == '0') {
        stmPut(stmp, (uint8_t)*s++);
        n++;
        i--;
      }
      do {
        stmPut(stmp, (uint8_t)filler);
        n++;
      } while (++width != 0);
    }
    while (--i >= 0) {
      stmPut(stmp, (uint8_t)*s++);
      n++;
    }

    while (width) {
      stmPut(stmp, (uint8_t)filler);
      n++;
      width--;
    }
  }
}

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p printf() like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] stmp      pointer to a @p sequential_stream_i interface
 * @param[in] fmt       formatting string
 * @return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 *
 * @api
 */
int chprintf(sequential_stream_i *stmp, const char *fmt, ...) {
  va_list ap;
  int formatted_bytes;

  va_start(ap, fmt);
  formatted_bytes = chvprintf(stmp, fmt, ap);
  va_end(ap);

  return formatted_bytes;
}

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p snprintf()-like functionality.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 * @post    @p str is NUL-terminated, unless @p size is 0.
 *
 * @param[in] str       pointer to a buffer
 * @param[in] size      maximum size of the buffer
 * @param[in] fmt       formatting string
 * @return              The number of characters (excluding the
 *                      terminating NUL byte) that would have been
 *                      stored in @p str if there was room.
 *
 * @api
 */
int chsnprintf(char *str, size_t size, const char *fmt, ...) {
  va_list ap;
  int retval;

  /* Performing the print operation.*/
  va_start(ap, fmt);
  retval = chvsnprintf(str, size, fmt, ap);
  va_end(ap);

  /* Return number of bytes that would have been written.*/
  return retval;
}

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vsnprintf()-like functionality.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 * @post    @p str is NUL-terminated, unless @p size is 0.
 *
 * @param[in] str       pointer to a buffer
 * @param[in] size      maximum size of the buffer
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number of characters (excluding the
 *                      terminating NUL byte) that would have been
 *                      stored in @p str if there was room.
 *
 * @api
 */
int chvsnprintf(char *str, size_t size, const char *fmt, va_list ap) {
  memory_stream_c ms;
  size_t size_wo_nul;
  int retval;

  if (size > 0)
    size_wo_nul = size - 1;
  else
    size_wo_nul = 0;

  /* Memory stream object to be used as a string writer, reserving one
     byte for the final zero.*/
  memstmObjectInit(&ms, (uint8_t *)str, size_wo_nul, 0U);

  /* Performing the print operation using the common code.*/
  retval = chvprintf(oopGetIf(&ms, stm), fmt, ap);

  /* Terminate with a zero, unless size==0.*/
  if (ms.eos < size) {
    str[ms.eos] = 0;
  }

  /* Return number of bytes that would have been written.*/
  return retval;
}

/** @} */
