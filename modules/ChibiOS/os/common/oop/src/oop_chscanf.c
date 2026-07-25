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
    This file was contributed by Alex Lewontin.
 */

/**
 * @file    chscanf.c
 * @brief   Mini scanf-like functionality.
 *
 * @addtogroup OOP_CHSCANF
 * @details Mini scanf-like functionality.
 * @{
 */

#include <ctype.h>

#include "hal.h"
#include "oop_chscanf.h"
#include "oop_memstreams.h"

static long sym_to_val(char sym, int base)
{
  sym = tolower(sym);
  if (sym <= '7' && sym >= '0') {
    return sym - '0';
  }
  switch (base) {
  case 16:
    if (sym <= 'f' && sym >= 'a') {
      return (sym - 'a' + 0xa);
    }
    /* fallthrough */
  case 10:
    if (sym == '8') {
      return 8;
    }
    if (sym == '9') {
      return 9;
    }
    /* fallthrough */
  default:
    return -1;
  }
}

#if CHSCANF_USE_FLOAT

/* Custom mixed-type power function. The internal promotion of the result to a double
   allows for a greater dynamic range than integral types. This function is mostly for
   simplicity, to allow us to do floating point math without either requiring any
   libc linkages, or actually having to write floating point algorithms ourselves */
static inline double ch_mpow(double x, unsigned long y)
{
  double res = 1;

  do {
    if (y & 1) {
      res *= x;
    }
    x *= x;
  } while (y >>= 1);

  return res;
}

#endif

/**
 * @brief   System formatted input function.
 * @details This function implements a minimal @p vscanf()-like functionality
 *          with input on a @p BaseSequentialStream.
 *          The general parameters format is: %[*][width][l|L]p
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
 * @param[in] chp       pointer to a @p BufferedStream implementing object
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number parameters in ap that have been successfully
 *                      filled. This does not conform to the standard in that if
 *                      a failure (either matching or input) occurs before any
 *                      parameters are assigned, the function will return 0.
 *
 * @api
 */
int chvscanf(sequential_stream_i *stmp, const char *fmt, va_list ap)
{
  char  f;
  int   c;
  int   width, base, i;
  int   n = 0;
  void* buf;
  bool  is_long, is_signed, is_positive;
  bool  has_digit;
  long  vall, digit;
#if CHSCANF_USE_FLOAT
  long   exp;
  double valf;
  char   exp_char;
  int    exp_base;
  bool   exp_is_positive, initial_digit;
  char*  match;
  int    fixed_point;
#endif

  /* Peek the first character of the format string. If it is null,
     we don't even need to take any input, just return 0 */
  f = *fmt++;
  if (f == 0) {
    return n;
  }

  /* Otherwise, get the first character from the input stream before we loop for the first time
     (no peek function for the stream means an extra character is taken out every iteration of the
     loop, so each loop iteration uses the value of c from the last one. However, the first iteration
     has no value to work with, so we initialize it here) */
  c = stmGet(stmp);

  while (c != STM_RESET && f != 0) {

    /* There are 3 options for f:
        - whitespace (take and discard as much contiguous whitespace as possible)
        - a non-whitespace, non-control sequence character (must 1:1 match)
        - a %, which indicates the beginning of a control sequence
    */

    if (isspace(f)) {
      while (isspace(c)) {
        c = stmGet(stmp);
      }
      f = *fmt++;
      continue;
    }

    if (f != '%') {
      if (f != c) {
        break;
      } else {
        c = stmGet(stmp);
        f = *fmt++;
        continue;
      }
    }

    /* So we have a formatting token... probably */
    f = *fmt++;
    /* Special case: a %% is equivalent to a '%' literal */
    if (f == '%') {
      if (f != c) {
        break;
      } else {
        c = stmGet(stmp);
        f = *fmt++;
        continue;
      }
    }

    if (f == '*') {
      buf = NULL;
      f   = *fmt++;
    } else {
      buf = va_arg(ap, void*);
    }

    /* Parse the optional width specifier */
    width = 0;
    while (isdigit(f)) {
      width = (width * 10) + (f - '0');
      f     = *fmt++;
    }

    if (!width) {
      width = -1;
    }

    /* Parse the optional length specifier */
    if (f == 'l' || f == 'L') {
      is_long = true;
      f       = *fmt++;
    } else {
      is_long = isupper(f);
    }

    is_positive = true;
    is_signed   = true;
    base        = 10;
    has_digit   = false;

    switch (f) {

    case 'c':
      /* Not supporting wchar_t, is_long is just ignored */
      if (width == 0) {
        width = 1;
      }
      for (i = 0; i < width; ++i) {
        if (buf) {
          ((char*)buf)[i] = c;
        }
        c = stmGet(stmp);
        if (c == STM_RESET) {
          return n;
        }
      }
      ++n;
      f = *fmt++;
      continue;

    case 's':
      /* S specifier discards leading whitespace */
      while (isspace(c)) {
        c = stmGet(stmp);
        if (c == STM_RESET) {
          return n;
        }
      }
      /* Not supporting wchar_t, is_long is just ignored */
      if (width == 0) {
        width = -1;
      }
      for (i = 0; i < width; ++i) {

        if (isspace(c)) {
          if (buf) {
            ((char*)buf)[i] = 0;
          }

          break;
        }

        if (buf) {
          ((char*)buf)[i] = c;
        }
        c = stmGet(stmp);
        if (c == STM_RESET) {
          return n;
        }
      }

      if (width != -1) {
        if (buf) {
          ((char*)buf)[width] = 0;
        }
      }
      ++n;
      f = *fmt++;
      continue;

#if CHSCANF_USE_FLOAT
    case 'f':
      valf          = -1;
      exp_char      = 'e';
      exp_base      = 10;
      fixed_point   = 0;
      initial_digit = false;
      while (isspace(c)) {
        c = stmGet(stmp);
      }

      if (c == '+') {
        if (--width == 0) {
          return n;
        }
        c = stmGet(stmp);

      } else if (c == '-') {
        if (--width == 0) {
          return n;
        }
        is_positive = false;
        c           = stmGet(stmp);
      }

      /* Special cases: a float can be INF(INITY) or NAN. As a note about this behavior:
          this consumes "the longest sequence of input characters which does not exceed any
          specified field width and which is, or is a prefix of, a matching input sequence" (from
          the C99 standard). Therefore, if a '%f' format token gets the input 'INFINITxyx',
          it will consume the 'INFINIT', leaving 'xyz' in the stream. Similarly, if it gets
          'NAxyz', it will consume the 'NA', leaving 'xyz' in the stream.

          Given that it seems a little odd to accept a short version and a long version, but not
          a version in between that contains the short version but isn't long enough to be the
          long version, This implementation is fairly permissive, and will accept anything from
          'INF' to 'INFINITY', case insensative, (e.g. 'INF', 'INfiN', 'INFit', or 'infinity')
          as a valid token meaning INF. It will not, however, accept less than 'INF' or 'NAN' as
          a valid token (so the above example 'NAxyz' would consume the 'NA', but not recognize it
          as signifying NaN)
      */

      if (tolower(c) == 'n') {
        c = stmGet(stmp);

        match = "an";
        while (*match != 0) {
          if (*match != tolower(c)) {
            stmUnget(chp, c);
            return n;
          }
          if (--width == 0) {
            stmUnget(chp, c);
            return n;
          }
          ++match;
          c = stmGet(stmp);
        }

        valf = NAN;
        goto float_common;
      }

      if (tolower(c) == 'i') {
        c = stmGet(stmp);

        match = "nf";
        while (*match != 0) {
          if (*match != tolower(c)) {
            stmUnget(chp, c);
            return n;
          }
          ++match;
          c = stmGet(stmp);
          if (--width == 0) {
            stmUnget(chp, c);
            return n;
          }
        }

        valf = INFINITY;

        match = "inity";
        while (*match != 0) {
          if (*match != tolower(c)) {
            break;
          }
          ++match;
          if (--width == 0) {
            break;
          }
          c = stmGet(stmp);
        }

        goto float_common;
      }

      if (c == '0') {
        c = stmGet(stmp);
        if (--width == 0) {
          valf = 0;
          goto float_common;
        }

        if (c == 'x' || c == 'X') {
          base     = 16;
          exp_char = 'p';
          exp_base = 2;
          c        = stmGet(stmp);
          if (--width == 0) {
            stmUnget(chp, c);
            return n;
          }
        } else {
          valf = 0;
        }
      }

      if (sym_to_val(c, base) != -1) {
        valf = 0;
      }

      while (width--) {
        digit = sym_to_val(c, base);
        if (digit == -1) {
          break;
        }
        valf = (valf * base) + (double)digit;
        c    = stmGet(stmp);
      }

      if (c == '.') {
        c = stmGet(stmp);

        while (width--) {
          digit = sym_to_val(c, base);
          if (digit == -1) {
            break;
          }
          if (valf == -1) {
            valf = 0;
          }
          valf = (valf * base) + (double)digit;
          ++fixed_point;
          c = stmGet(stmp);
        }
      }

      if (valf == -1.0) {
        stmUnget(chp, c);
        return n;
      }

      valf = valf / ch_mpow(base, fixed_point);

      if (tolower(c) == exp_char) {
        if (width-- == 0) {
          return n;
        }
        c               = stmGet(stmp);
        exp_is_positive = true;
        exp             = 0;

        if (c == '+') {
          if (width-- == 0) {
            return n;
          }
          c = stmGet(stmp);

        } else if (c == '-') {
          if (width-- == 0) {
            return n;
          }
          exp_is_positive = false;
          c               = stmGet(stmp);
        }
        /*
         "When parsing an incomplete floating-point value that ends in the exponent with no digits,
         such as parsing "100er" with the conversion specifier %f, the sequence "100e" (the longest
         prefix of a possibly valid floating-point number) is consumed, resulting in a matching
         error (the consumed sequence cannot be converted to a floating-point number), with "r"
         remaining." (https://en.cppreference.com/w/c/io/fscanf)
        */
        digit = sym_to_val(c, 10);
        if (digit == -1) {
          stmUnget(chp, c);
          return n;
        }
        while (width--) {
          /* Even if the significand was hex, the exponent is decimal */
          digit = sym_to_val(c, 10);
          if (digit == -1) {
            break;
          }
          exp = (exp * 10) + digit;
          c   = stmGet(stmp);
        }
        if (exp_is_positive) {
          valf = valf * (double)ch_mpow(exp_base, exp);
        } else {
          valf = valf / (double)ch_mpow(exp_base, exp);
        }
      }

    float_common:
      if (!is_positive) {
        valf = -1 * valf;
      }

      if (buf) {
        if (is_long) {
          *(double*)buf = valf;
        } else {
          *(float*)buf = valf;
        }
      }

      ++n;
      f = *fmt++;
      continue;
#endif

    case 'i':
    case 'I':
      /* I specifier discards leading whitespace */
      while (isspace(c)) {
        c = stmGet(stmp);
      }
      /* The char might be +, might be -, might be 0, or might be something else */
      if (c == '+') {
        if (--width == 0) {
          return n;
        }
        c = stmGet(stmp);
      } else if (c == '-') {
        if (--width == 0) {
          return n;
        }
        is_positive = false;
        c           = stmGet(stmp);
      }

      if (c == '0') {
        if (--width == 0) {
          return ++n;
        }
        c = stmGet(stmp);
        if (c == 'x' || c == 'X') {
          base = 16;
          if (--width == 0) {
            return n;
          }
          c = stmGet(stmp);

        } else {
          base = 8;
          has_digit = true;
        }
      }
      break;

    case 'd':
    case 'D':
      while (isspace(c)) {
        c = stmGet(stmp);
      }
      if (c == '+') {
        if (--width == 0) {
          return n;
        }
        c = stmGet(stmp);

      } else if (c == '-') {
        if (--width == 0) {
          return n;
        }
        is_positive = false;
        c           = stmGet(stmp);
      }
      break;
    case 'X':
    case 'x':
    case 'P':
    case 'p':
      is_signed = false;
      base      = 16;
      while (isspace(c)) {
        c = stmGet(stmp);
      }
      if (c == '+') {
        if (--width == 0) {
          return n;
        }
        c = stmGet(stmp);
      } else if (c == '-') {
        if (--width == 0) {
          return n;
        }
        is_positive = false;
        c           = stmGet(stmp);
      }
      if (c == '0') {
        if (--width == 0) {
          return ++n;
        }
        c = stmGet(stmp);
        if (c == 'x' || c == 'X') {
          if (--width == 0) {
            return n;
          }
          c = stmGet(stmp);
        } else {
          has_digit = true;
        }
      }
      break;
    case 'U':
    case 'u':
      is_signed = false;
      while (isspace(c)) {
        c = stmGet(stmp);
      }
      if (c == '+') {
        if (--width == 0) {
          return n;
        }
        c = stmGet(stmp);
      } else if (c == '-') {
        if (--width == 0) {
          return n;
        }
        is_positive = false;
        c           = stmGet(stmp);
      }
      break;
    case 'O':
    case 'o':
      is_signed = false;
      base      = 8;
      while (isspace(c)) {
        c = stmGet(stmp);
      }

      if (c == '+') {
        if (--width == 0) {
          return n;
        }
        c = stmGet(stmp);
      } else if (c == '-') {
        if (--width == 0) {
          return n;
        }
        is_positive = false;
        c           = stmGet(stmp);
      }

      break;
    default:
      stmUnget(stmp, c);
      return n;
    }

    vall = 0UL;

    /* If we don't have at least one eligible character, it's a matching failure. */
    digit = sym_to_val(c, base);
    if (digit == -1) {
      if (!has_digit) {
        break;
      }
    } else {
      while (width--) {
        digit = sym_to_val(c, base);
        if (digit == -1) {
          break;
        }
        vall = (vall * base) + digit;
        c    = stmGet(stmp);
      }
    }

    if (!is_positive) {
      vall = -1 * vall;
    }

    if (buf) {
      if (is_long && is_signed) {
        *((signed long*)buf) = vall;
      } else if (is_long && !is_signed) {
        *((unsigned long*)buf) = vall;
      } else if (!is_long && is_signed) {
        *((signed int*)buf) = vall;
      } else if (!is_long && !is_signed) {
        *((unsigned int*)buf) = vall;
      }
    }
    f = *fmt++;
    ++n;
  }
  stmUnget(stmp, c);
  return n;
}

/**
 * @brief   System formatted input function.
 * @details This function implements a minimal @p scanf() like functionality
 *          with input from a @p BufferedStream.
 *          The general parameters format is: %[*][width][l|L]p
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
 * @param[in] chp       pointer to a @p BufferedStream implementing object
 * @param[in] fmt       formatting string
 * @return              The number parameters in ap that have been successfully
 *                      filled. This does not conform to the standard in that if
 *                      a failure (either matching or input) occurs before any
 *                      parameters are assigned, the function will return 0.
 *
 * @api
 */
int chscanf(sequential_stream_i *stmp, const char *fmt, ...)
{
  va_list ap;
  int     retval;

  va_start(ap, fmt);
  retval = chvscanf(stmp, fmt, ap);
  va_end(ap);

  return retval;
}

/**
 * @brief   System formatted input function.
 * @details This function implements a minimal @p snscanf()-like functionality.
 *          The general parameters format is: %[*][width][l|L]p
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
 * @param[in] str       pointer to a buffer
 * @param[in] size      size of the buffer
 * @param[in] fmt       formatting string
 * @return              The number parameters in ap that have been successfully
 *                      filled. This does not conform to the standard in that if
 *                      a failure (either matching or input) occurs before any
 *                      parameters are assigned, the function will return 0.
 *
 * @api
 */
int chsnscanf(char *str, size_t size, const char *fmt, ...)
{
  va_list ap;
  int     retval;

  /* Performing the scan operation.*/
  va_start(ap, fmt);
  retval = chvsnscanf(str, size, fmt, ap);
  va_end(ap);

  /* Return number of receiving arguments successfully assigned.*/
  return retval;
}

/**
 * @brief   System formatted input function.
 * @details This function implements a minimal @p vsnscanf()-like functionality.
 *          The general parameters format is: %[*][width][l|L]p
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
 * @param[in] str       pointer to a buffer
 * @param[in] size      size of the buffer
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number parameters in ap that have been successfully
 *                      filled. This does not conform to the standard in that if
 *                      a failure (either matching or input) occurs before any
 *                      parameters are assigned, the function will return 0.
 *
 * @api
 */
int chvsnscanf(char *str, size_t size, const char *fmt, va_list ap)
{
  memory_stream_c ms;
  size_t          eos;

  eos = 0;
  if (size > 0) {
    while ((eos < size) && (str[eos] != '\0')) {
      eos++;
    }
  }

  /* Memory stream object to be used as a string reader. */
  memstmObjectInit(&ms, (uint8_t *)str, size, eos);

  /* Performing the scan operation using the common code and
     return number of receiving arguments successfully assigned.*/
  return chvscanf(oopGetIf(&ms, stm), fmt, ap);
}

/** @} */
