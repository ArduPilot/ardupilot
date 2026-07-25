/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ch_test_printf.c
 * @brief   Unit Tests Engine module code.
 *
 * @addtogroup CH_TEST
 * @{
 */

#include "ch_test.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define MAX_FILLER 11

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static char *test_ltoswd(char *p, long num, int radix, long divisor) {
  int i, c;
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
    c = (int)(l % (long)radix);
    c += '0';
    if (c > '9') {
      c += ('A' - '0') - 10;
    }
    *--q = (char)c;
    l /= radix;
    ll /= radix;
  } while (ll != 0);

  i = (int)(p + MAX_FILLER - q);
  do {
    *p++ = *q++;
  } while (--i);

  return p;
}

static char *test_ltoa(char *p, long num, int radix) {

  return test_ltoswd(p, num, radix, 0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Base character output function.
 *
 * @param[in] c         character to be printed
 */
void test_putchar(char c) {

  if (chtest.putchar != NULL) {
    chtest.putchar(c);
  }
}

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vprintf()-like functionality.
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
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number of bytes written.
 *
 * @api
 */
int test_vprintf(const char *fmt, va_list ap) {
  int n;
  char tmpbuf[MAX_FILLER + 1];

  n = 0;
  while (true) {
    char *p, *s, c, filler;
    int i, precision, width;
    bool is_long, left_align, do_sign;

    c = *fmt++;
    if (c == '\0') {
      return n;
    }

    if (c != '%') {
      test_putchar(c);
      n++;
      continue;
    }

    /* Pointers to the temporary buffer.*/
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
        if (c == '\0') {
          return n;
        }
        if ((c >= '0') && (c <= '9')) {
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
      if (c == '\0') {
        return n;
      }
      if (c == '*') {
        precision = va_arg(ap, int);
        c = *fmt++;
      }
      else {
        while ((c >= '0') && (c <= '9')) {
          c -= '0';
          precision = precision * 10 + c;
          c = *fmt++;
          if (c == '\0') {
            return n;
          }
        }
      }
    }

    /* Long modifier.*/
    if ((c == 'l') || (c == 'L')) {
      is_long = true;
      c = *fmt++;
      if (c == '\0') {
        return n;
      }
    }
    else {
      is_long = (c >= 'A') && (c <= 'Z');
    }

    /* Command decoding.*/
    switch (c) {
      int radix;
      long arg;

    case 'c':
      filler = ' ';
      *p++ = va_arg(ap, int);
      break;
    case 's':
      filler = ' ';
      if ((s = va_arg(ap, char *)) == NULL) {
        s = "(null)";
      }
      if (precision == 0) {
        precision = 32767;
      }
      for (p = s; (*p != '\0') && (--precision >= 0); p++) {
        /* Scanning string.*/
      }
      break;
    case 'D':
    case 'd':
    case 'I':
    case 'i':
      if (is_long) {
        arg = va_arg(ap, long);
      }
      else {
        arg = va_arg(ap, int);
      }
      if (arg < 0) {
        *p++ = '-';
        arg = -arg;
      }
      else
        if (do_sign) {
          *p++ = '+';
        }
      p = test_ltoa(p, arg, 10);
      break;
    case 'X':
    case 'x':
    case 'P':
    case 'p':
      radix = 16;
      goto unsigned_common;
    case 'U':
    case 'u':
      radix = 10;
      goto unsigned_common;
    case 'O':
    case 'o':
      radix = 8;
unsigned_common:
      if (is_long) {
        arg = va_arg(ap, unsigned long);
      }
      else {
        arg = va_arg(ap, unsigned int);
      }
      p = test_ltoa(p, arg, radix);
      break;
    default:
      *p++ = c;
      break;
    }
    i = (int)(p - s);
    width -= i;
    if (width < 0) {
      width = 0;
    }
    if (left_align == false) {
      width = -width;
    }
    if (width < 0) {
      if (((*s == '-') || (*s == '+')) && (filler == '0')) {
        test_putchar(*s++);
        n++;
        i--;
      }
      do {
        test_putchar(filler);
        n++;
      } while (++width != 0);
    }
    while (--i >= 0) {
      test_putchar(*s++);
      n++;
    }

    while (width) {
      test_putchar(filler);
      n++;
      width--;
    }
  }
}

/**
 * @brief   Test formatted output function.
 * @details This function implements a minimal @p printf() like functionality.
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
 * @param[in] fmt       formatting string
 * @return              The number of bytes written.
 *
 * @api
 */
int test_printf(const char *fmt, ...) {
  va_list ap;
  int formatted_bytes;

  va_start(ap, fmt);
  formatted_bytes = test_vprintf(fmt, ap);
  va_end(ap);

  return formatted_bytes;
}

/** @} */
