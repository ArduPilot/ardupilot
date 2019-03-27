/*
   Adapted from the avr-libc vfprintf:

   Copyright (c) 2002, Alexander Popov (sasho@vip.bg)
   Copyright (c) 2002,2004,2005 Joerg Wunsch
   Copyright (c) 2005, Helmut Wallner
   Copyright (c) 2007, Dmitry Xmelkov
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
   * Neither the name of the copyright holders nor the names of
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

/* From: Id: printf_p_new.c,v 1.1.1.9 2002/10/15 20:10:28 joerg_wunsch Exp */
/* $Id: vfprintf.c,v 1.18.2.1 2009/04/01 23:12:06 arcanum Exp $ */

#include "print_vprintf.h"

#include <cmath>
#include <stdarg.h>
#include <string.h>

#include <AP_HAL/AP_HAL.h>

#include "ftoa_engine.h"
#include "xtoa_fast.h"

#define FL_ZFILL    0x01
#define FL_PLUS     0x02
#define FL_SPACE    0x04
#define FL_LPAD     0x08
#define FL_ALT      0x10
#define FL_WIDTH    0x20
#define FL_PREC     0x40
#define FL_LONG     0x80
#define FL_LONGLONG 0x100

#define FL_NEGATIVE FL_LONG

#define FL_ALTUPP   FL_PLUS
#define FL_ALTHEX   FL_SPACE

#define FL_FLTUPP   FL_ALT
#define FL_FLTEXP   FL_PREC
#define FL_FLTFIX   FL_LONG

void print_vprintf(AP_HAL::BetterStream *s, const char *fmt, va_list ap)
{
        unsigned char c;        /* holds a char from the format string */
        uint16_t flags;
        unsigned char width;
        unsigned char prec;
        unsigned char buf[23];

        for (;;) {
            /*
             * Process non-format characters
             */
            for (;;) {
                c = *fmt++;
                if (!c) {
                    return;
                }
                if (c == '%') {
                    c = *fmt++;
                    if (c != '%') {
                        break;
                    }
                }
                /* emit cr before lf to make most terminals happy */
                if (c == '\n') {
                    s->write('\r');
                }
                s->write(c);
            }

            flags = 0;
            width = 0;
            prec = 0;

            /*
             * Process format adjustment characters, precision, width.
             */
            do {
                if (flags < FL_WIDTH) {
                    switch (c) {
                    case '0':
                        flags |= FL_ZFILL;
                        continue;
                    case '+':
                        flags |= FL_PLUS;
                        FALLTHROUGH;
                    case ' ':
                        flags |= FL_SPACE;
                        continue;
                    case '-':
                        flags |= FL_LPAD;
                        continue;
                    case '#':
                        flags |= FL_ALT;
                        continue;
                    }
                }

                if (flags < FL_LONG) {
                    if (c >= '0' && c <= '9') {
                        c -= '0';
                        if (flags & FL_PREC) {
                            prec = 10*prec + c;
                            continue;
                        }
                        width = 10*width + c;
                        flags |= FL_WIDTH;
                        continue;
                    }
                    if (c == '.') {
                        if (flags & FL_PREC) {
                            return;
                        }
                        flags |= FL_PREC;
                        continue;
                    }
                    if (c == 'l') {
                        flags |= FL_LONG;
                        continue;
                    }
                    if (c == 'h') {
                        continue;
                    }
                } else if ((flags & FL_LONG) && c == 'l') {
                    flags |= FL_LONGLONG;
                    continue;
                }

                break;
            } while ((c = *fmt++) != 0);

#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS || __FPU_PRESENT
            /*
             * Handle floating-point formats E, F, G, e, f, g.
             */
            if (c >= 'E' && c <= 'G') {
                flags |= FL_FLTUPP;
                c += 'e' - 'E';
                goto flt_oper;
            } else if (c >= 'e' && c <= 'g') {
                int exp;                /* exponent of master decimal digit     */
                int n;
                unsigned char vtype;    /* result of float value parse  */
                unsigned char sign;     /* sign character (or 0)        */
                unsigned char ndigs;

                flags &= ~FL_FLTUPP;

flt_oper:
                float value = va_arg(ap,double);

                if (!(flags & FL_PREC)) {
                    prec = 6;
                }
                flags &= ~(FL_FLTEXP | FL_FLTFIX);
                if (c == 'e') {
                    flags |= FL_FLTEXP;
                } else if (c == 'f') {
                    flags |= FL_FLTFIX;
                } else if (prec > 0) {
                    prec -= 1;
                }

                if ((flags & FL_FLTFIX) && fabsf(value) > 9999999) {
                    flags = (flags & ~FL_FLTFIX) | FL_FLTEXP;
                }

                if (flags & FL_FLTFIX) {
                    vtype = 7;              /* 'prec' arg for 'ftoa_engine' */
                    ndigs = prec < 60 ? prec + 1 : 60;
                } else {
                    if (prec > 10) {
                        prec = 10;
                    }
                    vtype = prec;
                    ndigs = 0;
                }
                memset(buf, 0, sizeof(buf));
                exp = ftoa_engine(value, (char *)buf, vtype, ndigs);
                vtype = buf[0];

                sign = 0;
                if ((vtype & FTOA_MINUS) && !(vtype & FTOA_NAN))
                    sign = '-';
                else if (flags & FL_PLUS)
                    sign = '+';
                else if (flags & FL_SPACE)
                    sign = ' ';

                if (vtype & (FTOA_NAN | FTOA_INF)) {
                    ndigs = sign ? 4 : 3;
                    if (width > ndigs) {
                        width -= ndigs;
                        if (!(flags & FL_LPAD)) {
                            do {
                                s->write(' ');
                            } while (--width);
                        }
                    } else {
                        width = 0;
                    }
                    if (sign) {
                        s->write(sign);
                    }

                    const char *p = "inf";
                    if (vtype & FTOA_NAN)
                        p = "nan";
                    while ((ndigs = *p) != 0) {
                        if (flags & FL_FLTUPP)
                            ndigs += 'I' - 'i';
                        s->write(ndigs);
                        p++;
                    }
                    goto tail;
                }

                /* Output format adjustment, number of decimal digits in buf[] */
                if (flags & FL_FLTFIX) {
                    ndigs += exp;
                    if ((vtype & FTOA_CARRY) && buf[1] == '1') {
                        ndigs -= 1;
                    }
                    if ((signed char)ndigs < 1) {
                        ndigs = 1;
                    } else if (ndigs > 8) {
                        ndigs = 8;
                    }
                } else if (!(flags & FL_FLTEXP)) {              /* 'g(G)' format */
                    if (exp <= prec && exp >= -4) {
                        flags |= FL_FLTFIX;
                    }
                    while (prec && buf[1+prec] == '0') {
                        prec--;
                    }
                    if (flags & FL_FLTFIX) {
                        ndigs = prec + 1;               /* number of digits in buf */
                        prec = prec > exp ? prec - exp : 0;       /* fractional part length  */
                    }
                }

                /* Conversion result length, width := free space length */
                if (flags & FL_FLTFIX) {
                    n = (exp>0 ? exp+1 : 1);
                } else {
                    n = 5;          /* 1e+00 */
                }
                if (sign) {
                    n += 1;
                }
                if (prec) {
                    n += prec + 1;
                }
                width = width > n ? width - n : 0;

                /* Output before first digit    */
                if (!(flags & (FL_LPAD | FL_ZFILL))) {
                    while (width) {
                        s->write(' ');
                        width--;
                    }
                }
                if (sign) {
                    s->write(sign);
                }
                if (!(flags & FL_LPAD)) {
                    while (width) {
                        s->write('0');
                        width--;
                    }
                }

                if (flags & FL_FLTFIX) {                /* 'f' format           */

                    n = exp > 0 ? exp : 0;          /* exponent of left digit */
                    unsigned char v = 0;
                    do {
                        if (n == -1) {
                            s->write('.');
                        }
                        v = (n <= exp && n > exp - ndigs)
                            ? buf[exp - n + 1] : '0';
                        if (--n < -prec || v == 0) {
                            break;
                        }
                        s->write(v);
                    } while (1);
                    if (n == exp
                        && (buf[1] > '5'
                            || (buf[1] == '5' && !(vtype & FTOA_CARRY)))) {
                        v = '1';
                    }
                    if (v) {
                        s->write(v);
                    }
                } else {                                /* 'e(E)' format        */
                    /* mantissa     */
                    if (buf[1] != '1')
                        vtype &= ~FTOA_CARRY;
                    s->write(buf[1]);
                    if (prec) {
                        s->write('.');
                        sign = 2;
                        do {
                            s->write(buf[sign++]);
                        } while (--prec);
                    }

                    /* exponent     */
                    s->write(flags & FL_FLTUPP ? 'E' : 'e');
                    ndigs = '+';
                    if (exp < 0 || (exp == 0 && (vtype & FTOA_CARRY) != 0)) {
                        exp = -exp;
                        ndigs = '-';
                    }
                    s->write(ndigs);
                    for (ndigs = '0'; exp >= 10; exp -= 10)
                        ndigs += 1;
                    s->write(ndigs);
                    s->write('0' + exp);
                }

                goto tail;
            }
#endif //#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS || __FPU_PRESENT
            /*
             * Handle string formats c, s, S.
             */
            {
                const char * pnt;
                size_t size;

                switch (c) {
                case 'c':
                    buf[0] = va_arg (ap, int);
                    pnt = (char *)buf;
                    size = 1;
                    break;

                case 's':
                    pnt = va_arg (ap, char *);
                    size = strnlen (pnt, (flags & FL_PREC) ? prec : ~0);
                    break;

                default:
                        goto non_string;
                }

                if (!(flags & FL_LPAD)) {
                    while (size < width) {
                        s->write(' ');
                        width--;
                    }
                }

                while (size) {
                    s->write(*pnt++);
                    if (width) width -= 1;
                    size -= 1;
                }
                goto tail;
            }

        non_string:
            
            /*
             * Handle integer formats variations for d/i, u, o, p, x, X.
             */
            if (c == 'd' || c == 'i') {
                if (flags & FL_LONGLONG) {
                    int64_t x = va_arg(ap,long long);
                    flags &= ~(FL_NEGATIVE | FL_ALT);
                    if (x < 0) {
                        x = -x;
                        flags |= FL_NEGATIVE;
                    }
                    c = ulltoa_invert (x, (char *)buf, 10) - (char *)buf;
                } else {
                    long x = (flags & FL_LONG) ? va_arg(ap,long) : va_arg(ap,int);
                    flags &= ~(FL_NEGATIVE | FL_ALT);
                    if (x < 0) {
                        x = -x;
                        flags |= FL_NEGATIVE;
                    }
                    c = ultoa_invert (x, (char *)buf, 10) - (char *)buf;
                }
            } else {
                int base;

                if (c == 'u') {
                    flags &= ~FL_ALT;
                    base = 10;
                    goto ultoa;
                }

                flags &= ~(FL_PLUS | FL_SPACE);

                switch (c) {
                case 'o':
                    base = 8;
                    goto ultoa;
                case 'p':
                    flags |= FL_ALT;

                    FALLTHROUGH;
                case 'x':
                    if (flags & FL_ALT)
                        flags |= FL_ALTHEX;
                    base = 16;
                    goto ultoa;
                case 'X':
                    if (flags & FL_ALT)
                        flags |= (FL_ALTHEX | FL_ALTUPP);
                    base = 16 | XTOA_UPPER;
ultoa:
                    if (flags & FL_LONGLONG) {
                        c = ulltoa_invert (va_arg(ap, unsigned long long),
                                           (char *)buf, base)  -  (char *)buf;
                    } else {
                        c = ultoa_invert ((flags & FL_LONG)
                                          ? va_arg(ap, unsigned long)
                                          : va_arg(ap, unsigned int),
                                          (char *)buf, base)  -  (char *)buf;
                    }
                    flags &= ~FL_NEGATIVE;
                    break;

                default:
                    return;
                }
            }

            /*
             * Format integers.
             */
            {
                unsigned char len;

                len = c;
                if (flags & FL_PREC) {
                    flags &= ~FL_ZFILL;
                    if (len < prec) {
                        len = prec;
                        if ((flags & FL_ALT) && !(flags & FL_ALTHEX)) {
                            flags &= ~FL_ALT;
                        }
                    }
                }
                if (flags & FL_ALT) {
                    if (buf[c-1] == '0') {
                        flags &= ~(FL_ALT | FL_ALTHEX | FL_ALTUPP);
                    } else {
                        len += 1;
                        if (flags & FL_ALTHEX) {
                            len += 1;
                        }
                    }
                } else if (flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE)) {
                    len += 1;
                }

                if (!(flags & FL_LPAD)) {
                    if (flags & FL_ZFILL) {
                        prec = c;
                        if (len < width) {
                            prec += width - len;
                            len = width;
                        }
                    }
                    while (len < width) {
                        s->write(' ');
                        len++;
                    }
                }

                width =  (len < width) ? width - len : 0;

                if (flags & FL_ALT) {
                    s->write('0');
                    if (flags & FL_ALTHEX) {
                        s->write(flags & FL_ALTUPP ? 'X' : 'x');
                    }
                } else if (flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE)) {
                    unsigned char z = ' ';
                    if (flags & FL_PLUS) {
                        z = '+';
                    }
                    if (flags & FL_NEGATIVE) {
                        z = '-';
                    }
                    s->write(z);
                }

                while (prec > c) {
                    s->write('0');
                    prec--;
                }

                do {
                    s->write(buf[--c]);
                } while (c);
            }

tail:
            /* Tail is possible.    */
            while (width) {
                s->write(' ');
                width--;
            }
        } /* for (;;) */
}
