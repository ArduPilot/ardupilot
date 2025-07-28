/* Copyright (c) 2005, Dmitry Xmelkov
   All rights reserved.

   Rewritten in C by Soren Kuula

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
  POSSIBILITY OF SUCH DAMAGE. */

#include "ftoa_engine.h"

#include <stdint.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

/*
 * 2^b ~= f * r * 10^e
 * where
 * i = b div 8
 * r = 2^(b mod 8)
 * f = factorTable[i]
 * e = exponentTable[i]
 */
static const int8_t exponentTable[32] = {
    -36, -33, -31, -29, -26, -24, -21, -19,
    -17, -14, -12, -9,  -7, -4, -2,  0,
    3, 5, 8, 10, 12, 15,  17, 20,
    22, 24, 27, 29,  32, 34, 36, 39
};

static const uint32_t factorTable[32] = {
    2295887404UL,
    587747175UL,
    1504632769UL,
    3851859889UL,
    986076132UL,
    2524354897UL,
    646234854UL,
    1654361225UL,
    4235164736UL,
    1084202172UL,
    2775557562UL,
    710542736UL,
    1818989404UL,
    465661287UL,
    1192092896UL,
    3051757813UL,
    781250000UL,
    2000000000UL,
    512000000UL,
    1310720000UL,
    3355443200UL,
    858993459UL,
    2199023256UL,
    562949953UL,
    1441151881UL,
    3689348815UL,
    944473297UL,
    2417851639UL,
    618970020UL,
    1584563250UL,
    4056481921UL,
    1038459372UL
};

int16_t ftoa_engine(float val, char *buf, uint8_t precision, uint8_t maxDecimals) 
{
    uint8_t flags;

    // Bit reinterpretation hacks. This will ONLY work on little endian machines.
    uint8_t *valbits = (uint8_t*)&val;
    union {
        float v;
        uint32_t u;
    } x;
    x.v = val;
    uint32_t frac = x.u & 0x007fffffUL;

    if (precision>7) precision=7;

    // Read the sign, shift the exponent in place and delete it from frac.
    if (valbits[3] & (1<<7)) flags = FTOA_MINUS; else flags = 0;
    uint8_t exp = valbits[3]<<1;
    if(valbits[2] & (1<<7)) exp++;

    // Test for easy cases
    if(exp==0) { // Zero or subnormal
        if(frac == 0) { // Zero
            buf[0] = flags | FTOA_ZERO;
            uint8_t i;
            for(i=0; i<=precision; i++) {
                buf[i+1] = '0';
            }
            return 0;
        } else { // Subnormal
            exp = 1; // Subnormal mantissa has same significance as exp=1
        }
    } else if(exp==0xff) { // Infinity or NaN
        if(frac==0) flags |= FTOA_INF; else flags |= FTOA_NAN;
    } else { // Normal number
        frac |= (1UL<<23); // The implicit leading 1 is made explicit
    }

    uint8_t idx = exp>>3;
    int8_t exp10 = exponentTable[idx];

    // We COULD try making the multiplication in situ, where we make
    // frac and a 64 bit int overlap in memory and select/weigh the
    // upper 32 bits that way. For starters, this is less risky:
    int64_t prod = (int64_t)frac * (int64_t)factorTable[idx];

    // The expConvFactorTable are factor are correct iff the lower 3 exponent
    // bits are 1 (=7). Else we need to compensate by dividing frac.
    // If the lower 3 bits are 7 we are right.
    // If the lower 3 bits are 6 we right-shift once
    // ..
    // If the lower 3 bits are 0 we right-shift 7x
    prod >>= (15-(exp & 7));

    // Now convert to decimal.
    uint8_t hadNonzeroDigit = 0; // a flag
    uint8_t outputIdx = 0;
    int64_t decimal = 100000000000000ull;

    do {
        char digit = '0';
        if(decimal == 0) {
            // Abnormal case, can't ever add enough digits to exit the loop,
            // bail out. This will happen for subnormals with frac <= 7 and
            // precision == 7.
            break;
        }
        while(1) {// find the first nonzero digit or any of the next digits.
            while ((prod -= decimal) >= 0)
                digit++;
            // Now we got too low. Fix it by adding again, once.
            // it might appear more efficient to check before subtract, or
            // to save and restore last nonnegative value - but in fact
            // they take as long time and more space.
            prod += decimal;
            decimal /= 10;

            // If already found a leading nonzero digit, accept zeros.
            if (hadNonzeroDigit) break;

            // Else, don't return results with a leading zero! Instead
            // skip those and decrement exp10 accordingly.
            if(digit == '0') {
                exp10--;
                continue;
            }

            hadNonzeroDigit = 1;

            // Compute how many digits N to output.
            if(maxDecimals != 0) {                        // If limiting decimals...
                int8_t beforeDP = exp10+1;                // Digits before point
                if (beforeDP < 1) beforeDP = 1;            // Numbers < 1 should also output at least 1 digit.
                /*
                 * Below a simpler version of this:
                int8_t afterDP = outputNum - beforeDP;
                if (afterDP > maxDecimals-1)
                    afterDP = maxDecimals-1;
                outputNum = beforeDP + afterDP;
                */
                maxDecimals = maxDecimals+beforeDP-1;
                if (precision > maxDecimals)
                    precision = maxDecimals;

            } else {
                precision++;                            // Output one more digit than the param value.
            }

            break;
        }

        // Now have a digit.
        outputIdx++;
        if(digit < '0' + 10) // normal case.
            buf[outputIdx] = digit;
        else {
            // Abnormal case, write 9s and bail.
            // We might as well abuse hadNonzeroDigit as counter, it will not be used again.
            for(hadNonzeroDigit=outputIdx; hadNonzeroDigit>0; hadNonzeroDigit--)
                buf[hadNonzeroDigit] = '9';
            goto roundup; // this is ugly but it _is_ code derived from assembler :)
        }
    } while (outputIdx<precision);

    // Rounding:
    decimal *= 10;

    if (prod - (decimal >> 1) >= 0) {

    roundup:
        // Increment digit, cascade
        while(outputIdx != 0) {
            if(++buf[outputIdx] == '0' + 10) {
                if(outputIdx == 1) {
                    buf[outputIdx] = '1';
                    exp10++;
                    flags |= FTOA_CARRY;
                    break;
                } else
                    buf[outputIdx--] = '0'; // and the loop continues, carrying to next digit.
            }
            else break;
        }
    }

    buf[0] = flags;
    return exp10;
}

