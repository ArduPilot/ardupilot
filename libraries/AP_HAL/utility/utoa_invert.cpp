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

#include <stdint.h>
#include "xtoa_fast.h"

char * ultoa_invert (uint32_t val, char *s, uint8_t base) {
	if (base == 8) {
		do {
			*s = '0' + (val & 0x7);
			val >>= 3;
		} while(val);
		return s;
	}

	if (base == 16) {
		do {
			uint8_t digit = '0' + (val & 0xf);
#if XTOA_UPPER == 0
			if (digit > '0' + 9)
				digit += ('a' - '0' - 10);
#else
			if (digit > '0' + 9)
				digit += ('A' - '0' - 10);
#endif
			*s++ = digit;
			val >>= 4;
		} while(val);
		return s;
	}

	// Every base which in not hex and not oct is considered decimal.

	// 33 bits would have been enough.
	uint64_t xval = val;
	do {
		uint8_t saved = xval;
		xval &= ~1;
		xval += 2;
		xval += xval >> 1;		// *1.5
		xval += xval >> 4;		// *1.0625
		xval += xval >> 8;		// *1.00390625
		xval += xval >> 16;		// *1.000015259
		xval += xval >> 32;		// it all amounts to *1.6
		xval >>= 4;				// /16 ... so *1.6/16 is /10, fraction truncated.
		*s++ = '0' + saved - 10 * (uint8_t)xval;
	} while (xval);
	return s;
}


char * ulltoa_invert (uint64_t val, char *s, uint8_t base) {
	if (base == 8) {
		do {
			*s = '0' + (val & 0x7);
			val >>= 3;
		} while(val);
		return s;
	}

	if (base == 16) {
		do {
			uint8_t digit = '0' + (val & 0xf);
#if XTOA_UPPER == 0
			if (digit > '0' + 9)
				digit += ('a' - '0' - 10);
#else
			if (digit > '0' + 9)
				digit += ('A' - '0' - 10);
#endif
			*s++ = digit;
			val >>= 4;
		} while(val);
		return s;
	}

	// Every base which in not hex and not oct is considered decimal.

	// 64 bits is not actually enough, we need 65, but it should
	// be good enough for the log dumping we're using this for
	uint64_t xval = val;
	do {
		uint8_t saved = xval;
		xval &= ~1;
		xval += 2;
		xval += xval >> 1;		// *1.5
		xval += xval >> 4;		// *1.0625
		xval += xval >> 8;		// *1.00390625
		xval += xval >> 16;		// *1.000015259
		xval += xval >> 32;		// it all amounts to *1.6
		xval >>= 4;				// /16 ... so *1.6/16 is /10, fraction truncated.
		*s++ = '0' + saved - 10 * (uint8_t)xval;
	} while (xval);
	return s;
}

