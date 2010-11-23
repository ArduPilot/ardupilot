/* Copyright (c) 2007, Dmitry Xmelkov
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
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: ntz.h 1217 2007-02-18 13:18:05Z dmix $	*/

#ifndef	_NTZ_H
#define	_NTZ_H

/* Number of Tail Zeros:  ntz(x)= (ffs(x) ? ffs(x)-1 : 16)
   It works with all: cpp, gcc and gas expressions.	*/
#define ntz(x)	\
	( (1 & (((x) & 1) == 0))        \
	+ (1 & (((x) & 3) == 0))        \
    	+ (1 & (((x) & 7) == 0))        \
        + (1 & (((x) & 017) == 0))      \
        + (1 & (((x) & 037) == 0))      \
	+ (1 & (((x) & 077) == 0))      \
	+ (1 & (((x) & 0177) == 0))     \
	+ (1 & (((x) & 0377) == 0))     \
	+ (1 & (((x) & 0777) == 0))     \
	+ (1 & (((x) & 01777) == 0))    \
	+ (1 & (((x) & 03777) == 0))    \
	+ (1 & (((x) & 07777) == 0))    \
	+ (1 & (((x) & 017777) == 0))   \
	+ (1 & (((x) & 037777) == 0))   \
	+ (1 & (((x) & 077777) == 0))   \
	+ (1 & (((x) & 0177777) == 0)) )

#endif	/* !_NTZ_H */
