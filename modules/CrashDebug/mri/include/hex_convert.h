/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Hexadecimal to/from text conversion helpers. */
#ifndef _HEX_CONVERT_H_
#define _HEX_CONVERT_H_

#include "try_catch.h"

#define EXTRACT_HI_NIBBLE(X) (((X) >> 4) & 0xF)
#define EXTRACT_LO_NIBBLE(X) ((X) & 0xF)

static const char NibbleToHexChar[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

static inline int HexCharToNibble(unsigned char HexChar)
{
    if (HexChar >= 'a' && HexChar <= 'f')
    {
        return HexChar - 'a' + 10;
    }
    if (HexChar >= 'A' && HexChar <= 'F')
    {
        return HexChar - 'A' + 10;
    }
    if (HexChar >= '0' && HexChar <= '9')
    {
        return HexChar - '0';
    }
    
    __throw_and_return(invalidHexDigitException, -1);
}

#endif /* _HEX_CONVERT_H_ */
