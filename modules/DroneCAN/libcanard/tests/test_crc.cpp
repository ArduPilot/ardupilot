/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Contributors: https://github.com/UAVCAN/libcanard/contributors
 */

#include <gtest/gtest.h>
#include "canard_internals.h"

/*
 * CRC-16-CCITT
 * Initial value: 0xFFFF
 * Poly: 0x1021
 * Reverse: no
 * Output xor: 0
 *
 * import crcmod
 * crc = crcmod.predefined.Crc('crc-ccitt-false')
 * crc.update('123456789')
 * crc.hexdigest()
 * '29B1'
 */

TEST(CRC, CRC)
{
    uint16_t crc = 0xFFFFU;

    crc = crcAdd(crc, reinterpret_cast<const uint8_t*>("1"), 1);
    crc = crcAdd(crc, reinterpret_cast<const uint8_t*>("2"), 1);
    crc = crcAdd(crc, reinterpret_cast<const uint8_t*>("3"), 1);

    ASSERT_TRUE(0x5BCE == crc);                                     // Using Libuavcan as reference

    crc = crcAdd(crc, reinterpret_cast<const uint8_t*>("456789"), 6);

    ASSERT_TRUE(0x29B1 == crc);
}
