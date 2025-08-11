/*!
 * \file            sbgTypes.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            17 March 2015
 *
 * \brief           Header file that defines all scalar types.
 *
 * The platform endianness should be defined here.
 *
 * \copyright       Copyright (C) 2007-2024, SBG Systems SAS. All rights reserved.
 * \beginlicense    The MIT license
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
 * \endlicense
 */

#ifndef SBG_TYPES_H
#define SBG_TYPES_H

// Standard headers
#include <stdint.h>

// Local headers
#include "sbgDefines.h"

//----------------------------------------------------------------------//
//- Limits definitions                                                 -//
//----------------------------------------------------------------------//
#define SBG_MIN_INT_24                  (-8388608l)
#define SBG_MAX_INT_24                  (8388607l)
#define SBG_MAX_UINT_24                 (16777215ul)

#define SBG_MIN_INT_40                  (-549755813887ll - 1)
#define SBG_MAX_INT_40                  (549755813887ll)
#define SBG_MAX_UINT_40                 (1099511627775ull)

#define SBG_MIN_INT_48                  (-140737488355327ll - 1)
#define SBG_MAX_INT_48                  (140737488355327ll)
#define SBG_MAX_UINT_48                 (281474976710655ull)

#define SBG_MIN_INT_56                  (-36028797018963967ll - 1)
#define SBG_MAX_INT_56                  (36028797018963967ll)
#define SBG_MAX_UINT_56                 (72057594037927935ull)

//----------------------------------------------------------------------//
//- Global typedef                                                     -//
//----------------------------------------------------------------------//
typedef uint32_t                        sbgIpAddress;                   /*!< Define an IP v4 address stored in 4 bytes. The format is A.B.C.D, each component is 8 bits and stored in Big Endian. */

//------------------------------------------------------------------//
//- Type punning safe conversion unions                            -//
//------------------------------------------------------------------//

/*!
 * Used to get a uint32_t from a uint8_t array.
 */
typedef union _SbgUint8PtrToUint32Ptr
{
    uint8_t     *m_pointerUint8;                /*!< Set the address used to access the uint32_t. */
    uint32_t    *m_pointerUint32;               /*!< Store the unint32 value. */
} SbgUint8PtrToUint32Ptr;

/*!
 * Union used to convert a buffer or 2 unit8 two's complement values to a int16_t
 */
typedef union _SbgUint8ToInt16
{
    int16_t     value;
    uint8_t     buffer[2];
} SbgUint8ToInt16;

/*!
 * Union used to convert a buffer or 2 unit8 values to a uint16_t
 */
typedef union _SbgUint8ToUint16
{
    uint16_t    value;
    uint8_t     buffer[2];
} SbgUint8ToUint16;

/*!
 * Union used to convert a buffer or 4 unit8 two's complement values to a int32_t
 */
typedef union _SbgUint8ToInt32
{
    int32_t     value;
    uint8_t     buffer[4];
} SbgUint8ToInt32;

/*!
 * Union used to convert a buffer or 4 unit8 values to a uint32_t
 */
typedef union _SbgUint8ToUint32
{
    uint32_t    value;
    uint8_t     buffer[4];
} SbgUint8ToUint32;

/*!
 * Union used to convert a buffer or 8 unit8 two's complement values to a int64_t
 */
typedef union _SbgUint8ToInt64
{
    int64_t     value;
    uint8_t     buffer[8];
} SbgUint8ToInt64;

/*!
 * Union used to convert a buffer or 8 unit8 values to a uint64_t
 */
typedef union _SbgUint8ToUint64
{
    uint64_t    value;
    uint8_t     buffer[8];
} SbgUint8ToUint64;

/*!
 * Union that allows type punning (access to a floating point number bits)
 */
typedef union _SbgFloatNint
{
    float       valF;
    int32_t     valI;
    uint32_t    valU;
} SbgFloatNint;

/*!
 * Union that allows type punning (access to a double number bits)
 */
typedef union _SbgDoubleNint
{
    double      valF;
    uint64_t    valU;
    int64_t     valI;
} SbgDoubleNint;

/*!
 * Set of 3 int32_t
 */
 typedef struct _SbgVector3i
 {
     int32_t    v[3];
 } SbgVector3i;

 /*!
 * Set of 3 int64_t
 */
 typedef struct _SbgVector3ll
 {
     int64_t    v[3];
 } SbgVector3ll;

//----------------------------------------------------------------------//
//- DEPRECATED: types definitions                                      -//
//----------------------------------------------------------------------//
#ifdef SBG_COMMON_USE_DEPRECATED
    SBG_DEPRECATED_TYPEDEF(typedef unsigned char            uint8);
    SBG_DEPRECATED_TYPEDEF(typedef unsigned short           uint16);
    SBG_DEPRECATED_TYPEDEF(typedef unsigned int             uint32);
    SBG_DEPRECATED_TYPEDEF(typedef unsigned long long int   uint64);

    SBG_DEPRECATED_TYPEDEF(typedef signed char              int8);
    SBG_DEPRECATED_TYPEDEF(typedef signed short             int16);
    SBG_DEPRECATED_TYPEDEF(typedef signed int               int32);
    SBG_DEPRECATED_TYPEDEF(typedef signed long long int     int64);

    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8PtrToUint32Ptr    Uint8PtrToUint32Ptr);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8ToInt16           Uint8ToInt16);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8ToUint16          Uint8ToUint16);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8ToInt32           Uint8ToInt32);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8ToUint32          Uint8ToUint32);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8ToInt64           Uint8ToInt64);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgUint8ToUint64          Uint8ToUint64);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgFloatNint              FloatNint);
    SBG_DEPRECATED_TYPEDEF(typedef union _SbgDoubleNint             DoubleNint);
#endif

#endif  // SBG_TYPES_H
