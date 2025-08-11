/*!
 * \file            sbgEComLib.h
 * \ingroup         main
 * \author          SBG Systems
 * \date            05 February 2013
 *
 * \brief           Main header file for the SBG Systems Enhanced Communication Library.
 *
 * Only this main header file should be included to use the library.
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

#ifndef SBG_ECOM_LIB_H
#define SBG_ECOM_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

// sbgCommonLib headers
#include <sbgCommon.h>
#include <crc/sbgCrc.h>
#include <interfaces/sbgInterface.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComIds.h"
#include "logs/sbgEComLog.h"
#include "protocol/sbgEComProtocol.h"

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif  // SBG_ECOM_LIB_H
