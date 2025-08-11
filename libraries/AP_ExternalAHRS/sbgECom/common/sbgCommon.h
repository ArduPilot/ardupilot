/*!
 * \file            sbgCommon.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            March 17, 2015
 *
 * \brief           Main header for the SBG Systems common C library.
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

/*!
 * \defgroup    common Common
 * \brief       SBG Systems foundation framework that is common to all C/C++ projects.
 */

#ifndef SBG_COMMON_H
#define SBG_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sbgConfig.h"

//----------------------------------------------------------------------//
//- Default configuration                                              -//
//----------------------------------------------------------------------//

/*!
 * If set to 0, the platform support only aligned memory access.
 * If set to 1, the platform support unaligned memory access.
 * Default: Support only Aligned access - Disabled
 */
#ifndef SBG_CONFIG_UNALIGNED_ACCESS_AUTH
#define SBG_CONFIG_UNALIGNED_ACCESS_AUTH                (0)
#endif

/*!
 * If set to 0, the platform is using little endian.
 * If set to 1, the platform is using big endian.
 * Default: Little Endian - Disabled
 */
#ifndef SBG_CONFIG_BIG_ENDIAN
#define SBG_CONFIG_BIG_ENDIAN                           (0)
#endif

/*!
 * If set to 1, error logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_ERROR
#define SBG_CONFIG_ENABLE_LOG_ERROR                     (1)
#endif

/*!
 * If set to 1, warning logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_WARNING
#define SBG_CONFIG_ENABLE_LOG_WARNING                   (1)
#endif

/*!
 * If set to 1, information logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_INFO
#define SBG_CONFIG_ENABLE_LOG_INFO                      (1)
#endif

/*!
 * If set to 1, debug logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_DEBUG
#define SBG_CONFIG_ENABLE_LOG_DEBUG                     (1)
#endif

/*!
 * Maximum error message size in bytes that can be generated including the NULL Char.
 * Default: 1024
 */
#ifndef SBG_CONFIG_LOG_MAX_SIZE
#define SBG_CONFIG_LOG_MAX_SIZE                         ((size_t)(1024))
#endif

/*!
 * Maximum number of chars for a file name including the NULL char.
 * Default: 256
 */
#ifndef SBG_CONFIG_PATH_MAX_SIZE
#define SBG_CONFIG_PATH_MAX_SIZE                        ((size_t)(256))
#endif

//----------------------------------------------------------------------//
//- Headers                                                            -//
//----------------------------------------------------------------------//

#include "sbgDefines.h"
#include "sbgErrorCodes.h"
#include "sbgTypes.h"

#include "debug/sbgDebug.h"
#include "platform/sbgPlatform.h"


#ifdef __cplusplus
}
#endif

#endif // SBG_COMMON_H
