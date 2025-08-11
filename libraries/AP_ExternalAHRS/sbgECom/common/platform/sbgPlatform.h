/*!
 * \file            sbgPlatform.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            March 17, 2015
 *
 * \brief           Platform-specific functions.
 *
 * This file should be modified to each targeted platform.
 * For example, all common headers should be included from this file.
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

#ifndef SBG_PLATFORM_H
#define SBG_PLATFORM_H

// System headers
#ifdef _WIN32
#include <direct.h>
#elif defined(__unix__)
#include <unistd.h>
#endif

// sbgCommonLib headers
#include <sbgDefines.h>
#include <sbgErrorCodes.h>
#include <debug/sbgDebug.h>

//----------------------------------------------------------------------//
//- Function pointer definitions                                       -//
//----------------------------------------------------------------------//

/*!
 * Type for logging functions.
 *
 * \param[in]   pFileName                   File name where the error occurred.
 * \param[in]   pFunctionName               Function name where the error occurred.
 * \param[in]   line                        Line number where the error occurred.
 * \param[in]   pCategory                   Category for this log or "None" if no category has been specified.
 * \param[in]   logType                     Define if we have an error, a warning, an info or a debug log.
 * \param[in]   errorCode                   The error code associated with the message.
 * \param[in]   pMessage                    The message to log.
 */
typedef void (*SbgCommonLibOnLogFunc)(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pMessage);

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

/*!
 * Get the current time.
 *
 * \return                                  The current time, in ms.
 */
SBG_COMMON_LIB_API uint32_t sbgGetTime(void);

/*!
 * Sleep.
 *
 * \param[in]   ms                          Time to wait, in ms.
 */
SBG_COMMON_LIB_API void sbgSleep(uint32_t ms);

/*!
 * Set the log function.
 *
 * Some platforms may not provide the ability to set a user-provided log function, in which
 * case this function does nothing.
 *
 * \param[in]   logCallback                 Log function.
 */
SBG_COMMON_LIB_API void sbgCommonLibSetLogCallback(SbgCommonLibOnLogFunc logCallback);

/*!
 * Log a message.
 *
 * \param[in]   pFileName                   File name where the error occurred.
 * \param[in]   pFunctionName               Function name where the error occurred.
 * \param[in]   line                        Line number where the error occurred.
 * \param[in]   pCategory                   Category for this log or "None" if no category has been specified.
 * \param[in]   logType                     Define if we have an error, a warning, an info or a debug log.
 * \param[in]   errorCode                   The error code associated with the message.
 * \param[in]   pFormat                     The error message that will be used with the variable list of arguments.
 */
SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...) SBG_CHECK_FORMAT(printf, 7, 8);

#endif // SBG_PLATFORM_H
