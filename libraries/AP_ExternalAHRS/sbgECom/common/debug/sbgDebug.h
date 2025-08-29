/*!
 * \file            sbgDebug.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            17 March 2015
 *
 * \brief           Define and handle error logging for the SBG Systems common C library.
 *
 * The methods defined here should be implemented in sbgPlatform.h/sbgPlatform.c
 * according to your platform and needs.
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
#ifndef SBG_DEBUG_H
#define SBG_DEBUG_H

// sbgCommonLib headers
#include <sbgDefines.h>

#ifndef SBG_DEBUG_LOG_CATEGORY
    #define SBG_DEBUG_LOG_CATEGORY      ("None")
#endif

//----------------------------------------------------------------------//
//- Errors and warning definitions                                     -//
//----------------------------------------------------------------------//

/*!
 * Enum that identify the type of error / warning that has been thrown.
 */
typedef enum _SbgDebugLogType
{
    SBG_DEBUG_LOG_TYPE_ERROR,                   /*!< The message to log is an error. */
    SBG_DEBUG_LOG_TYPE_WARNING,                 /*!< The message to log is a warning. */
    SBG_DEBUG_LOG_TYPE_INFO,                    /*!< The message to log is an information. */
    SBG_DEBUG_LOG_TYPE_DEBUG                    /*!< The message to log is a debug information. */
} SbgDebugLogType;

//----------------------------------------------------------------------//
//- Errors and warning macros                                          -//
//----------------------------------------------------------------------//

/*!
 * \brief Log an error with its associated message.
 *
 * This macro logs an error message along with the specified error code. The message can include formatted text similar to printf.
 *
 * \param[in]   errorCode                       The error code that triggered this error.
 * \param[in]   format                          The string literal for the associated error message, supporting printf-like formatting.
 * \param[in]   ...                             Additional arguments for the format string.
 */
#define SBG_LOG_ERROR_CALL(errorCode, format, ...)      sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__func__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_ERROR, errorCode, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_ERROR == 1
    #define SBG_LOG_ERROR                               SBG_LOG_ERROR_CALL
#else
    #define SBG_LOG_ERROR(format, ...)                  ((void)sizeof(SBG_LOG_ERROR_CALL(format, ## __VA_ARGS__), 0))
#endif

/*!
 * \brief Log a warning with its associated message.
 *
 * This macro logs a warning message along with the specified error code. The message can include formatted text similar to printf.
 *
 * \param[in]   errorCode                       The error code that triggered this warning.
 * \param[in]   format                          The string literal for the associated error message, supporting printf-like formatting.
 * \param[in]   ...                             Additional arguments for the format string.
 */
#define SBG_LOG_WARNING_CALL(errorCode, format, ...)    sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__func__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_WARNING, errorCode, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_WARNING == 1
    #define SBG_LOG_WARNING                             SBG_LOG_WARNING_CALL
#else
    #define SBG_LOG_WARNING(format, ...)                ((void)sizeof(SBG_LOG_WARNING_CALL(format, ## __VA_ARGS__), 0))
#endif

/*!
 * \brief Log an information message.
 *
 * This macro logs an information message. The message can include formatted text similar to printf.
 *
 * \param[in]   format                          The string literal for the associated information message, supporting printf-like formatting.
 * \param[in]   ...                             Additional arguments for the format string.
 */
#define SBG_LOG_INFO_CALL(format, ...)                  sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__func__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_INFO, SBG_NO_ERROR, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_INFO == 1
    #define SBG_LOG_INFO                                SBG_LOG_INFO_CALL
#else
    #define SBG_LOG_INFO(format, ...)                   ((void)sizeof(SBG_LOG_INFO_CALL(format, ## __VA_ARGS__), 0))
#endif

/*!
 * \brief Log a a debug message.
 *
 * This macro logs a debug message, which is only output when compiled in debug mode. The message can include formatted text similar to printf.
 *
 * \param[in]   format                          The string literal for the associated debug message, supporting printf-like formatting.
 * \param[in]   ...                             Additional arguments for the format string.
 */
#define SBG_LOG_DEBUG_CALL(format, ...)                 sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__func__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_DEBUG, SBG_NO_ERROR, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_DEBUG == 1
    #define SBG_LOG_DEBUG                               SBG_LOG_DEBUG_CALL
#else
    #define SBG_LOG_DEBUG(format, ...)                  ((void)sizeof(SBG_LOG_DEBUG_CALL(format, ## __VA_ARGS__), 0))
#endif

#endif // SBG_DEBUG_H
