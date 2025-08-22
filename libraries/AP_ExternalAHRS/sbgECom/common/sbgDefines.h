/*!
 * \file            sbgDefines.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            17 March 2015
 *
 * \brief           Header file that contains all common definitions.
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

#ifndef SBG_DEFINES_H
#define SBG_DEFINES_H

// Standard headers
#include <assert.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// Local headers
#include "sbgConfig.h"

//
// XXX If NDEBUG is defined, most libraries define assert() as ((void)0), which may
// cause "defined but not used" warnings. Redefine assert() in a way that safely
// prevents this warning, i.e. without triggering the expression side effects.
//
#ifdef NDEBUG
#undef assert
#define assert(expression) ((void)sizeof(expression))
#endif // NDEBUG

/*!
 * Macro used to handle export and import methods of the sbgCommon library
 */
#ifdef _MSC_VER
    #ifdef SBG_COMMON_STATIC_USE
        #define SBG_COMMON_LIB_API
    #else
        #ifdef SBG_COMMON_LIB_API_EXPORT
            #define SBG_COMMON_LIB_API __declspec(dllexport)
        #else
            #define SBG_COMMON_LIB_API __declspec(dllimport)
        #endif
    #endif
#else
    #define SBG_COMMON_LIB_API
#endif

//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//
#ifndef SBG_DISABLE
    #define SBG_DISABLE (0)
#endif

#ifndef SBG_ENABLE
    #define SBG_ENABLE  (1)
#endif

#ifndef FALSE
    #define FALSE        (0)
#endif

#ifndef TRUE
    #define TRUE        (1)
#endif

#ifndef NULL
    #define NULL        (0)
#endif

#ifndef SBG_INVALID_HANDLE
    #define SBG_INVALID_HANDLE  (NULL)
#endif

#ifndef SBG_ARRAY_SIZE
    #define SBG_ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#endif

#ifndef SBG_STRLEN
    #define SBG_STRLEN(s)       (sizeof(s) - 1)
#endif

#ifndef SBG_QUOTE_NX
    #define SBG_QUOTE_NX(x)     #x
#endif

#ifndef SBG_QUOTE
    #define SBG_QUOTE(x)        SBG_QUOTE_NX(x)
#endif

#ifndef SBG_CONCAT_NX
    #define SBG_CONCAT_NX(a, b) a ## b
#endif

#ifndef SBG_CONCAT
    #define SBG_CONCAT(a, b)    SBG_CONCAT_NX(a, b)
#endif

#ifndef SBG_UNPACK
    #define SBG_UNPACK(...) __VA_ARGS__
#endif

#ifndef SBG_CONTAINER_OF
    #define SBG_CONTAINER_OF(ptr, type, member)     ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

#ifndef SBG_LIKELY
    #if defined(__GNUC__) || defined(__clang__)
        #define SBG_LIKELY(expr)        __builtin_expect((bool)(expr), true)
    #else
        #define SBG_LIKELY(expr)        (expr)
    #endif
#endif

#ifndef SBG_UNLIKELY
    #if defined(__GNUC__) || defined(__clang__)
        #define SBG_UNLIKELY(expr)      __builtin_expect((bool)(expr), false)
    #else
        #define SBG_UNLIKELY(expr)      (expr)
    #endif
#endif

#ifndef SBG_CHECK_FORMAT
    #if defined(__GNUC__) || defined(__clang__)
        #define SBG_CHECK_FORMAT(style, format_index, va_args)  __attribute__((format(style, format_index, va_args)))
    #else
        #define SBG_CHECK_FORMAT(style, format_index, va_args)
    #endif
#endif

/*!
 * GCC typeof C extension
 * 
 * XXX Visual C (not C++) doesn't provide anything to implement typeof().
 * As a result, this macro is private and shouldn't be relied on.
 */
#ifndef __SBG_TYPEOF
    #ifdef __cplusplus
        #define __SBG_TYPEOF(x)         decltype(x)
    #elif defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__)
        #define __SBG_TYPEOF(x)         typeof(x)
    #endif
#endif

#ifndef SBG_CONST_CAST_AA
    #ifdef __SBG_TYPEOF
        #define SBG_CONST_CAST_AA(x)    ((const __SBG_TYPEOF((x)[0][0])(*)[SBG_ARRAY_SIZE((x)[0])])(x))
    #else
        #define SBG_CONST_CAST_AA(x)    x
    #endif
#endif

#ifndef SBG_CONST_CAST_PP
    #ifdef __SBG_TYPEOF
        #define SBG_CONST_CAST_PP(x)    ((const __SBG_TYPEOF(**(x))**)(x))
    #else
        #define SBG_CONST_CAST_PP(x)    x
    #endif
#endif

/*!
 * __BASE_FILE__ is gcc specific
 */
#if !defined(__GNUC__) && !defined(__clang__)
#ifndef __BASE_FILE__
    #define __BASE_FILE__ __FILE__
#endif
#endif

#ifdef __cplusplus
    #define SBG_DELETE(p)       if (p){delete (p); (p) = NULL;}
    #define SBG_DELETE_ARRAY(p) if (p){delete[] (p); (p) = NULL;}
    #define SBG_FREE(p)         if (p){free(p); (p) = NULL;}
    #define SBG_FREE_ARRAY(p)   if (p){free(p); (p) = NULL;}
#else
    #define SBG_DELETE          if (p){free(p); (p) = NULL;}
    #define SBG_DELETE_ARRAY    if (p){free(p); (p) = NULL;}
    #define SBG_FREE(p)         if (p){free(p); (p) = NULL;}
    #define SBG_FREE_ARRAY(p)   if (p){free(p); (p) = NULL;}
#endif

//----------------------------------------------------------------------//
//- Compiler definitions                                               -//
//----------------------------------------------------------------------//

/*!
 * Macro used to abstract the compiler specific inline keyword.
 */
#ifndef SBG_INLINE
    #if defined(_MSC_VER)
        #define SBG_INLINE          __inline
    #else
        #define SBG_INLINE          static inline
    #endif
#endif

/*!
 * Macro used to avoid compiler warning when a variable is not used.
 */
#ifndef SBG_UNUSED_PARAMETER
    #define SBG_UNUSED_PARAMETER(x)     (void)(x)
#endif

/*!
 * Compiler independent switch/case fallthrough attribute
 *
 * The fallthrough attribute is used to avoid compiler warning in swith case statements
 * when an intentional break is missing
 */
#ifndef SBG_FALLTHROUGH
    #if defined(__cplusplus) && __cplusplus >= 201703L
        #define SBG_FALLTHROUGH         [[fallthrough]]                 /* introduced in C++ 17 */
    #elif defined(__GNUC__) || defined(__clang__)
        #if (__GNUC__ >= 7) || defined(__clang__)
            #define SBG_FALLTHROUGH         __attribute__((fallthrough))
        #else
            #define SBG_FALLTHROUGH         ((void)0)
        #endif
    #else
        #define SBG_FALLTHROUGH
    #endif
#endif

//----------------------------------------------------------------------//
//- Macro used to defined packed structures                            -//
//----------------------------------------------------------------------//

/*!
 * Compiler independent struct members packing attribute
 * 
 * This macro is used to define a new section of packed structures.
 * All structures defined after this macro will be packed.
 */
#if defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__)
    #define SBG_BEGIN_PACKED()
#elif defined(_MSC_VER)
    #define SBG_BEGIN_PACKED()  __pragma(pack(push, 1))
#else
    #error you must byte-align these structures with the appropriate compiler directives
#endif

/*!
 * Compiler independent struct members packing attribute
 * 
 * This macro is used to specify that a structure is packed.
 */
#if defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__)
    #define SBG_PACKED          __attribute__((packed))
#elif defined(_MSC_VER)
    #define SBG_PACKED
#else
    #error you must byte-align these structures with the appropriate compiler directives
#endif

/*!
 * Compiler independent struct members packing attribute
 * 
 * This macro is used to close the section of packed structures and return to the default packing.
 */
#if defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__)
    #define SBG_END_PACKED()
#elif defined(_MSC_VER)
    #define SBG_END_PACKED()    __pragma(pack(pop))
#else
    #error you must byte-align these structures with the appropriate compiler directives
#endif

//----------------------------------------------------------------------//
//- Deprecation definitions                                            -//
//----------------------------------------------------------------------//

/*!
 * Macro used to indicate that a function is deprecated.
 */
#if defined(__GNUC__) || defined(__clang__)
    #define SBG_DEPRECATED(func) __attribute__((deprecated)) func
#elif defined(__TI_COMPILER_VERSION__)
    #define SBG_DEPRECATED(func) func __attribute__((deprecated))
#elif defined(_MSC_VER)
    #define SBG_DEPRECATED(func) __declspec(deprecated) func
#else
    #define SBG_DEPRECATED(func) func
#endif

/*!
 * Macro used to indicate that a macro is deprecated.
 */
#if defined(__GNUC__) || defined(__clang__)
    #define SBG_DEPRECATED_MACRO(func) __pragma(deprecated(func))
#elif defined(_MSC_VER)
#define SBG_DEPRECATED_MACRO(func) __pragma(deprecated(func))
#else
    #define SBG_DEPRECATED_MACRO(func) func
#endif

/*!
 * Set the default value of SBG_CONFIG_WARN_ABOUT_DEPRECATED_TYPES.
 */
#ifndef SBG_CONFIG_WARN_ABOUT_DEPRECATED_TYPES
#define SBG_CONFIG_WARN_ABOUT_DEPRECATED_TYPES (1)
#endif

/*!
 * Macro used to indicate that a type definition is deprecated.
 *
 * XXX In order to avoid excessive noise caused by deprecation warnings, the attribute
 * may currently be disabled by defining SBG_CONFIG_WARN_ABOUT_DEPRECATED_TYPES to 0.
 */
#if SBG_CONFIG_WARN_ABOUT_DEPRECATED_TYPES != 0
    #if defined(__GNUC__) || defined(__clang__)
        #define SBG_DEPRECATED_TYPEDEF(decl) decl __attribute__((deprecated))
    #elif defined(_MSC_VER)
        #define SBG_DEPRECATED_TYPEDEF(decl) __declspec(deprecated) decl
    #else
        #define SBG_DEPRECATED_TYPEDEF(decl) decl
    #endif
#else
    #define SBG_DEPRECATED_TYPEDEF(decl) decl
#endif

//----------------------------------------------------------------------//
//- Basic maths definitions                                            -//
//----------------------------------------------------------------------//
#ifndef SBG_PI
    #define SBG_PI 3.14159265358979323846
#endif

#ifndef SBG_PI_F
    #define SBG_PI_F 3.14159265358979323846f
#endif

/*!
 * Returns the absolute value of x.
 *
 * \param[in]   x                   Signed integer value.
 * \return                          The absolute value of x.
 */
#ifndef sbgAbs
    #define sbgAbs(x)               (((x) < 0) ? -(x) : (x))
#endif

/*!
 * Returns the maximum between a and b
 * 
 * \param[in]   a                   First operand.
 * \param[in]   b                   Second operand.
 * \return                          The maximum between a and b.
 */
#ifndef sbgMax
    #define sbgMax(a,b)            (((a) > (b)) ? (a) : (b))
#endif

/*!
 * Returns the minimum between a and b
 * 
 * \param[in]   a                   First operand.
 * \param[in]   b                   Second operand.
 * \return                          The minimum between a and b.
 */
#ifndef sbgMin
    #define sbgMin(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/*!
 * Clamp a value between minValue and maxValue ie minValue <= value <= maxValue
 * 
 * \param[in]   value               First operand.
 * \param[in]   minValue            First operand.
 * \param[in]   maxValue            Second operand.
 * \return                          The clamped value.
 */
#ifndef sbgClamp
    #define sbgClamp(value, minValue, maxValue)            (((value) < (minValue))?(minValue): ((value) > (maxValue)?maxValue:value))
#endif

/*!
 * Integer division with a result rounded up.
 * 
 * \param[in]   n                   Dividend.
 * \param[in]   d                   Divisor.
 * \return                          Rounded division
 */
#ifndef sbgDivCeil
    #define sbgDivCeil(n, d)        (((n) + (d) - 1) / (d))
#endif

/*!
 * Convert an angle from radians to degrees using double precision.
 *
 * \param[in]   angle               The angle to convert in radians.
 * \return                          The converted angle in degrees.
 */
SBG_INLINE double sbgRadToDegd(double angle)
{
    return angle * 180.0 / SBG_PI;
}

/*!
 * Convert an angle from degrees to radians using double precision.
 *
 * \param[in]   angle               The angle to convert in degrees.
 * \return                          The converted angle in radians.
 */
SBG_INLINE double sbgDegToRadd(double angle)
{
    return angle * SBG_PI / 180.0;
}

/*!
 * Convert an angle from radians to degrees using single (float) precision.
 *
 * \param[in]   angle               The angle to convert in radians.
 * \return                          The converted angle in degrees.
 */
SBG_INLINE float sbgRadToDegf(float angle)
{
    return angle * 180.0f / SBG_PI_F;
}

/*!
 * Convert an angle from degrees to radians using single (float) precision.
 *
 * \param[in]   angle               The angle to convert in degrees.
 * \return                          The converted angle in radians.
 */
SBG_INLINE float sbgDegToRadf(float angle)
{
    return angle * SBG_PI_F / 180.0f;
}

/*!
 * Test if two floating single-point numbers are equals or not.
 *
 * \param[in]   leftValue               The first operand to test for equality.
 * \param[in]   rightValue              The second operand to test for equality.
 * \return                              true if both left and right operands are almost equal.
 */
SBG_INLINE bool sbgAlmostEqualsFloat(float leftValue, float rightValue)
{
    //
    // The IEEE standard says that any comparison operation involving a NAN must return false.
    //
    if (isnan(leftValue) || isnan(rightValue))
    {
        return false;
    }

    //
    // This method is not good enough and should be updated using DistanceBetweenSignAndMagnitudeNumbers methods
    //
    if (fabsf(leftValue - rightValue) < FLT_EPSILON)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*!
 * Test if two floating double-point numbers are equals or not using the epsilon technique
 *
 * \param[in]   leftValue               The first operand to test for equality.
 * \param[in]   rightValue              The second operand to test for equality.
 * \return                              true if both left and right operands are almost equal.
 */
SBG_INLINE bool sbgAlmostEqualsDouble(double leftValue, double rightValue)
{
    //
    // The IEEE standard says that any comparison operation involving a NAN must return false.
    //
    if (isnan(leftValue) || isnan(rightValue))
    {
        return false;
    }

    //
    // This method is not good enough and should be updated using DistanceBetweenSignAndMagnitudeNumbers methods
    //
    if (fabs(leftValue - rightValue) < DBL_EPSILON)
    {
        return true;
    }
    else
    {
        return false;
    }
}

#endif  // SBG_DEFINES_H
