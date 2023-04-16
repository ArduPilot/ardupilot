/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
///
/// @file       AP_Common.h
/// @brief		Common definitions and utility routines for the ArduPilot
///				libraries.
///

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <type_traits>

// used to pack structures
#define PACKED __attribute__((__packed__))

// used to weaken symbols
#define WEAK __attribute__((__weak__))

// used to mark a function that may be unused in some builds
#define UNUSED_FUNCTION __attribute__((unused))

// used to mark an attribute that may be unused in some builds
#ifdef __clang__
#define UNUSED_PRIVATE_MEMBER __attribute__((unused))
#else
#define UNUSED_PRIVATE_MEMBER
#endif

// this can be used to optimize individual functions
#define OPTIMIZE(level) __attribute__((optimize(level)))

// sometimes we need to prevent inlining to prevent large stack usage
#ifndef NOINLINE
#define NOINLINE __attribute__((noinline))
#endif

// used to ignore results for functions marked as warn unused
#define IGNORE_RETURN(x) do {if (x) {}} while(0)

#define FMT_PRINTF(a,b) __attribute__((format(printf, a, b)))
#define FMT_SCANF(a,b) __attribute__((format(scanf, a, b)))

// used to forbid copy of objects
#define CLASS_NO_COPY(c) c(const c &other) = delete; c &operator=(const c&) = delete

#ifdef __has_cpp_attribute
#  if __has_cpp_attribute(fallthrough)
#    define FALLTHROUGH [[fallthrough]]
#  elif __has_cpp_attribute(gnu::fallthrough)
#    define FALLTHROUGH [[gnu::fallthrough]]
#  endif
#endif
#ifndef FALLTHROUGH
#  define FALLTHROUGH
#endif

#ifdef __GNUC__
 #define WARN_IF_UNUSED __attribute__ ((warn_unused_result))
#else
 #define WARN_IF_UNUSED
#endif

#define NORETURN __attribute__ ((noreturn))

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

/* Declare and implement const and non-const versions of the array subscript
 * operator. The object is treated as an array of type_ values. */
#define DEFINE_BYTE_ARRAY_METHODS                                                                   \
    inline uint8_t &operator[](size_t i) { return reinterpret_cast<uint8_t *>(this)[i]; }           \
    inline uint8_t operator[](size_t i) const { return reinterpret_cast<const uint8_t *>(this)[i]; }

/*
  check if bit bitnumber is set in value, returned as a
  bool. Bitnumber starts at 0 for the first bit
 */
#define BIT_IS_SET(value, bitnumber) (((value) & (1U<<(bitnumber))) != 0)
#define BIT_IS_SET_64(value, bitnumber) (((value) & (uint64_t(1U)<<(bitnumber))) != 0)

// get high or low bytes from 2 byte integer
#define LOWBYTE(i) ((uint8_t)(i))
#define HIGHBYTE(i) ((uint8_t)(((uint16_t)(i))>>8))

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

#define UINT16_VALUE(hbyte, lbyte) (static_cast<uint16_t>(((hbyte)<<8)|(lbyte)))
#define UINT32_VALUE(b3, b2, b1, b0) (static_cast<uint32_t>(((b3)<<24)|((b2)<<16)|((b1)<<8)|(b0)))

/*
 * See UNUSED_RESULT. The difference is that it receives @uniq_ as the name to
 * be used for its internal variable.
 *
 * @uniq_: a unique name to use for variable name
 * @expr_: the expression to be evaluated
 */
#define _UNUSED_RESULT(uniq_, expr_)                      \
    do {                                                  \
        decltype(expr_) uniq_ __attribute__((unused));    \
        uniq_ = expr_;                                    \
    } while (0)

/*
 * Allow to call a function annotated with warn_unused_result attribute
 * without getting a warning, because sometimes this is what we want to do.
 *
 * @expr_: the expression to be evaluated
 */
#define UNUSED_RESULT(expr_) _UNUSED_RESULT(__unique_name_##__COUNTER__, expr_)

// @}

// STR_VALUE returns the string equivalent for the passed cpp macro, so e.g.
// printf("%s", STR_VALUE(EINVAL)); will print "EINVAL"
#define STR_VALUE(x) #x

// assert_storage_size template: assert that the memory used to store an
// item is of a specific size.
// example invocation:
// assert_storage_size<class Location, 16> _assert_storage_size_Location;
// templates are used for this because the compiler's output will
// usually contain details of the template instantiation so you can
// see how the actual size differs from the expected size.
template<typename s, size_t s_size, size_t t> struct _assert_storage_size {
    static_assert(s_size == t, "wrong size");
};
template<typename s, size_t t> struct assert_storage_size {
    _assert_storage_size<s, sizeof(s), t> _member;
};

////////////////////////////////////////////////////////////////////////////////
/// @name	Conversions
///
/// Conversion macros and factors.
///
//@{

/*
  Return true if value is between lower and upper bound inclusive.
  False otherwise.
*/
bool is_bounded_int32(int32_t value, int32_t lower_bound, int32_t upper_bound);

bool hex_to_uint8(uint8_t a, uint8_t &res);  // return the uint8 value of an ascii hex character

/*
  strncpy without the warning for not leaving room for nul termination
 */
size_t strncpy_noterm(char *dest, const char *src, size_t n);

// return the numeric value of an ascii hex character
int16_t char_to_hex(char a);

/*
  Bit manipulation
 */
//#define BIT_SET(value, bitnumber) ((value) |= (((typeof(value))1U) << (bitnumber)))
template <typename T> void BIT_SET (T& value, uint8_t bitnumber) noexcept {
     static_assert(std::is_integral<T>::value, "Integral required.");
     ((value) |= ((T)(1U) << (bitnumber)));
 }
//#define BIT_CLEAR(value, bitnumber) ((value) &= ~(((typeof(value))1U) << (bitnumber)))
template <typename T> void BIT_CLEAR (T& value, uint8_t bitnumber) noexcept {
     static_assert(std::is_integral<T>::value, "Integral required.");
     ((value) &= ~((T)(1U) << (bitnumber)));
 }
