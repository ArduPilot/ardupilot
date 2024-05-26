/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_STD_HPP_INCLUDED
#define UAVCAN_STD_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <cstdarg>
#include <cstddef>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

# include <cstdint>
# include <cstdio>

namespace uavcan
{

typedef std::uint8_t uint8_t;
typedef std::uint16_t uint16_t;
typedef std::uint32_t uint32_t;
typedef std::uint64_t uint64_t;

typedef std::int8_t int8_t;
typedef std::int16_t int16_t;
typedef std::int32_t int32_t;
typedef std::int64_t int64_t;

}

#else

# include <stdint.h>  // Standard integer types from C library
# include <stdio.h>   // vsnprintf() from the C library

namespace uavcan
{

typedef ::uint8_t uint8_t;
typedef ::uint16_t uint16_t;
typedef ::uint32_t uint32_t;
typedef ::uint64_t uint64_t;

typedef ::int8_t int8_t;
typedef ::int16_t int16_t;
typedef ::int32_t int32_t;
typedef ::int64_t int64_t;

}

#endif

namespace uavcan
{
/**
 * Wrapper over the standard snprintf(). This wrapper is needed because different standards and different
 * implementations of C++ do not agree whether snprintf() should be defined in std:: or in ::. The solution
 * is to use 'using namespace std' hack inside the wrapper, so the compiler will be able to pick whatever
 * definition is available in the standard library. Alternatively, the user's application can provide a
 * custom implementation of uavcan::snprintf().
 */
#if __GNUC__
__attribute__ ((format(printf, 3, 4)))
#endif
extern int snprintf(char* out, std::size_t maxlen, const char* format, ...);

#if !UAVCAN_USE_EXTERNAL_SNPRINTF
inline int snprintf(char* out, std::size_t maxlen, const char* format, ...)
{
    using namespace std;  // This way we can pull vsnprintf() either from std:: or from ::.
    va_list args;
    va_start(args, format);
    const int return_value = ::vsnprintf(out, maxlen, format, args);
    va_end(args);
    return return_value;
}
#endif

}

#endif // UAVCAN_STD_HPP_INCLUDED
