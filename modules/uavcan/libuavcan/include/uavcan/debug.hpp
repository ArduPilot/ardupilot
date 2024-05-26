/*
 * Debug stuff, should only be used for library development.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_DEBUG_HPP_INCLUDED
#define UAVCAN_DEBUG_HPP_INCLUDED

#include <uavcan/build_config.hpp>

#if UAVCAN_DEBUG

# include <cstdio>
# include <cstdarg>

# if __GNUC__
__attribute__ ((format(printf, 2, 3)))
# endif
static void UAVCAN_TRACE(const char* src, const char* fmt, ...)
{
    va_list args;
    (void)std::printf("UAVCAN: %s: ", src);
    va_start(args, fmt);
    (void)std::vprintf(fmt, args);
    va_end(args);
    (void)std::puts("");
}

#else

# define UAVCAN_TRACE(...) ((void)0)

#endif

#endif // UAVCAN_DEBUG_HPP_INCLUDED
