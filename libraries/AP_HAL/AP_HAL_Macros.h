#pragma once

/*
  macros to allow code to build on multiple platforms more easily
 */

#ifdef __GNUC__
 #define WARN_IF_UNUSED __attribute__ ((warn_unused_result))
#else
 #define WARN_IF_UNUSED
#endif

// use this to avoid issues between C++11 with NuttX and C++10 on
// other platforms.
#if !(defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L)
# define constexpr const
#endif

#define NORETURN __attribute__ ((noreturn))
