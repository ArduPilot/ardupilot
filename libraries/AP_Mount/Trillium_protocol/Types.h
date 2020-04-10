#ifndef TYPES_H
#define TYPES_H

// All of our current platforms support the standard int header, so we can just
// use that to remove some of the pain of standard bit width determination
#include <stdint.h>

#ifdef WIN32
# include <WTypes.h>
#else
typedef unsigned long BOOL;
#endif

#ifndef TRUE
# define TRUE  (BOOL)1
#endif // TRUE

#ifndef FALSE
# define FALSE (BOOL)0
#endif // FALSE

#ifndef NULL
# define NULL        0
#endif // NULL

#ifndef UINT8_MAX
typedef   signed char  int8_t;
typedef unsigned char uint8_t;
#endif // !UINT8_MAX

typedef int8_t   SInt8;
typedef uint8_t  UInt8;

typedef int16_t  SInt16;
typedef uint16_t UInt16;

typedef int32_t  SInt24;
typedef uint32_t UInt24;

typedef int32_t  SInt32;
typedef uint32_t UInt32;

#ifdef UINT64_MAX
typedef int64_t  SInt64;
typedef uint64_t UInt64;
#endif // UINT64_MAX

#endif // TYPES_H
