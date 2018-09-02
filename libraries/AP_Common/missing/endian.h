#if defined(HAVE_ENDIAN_H) && HAVE_ENDIAN_H
#include_next <endian.h>
#else

/* minimal version defining only the macros we need in our codebase */

#define __LITTLE_ENDIAN  1234
#define __BIG_ENDIAN     4321

#ifdef __ARMEB__
#define __BYTE_ORDER     __BIG_ENDIAN
#else
#define __BYTE_ORDER     __LITTLE_ENDIAN
#endif

#endif
