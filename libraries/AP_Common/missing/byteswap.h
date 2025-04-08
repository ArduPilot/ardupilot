#if defined(HAVE_BYTESWAP_H) && HAVE_BYTESWAP_H
#include_next <byteswap.h>
#else

#include <inttypes.h>

/* minimal version defining only the macros we need in our codebase */

static inline uint16_t __bswap_16(uint16_t u)
{
    return (uint16_t) __builtin_bswap16(u);
}

static inline uint32_t __bswap_32(uint32_t u)
{
    return (uint32_t) __builtin_bswap32(u);
}

static inline uint64_t __bswap_64(uint64_t u)
{
    return (uint64_t) __builtin_bswap64(u);
}

#endif
