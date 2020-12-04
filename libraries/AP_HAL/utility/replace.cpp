/*
  replacement functions for systems that are missing required library functions
 */

#include "replace.h"

#ifndef HAVE_MEMRCHR
/*
  replacement for memrchr(). Note that we make the buffer non-const to
  avoid issues with converting const to non-const
 */
void *replace_memrchr(void *s, int c, size_t n)
{
    uint8_t *b = (uint8_t *)s;
    for (int32_t i=n-1; i>=0; i--) {
        if (b[i] == (uint8_t)c) {
            return (void *)&b[i];
        }
    }
    return nullptr;
}
#endif
