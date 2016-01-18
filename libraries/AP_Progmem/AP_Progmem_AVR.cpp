
/* Only build this file for AVR - it exists to provide overrides for
 * broken avr libc */
#ifdef __AVR__

#include <string.h>
#include "AP_Progmem.h"

// strlcat_P() in AVR libc seems to be broken
size_t strlcat_P(char *d, const prog_char_t *s, size_t bufsize)
{
    size_t len1 = strlen(d);
    size_t len2 = strlen_P(s);
    size_t ret  = len1 + len2;

    if (len1+len2 >= bufsize) {
        if (bufsize < (len1+1)) {
            return ret;
        }
        len2 = bufsize - (len1+1);
    }
    if (len2 > 0) {
        memcpy_P(d+len1, s, len2);
        d[len1+len2] = 0;
    }
    return ret;
}

#endif // __AVR__
