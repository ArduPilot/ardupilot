
#ifndef __AP_PROGMEM_AVR_H__
#define __AP_PROGMEM_AVR_H__

#include <avr/pgmspace.h>

// prog_char_t is used as a wrapper type for prog_char, which is
// a character stored in flash. By using this wrapper type we can
// auto-detect at compile time if a call to a string function is using
// a flash-stored string or not
typedef struct {
    char c;
} prog_char_t;

#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

#undef PSTR
/* Need const type for progmem - new for avr-gcc 4.6 */
#if __AVR__ && __GNUC__ == 4 && __GNUC_MINOR__ > 5 
#define PSTR(s) (__extension__({static const prog_char __c[] PROGMEM = (s); \
                                  (const prog_char_t *)&__c[0]; }))
#else
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); \
                                  (prog_char_t *)&__c[0]; }))
#endif


static inline int strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcasecmp_P(str1, (const prog_char *)pstr);
}

static inline int strcmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcmp_P(str1, (const prog_char *)pstr);
}

static inline size_t strlen_P(const prog_char_t *pstr)
{
    return strlen_P((const prog_char *)pstr);
}

static inline void *memcpy_P(void *dest, const prog_char_t *src, size_t n)
{
    return memcpy_P(dest, (const prog_char *)src, n);
}


static inline char *strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
    return strncpy_P(buffer, (const prog_char *)pstr, buffer_size);
}


// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t pgm_read_pointer(const void *s)
{
    if (sizeof(uintptr_t) == sizeof(uint16_t)) {
        return (uintptr_t)pgm_read_word(s);
    } else {
        union {
            uintptr_t p;
            uint8_t a[sizeof(uintptr_t)];
        } u;
        uint8_t        i;
        for (i=0; i< sizeof(uintptr_t); i++) {
            u.a[i] = pgm_read_byte(i + (const prog_char *)s);
        }
        return u.p;
    }
}

#endif // __AP_PROGMEM_AVR_H__
