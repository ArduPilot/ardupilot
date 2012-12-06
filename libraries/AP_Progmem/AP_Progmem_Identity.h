
#ifndef __AP_PROGMEM_IDENTITY__
#define __AP_PROGMEM_IDENTITY__

#include <string.h>

typedef char prog_char_t;
typedef char prog_char;

#define PSTR(s) s

static inline int strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcasecmp(str1, pstr);
}

static inline int strcmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcmp(str1, pstr);
}

static inline size_t strlen_P(const prog_char_t *pstr)
{
    return strlen(pstr);
}

static inline void *memcpy_P(void *dest, const prog_char_t *src, size_t n)
{
    return memcpy(dest, src, n);
}


static inline char *strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
    return strncpy(buffer, pstr, buffer_size);
}


// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t pgm_read_pointer(const void *s) {
	return *(const uintptr_t *)s;
}

#endif // __AP_PROGMEM_IDENTITY__

