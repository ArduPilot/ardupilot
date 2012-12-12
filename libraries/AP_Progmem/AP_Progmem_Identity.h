
#ifndef __AP_PROGMEM_IDENTITY__
#define __AP_PROGMEM_IDENTITY__

#include <string.h>
#include <stdint.h>

typedef char prog_char_t;
typedef char prog_char;

#define PSTR(s) s
#undef PROGMEM
#define PROGMEM __attribute__(())

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

static inline size_t strnlen_P(const prog_char_t *str, size_t size)
{
	return strnlen(str, size);
}

static inline int strncmp_P(const prog_char_t *str1, const prog_char_t *str2, size_t n)
{
	return strncmp(str1, str2, n);
}


// read something the size of a byte
static inline uint8_t pgm_read_byte(const void *s) {
	return *(const uint8_t *)s;
}

// read something the size of a byte, far version
static inline uint8_t pgm_read_byte_far(const void *s) {
	return *(const uint8_t *)s;
}

// read something the size of a word
static inline uint16_t pgm_read_word(const void *s) {
	return *(const uint16_t *)s;
}

// read something the size of a dword
static inline uint32_t pgm_read_dword(const void *s) {
	return *(const uint32_t *)s;
}

// read something the size of a float
static inline float pgm_read_float(const void *s) {
	return *(const float *)s;
}

// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t pgm_read_pointer(const void *s) {
	return *(const uintptr_t *)s;
}

#endif // __AP_PROGMEM_IDENTITY__

