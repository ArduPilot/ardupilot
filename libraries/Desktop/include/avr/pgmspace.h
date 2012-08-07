#ifndef _PGMSPACE_H
#define _PGMSPACE_H

#include <stdarg.h>
#include <unistd.h>
#include <inttypes.h>

#ifndef __ATTR_PROGMEM__
#define __ATTR_PROGMEM__ __attribute__(())
#endif

#ifndef __ATTR_PURE__
#define __ATTR_PURE__ __attribute__((__pure__))
#endif

#undef PROGMEM
#define PROGMEM __ATTR_PROGMEM__

#ifndef PGM_P
#define PGM_P const prog_char *
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef char PROGMEM prog_char;
extern int strcasecmp_P(const char *, PGM_P) __ATTR_PURE__;
extern int strcmp_P(const char *, PGM_P) __ATTR_PURE__;
extern int strncmp_P(const char *, PGM_P, size_t n) __ATTR_PURE__;
extern size_t strlcat_P (char *, PGM_P, size_t );
extern size_t strnlen_P (PGM_P, size_t );
extern size_t strlen_P (PGM_P);
extern size_t strlen_P (PGM_P);
extern char *strncpy_P(char *dest, PGM_P src, size_t n);
extern void *memcpy_P(void *dest, PGM_P src, size_t n);

static inline uint8_t pgm_read_byte(PGM_P s) { return (uint8_t)*s; }
	static inline uint8_t pgm_read_byte_far(const void *s) { return *(const uint8_t *)s; }
static inline uint16_t pgm_read_word(const void *s) { return *(const uint16_t *)s; }
static inline uint32_t pgm_read_dword(const void *s) { return *(const uint32_t *)s; }
static inline float    pgm_read_float(const void *s) { return *(const float *)s; }

#define GETBYTE(flag, mask, pnt)        ({      \
       unsigned char __c;                       \
       __c = ((flag) & (mask))                  \
             ? pgm_read_byte(pnt) : *pnt;       \
       pnt++;                                   \
       __c;                                     \
})


#ifdef __cplusplus
}
#endif

#endif
