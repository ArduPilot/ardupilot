#ifndef WIRISH_STRING_h
#define WIRISH_STRING_h

#include <stddef.h>
#include <stdint.h>


//typedef char prog_char;


// prog_char_t is used as a wrapper type for prog_char, which is
// a character stored in flash. By using this wrapper type we can
// auto-detect at compile time if a call to a string function is using
// a flash-stored string or not
/*
 typedef struct {
	char c;
} prog_char_t;
*/
/*
//funzioni non supportate
inline char pgm_read_byte(const prog_char * buff) {return buff[0]; }
inline uint16_t pgm_read_word(const void *s) { return *(const uint16_t *)s; }
inline uint8_t pgm_read_byte_far(const void *s) { return *(const uint8_t *)s; }
static inline float    pgm_read_float(const void *s) { return *(const float *)s; }
*/
#endif
