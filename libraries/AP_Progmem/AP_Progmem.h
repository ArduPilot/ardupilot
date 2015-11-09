#pragma once

#include <string.h>
#include <stdint.h>

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

// read something the size of a pointer. This makes the menu code more
// portable
static inline void pgm_read_block(const void *s, void *dest, uint8_t len) {
    memcpy(dest, s, len);
}
