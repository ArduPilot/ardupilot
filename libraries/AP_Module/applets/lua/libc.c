/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n]) {
        n++;
    }
    return n;
}
int strcmp(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return (unsigned char)*a - (unsigned char)*b;
}
int strncmp(const char *a, const char *b, size_t n)
{
    while (n && *a && *a == *b) {
        a++;
        b++;
        n--;
    }
    return n ? (unsigned char)*a - (unsigned char)*b : 0;
}
int strcoll(const char *a, const char *b)
{
    return strcmp(a, b);
}
char *strchr(const char *s, int c)
{
    do {
        if (*s == (char)c) {
            return (char *)s;
        }
    } while (*s++);
    return NULL;
}
char *strrchr(const char *s, int c)
{
    const char *last = NULL;
    do {
        if (*s == (char)c) {
            last = s;
        }
    } while (*s++);
    return (char *)last;
}
char *strstr(const char *s, const char *needle)
{
    size_t n = strlen(needle);
    for (; *s; s++) {
        if (strncmp(s, needle, n) == 0) {
            return (char *)s;
        }
    }
    return n ? NULL : (char *)s;
}
char *strpbrk(const char *s, const char *set)
{
    for (; *s; s++) {
        if (strchr(set, *s)) {
            return (char *)s;
        }
    }
    return NULL;
}
size_t strspn(const char *s, const char *set)
{
    size_t n = 0;
    while (s[n] && strchr(set, s[n])) {
        n++;
    }
    return n;
}
size_t strcspn(const char *s, const char *set)
{
    size_t n = 0;
    while (s[n] && !strchr(set, s[n])) {
        n++;
    }
    return n;
}
char *strcpy(char *dst, const char *src)
{
    char *start = dst;
    while ((*dst++ = *src++)) {}
    return start;
}
void *memchr(const void *p, int c, size_t n)
{
    const unsigned char *s = p;
    for (size_t i = 0; i < n; i++) {
        if (s[i] == (unsigned char)c) {
            return (void *)(s + i);
        }
    }
    return NULL;
}
char *strerror(int unused)
{
    (void)unused;
    return "module I/O error";
}
int abs(int value)
{
    return value < 0 ? -value : value;
}

float frexpf(float value, int *exponent)
{
    union {
        float f;
        uint32_t u;
    } bits = { .f = value };
    *exponent = 0;
    uint32_t e = (bits.u >> 23) & 255;
    if (e == 255 || (bits.u & 0x7fffffffU) == 0) {
        return value;
    }
    if (e == 0) {
        bits.f *= 16777216.0f;
        e = (bits.u >> 23) & 255;
        *exponent = -24;
    }
    *exponent += (int)e - 126;
    bits.u = (bits.u & 0x807fffffU) | (126U << 23);
    return bits.f;
}
float ldexpf(float value, int exponent)
{
    if (value == 0 || !__builtin_isfinite(value)) {
        return value;
    }
    // Clamp before iteration: Lua can supply any signed integer exponent.
    if (exponent > 512) {
        return __builtin_copysignf(HUGE_VALF, value);
    }
    if (exponent < -512) {
        return __builtin_copysignf(0, value);
    }
    while (exponent > 0) {
        value *= 2.0f;
        exponent--;
    }
    while (exponent < 0) {
        value *= 0.5f;
        exponent++;
    }
    return value;
}
float log2f(float value)
{
    return logf(value) * 1.4426950408889634f;
}
float log10f(float value)
{
    return logf(value) * 0.4342944819032518f;
}
