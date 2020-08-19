/*
  replacement functions for systems that are missing required library functions
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAVE_MEMRCHR
void *replace_memrchr(void *s, int c, size_t n);
#define memrchr(s,c,n) replace_memrchr(s,c,n)
#endif
