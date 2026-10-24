// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
static inline int isdigit(int c) { return c >= '0' && c <= '9'; }
static inline int islower(int c) { return c >= 'a' && c <= 'z'; }
static inline int isupper(int c) { return c >= 'A' && c <= 'Z'; }
static inline int isalpha(int c) { return islower(c) || isupper(c); }
static inline int isalnum(int c) { return isalpha(c) || isdigit(c); }
static inline int isspace(int c) { return c == ' ' || (c >= '\t' && c <= '\r'); }
static inline int iscntrl(int c) { return (c >= 0 && c < 32) || c == 127; }
static inline int isprint(int c) { return c >= 32 && c <= 126; }
static inline int isgraph(int c) { return c >= 33 && c <= 126; }
static inline int ispunct(int c) { return isgraph(c) && !isalnum(c); }
static inline int tolower(int c) { return isupper(c) ? c + ('a' - 'A') : c; }
static inline int toupper(int c) { return islower(c) ? c - ('a' - 'A') : c; }
static inline int isxdigit(int c) { return isdigit(c) || (tolower(c) >= 'a' && tolower(c) <= 'f'); }
