#include_next <fcntl.h>

/*
 * we only want to define as 0 for those targets in which it doesn't make
 * sense
 */
#if defined(HAVE_OCLOEXEC) && HAVE_OCLOEXEC == 0
#define O_CLOEXEC 0
#endif
