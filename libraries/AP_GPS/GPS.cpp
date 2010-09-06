// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "GPS.h"
#include <stdio.h>

void
GPS::_error(const char *fmt, ...)
{
	va_list	ap;

	if (print_errors && stderr)	{
		va_start(ap, fmt);
		vfprintf(stderr, fmt, ap);
		va_end(ap);
	}
}
