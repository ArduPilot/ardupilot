// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "GPS.h"
#include "WProgram.h"
#include <stdio.h>

int
GPS::status(void)
{
	if (millis() - _lastTime >= 500){
		return 0;
	} else if (fix == 0) {
		return 1;
	} else {
		return 2;
	}
}

void
GPS::_setTime(void)
{
	_lastTime = millis();
}

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
