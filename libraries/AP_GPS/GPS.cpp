// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "GPS.h"
#include "WProgram.h"

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
