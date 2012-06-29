/*
  SITL handling - utility functions

  This simulates the APM1 hardware sufficiently for the APM code to
  think it is running on real hardware

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <AP_Math.h>
#include "desktop.h"
#include "util.h"


void set_nonblocking(int fd)
{
	unsigned v = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, v | O_NONBLOCK);
}

double normalise(double v, double min, double max)
{
	while (v < min) {
		v += (max - min);
	}
	while (v > max) {
		v -= (max - min);
	}
	return v;
}

double normalise180(double v)
{
	return normalise(v, -180, 180);
}

// generate a random Vector3f of size 1
Vector3f rand_vec3f(void)
{
	Vector3f v = Vector3f(rand_float(),
			      rand_float(),
			      rand_float());
	if (v.length() != 0.0) {
		v.normalize();
	}
	return v;
}
