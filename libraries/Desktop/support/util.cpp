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
#include "desktop.h"


/*
  swap one double
 */
double swap_double(double d)
{
	union {
		double d;
		uint8_t b[8];
	} in, out;
	int i;
	in.d = d;
	for (i=0; i<8; i++) {
		out.b[7-i] = in.b[i];
	}
	return out.d;
}

/*
  swap an array of doubles
 */
void swap_doubles(double *d, unsigned count)
{
	while (count--) {
		*d = swap_double(*d);
		d++;
	}
}


/*
  swap one float
 */
float swap_float(float f)
{
	union {
		float f;
		uint8_t b[4];
	} in, out;
	int i;
	in.f = f;
	for (i=0; i<4; i++) {
		out.b[3-i] = in.b[i];
	}
	return out.f;
}

/*
  swap an array of floats
 */
void swap_floats(float *f, unsigned count)
{
	while (count--) {
		*f = swap_float(*f);
		f++;
	}
}


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
