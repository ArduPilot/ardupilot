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

/* convert the angular velocities from earth frame to
   body frame. Thanks to James Goppert for the formula
*/
void convert_body_frame(double rollDeg, double pitchDeg,
			double rollRate, double pitchRate, double yawRate,
			double *p, double *q, double *r)
{
	double phi, theta, phiDot, thetaDot, psiDot;

	phi = ToRad(rollDeg);
	theta = ToRad(pitchDeg);
	phiDot = ToRad(rollRate);
	thetaDot = ToRad(pitchRate);
	psiDot = ToRad(yawRate);

	*p = phiDot - psiDot*sin(theta);
	*q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta);
	*r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot;
}
