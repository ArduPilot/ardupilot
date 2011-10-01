// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	ACM_PI.cpp
/// @brief	Generic PI algorithm

#include <math.h>

#include "APM_PI.h"

long
APM_PI::get_pi(int32_t error, float dt)
{
	_integrator += ((float)error * _ki) * dt;
	_integrator = min(_integrator, (float)_imax);
	_integrator = max(_integrator, (float)-_imax);

	return (float)error * _kp + _integrator;
}

void
APM_PI::reset_I()
{
	_integrator = 0;
}

void
APM_PI::load_gains()
{
    _group.load();
}

void
APM_PI::save_gains()
{
    _group.save();
}
