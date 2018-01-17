/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 /* fourier class, determines the fourier magnitude and phase of the signal in case our frame is rotating fast on it's yaw axis
 This class is absolutely mandatory to fly gyropter or monoblade like drones
 
 designed by Philippe Crochat / Anemos Technologies : pcrochat@anemos-technologies.com
 */
 
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <Filter/LowPassFilter.h>
#include <stdio.h>
#include <stdlib.h>

#define RESULT_FILTER_FREQ 5 //in Hz

struct Timing_Struct{
	float dt;
	Vector3f omega;
	float T;
};

class Fourier_Analysis
{
public:
	Fourier_Analysis();
	~Fourier_Analysis();
	void Set_Fourier_Analysis(int buffer_size);
	void accumulate(Vector3f *new_sample, Timing_Struct *new_timing);
	void accumulate_discrete(Vector3f new_sample, float dt, Vector3f omega);
	float get_pitch_angle(void);
	Vector2f get_yaw_angle(void);
	float get_phase(void);
	void set_buffer_size(int buffer_size);
	void synchronize_fourier_phase(float instant_heading);
	Vector2f get_result(void);
protected:
	Vector2f _fourier_transform[2];
	Timing_Struct *_timing;
	float *_phase;
private:
	Vector3f *_signal;
	int _buffer_size;
	float _time_elapsed;
	int _buffer_index;
	Vector3f _signal_mean;
	Vector2f _angle_tot;
	LowPassFilterVector2f _result_filter;
	Vector2f _last_result;
};
