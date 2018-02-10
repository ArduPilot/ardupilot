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

struct Fourier_Struct{
	Vector3f signal;
	float phase;
	float dtheta;
};

const int32_t SAMPLE_RATE=1600;
const int32_t MIN_FREQ=1;
const int32_t BUF_SIZE=SAMPLE_RATE/MIN_FREQ;

class Fourier_Analysis
{
public:
	Fourier_Analysis();
	~Fourier_Analysis();
	void Set_Fourier_Analysis(int32_t buffer_size);
	void accumulate_discrete(const &Vector3f new_sample, float dt, const &Vector3f omega);
	Vector2f get_result(void);
	float get_phase(void);
	void set_buffer_size(int32_t buffer_size);
	void synchronize_fourier_phase(float instant_heading);
protected:
	Vector2f _fourier_transform[2];
	Timing_Struct *_timing;
private:
	void _add_signal(bool substract);
	float _get_pitch_angle(void);
	float _get_yaw_angle(void);
	
	Fourier_Struct *_signal_buffer;
	int32_t _buffer_size;
	float _time_elapsed;
	int32_t _buffer_index_inf;
	int32_t _buffer_index_sup;
	Vector3f _signal_mean;
	float _dtheta_sum;
	LowPassFilterVector2f _result_filter;
	Vector2f _last_result;
	bool _phase_synchronized;
};
