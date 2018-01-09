#pragma once

#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <stdlib.h>

struct Timing_Struct{
	float dt;
	float omega;
	float T;
};

class Fourier_Analysis
{
public:
	Fourier_Analysis(void);
	Fourier_Analysis(int buffer_size);
	~Fourier_Analysis();
	void accumulate(Vector2f *new_sample, Timing_Struct *new_timing);
	void accumulate_discrete(float new_sample_x, float new_sample_y, float dt, float omega);
	float get_magnitude(void);
	float get_angle(void);
	Vector2f get_transform(void);
	void set_buffer_size(int buffer_size);
protected:
	Vector2f _fourier_transform;
	Vector2f *_signal;
	Timing_Struct *_timing;
private:
	int _buffer_size;
	float _time_elapsed;
	int _buffer_index;
};
