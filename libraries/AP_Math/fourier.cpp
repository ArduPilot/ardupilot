#include "fourier.h"

Fourier_Analysis::Fourier_Analysis(void){
	Fourier_Analysis(0);
}

Fourier_Analysis::Fourier_Analysis(int buffer_size){
	int i;

	_buffer_size=buffer_size;

	_fourier_transform.x=0.0f;
	_fourier_transform.y=0.0f;

	_signal=(Vector2f *)malloc(buffer_size*sizeof(Vector2f));
	_timing=(Timing_Struct *)malloc(buffer_size*sizeof(Timing_Struct));

	for(i=0;i<_buffer_size;i++)
	{
		_timing[i].dt=0.0f;
		_timing[i].omega=0.0f;
		_timing[i].T=0.0f;
	}

	_time_elapsed=0.0f;
	_buffer_index=0;
}

Fourier_Analysis::~Fourier_Analysis(){
	free(_signal);
	free(_timing);
}

void Fourier_Analysis::accumulate(Vector2f *new_sample, Timing_Struct *new_timing){
	int previous_index;
	float phase;

	previous_index=_buffer_index-1;

	if(previous_index<0)
		previous_index=_buffer_size-1;

	phase=_timing[_buffer_index].omega*_timing[_buffer_index].T;

	_fourier_transform.x-= _signal[_buffer_index].x*cosf(phase);
	_fourier_transform.y-= _signal[_buffer_index].y*sinf(phase);

	new_timing->T=_timing[previous_index].T+new_timing->dt;

	phase=new_timing->omega*new_timing->T;
	_fourier_transform.x+= new_sample->x*cosf(phase);
	_fourier_transform.y+= new_sample->y*sinf(phase);

	_signal[_buffer_index]=*new_sample;
	_timing[_buffer_index]=*new_timing;

	_buffer_index++;

	if(_buffer_index>=_buffer_size)
		_buffer_index=0;
}

void Fourier_Analysis::accumulate_discrete(float new_sample_x, float new_sample_y, float dt, float omega)
{
	Vector2f new_sample;
	Timing_Struct new_timing;

	new_sample.x=new_sample_x;
	new_sample.y=new_sample_y;

	new_timing.dt=dt;
	new_timing.omega=omega;

	this->accumulate(&new_sample, &new_timing);
}

float Fourier_Analysis::get_magnitude(void){
	int next_index=_buffer_index+1;

	if(next_index==_buffer_size)
		next_index=0;

	float scale=(float)_buffer_size/2.0;

//	float time_lapsed=_timing[_buffer_index].T-_timing[next_index].T;

	return norm(_fourier_transform.x/scale, _fourier_transform.y/scale);
}

float Fourier_Analysis::get_angle(void){
	return atanf(_fourier_transform.y/_fourier_transform.x);
}

Vector2f Fourier_Analysis::get_transform(void){
	Vector2f transform_result;

	transform_result.x = get_magnitude();
	transform_result.y = get_angle();

	return transform_result;
}

void Fourier_Analysis::set_buffer_size(int buffer_size){
	_buffer_size=buffer_size;
}


