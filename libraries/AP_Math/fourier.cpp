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
 
 //designed by Philippe Crochat / Anemos Technologies : pcrochat@anemos-technologies.com

#include "fourier.h"

Fourier_Analysis::Fourier_Analysis(){
	int i;
	
	for(i=0;i<2;i++)
	{
		_fourier_transform[i].x=0.0f;
		_fourier_transform[i].y=0.0f;
	}

	_time_elapsed=0.0f;
	_buffer_index=0;
	
	_signal_mean.x=0.0f;
	_signal_mean.y=0.0f;
	_signal_mean.z=0.0f;
	
	_angle_tot.x=0.0f;
	_angle_tot.y=0.0f;
	
	_result_filter.set_cutoff_frequency(RESULT_FILTER_FREQ);
	
	_last_result.x=0.0f;
	_last_result.y=0.0f;
}

void Fourier_Analysis::Set_Fourier_Analysis(int buffer_size){
	int i;

	_buffer_size=buffer_size;
	
	_signal=(Vector3f *)malloc(buffer_size*sizeof(Vector3f));
	_timing=(Timing_Struct *)malloc(buffer_size*sizeof(Timing_Struct));
	_phase=(float *)malloc(buffer_size*sizeof(float));
	
	for(i=0;i<_buffer_size;i++)
	{
		_signal[i].x=0.0f;
		_signal[i].y=0.0f;
		_signal[i].z=0.0f;
		
		_timing[i].dt=0.0f;
		
		_timing[i].omega.x=0.0f;
		_timing[i].omega.y=0.0f;
		_timing[i].omega.z=0.0f;
		
		_timing[i].T=0.0f;
		
		_phase[i]=0.0f;
	}
}

Fourier_Analysis::~Fourier_Analysis(){
	free(_signal);
	free(_timing);
	free(_phase);
}

void Fourier_Analysis::accumulate(Vector3f *new_sample, Timing_Struct *new_timing){
	int previous_index;
	int i;
	float actual_heading;
	float actual_yaw;
	
	previous_index=_buffer_index-1;

	if(previous_index<0)
		previous_index=_buffer_size-1;
	
	Vector3f signal_current=_signal[_buffer_index];//-_signal_mean/_buffer_size;
	Vector2f fourier_coef;
	
	Vector2f new_result;
	
	fourier_coef.x=cosf(_phase[_buffer_index]);
	fourier_coef.y=-sinf(_phase[_buffer_index]);
	
	for(i=0;i<2;i++)
	{
		_fourier_transform[i].x -= (fourier_coef.x*signal_current.x);
		_fourier_transform[i].y -= (fourier_coef.y*signal_current.x);
	}
	
	actual_yaw=get_yaw_angle().x;
	actual_heading=_phase[_buffer_index]-actual_yaw;
	
	_angle_tot.x -= _timing[_buffer_index].dt*(_timing[_buffer_index].omega.x*cosf(actual_heading) - _timing[_buffer_index].omega.y*sinf(actual_heading));
	_angle_tot.y -= _timing[_buffer_index].dt*(_timing[_buffer_index].omega.y*cosf(actual_heading) + _timing[_buffer_index].omega.x*sinf(actual_heading));

	
	_signal_mean-=_signal[_buffer_index];
	
	_phase[_buffer_index]=_phase[previous_index]+new_timing->omega.z*new_timing->dt;
	
	if(_phase[_buffer_index]>M_PI)
		_phase[_buffer_index]-=2*M_PI;
	
	new_timing->T=_timing[previous_index].T+new_timing->dt;

	_signal_mean+=*new_sample;
	
	signal_current=*new_sample;//-_signal_mean/_buffer_size;
	fourier_coef.x=cosf(_phase[_buffer_index]);
	fourier_coef.y=-sinf(_phase[_buffer_index]);
	
	for(i=0;i<2;i++)
	{
		_fourier_transform[i].x += (fourier_coef.x*signal_current.x);
		_fourier_transform[i].y += (fourier_coef.y*signal_current.x);
	}

	actual_heading=_phase[_buffer_index]-actual_yaw;
	
	_angle_tot.x += _timing[_buffer_index].dt*(_timing[_buffer_index].omega.x*cosf(actual_heading) - _timing[_buffer_index].omega.y*sinf(actual_heading));
	_angle_tot.y += _timing[_buffer_index].dt*(_timing[_buffer_index].omega.y*cosf(actual_heading) + _timing[_buffer_index].omega.x*sinf(actual_heading));

	
	_signal[_buffer_index]=*new_sample;
	_timing[_buffer_index]=*new_timing;

	_buffer_index++;

	if(_buffer_index>=_buffer_size)
		_buffer_index=0;
	
	new_result.x=get_pitch_angle();
	new_result.y=get_yaw_angle().x;
	
	_last_result = _result_filter.apply(new_result,  new_timing->dt);
	
	if (_last_result.is_nan() || _last_result.is_inf()) {
		_result_filter.reset();
	}
}

void Fourier_Analysis::accumulate_discrete(Vector3f new_sample, float dt, Vector3f omega)
{
	Timing_Struct new_timing;

	new_timing.dt=dt;
	new_timing.omega=omega;
	
	this->accumulate(&new_sample, &new_timing);
}

float Fourier_Analysis::get_phase(void)
{
	int last_index=_buffer_index-1;
	
	if(last_index<0)
		last_index=_buffer_size-1;
	
	return _phase[last_index];
}

Vector2f Fourier_Analysis::get_result()
{
	return _last_result;
}

float Fourier_Analysis::get_pitch_angle(void){
	int next_index=_buffer_index+1;
	
	Vector2f result;
	
	if(next_index==_buffer_size)
		next_index=0;

	float scale=(float)_buffer_size/2.0;


	result.x = norm(_fourier_transform[0].x/scale, _fourier_transform[0].y/scale);
	result.y = norm(_fourier_transform[1].x/scale, _fourier_transform[1].y/scale);
	
	return atanf(0.5*(result.x+result.y)/_signal_mean.z*_buffer_size);
}

//synchronize heading with compass value
void Fourier_Analysis::synchronize_fourier_phase(float instant_heading)
{
	if(instant_heading>M_PI)
	{
		instant_heading-=2*M_PI;
	}
	
	_phase[_buffer_index]=instant_heading;
}

//return the calculated yaw between PI and -PI
Vector2f Fourier_Analysis::get_yaw_angle(void){
	Vector2f result;
	
	result.x = atanf(_fourier_transform[0].y/_fourier_transform[0].x);
	result.y = atanf(_fourier_transform[1].y/_fourier_transform[1].x);
	
	if(_fourier_transform[0].x<0)
	{
		result.x+=M_PI;
		
		if(result.x>M_PI)
			result.x-=2*M_PI;
	}
	
	if(_fourier_transform[1].x<0)
	{
		result.y+=M_PI;
		
		if(result.y>M_PI)
			result.y-=2*M_PI;
	}
	
	return result;
}

void Fourier_Analysis::set_buffer_size(int buffer_size){
	_buffer_size=buffer_size;
}


