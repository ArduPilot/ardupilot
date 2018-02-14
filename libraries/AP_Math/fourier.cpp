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

/* This library is here to transform a rotational frame motion into pitch and yaw angle.
The roll angle will always be set to 0, the pitch will be set to the one corresponding to the greatest slope swept 
by the rotor. A more complete explanation is available at https://discuss.ardupilot.org/t/gyropter-monocopter-support-with-the-help-of-a-fourier-based-transform/25168/14
*/


#include "fourier.h"


//constructor, it does initiates a filter and sets the fourier buffer size to it's default BUF_SIZE
Fourier_Analysis::Fourier_Analysis() {
	_result_filter.set_cutoff_frequency(RESULT_FILTER_FREQ);
	Set_Fourier_Analysis(BUF_SIZE);
}

//sets buffer size and allocates memory
void Fourier_Analysis::Set_Fourier_Analysis(int32_t buffer_size) {
	_buffer_size=buffer_size;
	
	_signal_buffer=(Fourier_Struct *)calloc(buffer_size,sizeof(Fourier_Struct));
}

//destructor
Fourier_Analysis::~Fourier_Analysis() {
	if (_signal_buffer != nullptr) {
		free(_signal_buffer);
		_signal_buffer=nullptr;
	}
}

//private method to add a new sample value to the filter (substract=false) or remove an old one (substract=true)
//this method does the fourier filter calculation. Please note that only the first fundamental frequency is analysed
void Fourier_Analysis::_add_signal(bool substract) {
	int8_t sign = substract==true ? -1 : 1;
	int32_t buffer_index = substract==true ? _buffer_index_inf : _buffer_index_sup;
	
	Vector2f fourier_coef;
	float signal_temp[2];
	
	fourier_coef.x=cosf(_signal_buffer[buffer_index].phase);
	fourier_coef.y=-sinf(_signal_buffer[buffer_index].phase);
	
	signal_temp[0]=_signal_buffer[buffer_index].signal.x;
	signal_temp[1]=_signal_buffer[buffer_index].signal.y;
	
	while(_dtheta_sum>M_2PI || substract==false || 
		(_buffer_index_sup==(_buffer_index_inf>0 ? _buffer_index_inf-1 : _buffer_size-1) && substract==true))
	{
		int32_t i;
		
		if (_signal_buffer[buffer_index].dtheta<0.0f) {
			_signal_buffer[buffer_index].dtheta=-_signal_buffer[buffer_index].dtheta;
		}
		
		for(i=0;i<2;i++) {
			_fourier_transform[i].x += sign * (fourier_coef.x*signal_temp[i]) * _signal_buffer[buffer_index].dtheta;
			_fourier_transform[i].y += sign * (fourier_coef.y*signal_temp[i]) * _signal_buffer[buffer_index].dtheta;
		}
		
		_dtheta_sum += _signal_buffer[buffer_index].dtheta * sign;
		
		_signal_mean += _signal_buffer[buffer_index].signal * _signal_buffer[buffer_index].dtheta * sign;
		buffer_index++;
		
		if (buffer_index>=_buffer_size) {
			buffer_index=0;
		}
		
		if (substract==true) {
			_buffer_index_inf=buffer_index;
		} else {
			_buffer_index_sup=buffer_index;
		}
		
		if (substract==false) {
			break;
		}
	}
}

//acumulates a new sample into the filter. dt is the time elapsed since last accumulaton and omega are the instant gyro value
void Fourier_Analysis::accumulate_discrete(const Vector3f new_sample, float dt, const Vector3f omega) {
	int32_t last_buffer_index_sup = _buffer_index_sup-1<0 ? _buffer_size-1 : _buffer_index_sup-1;
	
	_add_signal(true);
	
	_signal_buffer[_buffer_index_sup].signal = new_sample;
	_signal_buffer[_buffer_index_sup].dtheta = dt*omega.z;
	
	if (_phase_synchronized==false) {
		_signal_buffer[_buffer_index_sup].phase = _signal_buffer[last_buffer_index_sup].phase+dt*omega.z;
	}
	
	_add_signal(false);
	_phase_synchronized=false;
}

//return the current phase of the fourier filter
float Fourier_Analysis::get_phase(void) {
	int32_t last_index=_buffer_index_sup-1;
	
	if(last_index<0){
		last_index=_buffer_size-1;
	}
	
	return _signal_buffer[last_index].phase;
}

//synchronize heading with compass value
void Fourier_Analysis::synchronize_fourier_phase(float instant_heading) {
	if (instant_heading>M_PI) {
		instant_heading-=M_2PI;
	}
	
	if (instant_heading<-M_PI) {
		instant_heading+=M_2PI;
	}
	
	_signal_buffer[_buffer_index_sup].phase=instant_heading;
	_phase_synchronized=true;
}

//this methode calculates and returns the pitch and yaw angle 
Vector2f Fourier_Analysis::get_result(void) {
	Vector2f result;
	
	result.x=_get_pitch_angle();
	result.y=_get_yaw_angle();
	
	if (result.x>0.0f) {
		result.y=-result.y;
		result.x=-result.x;
	}
	
	return result;
}

//this method calculates the greatest slope of the disk area swept by the frame
float Fourier_Analysis::_get_pitch_angle(void) {
	Vector2f result;
	
	if (_dtheta_sum>0.0f) {
		float scale=2.0/_dtheta_sum;
		
		
		result.x = norm(_fourier_transform[0].x*scale, _fourier_transform[0].y*scale);
		result.y = norm(_fourier_transform[1].x*scale, _fourier_transform[1].y*scale);
		
		return _signal_mean.z != 0.0f ? atanf(0.5*(result.x+result.y)/_signal_mean.z*_dtheta_sum) : 0.0f;
	} else {
		return 0.0f;
	}
}

//return the calculated yaw between PI and -PI
float Fourier_Analysis::_get_yaw_angle(void) {
	Vector2f result;
	
	result.x = (_fourier_transform[0].x != 0.0f) ? atanf(_fourier_transform[0].y/_fourier_transform[0].x) : 0.0f;
	result.y = (_fourier_transform[1].x != 0.0f) ? atanf(_fourier_transform[1].y/_fourier_transform[1].x) : 0.0f;
	
	if (_fourier_transform[0].x<0) {
		result.x+=M_PI;
		
		if(result.x>M_PI){
			result.x-=M_2PI;
		}
	}
	
	if (_fourier_transform[1].x<0) {
		result.y+=M_PI;
		
		if(result.y>M_PI) {
			result.y-=M_2PI;
		}
	}
	
	return result.x;
}

//sets filter buffer size
void Fourier_Analysis::set_buffer_size(int32_t buffer_size) {
	_buffer_size=buffer_size;
}
