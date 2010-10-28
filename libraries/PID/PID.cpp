#include "PID.h"

PID::PID()
{

}

long
PID::get_pid(long err, long dt, float scaler)
{
	// Positive error produces positive output
	float output;
 	float error = (float)err;
 	float delta_time = (float)dt/1000.0;
 	
	// Compute proportional component
	if(_kp != 0){
		output 				+= (_kp * error);
	}

	if(_kd != 0){
		// Compute derivative component
		//derivative = (error - previous_error)/dt
		float derivative 	= (error - _last_error) / dt;
		
		//Serial.print("d: ");
		//Serial.println(derivative,DEC);

		_last_error 		= error;
		output 				+= _kd * derivative;			// Sum the derivative component
	}
	
	output *= scaler;
	
	// Compute integral component
	if(_ki != 0){
		_integrator 		+= (error * _ki) * scaler * dt *.001; 
		_integrator 		= constrain(_integrator, -_imax, _imax);
		output 				+= _integrator;
	}
	
	return output;
}


void
PID::reset_I(void)
{
	_integrator = 0;
	_last_error = 0;
}



/// setters
void
PID::set_P(float p)
{
	_kp = p;
}

void
PID::set_I(float i)
{
	_ki = i;
}

void
PID::set_D(float d)
{
	_kd = d;
}

void
PID::set_imax(int imax)
{
	_imax = imax;
	Serial.print("set imax ");
	Serial.println(_imax, DEC);
}

/// getters
float
PID::get_P(void)
{
	return _kp;
}

float
PID::get_I(void)
{
	return _ki;
}

float
PID::get_D(void)
{
	return _kd;
}

int
PID::get_imax(void)
{
	return _imax;
}


void
PID::load_gains(int address)
{
	//Serial.println("load gains ");
	//Serial.println(address, DEC);
	_kp 	= (float)(eeprom_read_word((uint16_t *)	address)) / 1000.0;
	_ki 	= (float)(eeprom_read_word((uint16_t *)	(address + 2))) / 1000.0;
	_kd 	= (float)(eeprom_read_word((uint16_t *)	(address + 4))) / 1000.0;
	_imax 	= eeprom_read_word((uint16_t *)	(address + 6)) * 100;
}

void
PID::save_gains(int address)
{
	eeprom_write_word((uint16_t *)	address, 		(int)(_kp * 1000));
	eeprom_write_word((uint16_t *)	(address + 2), 	(int)(_ki * 1000));
	eeprom_write_word((uint16_t *)	(address + 4), 	(int)(_kd * 1000));
	eeprom_write_word((uint16_t *)	(address + 6), 	(int)_imax/100);
}

/*
float
read_EE_compressed_float(int address, byte places)
{
	float scale = places * 10;
	int temp 	= eeprom_read_word((uint16_t *) address);
	return ((float)temp / scale);
}

void write_EE_compressed_float(float value, int address, byte places)
{
	float scale = places * 10;
	int temp 	= value * scale;
	eeprom_write_word((uint16_t *) 	address, temp);
}
*/

void
PID::test(void)
{
	/*
	int address = 0x46;
	Serial.print("imax= ");
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.print(eeprom_read_word((uint16_t *) address));
	Serial.print(" ");
	address += 8;
	Serial.println(eeprom_read_word((uint16_t *) address));
	*/
}