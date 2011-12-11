
extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}

#include "AP_Baro_BMP085_hil.h"

// Constructors ////////////////////////////////////////////////////////////////
AP_Baro_BMP085_HIL::AP_Baro_BMP085_HIL()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_Baro_BMP085_HIL::init(AP_PeriodicProcess * scheduler)
{
  BMP085_State=1;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state = 1) and then 4 times Pressure (states 2-5)
uint8_t AP_Baro_BMP085_HIL::read()
{
	uint8_t result = 0;

	if (BMP085_State == 1){
		BMP085_State++;
	}else{

		if (BMP085_State == 5){
			BMP085_State = 1;				// Start again from state = 1
			result = 1;						// New pressure reading
		}else{
			BMP085_State++;
			result = 1;						// New pressure reading
		}
	}
	return(result);
}

void AP_Baro_BMP085_HIL::setHIL(float _Temp, float _Press)
{
    // TODO: map floats to raw
	Temp 	= _Temp;
	Press 	= _Press;
}

int32_t AP_Baro_BMP085_HIL::get_pressure() {
    return Press;
}

int16_t AP_Baro_BMP085_HIL::get_temperature() {
    return Temp;
}

float AP_Baro_BMP085_HIL::get_altitude() {
    return 0.0; // TODO
}

int32_t AP_Baro_BMP085_HIL::get_raw_pressure() {
    return Press;
}

int32_t AP_Baro_BMP085_HIL::get_raw_temp() {
    return Temp;
}
