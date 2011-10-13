

extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}

#include "APM_BMP085_hil.h"

// Constructors ////////////////////////////////////////////////////////////////
APM_BMP085_HIL_Class::APM_BMP085_HIL_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void APM_BMP085_HIL_Class::Init(int initialiseWireLib)
{
  BMP085_State=1;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state = 1) and then 4 times Pressure (states 2-5)
uint8_t APM_BMP085_HIL_Class::Read()
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

void APM_BMP085_HIL_Class::setHIL(float _Temp, float _Press)
{
    // TODO: map floats to raw
	Temp 	= _Temp;
	Press 	= _Press;
}
