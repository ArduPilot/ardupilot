#ifndef __AP_RangeFinder_MaxsonarSerialXL_H__
#define __AP_RangeFinder_MaxsonarSerialXL_H__

#include "RangeFinder.h"

#define AP_RANGEFINDER_MAXSONARSERXL_SCALER 2.54
#define AP_RANGEFINDER_MAXSONARSERXL_MIN_DISTANCE 15
#define AP_RANGEFINDER_MAXSONARSERXL_MAX_DISTANCE 645 //254 * 2.54

/*NOTE - I wrote this because my hobbywing controller didn't have a spare analog input, but it DID have an extra UART. So Bam!
	This sensor puts out RS232(ish) serial. So to the TTL UART in the flight controller, it's INVERTED. You can't
	just plug it in. You can either buy a MAX3232 to convert it(pussy), or root around your junk drawer for a signal transistor and 2 resistors...
	I chose option 2 as I already had a 2n3906 a 10K and a 1k resistor handy. (if you don't know what I'm
	talking about, close this file. You ARE in WAY over your head.) While you're adding the logic inverter, get a larger 10 ohm 
	resistor and a couple 100uF capacitors and filter the sensor power supply. It'll save you issues later. 
	I was able to solder it all together tightly inline with the cabling, no PCB. 

	This code should work for all the serial enabled MaxSonarXL variants that support serial out. When using serial, the MAX range will ALWAYS 
	be 645cm regardless of the hardware capabilitiessince the max reading (8 bits) is 254 (in inches). So we don't care what variety 
	of MaxSonar we're hooked up to. 

	If you get random spurious readings FILTER THE POWER SUPPLY. If that doesn't work, move the sensor away from any prop wash or 
	shield it from visible line-of-sight of the propellers. Also when you power up, make sure the closest object is at least 12-14 inches away. 
	(On power up the unit calibrates. If there is an object too close, then the sensor won't see very close objects.)
	*/


#define AP_RANGEFINDER_MAXSONARSERXL_STARTCHAR 0x52 // 'R'
#define AP_RANGEFINDER_MAXSONARSERXL_ENDCHAR 0x13	// Newline

class AP_RangeFinder_MaxsonarSerialXL : public RangeFinder
{
public:
    AP_RangeFinder_MaxsonarSerialXL(AP_HAL::UARTDriver *ser, FilterInt16 *filter);

	void init(AP_HAL::UARTDriver *ser);

    int		convert_raw_to_distance(int _raw_value) {
				return _raw_value * _scaler;
			}                                                                                             


    // read value from sensor and return distance in cm
    int     read();


private:
	AP_HAL::UARTDriver* _ser;
	float _scaler;
//	float _old;
};
#endif
