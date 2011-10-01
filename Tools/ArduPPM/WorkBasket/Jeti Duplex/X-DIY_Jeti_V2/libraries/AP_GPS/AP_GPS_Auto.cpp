// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Auto-detecting pseudo-GPS driver
//

#include "AP_GPS.h"
#include <stdlib.h>
#include <stdio.h>
#include <wiring.h>

static unsigned int	baudrates[] = {38400U, 57600U, 9600U, 4800U};

void
AP_GPS_Auto::init(void)
{
}

//
// Called the first time that a client tries to kick the GPS to update.
//
// We detect the real GPS, then update the pointer we have been called through
// and return.
void
AP_GPS_Auto::update(void)
{
	GPS		*gps;
	int		i;

	// loop trying to find a GPS
	for (;;) {
		// loop through possible baudrates
		for (i = 0; i < (sizeof(baudrates) / sizeof(baudrates[0])); i++) {
			printf("GPS autodetect at %d:%u\n", i, baudrates[i]);
			_port->begin(baudrates[i]);
			if (NULL != (gps = _detect())) {
				// make the detected GPS the default
				*_gps = gps;
				
				// configure the detected GPS and run one update
				gps->print_errors = true;	// XXX
				gps->init();
				gps->update();

				// drop back to our caller - subsequent calls through
				// the global will not come here
				return;
			}
		}
	}
}

//
// Perform one iteration of the auto-detection process.
//
GPS *
AP_GPS_Auto::_detect(void)
{
	unsigned long then;
	int		fingerprint[4];
	int		tries;
	GPS		*gps;

	//
	// Loop attempting to detect a recognised GPS
	//
	gps = NULL;
	for (tries = 0; tries < 2; tries++) {

		//
		// Empty the serial buffer and wait for 50ms of quiet.
		//
		// XXX We can detect babble by counting incoming characters, but
		//     what would we do about it?
		//
		_port->flush();
		then = millis();
		do {
			if (_port->available()) {
				then = millis();
				_port->read();
			}
		} while ((millis() - then) < 50);
			
		//
		// Collect four characters to fingerprint a device
		//
		fingerprint[0] = _getc();
		fingerprint[1] = _getc();
		fingerprint[2] = _getc();
		fingerprint[3] = _getc();
		printf("fingerprints 0x%02x 0x%02x 0x%02x 0x%02x\n",
			   fingerprint[0],
			   fingerprint[1],
			   fingerprint[2],
			   fingerprint[3]);

		//
		// u-blox or MTK in DIYD binary mode (whose smart idea was
		// it to make the MTK look sort-of like it was talking UBX?)
		//
		if ((0xb5 == fingerprint[0]) &&
			(0x62 == fingerprint[1]) && 
			(0x01 == fingerprint[2])) {

			// message 5 is MTK pretending to talk UBX
			if (0x05 == fingerprint[3]) {
				printf("detected MTK in binary mode\n");
				gps = new AP_GPS_MTK(_port);
				break;
			}

			// any other message is u-blox
			printf("detected u-blox in binary mode\n");
			gps = new AP_GPS_UBLOX(_port);
			break;
		} 

		//
		// SIRF in binary mode
		//
		if ((0xa0 == fingerprint[0]) &&
			(0xa2 == fingerprint[1])) {
			printf("detected SIRF in binary mode\n");
			gps = new AP_GPS_SIRF(_port);
			break;
		}

		//
		// If we haven't spammed the various init strings, send them now
		// and retry to avoid a false-positive on the NMEA detector.
		//
		if (0 == tries) {
			printf("sending setup strings and trying again\n");
			_port->println(MTK_SET_BINARY);
			_port->println(UBLOX_SET_BINARY);
			_port->println(SIRF_SET_BINARY);
			continue;
		}

		//
		// Something talking NMEA
		//
		if (('$' == fingerprint[0]) &&
			(('G' == fingerprint[1]) || ('P' == fingerprint[1]))) {

			// XXX this may be a bit presumptive, might want to give the GPS a couple of
			//     iterations around the loop to react to init strings?
			printf("detected NMEA\n");
			gps = new AP_GPS_NMEA(_port);
			break;
		}
	}
	return(gps);
}

int
AP_GPS_Auto::_getc(void)
{
	while (0 == _port->available())
		;
	return(_port->read());
}
