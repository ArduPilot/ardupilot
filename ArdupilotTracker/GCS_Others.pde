#include "GCS.h"
#include "tracking.h"

/*
 *  look for incoming commands on the GCS links
 */
static void datacomm_update(void) {
	if (datacomm0.initialised) {
		datacomm0.update();
	}
	if (datacomm1.initialised) {
		datacomm1.update();
	}
}

#if CLI_ENABLED == ENABLED
#endif

void Datacomm_Class_Multiple::update(void) {
	// process received bytes
	uint16_t nbytes = _port->available();
	for (uint16_t i = 0; i < nbytes; i++) {
		uint8_t c = _port->read();

        if (c == '\n' || c == '\r') {
            crlf_count++;
            digitalWrite(A_LED_PIN, crlf_count ? LED_ON : LED_OFF);
        } else {
            crlf_count = 0;
        }

		for (uint8_t j=0; j<numInterpreters; j++) {
			interpreters[j]->receive(c);
		}
	}
	
	bool startCLI = (crlf_count >= 3);
	for (uint8_t j=0; startCLI && j<numInterpreters; j++) {
		if (interpreters[j]->detected || interpreters[j]->isReceivingMessage()) {
			// startCLI = false;
		}
	}	
	
	if (startCLI) {
		run_cli(_port);
	}
}

void GCS_MAVLINK::init(AP_HAL::BetterStream *port) {
	Datacomm_Class::init(port);
	if (port == (AP_HAL::BetterStream*) hal.uartA) {
		mavlink_comm_0_port = port;
		chan = MAVLINK_COMM_0;
	} else {
		mavlink_comm_1_port = port;
		chan = MAVLINK_COMM_1;
	}
	_queued_parameter = NULL;
}

const prog_char ArduTrackerDataInterpreter::PAN[] PROGMEM = "PAN";
const prog_char ArduTrackerDataInterpreter::TLT[] PROGMEM = "TLT";

ArduTrackerDataInterpreter::ArduTrackerDataInterpreter() {
}

void ArduTrackerDataInterpreter::init(AP_HAL::BetterStream *port) {
	Datacomm_Class::init(port);
}

void ArduTrackerDataInterpreter::update(void) {

}

void ArduTrackerDataInterpreter::fail() {
	parseState = PANTTILT_PARSE_EXC0;
}

// Format is :
// !!!PAN:XXXX,TLT:YYYY

static int32_t BDC2Number(uint8_t* bcd) {
	int32_t result = 0;
	for (uint8_t i=0; i<4; i++) {
		result *= 10;
		result += bcd[i];
	}
	return result;
}

void ArduTrackerDataInterpreter::receive(uint8_t c) {
	switch (parseState) {
	case PANTTILT_PARSE_EXC0: {
		// Accept one CR LF char at start without failing.
		if (c == '\n' || c == '\r')
			return;
	}
	case PANTTILT_PARSE_EXC0 + 1:
	case PANTTILT_PARSE_EXC0 + 2: {
		if (c != '!') {
			fail();
			return;
		}
		break;
	}

	case PANTTILT_PARSE_LIT0:
	case PANTTILT_PARSE_LIT0 + 1:
	case PANTTILT_PARSE_LIT0 + 2: {
		char t = pgm_read_byte(PAN + parseState - PANTTILT_PARSE_LIT0);
		if (c != t) {
			fail();
			return;
		}
		break;
	}
	case PANTTILT_PARSE_LIT1:
	case PANTTILT_PARSE_LIT1 + 1:
	case PANTTILT_PARSE_LIT1 + 2: {
		char t = pgm_read_byte(TLT + parseState - PANTTILT_PARSE_LIT1);
		if (c != t) {
			fail();
			return;
		}
		break;
	}
	case PANTTILT_PARSE_COL0:
	case PANTTILT_PARSE_COL1: {
		if (c != ':') {
			fail();
			return;
		}
		break;
	}
	case PANTTILT_PARSE_COMMA0: {
		if (c != ',') {
			fail();
			return;
		}
		break;
	}

	case PANTTILT_PARSE_PAN0:
	case PANTTILT_PARSE_PAN0 + 1:
	case PANTTILT_PARSE_PAN0 + 2:
	case PANTTILT_PARSE_PAN0 + 3: {
		if (c < '0' || c > '9') {
			fail();
			return;
		}
		aziBCD[parseState - PANTTILT_PARSE_PAN0] = c - '0';
		break;
	}

	case PANTTILT_PARSE_TLT0:
	case PANTTILT_PARSE_TLT0 + 1:
	case PANTTILT_PARSE_TLT0 + 2:
	case PANTTILT_PARSE_TLT0 + 3: {
		if (c < '0' || c > '9') {
			fail();
			return;
		}
		eleBCD[parseState - PANTTILT_PARSE_TLT0] = c - '0';
		break;
	}
	case PANTTILT_PARSE_END: {
		// success!
		parseState = PANTTILT_PARSE_EXC0;
		
		struct AzimuthElevation result;
		/*
		 * Here we could have used an RC_Channel object to do the conversion between
		 * PWM values and centidegrees, and even have configurable end points, trim,
		 * direction etc. However all the configuration can also be done in MP so
		 * there is no reason to repeat it here. We just do a simple calculation.
		 */
		result.azimuth_cd = (BDC2Number(aziBCD) - 1500) * SERVO_MAX / 500;
		result.elevation_cd = (BDC2Number(eleBCD) - 1500) * SERVO_MAX / 500;
		result.timestamp = millis();
		
		incomingAzimuthElevation = result;
		
		detected = true;
		return;
	}
	default: {
		fail();
		return;
	}
	}
	parseState++;
}

bool ArduTrackerDataInterpreter::isReceivingMessage() {
	return parseState != 0;
} 