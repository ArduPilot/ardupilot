#ifndef _APO_COMMON_H
#define _APO_COMMON_H

FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);

/*
 * Required Global Declarations
 */

static apo::AP_Autopilot * autoPilot;

void setup() {

	using namespace apo;
	
	AP_Var::load_all();

	/*
	 * Communications
	 */
	Serial.begin(DEBUG_BAUD, 128, 128); // debug
	if (board==BOARD_ARDUPILOTMEGA_2) Serial2.begin(TELEM_BAUD, 128, 128); // gcs
	else Serial3.begin(TELEM_BAUD, 128, 128); // gcs

	// hardware abstraction layer
	AP_HardwareAbstractionLayer * hal = new AP_HardwareAbstractionLayer(
			halMode, board, vehicle, heartBeatTimeout);
	
	// debug serial
	hal->debug = &Serial;
	hal->debug->println_P(PSTR("initializing debug line"));

	/*
	 * Initialize Comm Channels
	 */
	hal->debug->println_P(PSTR("initializing comm channels"));
	if (hal->getMode() == MODE_LIVE) {
		Serial1.begin(GPS_BAUD, 128, 16); // gps
	} else { // hil
		Serial1.begin(HIL_BAUD, 128, 128);
	}

	/*
	 * Sensor initialization
	 */
	if (hal->getMode() == MODE_LIVE) {
		hal->debug->println_P(PSTR("initializing adc"));
		hal->adc = new ADC_CLASS;
		hal->adc->Init();

		if (gpsEnabled) {
			hal->debug->println_P(PSTR("initializing gps"));
			AP_GPS_Auto gpsDriver(&Serial1, &(hal->gps));
			hal->gps = &gpsDriver;
			hal->gps->init();
		}

		if (baroEnabled) {
			hal->debug->println_P(PSTR("initializing baro"));
			hal->baro = new BARO_CLASS;
			hal->baro->Init();
		}

		if (compassEnabled) {
			hal->debug->println_P(PSTR("initializing compass"));
			hal->compass = new COMPASS_CLASS;
			hal->compass->set_orientation(AP_COMPASS_COMPONENTS_UP_PINS_FORWARD);
			hal->compass->init();
		}

		/**
		 * Initialize ultrasonic sensors. If sensors are not plugged in, the navigator will not
		 * initialize them and NULL will be assigned to those corresponding pointers.
		 * On detecting NULL assigned to any ultrasonic sensor, its corresponding block of code
		 * will not be executed by the navigator.
		 * The coordinate system is assigned by the right hand rule with the thumb pointing down.
		 * In set_orientation, it is defind as (front/back,left/right,down,up)
		 */

		if (rangeFinderFrontEnabled) {
			hal->debug->println_P(PSTR("initializing front range finder"));
			RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
			rangeFinder->set_analog_port(1);
			rangeFinder->set_orientation(1, 0, 0);
			hal->rangeFinders.push_back(rangeFinder);
		}

		if (rangeFinderBackEnabled) {
			hal->debug->println_P(PSTR("initializing back range finder"));
			RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
			rangeFinder->set_analog_port(2);
			rangeFinder->set_orientation(-1, 0, 0);
			hal->rangeFinders.push_back(rangeFinder);
		}

		if (rangeFinderLeftEnabled) {
			hal->debug->println_P(PSTR("initializing left range finder"));
			RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
			rangeFinder->set_analog_port(3);
			rangeFinder->set_orientation(0, -1, 0);
			hal->rangeFinders.push_back(rangeFinder);
		}

		if (rangeFinderRightEnabled) {
			hal->debug->println_P(PSTR("initializing right range finder"));
			RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
			rangeFinder->set_analog_port(4);
			rangeFinder->set_orientation(0, 1, 0);
			hal->rangeFinders.push_back(rangeFinder);
		}

		if (rangeFinderUpEnabled) {
			hal->debug->println_P(PSTR("initializing up range finder"));
			RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
			rangeFinder->set_analog_port(5);
			rangeFinder->set_orientation(0, 0, -1);
			hal->rangeFinders.push_back(rangeFinder);
		}

		if (rangeFinderDownEnabled) {
			hal->debug->println_P(PSTR("initializing down range finder"));
			RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
			rangeFinder->set_analog_port(6);
			rangeFinder->set_orientation(0, 0, 1);
			hal->rangeFinders.push_back(rangeFinder);
		}

	}

	/*
	 * Select guidance, navigation, control algorithms
	 */
	AP_Navigator * navigator = new NAVIGATOR_CLASS(hal);
	AP_Guide * guide = new GUIDE_CLASS(navigator, hal);
	AP_Controller * controller = new CONTROLLER_CLASS(navigator, guide, hal);

	/*
	 * CommLinks
	 */
	if (board==BOARD_ARDUPILOTMEGA_2) hal->gcs = new COMMLINK_CLASS(&Serial2, navigator, guide, controller, hal);
	else hal->gcs = new COMMLINK_CLASS(&Serial3, navigator, guide, controller, hal);
	
	hal->hil = new COMMLINK_CLASS(&Serial1, navigator, guide, controller, hal);

	/*
	 * Start the autopilot
	 */
	hal->debug->printf_P(PSTR("initializing arduplane\n"));
	hal->debug->printf_P(PSTR("free ram: %d bytes\n"),freeMemory());
	autoPilot = new apo::AP_Autopilot(navigator, guide, controller, hal,
			loop0Rate, loop1Rate, loop2Rate, loop3Rate);
}

void loop() {
	autoPilot->update();
}

#endif _APO_COMMON_H
