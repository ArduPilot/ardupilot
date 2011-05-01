/*
 * UgvTraxxasStampede.pde
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

/*
 * ArduPilotOne.pde
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

// Libraries
#include <APO.h>
#include <AP_Common.h>
#include <FastSerial.h>
#include <APM_RC.h>
#include <AP_RangeFinder.h>
#include <GCS_MAVLink.h>
#include <AP_ADC.h>
#include <AP_DCM.h>
#include <AP_Compass.h>
#include <Wire.h>
#include <AP_GPS.h>
#include <AP_IMU.h>
#include <APM_BMP085.h>

// Serial 0: debug      /dev/ttyUSB0
// Serial 1: gps/hil    /dev/ttyUSB1
// Serial 2: gcs        /dev/ttyUSB2

// select hardware absraction mode from
// 	MODE_LIVE, actual flight
// 	TODO: IMPLEMENT --> MODE_HIL_NAV, hardware in the loop with sensors running, tests navigation system and control
// 	MODE_HIL_CNTL, hardware in the loop with only controller running, just tests controller
apo::halMode_t halMode = apo::MODE_LIVE;

// select from, BOARD_ARDUPILOTMEGA
apo::board_t board = apo::BOARD_ARDUPILOTMEGA;

// select from, VEHICLE_CAR, VEHICLE_QUAD, VEHICLE_PLANE
apo::vehicle_t vehicle = apo::VEHICLE_CAR;

// optional sensors
static bool gpsEnabled = true;
static bool baroEnabled = true;
static bool compassEnabled = true;

static bool rangeFinderFrontEnabled = true;
static bool rangeFinderBackEnabled = true;
static bool rangeFinderLeftEnabled = true;
static bool rangeFinderRightEnabled = true;
static bool rangeFinderUpEnabled = true;
static bool rangeFinderDownEnabled = true;


//---------ADVANCED SECTION ----------------//

// loop rates
const float loop0Rate = 150;
const float loop1Rate = 50;
const float loop2Rate = 10;
const float loop3Rate = 1;
const float loop4Rate = 0.1;

// max time in seconds to allow flight without ground station comms
// zero will ignore timeout
const uint8_t heartbeatTimeout = 3.0;

//---------HARDWARE CONFIG ----------------//

//Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41
#define A_LED_PIN 37 //36 = B,3637 = A,363735 = C
#define B_LED_PIN 36
#define C_LED_PIN 35
#define EEPROM_MAX_ADDR	2048
#define RANGE_FINDER_CLASS AP_RangeFinder_MaxsonarLV


//---------MAIN ----------------//


/*
 * Required Global Declarations
 */
FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);
apo::AP_Autopilot * autoPilot;

void setup() {

	using namespace apo;

	AP_HardwareAbstractionLayer * hal = new AP_HardwareAbstractionLayer(halMode,board,vehicle);

	/*
	 * Communications
	 */
	Serial.begin(57600, 128, 128); // debug
	Serial3.begin(57600, 128, 128); // gcs

	hal->debug = &Serial;
	hal->debug->println_P(PSTR("initializing debug line"));
	hal->debug->println_P(PSTR("initializing radio"));
	APM_RC.Init(); // APM Radio initialization

	/*
	 * Pins
	 */
	hal->debug->println_P(PSTR("settings pin modes"));
	pinMode(A_LED_PIN, OUTPUT); //  extra led
	pinMode(B_LED_PIN, OUTPUT); //  imu ledclass AP_CommLink;
	pinMode(C_LED_PIN, OUTPUT); //  gps led
	pinMode(SLIDE_SWITCH_PIN, INPUT);
	pinMode(PUSHBUTTON_PIN, INPUT);
	DDRL |= B00000100; // set port L, pint 2 to output for the relay

	/*
	 * Initialize Comm Channels
	 */
	hal->debug->println_P(PSTR("initializing comm channels"));
	if (hal->mode()==MODE_LIVE) {
		Serial1.begin(38400, 128, 16); // gps
	} else { // hil
		Serial1.begin(57600, 128, 128);
	}

	/*
	 * Sensor initialization
	 */
	if (hal->mode()==MODE_LIVE)
	{
		hal->debug->println_P(PSTR("initializing adc"));
		hal->adc =  new AP_ADC_ADS7844;
		hal->adc->Init();

		if (gpsEnabled) {
			hal->debug->println_P(PSTR("initializing gps"));
			AP_GPS_Auto gpsDriver(&Serial1,&(hal->gps));
			hal->gps = &gpsDriver;
			hal->gps->init();
		}

		if (baroEnabled) {
			hal->debug->println_P(PSTR("initializing baro"));
			hal->baro = new APM_BMP085_Class;
			hal->baro->Init();
		}

		if (compassEnabled) {
			hal->debug->println_P(PSTR("initializing compass"));
			hal->compass = new AP_Compass_HMC5843;
			hal->compass->set_orientation(AP_COMPASS_COMPONENTS_UP_PINS_FORWARD);
			hal->compass->init();
		}

	}

	/**
	 * Initialize ultrasonic sensors. If sensors are not plugged in, the navigator will not
	 * initialize them and NULL will be assigned to those corresponding pointers.
	 * On detecting NULL assigned to any ultrasonic sensor, its corresponding block of code
	 * will not be executed by the navigator.
	 * The coordinate system is assigned by the right hand screw rule with the thumb pointing down.
	 * In set_orientation, it is defind as (front/back,left/right,down,up)
	 */

	if (rangeFinderFrontEnabled) {
		hal->debug->println_P(PSTR("initializing front range finder"));
		RangeFinder * rangeFinder = new RANGE_FINDER_CLASS;
		rangeFinder->init(1);
		rangeFinder->set_orientation(1,0,0);
		hal->rangeFinders.push_back(rangeFinder);
	}

	if (rangeFinderBackEnabled) {
		hal->debug->println_P(PSTR("initializing back range finder"));
		RangeFinder * rangeFinder = new RANGE_FINDER_CLASS;
		rangeFinder->init(2);
		rangeFinder->set_orientation(-1,0,0);
		hal->rangeFinders.push_back(rangeFinder);
	}

	if (rangeFinderLeftEnabled) {
		hal->debug->println_P(PSTR("initializing left range finder"));
		RangeFinder * rangeFinder = new RANGE_FINDER_CLASS;
		rangeFinder->init(3);
		rangeFinder->set_orientation(0,-1,0);
		hal->rangeFinders.push_back(rangeFinder);
	}

	if (rangeFinderRightEnabled) {
		hal->debug->println_P(PSTR("initializing right range finder"));
		RangeFinder * rangeFinder = new RANGE_FINDER_CLASS;
		rangeFinder->init(4);
		rangeFinder->set_orientation(0,1,0);
		hal->rangeFinders.push_back(rangeFinder);
	}

	if (rangeFinderUpEnabled) {
		hal->debug->println_P(PSTR("initializing up range finder"));
		RangeFinder * rangeFinder = new RANGE_FINDER_CLASS;
		rangeFinder->init(5);
		rangeFinder->set_orientation(0,0,-1);
		hal->rangeFinders.push_back(rangeFinder);
	}

	if (rangeFinderDownEnabled) {
		hal->debug->println_P(PSTR("initializing down range finder"));
		RangeFinder * rangeFinder = new RANGE_FINDER_CLASS;
		rangeFinder->init(6);
		rangeFinder->set_orientation(0,0,1);
		hal->rangeFinders.push_back(rangeFinder);
	}

	/*
	 * Navigator
	 */
	AP_Navigator * navigator = new DcmNavigator(hal);

	/*
	 * Guide
	 */
	AP_Guide * guide = new MavlinkGuide(k_guide,navigator,hal);

	/*
	 * Controller Initialization
	 */
	AP_Controller * controller = NULL;
	switch(vehicle)
	{
	case VEHICLE_CAR:
		controller = new CarController(k_cntrl,k_pidStr,k_pidThr,navigator,guide,hal);
		break;
	case VEHICLE_QUAD:
		controller = new QuadController(navigator,guide,hal);
		break;
	}

	/*
	 * CommLinks
	 */
	hal->gcs = new MavlinkComm(&Serial3,navigator,guide,controller,hal);
	hal->hil = new MavlinkComm(&Serial1,navigator,guide,controller,hal);

	/*
	 * Start the autopilot
	 */
	hal->debug->printf_P(PSTR("initializing ArduPilotOne\n"));
	hal->debug->printf_P(PSTR("free ram: %d bytes\n"),freeMemory());

	autoPilot = new apo::AP_Autopilot(navigator,guide,controller,hal);
}

void loop() {
	autoPilot->update();
}
