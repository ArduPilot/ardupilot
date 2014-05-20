// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// HoTT v4 telemetry for ArduCopter
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//
//
// 01/2013 by Adam Majerczyk (majerczyk.adam@gmail.com)
// Texmode add-on by Michi (mamaretti32@gmail.com)
// v0.9.7.2b (beta software)
// Adapted for Ardupilot 3.1-rc5 10/2013 by Michel Kosloff aka msk7-ripe (michel.kosloff@gmail.com)
// Textmode not supported due to APM overloading (msk7-ripe)
//
// Developed and tested with:
// Transmitter MC-32 (v1.030,v1.041), MX-20, MC-20
// Receiver GR-16 v6a10_f9
// Receiver GR-12 v3a40_e9
//
//
/**
*   2013-10-27	msk7-ripe	Added electric time EAM functionality
*   2014-12.15  msk7-ripe	Replaced float calculations with integer where possible
*   2014-04-07  msk7-ripe	Added AC3.1.2dev compatibility (usage of AP_GPS object)
*   2014-04-08  msk7-ripe	Clered old comments
*   2014-05-12  msk7-ripe	Added HYBRID mode support
*/

#ifdef HOTT_TELEMETRY

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    #define HOTT_TELEMETRY_SERIAL_PORT	2
#else
    #error Boards other than APM 2.x not supported (not tested) yet
#endif

#define HOTT_SIM_GPS_SENSOR
#define HOTT_SIM_EAM_SENSOR
#define HOTT_SIM_VARIO_SENSOR
#define HOTT_SIM_GAM_SENSOR

#ifdef HOTT_SIM_TEXTMODE
#undef HOTT_SIM_TEXTMODE
#endif

#if HOTT_TELEMETRY_SERIAL_PORT == 2
	//FastSerialPort2(Serial2);      //HOTT serial port
	#define _HOTT_PORT	hal.uartC
#endif
#if HOTT_TELEMETRY_SERIAL_PORT == 3
	//FastSerialPort3(Serial3);      //HOTT serial port
	#error Not supported yet
//	#define _HOTT_PORT	hal.uartC
#endif

#ifndef HOTT_TELEMETRY_SERIAL_PORT
#error HOTT serial port undefined. Please define HOTT_TELEMETRY_SERIAL_PORT
#endif


#define HOTT_TEXT_MODE_REQUEST_ID	0x7f
#define HOTT_BINARY_MODE_REQUEST_ID	0x80
//Sensor Ids
//Graupner #33611 General Air Module
#define HOTT_TELEMETRY_GAM_SENSOR_ID	0x8d
//Graupner #33600 GPS Module
#define HOTT_TELEMETRY_GPS_SENSOR_ID	0x8a
//Graupner #33620 Electric Air Module
#define HOTT_TELEMETRY_EAM_SENSOR_ID	0x8e
//Graupner #33601 Vario Module
#define HOTT_TELEMETRY_VARIO_SENSOR_ID	0x89



static bool _hott_telemetry_is_sending = false;
static int8_t _hott_telemetry_sendig_msgs_id = 0;

#ifdef HOTT_SIM_TEXTMODE
#define HOTT_TEXTMODE_MSG_TEXT_LEN 168
//Text mode msgs type
struct HOTT_TEXTMODE_MSG {
	int8_t start_byte;	//#01 constant value 0x7b
	int8_t fill1;			//#02 constant value 0x00
	int8_t warning_beeps;	//#03 1=A 2=B ...
	int8_t msg_txt[HOTT_TEXTMODE_MSG_TEXT_LEN];	//#04 ASCII text to display to
							// Bit 7 = 1 -> Inverse character display
                            // Display 21x8
	int8_t stop_byte;		//#171 constant value 0x7d
//	int8_t parity;		//#172 Checksum / parity
};
static HOTT_TEXTMODE_MSG hott_txt_msg;
#endif

#ifdef HOTT_SIM_GAM_SENSOR
struct HOTT_GAM_MSG {
	int8_t start_byte;			//#01 start int8_t constant value 0x7c
	int8_t gam_sensor_id; 		//#02 EAM sensort id. constat value 0x8d
	int8_t warning_beeps;			//#03 1=A 2=B ... 0x1a=Z  0 = no alarm
								// Q	Min cell voltage sensor 1
								// R	Min Battery 1 voltage sensor 1
								// J	Max Battery 1 voltage sensor 1
								// F	Min temperature sensor 1
								// H	Max temperature sensor 1
								// S	Min Battery 2 voltage sensor 2
								// K	Max Battery 2 voltage sensor 2
								// G	Min temperature sensor 2
								// I	Max temperature sensor 2
								// W	Max current
								// V	Max capacity mAh
								// P	Min main power voltage
								// X	Max main power voltage
								// O	Min altitude
								// Z	Max altitude
								// C	negative difference m/s too high
								// A	negative difference m/3s too high
								// N	positive difference m/s too high
								// L	positive difference m/3s too high
								// T	Minimum RPM
								// Y	Maximum RPM

	int8_t sensor_id;	            //#04 constant value 0xd0
	int8_t alarm_invers1;			//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	all cell voltage
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Fuel
								// 6	mAh
								// 7	Altitude
	int8_t alarm_invers2;			//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	main power current
								// 1	main power voltage
								// 2	Altitude
								// 3	m/s
								// 4	m/3s
								// 5	unknown
								// 6	unknown
								// 7	"ON" sign/text msg active

	int8_t cell1;					//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	int8_t cell2;					//#08
	int8_t cell3;					//#09
	int8_t cell4;					//#10
	int8_t cell5;					//#11
	int8_t cell6;					//#12
	int16_t batt1;				//#13 battery 1 voltage value. 0.1V steps. 50 = 5.5V
	int16_t batt2;				//#15 battery 2 voltage value. 0.1V steps. 50 = 5.5V
	int8_t temperature1;			//#17 temperature 1. offset of 20. a value of 20 = 0 dC
	int8_t temperature2;			//#18 temperature 2. offset of 20. a value of 20 = 0 dC
	int8_t fuel_procent;			//#19 Fuel capacity in %. Values 0--100
								// graphical display ranges: 0-25% 50% 75% 100%
	int16_t fuel_ml;				//#20 Fuel in ml scale. Full = 65535!
	int16_t rpm;					//#22 RPM in 10 RPM steps. 300 = 3000rpm
	int16_t altitude;			//#24 altitude in meters. offset of 500, 500 = 0m
	int16_t climbrate;			//#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	int8_t climbrate3s;			//#28 climb rate in m/3sec. Value of 120 = 0m/3sec
	int16_t current;				//#29 current in 0.1A steps
	int16_t main_voltage;		//#31 Main power voltage using 0.1V steps
	int16_t batt_cap;			//#33 used battery capacity in 10mAh steps
	int16_t speed;				//#35 (air?) speed in km/h(?) we are using ground speed here per default
	int8_t min_cell_volt;			//#37 minimum cell voltage in 2mV steps. 124 = 2,48V
	int8_t min_cell_volt_num;		//#38 number of the cell with the lowest voltage
	int16_t rpm2;				//#39 RPM in 10 RPM steps. 300 = 3000rpm
	int8_t general_error_number;	//#41 Voice error == 12. TODO: more docu
	int8_t pressure;				//#42 Pressure up to 16bar. 0,1bar scale. 20 = 2bar
	int8_t version;				//#43 version number TODO: more info?
	int8_t stop_byte;				//#44 stop int8_t
//	int8_t parity;				//#45 CRC/Parity
};

static struct HOTT_GAM_MSG hott_gam_msg;
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
#define HOTT_VARIO_MSG_TEXT_LEN	21

struct HOTT_VARIO_MSG {
	int8_t start_byte;			//#01 start int8_t constant value 0x7c
	int8_t vario_sensor_id; 	//#02 VARIO sensort id. constat value 0x89
	int8_t warning_beeps;		//#03 1=A 2=B ...
								// Q	Min cell voltage sensor 1
								// R	Min Battery 1 voltage sensor 1
								// J	Max Battery 1 voltage sensor 1
								// F	Min temperature sensor 1
								// H	Max temperature sensor 1
								// S	Min Battery voltage sensor 2
								// K	Max Battery voltage sensor 2
								// G	Min temperature sensor 2
								// I	Max temperature sensor 2
								// W	Max current
								// V	Max capacity mAh
								// P	Min main power voltage
								// X	Max main power voltage
								// O	Min altitude
								// Z	Max altitude
								// T	Minimum RPM
								// Y	Maximum RPM
								// C	m/s negative difference
								// A	m/3s negative difference


	int8_t sensor_id;	        //#04 constant value 0x90
	int8_t alarm_invers1;		//#05 Inverse display (alarm?) bitmask
								//TODO: more info
	int16_t altitude;			//#06 Altitude low int8_t. In meters. A value of 500 means 0m
	int16_t altitude_max;		//#08 Max. measured altitude low int8_t. In meters. A value of 500 means 0m
	int16_t altitude_min;		//#10 Min. measured altitude low int8_t. In meters. A value of 500 means 0m
	int16_t climbrate;			//#12 Climb rate in m/s. Steps of 0.01m/s. Value of 30000 = 0.00 m/s
	int16_t climbrate3s;		//#14 Climb rate in m/3s. Steps of 0.01m/3s. Value of 30000 = 0.00 m/3s
	int16_t climbrate10s;		//#16 Climb rate m/10s. Steps of 0.01m/10s. Value of 30000 = 0.00 m/10s
	int8_t text_msg[HOTT_VARIO_MSG_TEXT_LEN];			//#18 Free ASCII text message
	int8_t free_char1;			//#39 Free ASCII character.  appears right to home distance
	int8_t free_char2;			//#40 Free ASCII character.  appears right to home direction
	int8_t free_char3;			//#41 Free ASCII character.  appears? TODO: Check where this char appears
	int8_t compass_direction;		//#42 Compass heading in 2 deg steps. 1 = 2 degrees
	int8_t version;				//#43 version number TODO: more info?
	int8_t stop_byte;				//#44 stop int8_t, constant value 0x7d
//	int8_t parity;				//#45 checksum / parity
};

static HOTT_VARIO_MSG hott_vario_msg;
#endif

#ifdef HOTT_SIM_EAM_SENSOR
struct HOTT_EAM_MSG {
	int8_t start_byte;			//#01 start int8_t
	int8_t eam_sensor_id; 		//#02 EAM sensort id. constat value 0x8e
	int8_t warning_beeps;			//#03 1=A 2=B ... or 'A' - 0x40 = 1
								// Q	Min cell voltage sensor 1
								// R	Min Battery 1 voltage sensor 1
								// J	Max Battery 1 voltage sensor 1
								// F	Mim temperature sensor 1
								// H	Max temperature sensor 1
								// S	Min cell voltage sensor 2
								// K	Max cell voltage sensor 2
								// G	Min temperature sensor 2
								// I	Max temperature sensor 2
								// W	Max current
								// V	Max capacity mAh
								// P	Min main power voltage
								// X	Max main power voltage
								// O	Min altitude
								// Z	Max altitude
								// C	(negative) sink rate m/sec to high
								// B	(negative) sink rate m/3sec to high
								// N	climb rate m/sec to high
								// M	climb rate m/3sec to high

	int8_t sensor_id;	            //#04 constant value 0xe0
	int8_t alarm_invers1;			//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	mAh
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Altitude
								// 6	Current
								// 7	Main power voltage
	int8_t alarm_invers2;			//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	m/s
								// 1	m/3s
								// 2	Altitude (duplicate?)
								// 3	m/s	(duplicate?)
								// 4	m/3s (duplicate?)
								// 5	unknown/unused
								// 6	unknown/unused
								// 7	"ON" sign/text msg active

	int8_t cell1_L;				//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	int8_t cell2_L;				//#08
	int8_t cell3_L;				//#09
	int8_t cell4_L;				//#10
	int8_t cell5_L;				//#11
	int8_t cell6_L;				//#12
	int8_t cell7_L;				//#13
	int8_t cell1_H;				//#14 cell 1 voltage high value. 0.02V steps, 124=2.48V
	int8_t cell2_H;				//#15
	int8_t cell3_H;				//#16
	int8_t cell4_H;				//#17
	int8_t cell5_H;				//#18
	int8_t cell6_H;				//#19
	int8_t cell7_H;				//#20

	int16_t batt1_voltage;		//#21 battery 1 voltage lower value. opt. cell8_L 0.02V steps
	int16_t batt2_voltage;		//#23 battery 2 voltage lower value. opt cell8_H value-. 0.02V steps

	int8_t temp1;					//#25 Temperature sensor 1. 0 degrees = 20, 26 dgrees = 46
	int8_t temp2;					//#26 temperature sensor 2

	int16_t altitude;			//#27 Attitude lower value. unit: meters. Value of 500 = 0m

	int16_t current;				//#29 Current in 0.1 steps

	int16_t main_voltage;		//#30 Main power voltage (drive) in 0.1V steps

	int16_t batt_cap;			//#32 used battery capacity in 10mAh steps

	int16_t climbrate;			//#34 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s

	int8_t climbrate3s;			//#36 climbrate in m/3sec. Value of 120 = 0m/3sec

	int16_t rpm;					//#37 RPM. Steps: 10 U/min

	int8_t electric_min;			//#39 Electric minutes. Time does start, when motor current is > 3 A
	int8_t electric_sec;			//#40

	int16_t speed;				//#41 (air?) speed in km/h. Steps 1km/h
	int8_t stop_byte;				//#43 stop int8_t
//	int8_t parity;				//#44 CRC/Parity
};

static HOTT_EAM_MSG hott_eam_msg;
#endif

#ifdef HOTT_SIM_GPS_SENSOR
//HoTT GPS Sensor response to Receiver (?!not?! Smartbox)
struct HOTT_GPS_MSG {
  int8_t start_byte;      //#01 constant value 0x7c
  int8_t gps_sensor_id;   //#02 constant value 0x8a
  int8_t warning_beeps;   //#03 1=A 2=B ...
						// A	Min Speed
						// L	Max Speed
						// O	Min Altitude
						// Z	Max Altitude
						// C	(negative) sink rate m/sec to high
						// B	(negative) sink rate m/3sec to high
						// N	climb rate m/sec to high
						// M	climb rate m/3sec to high
						// D	Max home distance
						//

  int8_t sensor_id;       //#04 constant (?) value 0xa0
  int8_t alarm_invers1;	//#05
  						//TODO: more info
  int8_t alarm_invers2;	//#06  1 = No GPS signal
  								//TODO: more info

  int8_t flight_direction; //#07 flight direction in 2 degreees/step (1 = 2degrees);
  int16_t gps_speed;  //08 km/h

  int8_t pos_NS;  //#10 north = 0, south = 1
  int16_t pos_NS_dm;  //#11 degree minutes ie N48d39 988
  int16_t pos_NS_sec; //#13 position seconds

  int8_t pos_EW;  //#15 east = 0, west = 1
  int16_t pos_EW_dm; //#16 degree minutes ie. E9d25 9360
  int16_t pos_EW_sec;  //#18 position seconds

  int16_t home_distance;  //#20 meters

  int16_t altitude; //#22 meters. Value of 500 = 0m

  int16_t climbrate;  //#24 m/s 0.01m/s resolution. Value of 30000 = 0.00 m/s

  int8_t climbrate3s;  //#26 climbrate in m/3s resolution, value of 120 = 0 m/3s

  int8_t gps_satelites;  //#27 sat count
  int8_t gps_fix_char; //#28 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix. Where appears this char???

  int8_t home_direction; //#29 direction from starting point to Model position (2 degree steps)
  int8_t angle_roll;    //#30 angle roll in 2 degree steps
  int8_t angle_nick;    //#31 angle in 2degree steps
  int8_t angle_compass; //#32 angle in 2degree steps. 1 = 2d, 255 = - 2d (1 int8_t) North = 0d

  int8_t gps_time_h;  //#33 UTC time hours
  int8_t gps_time_m;  //#34 UTC time minutes
  int8_t gps_time_s;  //#35 UTC time seconds
  int8_t gps_time_sss;//#36 UTC time milliseconds

  int16_t msl_altitude;  //#37 mean sea level altitude

  int8_t vibration; //#39 vibrations level in %

  int8_t free_char1;  //#40 appears right to home distance
  int8_t free_char2;  //#41 appears right to home direction
  int8_t free_char3;  //#42 GPS ASCII D=DGPS 2=2D 3=3D -=No Fix
  int8_t version;  //#43
  				 // 0	GPS Graupner #33600
                 // 1	Gyro Receiver
                 // 255 Mikrokopter
  int8_t end_byte;  //#44 constant value 0x7d
//  int8_t parity;    //#45
};

static HOTT_GPS_MSG hott_gps_msg;
#endif


static bool HOTT_REQ_UPDATE_GPS = false;
static bool HOTT_REQ_UPDATE_EAM = false;
static bool HOTT_REQ_UPDATE_VARIO = false;
static bool HOTT_REQ_UPDATE_GAM = false;


// HoTT serial send buffer pointer
static int8_t *_hott_msg_ptr = 0;
// Len of HoTT serial buffer
static int _hott_msg_len = 0;

#if HOTT_TELEMETRY_SERIAL_PORT == 2
void _hott_enable_transmitter() {
  //enables serial transmitter, disables receiver
    UCSR2B &= ~_BV(RXEN2);
    UCSR2B |= _BV(TXEN2);
}

void _hott_enable_receiver() {
   //enables serial receiver, disables transmitter
  UCSR2B &= ~_BV(TXEN2);
  UCSR2B |= _BV(RXEN2);
}
#endif

#if HOTT_TELEMETRY_SERIAL_PORT == 3
void _hott_enable_transmitter() {
  //enables serial transmitter, disables receiver
    UCSR3B &= ~_BV(RXEN3);
    UCSR3B |= _BV(TXEN3);
}

void _hott_enable_receiver() {
   //enables serial receiver, disables transmitter
  UCSR3B &= ~_BV(TXEN3);
  UCSR3B |= _BV(RXEN3);
}
#endif

//*****************************************************************************

/*
  Initializes a HoTT GPS message (Receiver answer type  !Not Smartbox)
*/
void _hott_msgs_init() {
#ifdef HOTT_SIM_GPS_SENSOR
  memset(&hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));
  hott_gps_msg.start_byte = 0x7c;
  hott_gps_msg.gps_sensor_id = 0x8a;
  hott_gps_msg.sensor_id = 0xa0;

  hott_gps_msg.version = 0x00;
  hott_gps_msg.end_byte = 0x7d;
#endif

#ifdef HOTT_SIM_EAM_SENSOR
 memset(&hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));
 hott_eam_msg.start_byte = 0x7c;
 hott_eam_msg.eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
 hott_eam_msg.sensor_id = 0xe0;
 hott_eam_msg.stop_byte = 0x7d;
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
 memset(&hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));
 hott_vario_msg.start_byte = 0x7c;
 hott_vario_msg.vario_sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
 hott_vario_msg.altitude = 500; // 0m
 hott_vario_msg.altitude_max = 500; // 0m
 hott_vario_msg.altitude_min = 500; // 0m
 hott_vario_msg.sensor_id = 0x90;
 hott_vario_msg.stop_byte = 0x7d;
#endif

#ifdef HOTT_SIM_GAM_SENSOR
 memset(&hott_gam_msg, 0, sizeof(struct HOTT_GAM_MSG));
 hott_gam_msg.start_byte = 0x7c;
 hott_gam_msg.gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
 hott_gam_msg.sensor_id = 0xd0;
 hott_gam_msg.stop_byte = 0x7d;
#endif

#ifdef HOTT_SIM_TEXTMODE
  memset(&hott_txt_msg, 0, sizeof(struct HOTT_TEXTMODE_MSG));
  hott_txt_msg.start_byte = 0x7b;
  hott_txt_msg.fill1 = 0x00;
  hott_txt_msg.warning_beeps = 0x00;
  hott_txt_msg.stop_byte = 0x7d;
#endif
}


/*
  called by timer_scheduler
*/
/*
  Called periodically (1ms) by timer scheduler

  @param tnow  timestamp in uSecond
*/
void _hott_serial_scheduler(void) {
	uint32_t tnow = micros();
	static uint32_t _hott_serial_timer;
   _hott_check_serial_data(tnow);	// Check for data request
  if(_hott_msg_ptr == 0) return;   //no data to send
  if(_hott_telemetry_is_sending) {
    //we are sending already, wait for a delay of 2ms between data bytes
    if(tnow - _hott_serial_timer < 3000)  //delay ca. 3,5 mS. 19200 baud = 520uS / int8_t + 3ms required delay
    					// Graupner Specs claims here 2ms but in reality it's 3ms, they using 3ms too...
      return;
  } else {
  	//new data request
  	tnow = micros(); //correct the 5ms  delay in _hott_check_serial_data()...
  }
  _hott_send_telemetry_data();
  _hott_serial_timer = tnow;
}

/*
  Transmitts a HoTT message
*/
void _hott_send_telemetry_data() {
  static int msg_crc = 0;
  if(!_hott_telemetry_is_sending) {
  					// new message to send
    _hott_telemetry_is_sending = true;
    _hott_enable_transmitter();  	//switch to transmit mode
  }

  					//send data
  if(_hott_msg_len == 0) {
    					//all data sent
    _hott_msg_ptr = 0;
    _hott_telemetry_is_sending = false;
    _hott_telemetry_sendig_msgs_id = 0;	//clear current hott msg id
    msg_crc = 0;
    _hott_enable_receiver();
    _HOTT_PORT->flush();
  } else {
    if(--_hott_msg_len != 0) {
       	msg_crc += *_hott_msg_ptr;
	_HOTT_PORT->write(*_hott_msg_ptr++);

    } else {
    	//last int8_t, send crc
	_HOTT_PORT->write((int8_t) (msg_crc));
    }
  }
}

/**********************************************************************************************************************
  Onetime setup for HoTT
*/
void _hott_setup() {

   //init msgs
  _hott_msgs_init();  //set default values

  _HOTT_PORT->begin(19200);
  _hott_enable_receiver();
//check AVR_SCHEDULER_MAX_TIMER_PROCS
  hal.scheduler->register_timer_process(_hott_serial_scheduler);
}


/************************************************************************************************************************
  Check for a valid HoTT requests on serial bus
*/

void _hott_check_serial_data(uint32_t tnow) {
	static uint32_t _hott_serial_request_timer = 0;
	if(_hott_telemetry_is_sending == true) return;
    if(_HOTT_PORT->available() > 1) {
      if(_HOTT_PORT->available() == 2) {
        if(_hott_serial_request_timer == 0) {
        	//new request, check required
        	_hott_serial_request_timer = tnow;	//save timestamp
        	return;
        } else {
        	if(tnow - _hott_serial_request_timer < 4600)	//wait ca. 5ms
        		return;
        	_hott_serial_request_timer = 0;	//clean timer
        }
        // we never reach this point if there is additionally data on bus, so is has to be valid request
        unsigned char c = _HOTT_PORT->read();
        unsigned char addr = _HOTT_PORT->read();
        //ok, we have a valid request, check for address
        switch(c) {
//*****************************************************************************
#ifdef HOTT_SIM_TEXTMODE
            case HOTT_TEXT_MODE_REQUEST_ID:
           //Text mode, handle only if not armed!
           if(!motors.armed())
           {
				hott_txt_msg.start_byte = 0x7b;
				hott_txt_msg.stop_byte = 0x7d;
				uint8_t tmp = (addr >> 4);  // Sensor type
				if(tmp == (HOTT_SIM_TEXTMODE_ADDRESS & 0x0f))   {
					HOTT_Clear_Text_Screen();
					HOTT_HandleTextMode(addr);
					_hott_send_text_msg();   //send message
				}
           }
           break;
#endif
//*****************************************************************************
          case HOTT_BINARY_MODE_REQUEST_ID:
//          cliSerial->printf_P(PSTR("\nHott\n"));
#ifdef HOTT_SIM_GPS_SENSOR
			//GPS module binary mode
            if(addr == HOTT_TELEMETRY_GPS_SENSOR_ID) {
              _hott_send_gps_msg();
              HOTT_REQ_UPDATE_GPS = true;
              break;
            }
#endif
#ifdef HOTT_SIM_EAM_SENSOR
            if(addr == HOTT_TELEMETRY_EAM_SENSOR_ID) {
		      _hott_send_eam_msg();
		      HOTT_REQ_UPDATE_EAM = true;
              break;
		    }
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
            if(addr == HOTT_TELEMETRY_VARIO_SENSOR_ID) {
		      _hott_send_vario_msg();
		       HOTT_REQ_UPDATE_VARIO = true;
              break;
		    }
#endif
#ifdef HOTT_SIM_GAM_SENSOR
            if(addr == HOTT_TELEMETRY_GAM_SENSOR_ID) {
		      _hott_send_gam_msg();
		       HOTT_REQ_UPDATE_GAM = true;
              break;
		    }
#endif

//END: case HOTT_BINARY_MODE_REQUEST_ID:
           break;
//*****************************************************************************
          default:
            break;
        }
      } else {
        //ignore data from other sensors
        _HOTT_PORT->flush();
        _hott_serial_request_timer = 0;
      }
    }
}

void _hott_send_msg(int8_t *buffer, int len) {
  if(_hott_telemetry_is_sending == true) return;
  _hott_msg_ptr = buffer;
  _hott_msg_len = len +1; //len + 1 byte for crc
  _hott_telemetry_sendig_msgs_id = buffer[1];   //HoTT msgs id is the 2nd byte
}

#ifdef HOTT_SIM_GAM_SENSOR
void _hott_send_gam_msg() {
	_hott_send_msg((int8_t *)&hott_gam_msg, sizeof(struct HOTT_GAM_MSG));
}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
void _hott_send_vario_msg() {
	_hott_send_msg((int8_t *)&hott_vario_msg, sizeof(struct HOTT_VARIO_MSG));
}
#endif

#ifdef HOTT_SIM_GPS_SENSOR
void _hott_send_gps_msg() {
  _hott_send_msg((int8_t *)&hott_gps_msg, sizeof(struct HOTT_GPS_MSG));
}
#endif

#ifdef HOTT_SIM_EAM_SENSOR
//Send EMA sensor data
void _hott_send_eam_msg() {
  _hott_send_msg((int8_t *)&hott_eam_msg, sizeof(struct HOTT_EAM_MSG));
}
#endif

#ifdef HOTT_SIM_TEXTMODE
void _hott_send_text_msg() {
  _hott_send_msg((int8_t *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
}
#endif

#define HOTT_CELSIUS_ZERO 20   // 0 degrees C in Hott spec
#define HOTT_APM_GET_TEMPERATURE (int8_t)((int8_t)barometer.get_temperature() + HOTT_CELSIUS_ZERO )
#define HOTT_APM_GET_PRESSURE	(int8_t)((barometer.get_pressure()/10132.5))
#define HOTT_APM_GET_CURRENT	(int16_t)(battery.current_amps()*10.0)
#define HOTT_APM_GET_COMPASS_DIRECTION	ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) / 2
#define HOTT_APM_GET_COMPASS_YAW ((ahrs.yaw_sensor /100) %360) / 2
//#define HOTT_APM_GET_GPS_GROUND_SPEED  (int16_t)(((float)g_gps->ground_speed_cm * 0.036))
//#define HOTT_APM_GET_GPS_GROUND_SPEED  (int16_t)(((float)gps.ground_speed_cm() * 0.036))
#define HOTT_APM_GET_GPS_GROUND_SPEED  (int16_t)(gps.ground_speed_cm()/28)
//#define HOTT_APM_GET_GPS_ALTITUDE (int16_t)(gps.altitude_cm() / 100)
#define HOTT_APM_GET_GPS_ALTITUDE (int16_t)(gps.location().alt / 100)
//#define HOTT_APM_GET_GPS_GROUND_COURSE (int8_t)(g_gps->ground_course_cd / 200)
#define HOTT_APM_GET_GPS_GROUND_COURSE (int8_t)(gps.ground_course_cd() / 200)
#define HOTT_APM_GET_BATTERY_VOLTAGE (int16_t)(battery.voltage() * 10.0)
#define HOTT_APM_GET_BATTERY_USED (int16_t)(battery.current_total_mah() / 10.0)
#define HOTT_APM_GET_BATTERY_PERCENT battery.capacity_remaining_pct()
#define HOTT_APM_GET_LOCAL_ALTITUDE (int16_t)((current_loc.alt - home.alt) / 100)+500
#define HOTT_APM_GET_CLIMBRATE           (int16_t)(30000 + climb_rate)               // climbrate in cm/s, 3000=0
#define HOTT_APM_GET_VARIO_CLIMBRATE_3S          (int16_t)(30000 + climb_rate *3)
#define HOTT_APM_GET_VARIO_CLIMBRATE_10S         (int16_t)(30000 + climb_rate *10)
#define HOTT_APM_GET_GPS_CLIMBRATE_3S    (int8_t)(120 + (climb_rate / 100)*3)   // climbrate in m/3s, 120=0

#ifdef HOTT_SIM_GAM_SENSOR
void _hott_update_gam_msg() {
	hott_gam_msg.temperature1 = HOTT_APM_GET_TEMPERATURE;
	hott_gam_msg.temperature2 = hott_gam_msg.temperature1; //HOTT_CELSIUS_ZERO;
	hott_gam_msg.pressure = HOTT_APM_GET_PRESSURE;
	hott_gam_msg.altitude = HOTT_APM_GET_LOCAL_ALTITUDE;
    hott_gam_msg.climbrate = HOTT_APM_GET_CLIMBRATE;
    hott_gam_msg.climbrate3s = HOTT_APM_GET_GPS_CLIMBRATE_3S;  // 0 m/3s using filtered data here
	hott_gam_msg.current = HOTT_APM_GET_CURRENT;
	hott_gam_msg.main_voltage = HOTT_APM_GET_BATTERY_VOLTAGE;
	hott_gam_msg.batt_cap = HOTT_APM_GET_BATTERY_USED;
	hott_gam_msg.fuel_procent = HOTT_APM_GET_BATTERY_PERCENT;	// my fuel are electrons :)
	hott_gam_msg.speed = HOTT_APM_GET_GPS_GROUND_SPEED;
	// TODO hott_gam_msg.fuel_ml =  battery.capacity - battery.current_total_mah();
	hott_gam_msg.fuel_ml =  battery.current_total_mah(); // Now showing used batt capacity in mAh
    //display ON when motors are armed

   // if (motors.armed()) {
   //    hott_gam_msg.alarm_invers2 |= 0x80;
   // } else {
   //     hott_gam_msg.alarm_invers2 &= 0x7f;
   // }
}
#endif

#ifdef HOTT_SIM_EAM_SENSOR
//Update EAM sensor data
void _hott_update_eam_msg() {

	static uint32_t	electric_msecs =0;	// electric time ( time with batt current > 2A) in milliseconds
	static uint32_t last_msecs=0; 	        // previous call msecs value, ==0 if in previous call current was < 2A and we weren't counting

    hott_eam_msg.main_voltage =  HOTT_APM_GET_BATTERY_VOLTAGE;
    hott_eam_msg.batt1_voltage = hott_eam_msg.main_voltage;
    hott_eam_msg.batt2_voltage = hott_eam_msg.main_voltage;
	hott_eam_msg.temp1 = HOTT_APM_GET_TEMPERATURE;
    hott_eam_msg.temp2 = hott_eam_msg.temp1;
    hott_eam_msg.altitude = HOTT_APM_GET_LOCAL_ALTITUDE;
	hott_eam_msg.current = HOTT_APM_GET_CURRENT;
    hott_eam_msg.main_voltage = hott_eam_msg.batt1_voltage;
	hott_eam_msg.batt_cap = HOTT_APM_GET_BATTERY_USED;
	hott_eam_msg.speed = HOTT_APM_GET_GPS_GROUND_SPEED;

    hott_eam_msg.climbrate = HOTT_APM_GET_CLIMBRATE;
    hott_eam_msg.climbrate3s = HOTT_APM_GET_GPS_CLIMBRATE_3S;  // 0 m/3s using filtered data here

  	{
        register uint32_t current_msecs = millis(); //   g_gps->time_week_ms;

       if(last_msecs) {                       // if we were counting time since previous call, update time
            electric_msecs+=(current_msecs - last_msecs);
            hott_eam_msg.electric_min=(int8_t)(electric_msecs / 60000); // milliseconds in minute
            hott_eam_msg.electric_sec=(int8_t)((electric_msecs / 1000) % 60);
	    }

  	    if(battery.current_amps() >= 2) last_msecs=current_msecs;  // it there is battery current, update timestamp: we continue counting
	    else last_msecs=0;					// if no current, clear timestamp, stop counting
	}
    //display ON when motors are armed
   // if (motors.armed()) {
   //    hott_eam_msg.alarm_invers2 |= 0x80;
   //  } else {
   //    hott_eam_msg.alarm_invers2 &= 0x7f;
   //  }
}

#endif

#ifdef HOTT_SIM_GPS_SENSOR
// Updates GPS message values
void _hott_update_gps_msg() {
  // update GPS telemetry data

  hott_gps_msg.msl_altitude = HOTT_APM_GET_GPS_ALTITUDE;  //meters above sea level
  hott_gps_msg.flight_direction = HOTT_APM_GET_GPS_GROUND_COURSE;  // in 2* steps
  hott_gps_msg.gps_speed = HOTT_APM_GET_GPS_GROUND_SPEED;
  hott_gps_msg.altitude = HOTT_APM_GET_LOCAL_ALTITUDE;  //meters above ground
  hott_gps_msg.climbrate = HOTT_APM_GET_CLIMBRATE;
  hott_gps_msg.climbrate3s = HOTT_APM_GET_GPS_CLIMBRATE_3S;
  hott_gps_msg.gps_satelites = (int8_t)gps.num_sats();
  hott_gps_msg.angle_roll = (int8_t)(ahrs.roll_sensor / 200);
  hott_gps_msg.angle_nick = (int8_t)(ahrs.pitch_sensor / 200);

  hott_gps_msg.angle_compass = HOTT_APM_GET_COMPASS_YAW;

  switch(control_mode) {
	case AUTO:
        	  //Use home direction field to display direction and distance to next waypoint
          {
          	  // int32_t dist = wp_nav.get_distance_to_target();
	          //hott_gps_msg.home_distance = dist < 0 ? 0 : (int16_t)(dist / 100);
              hott_gps_msg.home_distance = (int16_t)(wp_nav.get_wp_distance_to_destination() /100);
    	       //	  hott_gps_msg.home_direction = wp_nav.get_bearing_to_destination() / 200; //get_bearing() return value in degrees * 100
              hott_gps_msg.home_direction = (int8_t)(wp_nav.get_wp_bearing_to_destination() /200);
			  //Display WP to mark the change of meaning!
           	hott_gps_msg.free_char1 ='W';
           	hott_gps_msg.free_char2 ='P';
           }
           break;

        default:
          //Display Home direction and distance
          {
            //float dist = get_distance(current_loc, home);
            //hott_gps_msg.home_distance = dist < 0 ? 0 : (int16_t)dist;
			hott_gps_msg.home_distance = (int16_t)(home_distance/100);
			//hott_gps_msg.home_direction = get_bearing_cd(current_loc, home) / 200; //get_bearing() return value in degrees * 100
            hott_gps_msg.home_direction = (int8_t)(home_bearing /200);
			hott_gps_msg.free_char1 = 0;
            hott_gps_msg.free_char2 = 0;
            break;
          }
	}

 if(gps.status() == AP_GPS::GPS_OK_FIX_3D) {
    hott_gps_msg.alarm_invers2 = 0;
    hott_gps_msg.gps_fix_char = '3';
    hott_gps_msg.free_char3 = '3';  //3D Fix according to specs...
  } else {
    //No GPS Fix
    hott_gps_msg.alarm_invers2 = 1;
    hott_gps_msg.gps_fix_char = '-';
    hott_gps_msg.free_char3 = '-';
    hott_gps_msg.home_distance = (int16_t)0; // set distance to 0 since there is no GPS signal
    hott_gps_msg.home_direction = 0;                                  // also no direction without GPS fix
  }

    //latitude

    register uint32_t   coord = labs(current_loc.lat);     // get current coord in D.ddddddd format (degrees*10**7)
    hott_gps_msg.pos_NS = (int8_t)(current_loc.lat < 0);  // N=0, S=1
    hott_gps_msg.pos_NS_dm = coord / 10000000;            // get integer degrees
    coord %= 10000000;                                    // get rid of degrees
    coord *= 6;                                           // translate to minutes * 10**6
    hott_gps_msg.pos_NS_dm = hott_gps_msg.pos_NS_dm *100 + coord / 1000000; // translate to deg*100+minutes
    hott_gps_msg.pos_NS_sec = ((coord % 1000000)*3)/50;    //translate to seconds * 10**5



    // Longitude

    coord = labs(current_loc.lng);                         // get current coord in D.ddddddd format (degrees*10**7)
    hott_gps_msg.pos_EW = (int8_t)(current_loc.lng < 0) ;  // E=0, W=1
    hott_gps_msg.pos_EW_dm = coord / 10000000;            // get integer degrees
    coord %= 10000000;                                    // get rid of degrees
    coord *= 6;                                           // translate to minutes * 10**6
    hott_gps_msg.pos_EW_dm = hott_gps_msg.pos_EW_dm *100 + coord / 1000000; // translate to deg*100+minutes
    hott_gps_msg.pos_EW_sec = ((coord % 1000000)*3)/50;    //translate to seconds * 10**5


    // prepare time
    uint32_t t = gps.last_fix_time_ms(0);
    hott_gps_msg.gps_time_sss = t % 1000;    // milliseconds are remainder from division by 1000
    t /= 1000;                               // throw away milliseconds
    hott_gps_msg.gps_time_s = t % 60;        // seconds are remainder form division by 60
    t /= 60;                                 // throw away seconds
    hott_gps_msg.gps_time_m = t % 60;        // minutes are remainder from divisiion by 60
    hott_gps_msg.gps_time_h = t / 60;        // throw away minutes, we have hours

}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR

#define HOTT_FLIGHT_MODE_STRING_LEN 11
#define HOTT_ARMED_STR_LEN 9
#define HOTT_DISPLAY_WP_NUM_OFFSET  17

static void u8toa10r3(uint8_t n, char s[])      // converts uint8_t to 3-digit char string overwriting chars
 {
     int8_t i=2;

     do {
         s[i--] = n % 10 + '0';
     } while (((n /= 10) > 0) && (i > 0));
 }

static const char hott_flight_mode_strings[NUM_MODES+1][HOTT_FLIGHT_MODE_STRING_LEN+1] PROGMEM = {
    "STABILIZE  ",                // 0
    "ACRO       ",                // 1
    "ALT HOLD   ",                // 2
    "AUTO ->    ",                // 3
    "GUIDED     ",                // 4
    "LOITER     ",                // 5
    "RTL        ",                // 6
    "CIRCLE     ",                // 7
    "POSITION   ",                // 8
    "LAND       ",                // 9
    "OF LOITER  ",                // 10
    "DRIFT      ",                // 11
    "TOY A      ",                // 12
    "SPORT      ",                // 13
    "FLIP       ",                // 14
    "AUTOTUNE   ",                // 15
    "HYBRID     ",		  // 16
    "???????????"
};

const char hott_ARMED_STR[] PROGMEM =     "  ARMED  ";
const char hott_DISARMED_STR[] PROGMEM	= "DISARMED ";


void _hott_update_vario_msg() {

    hott_vario_msg.altitude = HOTT_APM_GET_LOCAL_ALTITUDE;

	if ( hott_vario_msg.altitude > hott_vario_msg.altitude_max) hott_vario_msg.altitude_max =  hott_vario_msg.altitude;
	if ( hott_vario_msg.altitude < hott_vario_msg.altitude_min) hott_vario_msg.altitude_min =  hott_vario_msg.altitude;

    hott_vario_msg.climbrate = HOTT_APM_GET_CLIMBRATE;
    hott_vario_msg.climbrate3s = HOTT_APM_GET_VARIO_CLIMBRATE_3S;
    hott_vario_msg.climbrate10s = HOTT_APM_GET_VARIO_CLIMBRATE_10S;

	hott_vario_msg.compass_direction = HOTT_APM_GET_COMPASS_YAW; // use ahrs yaw sensor

	//Free text processing
	if(control_mode > NUM_MODES) control_mode = NUM_MODES;


	memcpy_P((char*)hott_vario_msg.text_msg, (motors.armed()?(char *)hott_ARMED_STR:(char *)hott_DISARMED_STR), HOTT_ARMED_STR_LEN);
	memcpy_P((char*)&hott_vario_msg.text_msg[HOTT_ARMED_STR_LEN], hott_flight_mode_strings[control_mode],HOTT_FLIGHT_MODE_STRING_LEN);
//    if ( control_mode == AUTO) u8toa10r3((uint8_t)g.command_index,(char*)&hott_vario_msg.text_msg[HOTT_DISPLAY_WP_NUM_OFFSET]);
    if ( control_mode == AUTO) u8toa10r3((uint8_t)(mission.get_current_nav_cmd().index),(char*)&hott_vario_msg.text_msg[HOTT_DISPLAY_WP_NUM_OFFSET]);

	}
#endif  // HOTT_SIM_VARIO_SENSOR


#ifdef HOTT_SIM_TEXTMODE
/*
  Converts a "black on white" string into inverted one "white on black"
  Works in text mode only!
*/
char * _hott_invert_chars(char *str, int cnt) {
	if(str == 0) return str;
	int len = strlen(str);
	if((len < cnt)  && cnt > 0) len = cnt;
	for(int i=0; i< len; i++) {
       //    str[i] = (int8_t)(0x80 + (int8_t)str[i]);
        str[i] |= 0x80;
	}
	return str;
}

char * _hott_invert_all_chars(char *str) {
    return _hott_invert_chars(str, 0);
}
#endif

/*
  Updates HoTT Messages values. Called in slow loop
*/
void _hott_update_telemetry_data() {
    //no update while sending
    static uint8_t update_counter = 0;

    switch ( update_counter )
      {
          #ifdef HOTT_SIM_GPS_SENSOR
          case 0 :
                  if((_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GPS_SENSOR_ID) &&  HOTT_REQ_UPDATE_GPS == true )
                  {
                      _hott_update_gps_msg();
                      HOTT_REQ_UPDATE_GPS = false;
                      break;
                  }
          #endif

          #ifdef HOTT_SIM_EAM_SENSOR
          case 1 :
                  if((_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_EAM_SENSOR_ID) &&  HOTT_REQ_UPDATE_EAM == true)
                  {
                      _hott_update_eam_msg();
                      HOTT_REQ_UPDATE_EAM = false;
                      break;
                  }
          #endif

          #ifdef HOTT_SIM_VARIO_SENSOR
          case 2 :
                  if((_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_VARIO_SENSOR_ID) &&  HOTT_REQ_UPDATE_VARIO == true)
                  {
                      _hott_update_vario_msg();
                      HOTT_REQ_UPDATE_VARIO = false;
                      break;
                  }
          #endif

          #ifdef HOTT_SIM_GAM_SENSOR
          case 3 :
                  if((_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GAM_SENSOR_ID) &&  HOTT_REQ_UPDATE_GAM == true)
                  {
                      _hott_update_gam_msg();
                      HOTT_REQ_UPDATE_GAM = false;
                      break;
                  }
         #endif
         case 4 :
              _hott_check_alarm();
              _hott_alarm_scheduler();
              _hott_update_replay_queue();
              break;


      }
      if (++update_counter > 4) update_counter =0;

}

//****************************************************************************************
// Alarm stuff
//
//HoTT alarm macro
#define HOTT_ALARM_NUM(a) (a-0x40)

struct _hott_alarm_event_T {
	uint16_t alarm_time; 		//Alarm play time in 1sec units
	uint16_t alarm_time_replay;	//Alarm repeat time in 1sec. units. 0 -> One time alarm
								//forces a delay between new alarms of the same kind
	uint8_t visual_alarm1;		//Visual alarm bitmask
	uint8_t visual_alarm2;		//Visual alarm bitmask
	uint8_t alarm_num;			//Alarm number 0..255 (A-Z)
	uint8_t alarm_profile;		//profile id ie HOTT_TELEMETRY_GPS_SENSOR_ID
};
typedef struct _hott_alarm_event_T _hott_alarm_event;

#define HOTT_ALARM_QUEUE_MAX		5
//TODO: better queueing solution
static _hott_alarm_event _hott_alarm_queue[HOTT_ALARM_QUEUE_MAX];
static _hott_alarm_event _hott_alarm_replay_queue[HOTT_ALARM_QUEUE_MAX];
static uint8_t _hott_alarmCnt = 0;
static uint8_t _hott_alarm_ReplayCnt = 0;

//
// checks if an alarm exists in active queue
//
bool _hott_alarm_active_exists(struct _hott_alarm_event_T *alarm) {
	//check active alarms
	for(uint8_t i=0; i<_hott_alarmCnt; i++) {
		if(_hott_alarm_queue[i].alarm_num == alarm->alarm_num &&
			_hott_alarm_queue[i].alarm_profile == alarm->alarm_profile) {
			//alarm exists.
			return true;
		}
	}
	return false;
}

//
// checks if an alarm exists in replay queue
//
bool _hott_alarm_replay_exists(struct _hott_alarm_event_T *alarm) {
	//check replay delay queue
	for(uint8_t i=0; i<_hott_alarm_ReplayCnt; i++) {
		if(_hott_alarm_replay_queue[i].alarm_num == alarm->alarm_num &&
			_hott_alarm_replay_queue[i].alarm_profile == alarm->alarm_profile) {
			//alarm exists
			return true;
		}
	}
	return false;
}

//
// checks if an alarm exists
//
bool _hoot_alarm_exists(struct _hott_alarm_event_T *alarm) {
	if(_hott_alarm_active_exists(alarm))
		return true;
	if(_hott_alarm_replay_exists(alarm))
		return true;
	return false;
}

//
// adds an alarm to active queue
//
void _hott_add_alarm(struct _hott_alarm_event_T *alarm) {
	if(alarm == 0)
		return;
	if(_hott_alarmCnt >= HOTT_ALARM_QUEUE_MAX)
		return;	//no more space left...
	if(_hoot_alarm_exists(alarm))
		return;
	// we have a new alarm
	memcpy(&_hott_alarm_queue[_hott_alarmCnt++], alarm, sizeof(struct _hott_alarm_event_T));
}

//
// adds an alarm to replay queue
//
void _hott_add_replay_alarm(struct _hott_alarm_event_T *alarm) {
	if(alarm == 0)
		return;
	if(_hott_alarm_ReplayCnt >= HOTT_ALARM_QUEUE_MAX)
		return;	//no more space left...
	if(_hott_alarm_replay_exists(alarm))
		return;
	// we have a new alarm
	memcpy(&_hott_alarm_replay_queue[_hott_alarm_ReplayCnt++], alarm, sizeof(struct _hott_alarm_event_T));
}

//
//removes an alarm from active queue
//first alarm at offset 1
//
void _hott_remove_alarm(uint8_t num) {
	if(num > _hott_alarmCnt || num == 0)	//has to be > 0
		return; // possibile error

	if(_hott_alarmCnt != 1) {
		memcpy(&_hott_alarm_queue[num-1], &_hott_alarm_queue[num], sizeof(struct _hott_alarm_event_T) * (_hott_alarmCnt - num) );
	}
	--_hott_alarmCnt;
}

//
//removes an alarm from replay queue
//first alarm at offset 1
//
void _hott_remove_replay_alarm(uint8_t num) {
	if(num > _hott_alarm_ReplayCnt || num == 0)	//has to be > 0
		return; // possibile error

	if(_hott_alarm_ReplayCnt != 1) {
		memcpy(&_hott_alarm_replay_queue[num-1], &_hott_alarm_replay_queue[num], sizeof(struct _hott_alarm_event_T) * (_hott_alarm_ReplayCnt - num) );
	}
	--_hott_alarm_ReplayCnt;
}

//
// Updates replay delay queue
//
void _hott_update_replay_queue(void) {
static uint8_t t = 0;
	if(++t < 50)
		return;
	//every second
	t = 0;

	for(uint8_t i=0; i< _hott_alarm_ReplayCnt; i++) {
		if(--_hott_alarm_replay_queue[i].alarm_time_replay == 0) {
			//remove it
			_hott_remove_replay_alarm(i+1);
			i--;
			continue;
		}
	}
}

//
// Sets a voice alarm value
//
void _hott_set_voice_alarm(uint8_t profile, uint8_t value) {
	switch(profile) {
#ifdef HOTT_SIM_EAM_SENSOR
  		case HOTT_TELEMETRY_EAM_SENSOR_ID:
  			if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_EAM_SENSOR_ID)
				hott_eam_msg.warning_beeps = value;
			break;
#endif
#ifdef HOTT_SIM_GPS_SENSOR
		case HOTT_TELEMETRY_GPS_SENSOR_ID:
			if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GPS_SENSOR_ID)
				hott_gps_msg.warning_beeps = value;
			break;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
		case HOTT_TELEMETRY_VARIO_SENSOR_ID:
			if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_VARIO_SENSOR_ID)
				hott_vario_msg.warning_beeps = value;
			break;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
		case HOTT_TELEMETRY_GAM_SENSOR_ID:
			if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GAM_SENSOR_ID)
				hott_gam_msg.warning_beeps = value;
			break;
#endif
		default:
			break;
	}
}

//
// active alarm scheduler
//
void _hott_alarm_scheduler() {
	static uint8_t activeAlarmTimer = 3* 50;
	static uint8_t activeAlarm = 0;

	if(_hott_alarmCnt < 1)
		return;	//no alarms

	uint8_t vEam = 0;
	uint8_t vEam2 = 0;
	uint8_t vVario = 0;
	uint8_t vGps = 0;
	uint8_t vGps2 = 0;
#ifdef HOTT_SIM_GAM_SENSOR
	uint8_t vGam = 0;
	uint8_t vGam2 = 0;
#endif

	for(uint8_t i = 0; i< _hott_alarmCnt; i++) {
		if(_hott_alarm_queue[i].alarm_time == 0) {
			//end of alarm, remove it
			//no check for current msg to be send, so maybe the crc will be wrong
			_hott_set_voice_alarm(_hott_alarm_queue[i].alarm_profile, 0);
			if(_hott_alarm_queue[i].alarm_time_replay != 0)
				_hott_add_replay_alarm(&_hott_alarm_queue[i]);
			_hott_remove_alarm(i+1);	//first alarm at offset 1
			--i;	//correct counter
			continue;
		}

		//
		switch(_hott_alarm_queue[i].alarm_profile) {
#ifdef HOTT_SIM_EAM_SENSOR
			case HOTT_TELEMETRY_EAM_SENSOR_ID:
				vEam |= _hott_alarm_queue[i].visual_alarm1;
				vEam2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif
#ifdef HOTT_SIM_GPS_SENSOR
			case HOTT_TELEMETRY_GPS_SENSOR_ID:
				vGps |= _hott_alarm_queue[i].visual_alarm1;
				vGps2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
			case HOTT_TELEMETRY_VARIO_SENSOR_ID:
				vVario |= _hott_alarm_queue[i].visual_alarm1;
				break;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
			case HOTT_TELEMETRY_GAM_SENSOR_ID:
				vGam |= _hott_alarm_queue[i].visual_alarm1;
				vGam2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif
			default:
				break;
		}
	} //end: visual alarm loop
#ifdef HOTT_SIM_EAM_SENSOR
	// Set all visual alarms
	if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_EAM_SENSOR_ID) {
		hott_eam_msg.alarm_invers1 |= vEam;
		hott_eam_msg.alarm_invers2 |= vEam2;
	}
#endif
#ifdef HOTT_SIM_GAM_SENSOR
	if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GAM_SENSOR_ID) {
		hott_gam_msg.alarm_invers1 |= vGam;
		hott_gam_msg.alarm_invers2 |= vGam2;
	}
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
	if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_VARIO_SENSOR_ID) {
		hott_vario_msg.alarm_invers1 |= vVario;
	}
#endif
#ifdef HOTT_SIM_GPS_SENSOR
	if(_hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GPS_SENSOR_ID) {
		hott_gps_msg.alarm_invers1 |= vGps;
		hott_gps_msg.alarm_invers2 |= vGps2;
	}
#endif
	if(activeAlarm != 0) { //is an alarm active
		if ( ++activeAlarmTimer % 50 == 0 ) {	//every 1sec
			_hott_alarm_queue[activeAlarm-1].alarm_time--;
		}
		if ( activeAlarmTimer < 50 * 2) //alter alarm every 2 sec
			return;
	}
	activeAlarmTimer = 0;

	if(++activeAlarm > _hott_alarmCnt) {
		activeAlarm = 1;
	}
	if(_hott_alarmCnt <= 0) {
		activeAlarm = 0;
		return;
	}
	_hott_set_voice_alarm(_hott_alarm_queue[activeAlarm-1].alarm_profile, _hott_alarm_queue[activeAlarm-1].alarm_num);
}

//****************************************************************************************
// Sensor specific code
//
#ifdef HOTT_SIM_EAM_SENSOR
//
// triggers max consumed mAh alarm
//
void _hott_eam_check_mAh() {
	_hott_alarm_event _hott_ema_alarm_event;
	if( (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
		_hott_ema_alarm_event.alarm_time = 6;	//1sec units
		_hott_ema_alarm_event.alarm_time_replay = 15;	//1sec units
		_hott_ema_alarm_event.visual_alarm1 = 0x01;	//blink mAh
		_hott_ema_alarm_event.visual_alarm2 = 0;
		_hott_ema_alarm_event.alarm_num = HOTT_ALARM_NUM('V');
		_hott_ema_alarm_event.alarm_profile = HOTT_TELEMETRY_EAM_SENSOR_ID;
		_hott_add_alarm(&_hott_ema_alarm_event);
	}

}

//
// triggers low main power voltage alarm
//
void _hott_eam_check_mainPower() {
	//voltage sensor needs some time at startup
	if((millis() > 10000) && (battery.monitoring()) && (battery.voltage() < g.fs_batt_voltage)) {
		_hott_alarm_event _hott_ema_alarm_event;
		_hott_ema_alarm_event.alarm_time = 6;	//1sec units
		_hott_ema_alarm_event.alarm_time_replay = 30; //1sec unit
		_hott_ema_alarm_event.visual_alarm1 = 0x80;	//blink main power
		_hott_ema_alarm_event.visual_alarm2 = 0;
		_hott_ema_alarm_event.alarm_num = HOTT_ALARM_NUM('P');
		_hott_ema_alarm_event.alarm_profile = HOTT_TELEMETRY_EAM_SENSOR_ID;
		_hott_add_alarm(&_hott_ema_alarm_event);
	}
}
#endif

//
//	alarm triggers to check
//
void _hott_check_alarm()  {
#ifdef HOTT_SIM_EAM_SENSOR
	_hott_eam_check_mAh();
	_hott_eam_check_mainPower();
#endif
}


#endif	//End of #ifdef HOTT_TELEMETRY
