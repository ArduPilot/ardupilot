#pragma once
//

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RSSI/AP_RSSI.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <GCS_MAVLink/GCS.h>



#define LIGHTTELEMETRY_START1       0x24 //$
#define LIGHTTELEMETRY_START2       0x54 //T
#define LIGHTTELEMETRY_GFRAME     0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME     0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME     0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LTM_GFRAME_SIZE                  18
#define LTM_AFRAME_SIZE                  10
#define LTM_SFRAME_SIZE                  11
#define ROLL_LIMIT                               0x7FF
#define PITCH_LIMIT                              0x3FF
#define YAW_LIMIT                               0x7FF

class AP_LTM{
public:
	// Constructor
	AP_LTM(const AP_AHRS &ahrs, const AP_BattMonitor &battery);
	
     /* Do not allow copies */
    AP_LTM(const AP_LTM &other) = delete;
    AP_LTM &operator=(const AP_LTM&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager, 
			   const uint8_t mav_type);
			   
/* 	void update_control_mode(uint8_t mode){
		_uav.flightmode = mode;
	} */
	
private:
    AP_HAL::UARTDriver *_port;                             // UART 
    AP_SerialManager::SerialProtocol _protocol;      // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
	const AP_AHRS &_ahrs; 
	const AP_BattMonitor &_battery;
	
	uint8_t    _ltm_scheduler;
	uint32_t  _last_frame_ms;

    struct
    {
        int32_t   lat;						// latitude
		int32_t   lon;						// longtitude
        int32_t   alt;						// altitude
		uint8_t   sats_visible;				// number of visible gps sats
		uint8_t   fix_type;					// gps fix type
		uint8_t   gndspeed;					// gps ground speed
		int16_t   pitch;                 	// attitude pitch
		int16_t   roll;                  	// attitude roll
		int16_t   heading;                	// heading
		uint16_t  volt;                    	// battery voltage (mv)
		uint16_t  amp;                		// actual current in mAh
		uint8_t   rssi;                  	// radio RSSI (%)
		uint8_t   airspeed;               	// Airspeed sensor (m/s)
		uint8_t   armstat;                  // 0: disarmed, 1: armed
		uint8_t   failsafe;               	// 0: normal,   1: failsafe 
		uint8_t   flightmode;            	// Flight mode
		uint8_t   mav_type;              // Vehicle type
    } _uav;
       	
	void send_Gframe(void);
	void send_Sframe(void);
	void send_Aframe(void);
	void send_LTM(void);
	void send_LTM_frame(uint8_t LTPacket[], uint8_t LTPacket_size);
	void get_position(void);
	void get_sense(void);
	void get_attitude(void);
    void tick(void);  						// tick - main call to send updates to transmitter (called by scheduler at 1kHz)
	
};