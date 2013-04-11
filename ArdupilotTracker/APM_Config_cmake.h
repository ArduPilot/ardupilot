//automatically generated, do not edit
#define OFF 0
#define ON 1

#define APM_PROGRAMMING_PORT /dev/ttyACM0 // Programming upload port?
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2 // APM Hardware?
//#define APM2_BETA_HARDWARE // Is this an APM 2.0 Beta board?
#define APM_PROCESSOR mega2560 // ArduPilotMega processor (2560 for APM2 and later APM1)?
#define LOGGING_ENABLED OFF // Enable logging?
#define GPS_PROTOCOL GPS_PROTOCOL_AUTO // GPS protocol?
#define AIRSPEED_SENSOR OFF // Enable airspeed sensor?
#define PITOT_ENABLED OFF // Enable pitot static system?
#define SONAR_ENABLED OFF // Enable sonar?
#define AIRSPEED_RATIO 1.9936 // Airspeed ratio?
#define HIL_MODE HIL_MODE_DISABLED // Hardware-in-the-loop- mode?
#define MAV_SYSTEM_ID 100 // MAVLink System ID?
#define MAV_TARGET_ID 1 // MAVLink System ID of drone to track?
#define SERIAL0_BAUD 115200 // Serial 0 baudrate?
#define SERIAL3_BAUD 57600 // Serial 3 baudrate?
#define BATTERY_EVENT OFF // Enable low voltage/ high discharge warnings?
#define LOW_VOLTAGE 9.6 // Voltage to consider low (volts)?
#define VOLT_DIV_RATIO 3.56 // Voltage division ratio?
#define CUR_AMPS_PER_VOLT 27.32 // Current amps/volt?
#define CUR_AMPS_OFFSET 0.0 // Current amps offset?
#define HIGH_DISCHARGE 1760 // What to consider high discharge rate (milliamps)?
#define INPUT_VOLTAGE 4.68 // Voltage measured at the process (V)? (affects ADC measurements)
#define FLIGHT_MODE_CHANNEL 8 // The radio channel to control the flight mode.
#define FLIGHT_MODE_1 RTL // Flight mode 1 (pwm 1165 ms)?
#define FLIGHT_MODE_2 RTL // Flight mode 2 (pwm 1295 ms)?
#define FLIGHT_MODE_3 STABILIZE // Flight mode 3 (pwm 1425 ms)?
#define FLIGHT_MODE_4 STABILIZE // Flight mode 4 (pwm 1555 ms)?
#define FLIGHT_MODE_5 MANUAL // Flight mode 5 (pwm 1685 ms)?
#define FLIGHT_MODE_6 MANUAL // Flight mode 6 (pwm 1815 ms)?

#define MOBILE 2
// OFF is off. ON is raw MAVLink. MOBILE is cellular modem.
#define SERIAL3_MODE MOBILE
