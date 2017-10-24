/*******************************************************************************
* roboticscape.h
*
* This contains the complete Robotics Cape API. All functions declared here can 
* be executed by linking to /usr/lib/robotics_cape.so
*
* All functions return 0 on success or -1 on failure unless otherwise stated.
*
* James Strawson - 2016
*******************************************************************************/

#ifndef ROBOTICS_CAPE
#define ROBOTICS_CAPE

// library version, can also be printed from the command line with the included
// example program rc_version
#define RC_LIB_VERSION_FLOAT	0.34
#define RC_LIB_VERSION_STRING	0.3.4

// necessary types for function prototypes
#include <stdint.h> // for uint8_t types etc
typedef struct timespec	timespec;
typedef struct timeval timeval;

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif	


/*******************************************************************************
* INITIALIZATION AND CLEANUP
*
* Because the Robotics Cape library is tightly integrated with hardware and 
* utilizes background threads, it is critical that the user calls 
* rc_initialize() and rc_cleanup() at the beginning and end of their programs to
* ensure predictable operation. These methods, also enable the following two 
* features.
*
* Firstly, to ensure only one program interfaces with the Robotics Cape core 
* functions at once, it  makes a PID file in /var/run/ called robotics.pid that
* contains the process ID of the currently running robotics cape project. If a
* second process is started that uses the Robotics Cape library, the call to
* rc_initialize() in the new process cleanly shuts down the existing process
* before returning. This PID file also helps systemd monitor robotics cape
* projects that are started on boot as a service. See the services section of
* the manual for more details on systemd services.
*
* Secondly, the call to rc_initialize() registers a system signal handler to 
* intercept shutdown and halt signals. It is typical to stop a Linux program by
* pressing Ctrl-C but this can halt the program in a problematic state if not
* handled correctly. When the user presses Ctrl-C, the Linux kernel sends the
* SIGINT signal to the program which is intercepted by the Robotics Cape signal
* handler to set the program flow state to EXITING. The user's program should
* then monitor for this condition in each thread and exit in a safe manner. This
* is particularly important when driving motors and other actuators which should
* be turned off or put in a safe position before the program closes. See the
* flow state section of this manual for more details. The SIGHUP signal is also
* caught and ignored so that robotics programs started from the command line
* over a USB connection will continue to run if the USB cable is disconnected.

* @ int rc_initialize() 
*
* To ensure full library functionality and to take advantage of the above
* features, the user must call rc_initialize() at beginning of their program.
* rc_initialize() will make sure any existing Robotics Cape project is shut down
* cleanly before continuing to avoid conflicts. It then proceeds to open file
* descriptors and start background threads that are necessary for the Robotics
* Cape library to run. If you are running a kernel with missing drivers or the
* Robotics Cape device tree is not loaded then this function will return -1 and
* print an error message to indicate what is wrong. Otherwise it will return 0
* to indicate success.
* 
* @ int rc_cleanup() 
*
* rc_cleanup() undoes everything done by initialize cape. It closes file
* pointers, waits for background threads to stop cleanly, and finally removes
* the PID file from /var/run/. Additionally it sets LEDs and slave select pins
* in an 'OFF' state and puts the h-bridges into a low power standby state. This
* should be called at the very end of the user's program. It returns 0 on
* success or -1 on error. 
* 
* * @ int rc_kill()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the kill_robot example program from the command line to close whatever
* program is running in the background.
* return values:
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*
* All example programs use these functions. See the bare_minimum example 
* for a skeleton outline.
*
* @ void rc_disable_signal_handler()
*
* Disables the built-in signal handler. Use only if you want to implement your
* own signal handler. Make sure your handler sets rc_state to EXITING or calls
* cleanup_cape on shutdown to ensure Robotics Cape library threads close
* cleanly.
*
* @ void rc_enable_signal_handler()
*
* Re-enables the built-in signal handler if it was disabled before. The built-in 
* signal handler is enabled by default in rc_initialize().
*******************************************************************************/
EXTERNC int rc_initialize();	// call at the beginning of main()
EXTERNC int rc_cleanup();		// call at the end of main()
EXTERNC int rc_kill();	// not usually necessary, use kill_robot example instead
void rc_disable_signal_handler();
EXTERNC void rc_enable_signal_handler();


/*******************************************************************************
* FLOW STATE FOR HIGH LEVEL PROGRAM CONTROL
*
* It can be tricky to manage the starting and stopping of mutiple threads and
* loops. Since the robotics cape library has several background threads in
* addition to any user-created threads, we encourage the use of the consolidated
* high-level program flow control method described here.
*
* The rc_state_t struct tries to cover the majority of use cases in the context
* of a robotics application. After the user has called rc_initialize(), the 
* program flow state will be set to UNINITIALIZED. When the user's own 
* initialization sequence is complete they should set the flow state to PAUSED 
* or RUNNING to indicate to their own threads that the program should now behave
* in normal ongoing operational mode.
*
* During normal operation, the user may elect to implement a PAUSED state where 
* the user's own threads may keep running to read sensors but do not actuate 
* motors, leaving their robot in a safe state. For example, pressing the pause 
* button could be assigned to change this state back and forth between RUNNING 
* and PAUSED. This is entirely optional.
*
* The most important state here is EXITING. The signal handler described in the 
* Init & Cleanup section intercepts the SIGINT signal when the user pressed 
* Ctrl-C and sets the flow state to EXITING. It is then up to the user's threads
* to watch for this condition and exit quickly and cleanly. The user may also 
* set the flow state to EXITING at any time to trigger the closing of their own 
* threads and Robotics Cape library's own background threads.
*
* The flow state variable is kept safely in the robotics cape library's memory 
* space and should be read and modified by the rc_get_state() and rc_set_state()
* functions above. The user may optionally use the rc_print_state() function to 
* print a human readable version of the state enum to the screen.

* All example programs use these functions. See the rc_bare_minimum example 
* for a skeleton outline.
*******************************************************************************/
typedef enum rc_state_t {
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
} rc_state_t;

rc_state_t rc_get_state();
EXTERNC int rc_set_state(rc_state_t new_state);
int rc_print_state();


/*******************************************************************************
* LEDs
*
* Since the 4 blue USR LEDs on the Beaglebone are normally used by the OS,
* the Robotics Cape provides two LEDs for sole use by the user. One is red
* and one is green. The included examples use the red LED to indicate a paused
* or stopped state, and the green LED to indicate a running state. However
* they are not tied to any other robotics cape library functions and can be 
* used for whatever the user desires.
*
* @ typedef enum rc_led_t
* 
* Two LEDs are available and defined by an enumerated type led_t which can be
* RED or GREEN. Just like most boolean states in the C language, a 0 indicates
* 'false' or 'off' and anything else indicates 'on' or 'true'. To make code 
* easier to read, #defines are provided for 'ON' and 'OFF'.
*
* @ int rc_set_led(rc_led_t led, int state)
* 
* If state is 0 the LED will be turned off. If int state is non-zero then the 
* LED will be turned on. Returns 0 on success, -1 on failure.
*
* @ int rc_get_led_state(rc_led_t led)
*
* returns 1 if the LED is on or 0 if it is off.
* This function is typically used in multithreded applications where multiple
* threads may wish to use the same LED.
*
* @ int rc_blink_led(rc_led_t led, float hz, float period)
* 
* Flash an LED at a set frequency for a finite period of time.
* This is a blocking call and only returns after flashing.
*
* See the blink example for sample use case of all of these functions.
*******************************************************************************/
#ifndef ON
#define ON 	1
#endif
#ifndef OFF
#define OFF	0
#endif
typedef enum rc_led_t {
	GREEN,
	RED
} rc_led_t;
int rc_set_led(rc_led_t led, int state);
int rc_get_led(rc_led_t led);
int rc_blink_led(rc_led_t led, float hz, float period);


/*******************************************************************************
* BUTTONS
*
* The Robotics Cape includes two buttons labeled PAUSE and MODE. Like the LEDs,
* they are not used by any background library functions and the user can assign
* them to any function they wish. However, the user is encouraged to use the
* pause button to toggle the program flow state between PAUSED and RUNNING
* using the previously described rc_set_state() function.
*
* @ typedef enum rc_button_state_t
* 
* A button state can be either RELEASED or PRESSED as defined by this enum.
*
* @ int rc_set_pause_pressed_func(int (*func)(void))
* @ int rc_set_pause_released_func(int (*func)(void))
* @ int rc_set_mode_pressed_func(int (*func)(void))
* @ int rc_set_mode_released_func(int (*func)(void))
*
* rc_initialize() sets up interrupt handlers that run in the background to
* poll changes in button state in a way that uses minimal resources. The 
* user can assign which function should be called when either button is pressed
* or released. Functions can also be assigned under both conditions.
* for example, a timer could be started when a button is pressed and stopped
* when the button is released. Pass
*
* For simple tasks like pausing the robot, the user is encouraged to assign
* their function to be called when the button is released as this provides 
* a more natural user experience aligning with consumer product functionality.
* 
* The user can also just do a basic call to rc_get_pause_button_state() or
* rc_get_mode_buttom_state() which returns the enumerated type RELEASED or 
* PRESSED.
*
* See the rc_blink and rc_test_buttons example programs for sample use cases.
******************************************************************************/
typedef enum rc_button_state_t {
	RELEASED,
	PRESSED
} rc_button_state_t;
int rc_set_pause_pressed_func(void (*func)(void));
int rc_set_pause_released_func(void (*func)(void));
int rc_set_mode_pressed_func(void (*func)(void));
int rc_set_mode_released_func(void (*func)(void));
rc_button_state_t rc_get_pause_button();
rc_button_state_t rc_get_mode_button();


/******************************************************************************
* DC MOTOR CONTROL
*
* The robotics cape can drive 4 DC motors bidirectionally powered only from a
* 2-cell lithium battery pack connected to the cape. The motors will not draw
* power from USB or the 9-18v DC Jack. Each channel can support 1.2A continuous
* and the user must be careful to choose motors which will not exceed this
* rating when stalled. Each channel is broken out on an independent 2-pin 
* JST ZH connector.
* 
* @ int rc_enable_motors()
* @ int rc_disable_motors()
*
* The motor drivers are initially in a low-power standby state and must be
* woken up with rc_enable_motors() before use. The user can optionally put the 
* motor drivers back into low power state with rc_disable_motors().
* 
* @ int rc_set_motor(int motor, float duty)
* @ int rc_set_motor_all(float duty)
*
* These will take in a motor index from 1 to 4 and a duty between -1 & 1
* corresponding to full power reverse to full power forward.
* rc_set_motor_all() applies the same duty cycle to all 4 motor channels.
*
* @ int rc_set_motor_free_spin(int motor)
* @ int set motor_free_spin_all()
*
* This puts one or all motor outputs in high-impedance state which lets the 
* motor spin freely as if it wasn't connected to anything.
*
* @ int rc_set_motor_brake(int motor)
* @ int rc_set_motor_brake_all()
*
* These will connect one or all motor terminal pairs together which
* makes the motor fight against its own back EMF turning it into a brake.
*
* See the test_motors example for sample use case.
******************************************************************************/

//#ifdef __cplusplus
//#define EXTERNC extern "C"
//#else
//#define EXTERNC
//#endif	

EXTERNC int rc_enable_motors();
EXTERNC int rc_disable_motors();
EXTERNC int rc_set_motor(int motor, float duty);
EXTERNC int rc_set_motor_all(float duty);
EXTERNC int rc_set_motor_free_spin(int motor);
EXTERNC int rc_set_motor_free_spin_all();
EXTERNC int rc_set_motor_brake(int motor);
EXTERNC int rc_set_motor_brake_all();

/******************************************************************************
* QUADRATURE ENCODER
*
* @ int rc_get_encoder_pos(int ch)
* @ int rc_set_encoder_pos(int ch, int value)
*
* The Robotics Cape includes 4 JST-SH sockets {E1 E2 E3 E4} for connecting
* quadrature encoders. The pins for each are assigned as follows:
*
* 1 - Ground
* 2 - 3.3V
* 3 - Signal A
* 4 - Signal B
*
* The first 3 channels are counted by the Sitara's eQEP hardware encoder
* counters. The 4th channel is counted by the PRU. As a result, no CPU cycles
* are wasted counting encoders and the user only needs to read the channel
* at any point in their code to get the current position. All channels are 
* reset to 0 when initialize_cape() is called. However, the user can reset
* the counter to zero or any other signed 32 bit value with rc_set_encoder_pos().
*
* See the test_encoders example for sample use case.
******************************************************************************/
EXTERNC int rc_get_encoder_pos(int ch);
EXTERNC int rc_set_encoder_pos(int ch, int value);

/******************************************************************************
* ANALOG VOLTAGE SIGNALS
*
*
* @ float rc_battery_voltage()
* @ float rc_dc_jack_voltage()
* The Robotics cape includes two voltage dividers for safe measurement of the
* 2-cell lithium battery voltage and the voltage of any power source connected
* to the 6-16V DC power jack. These can be read with rc_battery_voltage()
* and rc_dc_jack_voltage()
* 
* @ int rc_adc_raw(int ch)
* @ float rc_adc_volt(int ch)
* 
* There is also a 6-pin JST-SH socket on the Cape for connecting up to 4
* potentiometers or general use analog signals. The pinout of this socket is
* as follows:
*
* 1 - Ground
* 2 - VDD_ADC (1.8V)
* 3 - AIN0
* 4 - AIN1
* 5 - AIN2
* 6 - AIN3
*
* All 7 ADC channels on the Sitara including the 4 listed above can be read
* with rc_adc_raw(int ch) which returns the raw integer output of the 
* 12-bit ADC. rc_adc_volt(int ch) additionally converts this raw value to 
* a voltage. ch must be from 0 to 6.
*
* See the test_adc example for sample use case.
******************************************************************************/
float rc_battery_voltage();
float rc_dc_jack_voltage();
int   rc_adc_raw(int ch);
float rc_adc_volt(int ch);


/******************************************************************************
* SERVO AND ESC 
*
* The Robotics Cape has 8 3-pin headers for connecting hobby servos and ESCs.
* The connectors are not polarized so pay close attention to the symbols
* printed in white silkscreen on the cape before plugging anything in.
* The standard pinnout for these 3 pin connectors is as follows.
*
* 1 - Ground
* 2 - 6V Power
* 3 - Pulse width signal
*
* @ int rc_enable_servo_power_rail()
* @ int rc_disable_servo_power_rail()
*
* The user must call rc_enable_servo_power_rail() to enable the 6V voltage 
* regulator and send power to servos. This can be ignored if using ESCs or 
* servos that are driven by an external power source.
* rc_disable_servo_power_rail() can be called to turn off power to the servos for
* example when the robot is in a paused state to save power or prevent noisy
* servos from buzzing.
*
* @ int rc_send_servo_pulse_normalized(int ch, float input)
* @ int rc_send_servo_pulse_normalized_all(float input)
*
* The normal operating range of hobby servos is usually +- 60 degrees of 
* rotation from the neutral position but they often work up to +- 90 degrees.
* rc_send_servo_pulse_normalized(int ch, float input) will send a single pulse to
* the selected channel. the normalized input should be between -1.5 and 1.5
* corresponding to the following pulse width range and angle.
*
* input     width   angle  
* -1.5		600us	90 deg anticlockwise
* -1.0		900us	60 deg anticlockwise
*  0.0		1500us	0 deg neutral
*  1.0		2100us	60 deg clockwise
*  1.5		2400us	90 deg clockwise
*
* Note that all servos are different and do not necessarily allow the full
* range of motion past +-1.0. DO NOT STALL SERVOS.
*
* @ int rc_send_esc_pulse_normalized(int ch, float input)
* @ int rc_send_esc_pulse_normalized_all(float input)
*
* Brushless motor controllers (ESCs) for planes and multirotors are
* unidirectional and lend themselves better to a normalized range from 0 to 1.
* rc_send_esc_pulse_normalized(int ch, float input) also sends a single pulse
* but the range is set for common ESCs
*
* input     width   power  
* -0.1		900     armed but idle
* 0.0		1000us	0%   off
* 0.5		1500us	50%  half-throttle
* 1.0		2000us	100% full-throttle
*
* This assumes the ESCs have been calibrated for the 1000-2000us range. Use the
* calibrate_escs example program to be sure.
*
* @ int rc_send_servo_pulse_us(int ch, int us)
* @ int rc_send_servo_pulse_us_all(int us)
*
* The user may also elect to manually specify the exact pulse width in
* in microseconds with rc_send_servo_pulse_us(int ch, int us). When using any of
* these functions, be aware that they only send a single pulse to the servo
* or ESC. Servos and ESCs typically require an update frequency of at least 
* 10hz to prevent timing out. The timing accuracy of this loop is not critical
* and the user can choose to update at whatever frequency they wish.
*
* See the test_servos, sweep_servos, and calibrate_escs examples.
******************************************************************************/
int rc_enable_servo_power_rail();
EXTERNC int rc_disable_servo_power_rail();
int rc_send_servo_pulse_us(int ch, int us);
int rc_send_servo_pulse_us_all(int us);
int rc_send_servo_pulse_normalized(int ch, float input);
int rc_send_servo_pulse_normalized_all(float input);
int rc_send_esc_pulse_normalized(int ch, float input);
int rc_send_esc_pulse_normalized_all(float input);
int rc_send_oneshot_pulse_normalized(int ch, float input);
int rc_send_oneshot_pulse_normalized_all(float input);


/******************************************************************************
* DSM2/DSMX RC radio functions
*
* The Robotics Cape features a 3-pin JST ZH socket for connecting a DSM2/DSMX
* compatible satellite receiver. See the online manual for more details.
*
* @ int rc_initialize_dsm()
* Starts the background service.
*
* @ rc_is_new_dsm_data()
* 
* Returns 1 when new data is available. 
*
* @ int rc_is_dsm_active()
* 
* Returns 1 if packets are arriving in good health without timeouts.
* Returns 0 otherwise.
*
* @ int rc_set_dsm_data_func(int (*func)(void));
*
* Much like the button handlers, this assigns a user function to be called when
* new data arrives. Be careful as you should still check for radio disconnects 
*
* @ int rc_get_dsm_ch_raw(int channel) 
* 
* Returns the pulse width in microseconds commanded by the transmitter for a
* particular channel. The user can specify channels 1 through 9 but non-zero 
* values will only be returned for channels the transmitter is actually using. 
* The raw values in microseconds typically range from 900-2100us for a standard
* radio with default settings.
*
* @ rc_get_dsm_ch_normalized(int channel) 
*
* Returns a scaled value from -1 to 1 corresponding to the min and max values
* recorded during calibration. The user
* MUST run the clalibrate_dsm example to ensure the normalized values returned
* by this function are correct.
*
* @ uint64_t rc_nanos_since_last_dsm_packet();
* 
* returns the number of nanoseconds since the last dsm packet was received.
* if no packet has ever been received, returns UINT64_MAX;
*
* @ int rc_stop_dsm_service()
*
* stops the background thread. Not necessary to be called by the user as
* cleanup_cape() calls this anyway.
*
* @ int rc_bind_dsm()
*
* Puts a satellite receiver in bind mode. Use the rc_bind_dsm example program
* instead of calling this in your own program.
*
* int rc_calibrate_dsm_routine()
*
* Starts a calibration routine. 
*
* see rc_test_dsm, rc_calibrate_dsm, and rc_dsm_passthroguh examples
******************************************************************************/
int   rc_initialize_dsm();
EXTERNC int   rc_stop_dsm_service();
int   rc_get_dsm_ch_raw(int channel);
float rc_get_dsm_ch_normalized(int channel);
int   rc_is_new_dsm_data();
int   rc_set_dsm_data_func(void (*func)(void));
int   rc_is_dsm_active();
uint64_t rc_nanos_since_last_dsm_packet();
int   rc_get_dsm_resolution();
int   rc_num_dsm_channels();
int   rc_bind_dsm();
int   rc_calibrate_dsm_routine();


/******************************************************************************
* 9-AXIS IMU
*
* The Robotics Cape features an Invensense MPU9250 9-axis IMU. This API allows
* the user to configure this IMU in two modes: RANDOM and DMP
*
* RANDOM: The accelerometer, gyroscope, magnetometer, and thermometer can be
* read directly at any time. To use this mode, call rc_initialize_imu() with your
* imu_config and imu_data structs as arguments as defined below. You can then
* call rc_read_accel_data, rc_read_gyro_data, rc_read_mag_data, or rc_read_imu_temp
* at any time to get the latest sensor values.
*
* DMP: Stands for Digital Motion Processor which is a feature of the MPU9250.
* in this mode, the DMP will sample the sensors internally and fill a FIFO
* buffer with the data at a fixed rate. Furthermore, the DMP will also calculate
* a filtered orientation quaternion which is placed in the same buffer. When
* new data is ready in the buffer, the IMU sends an interrupt to the BeagleBone
* triggering the buffer read followed by the execution of a function of your
* choosing set with the rc_set_imu_interrupt_func() function.
*
* @ enum rc_accel_fsr_t rc_gyro_fsr_t
* 
* The user may choose from 4 full scale ranges of the accelerometer and
* gyroscope. They have units of gravity (G) and degrees per second (DPS)
* The defaults values are A_FSR_4G and G_FSR_1000DPS respectively.
*
* enum rc_accel_dlpf_t rc_gyro_dlpf_t 
*
* The user may choose from 7 digital low pass filter constants for the 
* accelerometer and gyroscope. The filter runs at 1kz and helps to reduce sensor
* noise when sampling more slowly. The default values are ACCEL_DLPF_184
* GYRO_DLPF_250. Lower cut-off frequencies incur phase-loss in measurements.
*
* @ struct rc_imu_config_t
*
* Configuration struct passed to rc_initialize_imu and rc_initialize_imu_dmp. It is 
* best to get the default config with rc_default_imu_config() function and
* modify from there.
*
* @ struct rc_imu_data_t 
*
* This is the container for holding the sensor data from the IMU.
* The user is intended to make their own instance of this struct and pass
* its pointer to imu read functions.
*
* @ rc_imu_config_t rc_default_imu_config()
* 
* Returns an rc_imu_config_t struct with default settings. Use this as a starting
* point and modify as you wish.
*
* @ int rc_initialize_imu(rc_imu_data_t *data, rc_imu_config_t conf)
*
* Sets up the IMU in random-read mode. First create an instance of the imu_data
* struct to point to as rc_initialize_imu will put useful data in it.
* rc_initialize_imu only reads from the config struct. After this, you may read
* sensor data.
*
* @ int rc_read_accel_data(rc_imu_data_t *data)
* @ int rc_read_gyro_data(rc_imu_data_t *data)
* @ int rc_read_mag_data(rc_imu_data_t *data)
* @ int rc_read_imu_temp(rc_imu_data_t* data)
*
* These are the functions for random sensor sampling at any time. Note that
* if you wish to read the magnetometer then it must be enabled in the
* configuration struct. Since the magnetometer requires additional setup and
* is slower to read, it is disabled by default.
*
******************************************************************************/
// defines for index location within TaitBryan and quaternion vectors
#define TB_PITCH_X	0
#define TB_ROLL_Y	1
#define TB_YAW_Z	2
#define QUAT_W		0
#define QUAT_X		1
#define QUAT_Y		2
#define QUAT_Z		3

typedef enum rc_accel_fsr_t{
  A_FSR_2G,
  A_FSR_4G,
  A_FSR_8G,
  A_FSR_16G
} rc_accel_fsr_t;

typedef enum rc_gyro_fsr_t{
  G_FSR_250DPS,
  G_FSR_500DPS,
  G_FSR_1000DPS,
  G_FSR_2000DPS 
} rc_gyro_fsr_t;

typedef enum rc_accel_dlpf_t{
	ACCEL_DLPF_OFF,
	ACCEL_DLPF_184,
	ACCEL_DLPF_92,
	ACCEL_DLPF_41,
	ACCEL_DLPF_20,
	ACCEL_DLPF_10,
	ACCEL_DLPF_5
} rc_accel_dlpf_t;

typedef enum rc_gyro_dlpf_t{
	GYRO_DLPF_OFF,
	GYRO_DLPF_184,
	GYRO_DLPF_92,
	GYRO_DLPF_41,
	GYRO_DLPF_20,
	GYRO_DLPF_10,
	GYRO_DLPF_5
} rc_gyro_dlpf_t;

typedef enum rc_imu_orientation_t{
	ORIENTATION_Z_UP		= 136,
	ORIENTATION_Z_DOWN		= 396,
	ORIENTATION_X_UP		= 14,
	ORIENTATION_X_DOWN		= 266,
	ORIENTATION_Y_UP		= 112,
	ORIENTATION_Y_DOWN		= 336,
	ORIENTATION_X_FORWARD	= 133,
	ORIENTATION_X_BACK		= 161
} rc_imu_orientation_t;

typedef struct rc_imu_config_t{
	// full scale ranges for sensors
	rc_accel_fsr_t accel_fsr; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	rc_gyro_fsr_t gyro_fsr;  // GFS_250,GFS_500,GFS_1000,GFS_2000
	
	// internal low pass filter constants
	rc_gyro_dlpf_t gyro_dlpf;
	rc_accel_dlpf_t accel_dlpf;
	
	// magnetometer use is optional 
	int enable_magnetometer; // 0 or 1
	
	// DMP settings, only used with DMP interrupt
	int dmp_sample_rate;
	rc_imu_orientation_t orientation; //orientation matrix
	// higher mix_factor means less weight the compass has on fused_TaitBryan
	float compass_time_constant; 	// time constant for filtering fused yaw
	int dmp_interrupt_priority; // scheduler priority for handler
	int show_warnings;	// set to 1 to enable showing of rc_i2c_bus warnings

} rc_imu_config_t;

typedef struct rc_imu_data_t{
	// last read sensor values in real units
	float accel[3];	// units of m/s^2
	float gyro[3];	// units of degrees/s
	float mag[3];	// units of uT
	float temp;		// units of degrees Celsius
	
	// 16 bit raw adc readings from each sensor
	int16_t raw_gyro[3];	
	int16_t raw_accel[3];
	
	// FSR-derived conversion ratios from raw to real units
	float accel_to_ms2;	// to m/s^2
	float gyro_to_degs;	// to degrees/s
	
	// everything below this line is available in DMP mode only
	// quaternion and TaitBryan angles from DMP based on ONLY Accel/Gyro
	float dmp_quat[4];		// normalized quaternion
	float dmp_TaitBryan[3];	// radians pitch/roll/yaw X/Y/Z
	
	// If magnetometer is enabled in DMP mode, the following quaternion and 
	// TaitBryan angles will be available which add magnetometer data to filter
	float fused_quat[4];		// normalized quaternion
	float fused_TaitBryan[3];	// radians pitch/roll/yaw X/Y/Z
	float compass_heading;		// heading filtered with gyro and accel data
	float compass_heading_raw;	// heading in radians from magnetometer
} rc_imu_data_t;

// Thread control
#include <pthread.h>
extern pthread_mutex_t rc_imu_read_mutex;
extern pthread_cond_t  rc_imu_read_condition;

// General functions
rc_imu_config_t rc_default_imu_config();
int rc_set_imu_config_to_defaults(rc_imu_config_t* conf);
int rc_power_off_imu();

// one-shot sampling mode functions
int rc_initialize_imu(rc_imu_data_t* data, rc_imu_config_t conf);
int rc_read_accel_data(rc_imu_data_t* data);
int rc_read_gyro_data(rc_imu_data_t* data);
int rc_read_mag_data(rc_imu_data_t* data);
int rc_read_imu_temp(rc_imu_data_t* data);

// interrupt-driven sampling mode functions
int rc_initialize_imu_dmp(rc_imu_data_t* data, rc_imu_config_t conf);
int rc_set_imu_interrupt_func(void (*func)(void));
int rc_stop_imu_interrupt_func();
int rc_was_last_imu_read_successful();
uint64_t rc_nanos_since_last_imu_interrupt();

// other
int rc_calibrate_gyro_routine();
int rc_calibrate_mag_routine();
int rc_is_gyro_calibrated();
int rc_is_mag_calibrated();

/*******************************************************************************
* BMP280 Barometer
*
* The robotics cape features a Bosch BMP280 barometer for measuring temperature, 
* pressure, and altitude.
*
* @ enum rc_bmp_oversample_t 
* Setting given to rc_initialize_barometer() which defines the oversampling done
* internally to the barometer. For example, if BMP_OVERSAMPLE_16 is used then
* the barometer will average 16 samples before updating the data registers.
* The more oversampling used, the slower the data registers will update. You
* should pick an oversample that provides an update rate slightly slower than 
* the rate at which you will be reading the barometer. 
* 
* @ enum rc_bmp_filter_t
* Setting given to rc_initialize_barometer() to configure the coefficient of the 
* internal first order filter. We recommend disabling the filter with
* BMP_FILTER_OFF and doing your own filtering with the discrete filter functions
* below.
*
* @ int rc_initialize_barometer(rc_bmp_oversample_t oversample, rc_bmp_filter_t filter)
* powers on the barometer and initializes it with the given oversample and
* filter settings. returns 0 on success, otherwise -1.
*
* @ int rc_power_off_barometer()
* Puts the barometer into a low power state, should be called at the end of
* your program before close. return 0 on success, otherwise -1.
*
* @ int rc_read_barometer()
* Reads the newest temperature and pressure measurments from the barometer over
* the I2C bus. To access the data use the rc_bmp_get_temperature(), 
* rc_bmp_get_pressure_pa(), or rc_bmp_get_altitude_m() functions. 
* returns 0 on success, otherwise -1.
*
* @ float rc_bmp_get_temperature()
* This does not start an I2C transaction but simply returns the temperature in
* degrees celcius that was read by the last call to the rc_read_barometer() 
* function.
*
* @ float rc_bmp_get_pressure_pa()
* This does not start an I2C transaction but simply returns the pressure in
* pascals that was read by the last call to the rc_read_barometer() function.
* 
* @ float rc_bmp_get_altitude_m()
* This does not start an I2C transaction but simply returns the altitude in 
* meters based on the pressure received by the last call to the rc_read_barometer()
* function. Assuming current pressure at sea level is the default 101325 Pa.
* Use rc_set_sea_level_pressure_pa() if you know the current sea level pressure
* and desire more accuracy. 
* 
* @ int rc_set_sea_level_pressure_pa(float pa)
* If you know the current sea level pressure for your region and weather, you 
* can use this to correct the altititude reading. This is not necessary if you
* only care about differential altitude from a starting point.
*******************************************************************************/
typedef enum rc_bmp_oversample_t{
	BMP_OVERSAMPLE_1  =	(0x01<<2), // update rate 182 HZ
	BMP_OVERSAMPLE_2  =	(0x02<<2), // update rate 133 HZ
	BMP_OVERSAMPLE_4  =	(0x03<<2), // update rate 87 HZ
	BMP_OVERSAMPLE_8  =	(0x04<<2), // update rate 51 HZ
	BMP_OVERSAMPLE_16 =	(0x05<<2)  // update rate 28 HZ
} rc_bmp_oversample_t;

typedef enum rc_bmp_filter_t{
	BMP_FILTER_OFF = (0x00<<2),
	BMP_FILTER_2   = (0x01<<2),
	BMP_FILTER_4   = (0x02<<2),
	BMP_FILTER_8   = (0x03<<2),
	BMP_FILTER_16  = (0x04<<2)
}rc_bmp_filter_t;

int rc_initialize_barometer(rc_bmp_oversample_t oversample, rc_bmp_filter_t filter);
int rc_power_off_barometer();
int rc_read_barometer();
float rc_bmp_get_temperature();
float rc_bmp_get_pressure_pa();
float rc_bmp_get_altitude_m();
int rc_set_sea_level_pressure_pa(float pa);

/*******************************************************************************
* I2C functions
*
* I2C bus 1 is broken out on the robotics cape on socket "I2C1" and is free for 
* the user to have full authority over. Bus 0 is used internally on the cape 
* for the IMU and barometer. The user should not use bus 0 unless they know what 
* they are doing. The IMU and barometer functions 
*
* @ int rc_i2c_init(int bus, uint8_t devAddr)
* This initializes an I2C bus (0 or 1) at 400khz as defined in the device tree.
* The bus speed cannot be modified. devAddr is the 8-bit i2c address of the 
* device you wish to communicate with. This devAddr can be changed later 
* without initializing. rc_i2c_init only needs to be called once per bus.
* 
* @ int set_device_address(int bus, uint8_t devAddr)
* Use this to change to another device address after initialization.
* 
* @ int rc_i2c_close(int bus) 
* Closes the bus and device file descriptors.
*
* @int rc_i2c_claim_bus(int bus)
* @int rc_i2c_release_bus(int bus)
* @int rc_i2c_get_in_use_state(int bus)
* Claim and release bus are purely for the convenience of the user and are not 
* necessary. They simply set a flag indicating that the bus is in use to help 
* manage multiple device access in multithreaded applications.
*
* @ int rc_i2c_read_byte(int bus, uint8_t regAddr, uint8_t *data)
* @ int rc_i2c_read_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t *data)
* @ int rc_i2c_read_word(int bus, uint8_t regAddr, uint16_t *data)
* @ int rc_i2c_read_words(int bus, uint8_t regAddr, uint8_t length, uint16_t *data)
* @ int rc_i2c_read_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
* These rc_i2c_read functions are for reading data from a particular register.
* This sends the device address and register address to be read from before
* reading the response. 
*
* @ int rc_i2c_write_byte(int bus, uint8_t regAddr, uint8_t data);
* @ int rc_i2c_write_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t* data)
* @ int rc_i2c_write_word(int bus, uint8_t regAddr, uint16_t data);
* @ int rc_i2c_write_words(int bus,uint8_t regAddr, uint8_t length, uint16_t* data)
* @ int rc_i2c_write_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t data)
* These write values write a value to a particular register on the previously
* selected device.
*
* @ int rc_i2c_send_bytes(int bus, uint8_t length, uint8_t* data)
* @ int rc_i2c_send_byte(int bus, uint8_t data)
* Instead of automatically sending a device address before the data which is 
* what happens in the above read and write functions, the rc_i2c_send functions 
* send only the data given by the data argument. This is useful for more
* complicated IO such as uploading firmware to a device.
*******************************************************************************/
int rc_i2c_init(int bus, uint8_t devAddr);
int rc_i2c_close(int bus);
int rc_i2c_set_device_address(int bus, uint8_t devAddr);
 
int rc_i2c_claim_bus(int bus);
int rc_i2c_release_bus(int bus);
int rc_i2c_get_in_use_state(int bus);

int rc_i2c_read_byte(int bus, uint8_t regAddr, uint8_t *data);
int rc_i2c_read_bytes(int bus, uint8_t regAddr, uint8_t length,  uint8_t *data);
int rc_i2c_read_word(int bus, uint8_t regAddr, uint16_t *data);
int rc_i2c_read_words(int bus, uint8_t regAddr, uint8_t length, uint16_t *data);
int rc_i2c_read_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

int rc_i2c_write_byte(int bus, uint8_t regAddr, uint8_t data);
int rc_i2c_write_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t* data);
int rc_i2c_write_word(int bus, uint8_t regAddr, uint16_t data);
int rc_i2c_write_words(int bus, uint8_t regAddr, uint8_t length, uint16_t* data);
int rc_i2c_write_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t data);

int rc_i2c_send_bytes(int bus, uint8_t length, uint8_t* data);
int rc_i2c_send_byte(int bus, uint8_t data);

/*******************************************************************************
* SPI - Serial Peripheral Interface
*
* The Sitara's SPI bus is broken out on two JST SH 6-pin sockets labeled SPI1.1 
* and SPI1.2 These share clock and serial IO signals, but have independent slave
* select lines. 
* 
* The slaves can be selected automatically by the SPI linux driver or manually
* with select/deselect_spi_slave() functions. On the Robotics Cape, slave 1
* can be used in either mode, but slave 2 must be selected manually. On the
* BB Blue either slave can be used in manual or automatic modes. 
*******************************************************************************/
typedef enum ss_mode_t{
	SS_MODE_AUTO,
	SS_MODE_MANUAL
} ss_mode_t;

#define SPI_MODE_CPOL0_CPHA0 0
#define SPI_MODE_CPOL0_CPHA1 1
#define SPI_MODE_CPOL1_CPHA0 2
#define SPI_MODE_CPOL1_CPHA1 3

int rc_spi_init(ss_mode_t ss_mode, int spi_mode, int speed_hz, int slave);
int rc_spi_fd(int slave);
int rc_spi_close(int slave);
int rc_manual_select_spi_slave(int slave);
EXTERNC int rc_manual_deselect_spi_slave(int slave);	
int rc_spi_send_bytes(char* data, int bytes, int slave);
int rc_spi_read_bytes(char* data, int bytes, int slave);
int rc_spi_transfer(char* tx_data, int tx_bytes, char* rx_data, int slave);
int rc_spi_write_reg_byte(char reg_addr, char data, int slave);
char rc_spi_read_reg_byte(char reg_addr, int slave);
int rc_spi_read_reg_bytes(char reg_addr, char* data, int bytes, int slave);



/*******************************************************************************
* UART
*******************************************************************************/
int rc_uart_init(int bus, int speed, float timeout);
int rc_uart_close(int bus);
int rc_uart_fd(int bus);
int rc_uart_send_bytes(int bus, int bytes, char* data);
int rc_uart_send_byte(int bus, char data);
int rc_uart_read_bytes(int bus, int bytes, char* buf);
int rc_uart_read_line(int bus, int max_bytes, char* buf);
int rc_uart_flush(int bus);
int rc_uart_bytes_available(int bus);

/*******************************************************************************
* CPU Frequency Control
*
* @ int rc_set_cpu_freq(rc_cpu_freq_t freq)
*
* Sets the CPU frequency to either a fixed value or to on-demand automatic
* scaling mode. Returns 0 on success, -1 on failure.
*
* @ rc_cpu_freq_t rc_get_cpu_freq()
*
* Returns the current clock speed of the Beaglebone's Sitara processor in the
* form of the provided enumerated type. It will never return the FREQ_ONDEMAND
* value as the intention of this function is to see the clock speed as set by
* either the user or the ondemand governor itself.
*
* @ int rc_print_cpu_freq()
*
* Prints the current frequency to the screen. For example "300MHZ".
* Returns 0 on success or -1 on failure.
*******************************************************************************/
typedef enum rc_cpu_freq_t{
	FREQ_ONDEMAND,
	FREQ_300MHZ,
	FREQ_600MHZ,
	FREQ_800MHZ,
	FREQ_1000MHZ
} rc_cpu_freq_t;

int rc_set_cpu_freq(rc_cpu_freq_t freq);
rc_cpu_freq_t rc_get_cpu_freq();
int rc_print_cpu_freq();

/*******************************************************************************
* Board identification
*
* Because we wish to support different beagleboard products with this same
* library, we must internally determine which board we are running on to decide
* which pins to use. We make these functions available to the user in case they
* wish to do the same. 
* See the check_board example for a demonstration.
*******************************************************************************/
typedef enum rc_bb_model_t{
	UNKNOWN_MODEL,
	BB_BLACK,
	BB_BLACK_RC,
	BB_BLACK_W,
	BB_BLACK_W_RC,
	BB_GREEN,
	BB_GREEN_W,
	BB_BLUE
} rc_bb_model_t;

EXTERNC rc_bb_model_t rc_get_bb_model();
void rc_print_bb_model();

/*******************************************************************************
* Pin Multiplexing (pinmux)
*
* On the Robotics Cape, we allow changing the pinmux on the SPI, GPS, and UART1
* headers in case you wish to expose GPIO, CAN, or PWM functionality.
* We use the GPIO number to identify the pins even though they may be used
* for things other than GPIO as this provides consistency with the GPIO
* functions which will likely be used. A list of defines are also given here
* to make your code easier to read and to indicate which pins are available
* for pinmuxing.
*
* enum rc_pinmux_mode_t gives options for pinmuxing. Not every mode if available on
* each pin. refer to the pin table for which to use. 
*
* rc_set_default_pinmux() puts everything back to standard and is used by 
* initialize_cape
*******************************************************************************/
// Cape and Blue
#define GPS_HEADER_PIN_3		2	// P9_22, normally GPS UART2 RX
#define GPS_HEADER_PIN_4		3	// P9_21, normally GPS UART2 TX
#define UART1_HEADER_PIN_3		14	// P9_26, normally UART1 RX
#define UART1_HEADER_PIN_4		15	// P9_24, normally UART1 TX
#define SPI_HEADER_PIN_3		112	// P9_30, normally SPI1 MOSI		
#define SPI_HEADER_PIN_4		111	// P9_29, normally SPI1 MISO	
#define SPI_HEADER_PIN_5		110	// P9_31, normally SPI1 SCLK

// Cape Only
#define CAPE_SPI_PIN_6_SS1		113	// P9_28, normally SPI mode
#define CAPE_SPI_PIN_6_SS2		49	// P9_23, normally GPIO mode

// Blue Only
#define BLUE_SPI_PIN_6_SS1		29	// gpio 0_29  pin H18
#define BLUE_SPI_PIN_6_SS2		7	// gpio 0_7  pin C18		
#define BLUE_GP0_PIN_3			57	// gpio 1_25 pin U16
#define BLUE_GP0_PIN_4			49	// gpio 1_17 pin P9.23
#define BLUE_GP0_PIN_5			116	// gpio 3_20 pin D13
#define BLUE_GP0_PIN_6			113	// gpio 3_17 pin P9_28
#define BLUE_GP1_PIN_3			98	// gpio 3_2  pin J15
#define BLUE_GP1_PIN_4			97	// gpio 3_1  pin H17

typedef enum rc_pinmux_mode_t{
	PINMUX_GPIO,
	PINMUX_GPIO_PU,
	PINMUX_GPIO_PD,
	PINMUX_PWM,
	PINMUX_SPI,
	PINMUX_UART,
	PINMUX_CAN
} rc_pinmux_mode_t;

int rc_set_pinmux_mode(int pin, rc_pinmux_mode_t mode);
EXTERNC int rc_set_default_pinmux();

/*******************************************************************************
* GPIO
*******************************************************************************/
#define HIGH 1
#define LOW 0

typedef enum rc_pin_direction_t{
	INPUT_PIN,
	OUTPUT_PIN
}rc_pin_direction_t;

typedef enum rc_pin_edge_t{
	EDGE_NONE,
	EDGE_RISING,
	EDGE_FALLING,
	EDGE_BOTH
}rc_pin_edge_t;

int rc_gpio_export(unsigned int gpio);
int rc_gpio_unexport(unsigned int gpio);
int rc_gpio_set_dir(int gpio, rc_pin_direction_t dir);
int rc_gpio_set_value(unsigned int gpio, int value);
int rc_gpio_get_value(unsigned int gpio);
int rc_gpio_set_edge(unsigned int gpio, rc_pin_edge_t edge);
int rc_gpio_fd_open(unsigned int gpio);
int rc_gpio_fd_close(int fd);



EXTERNC int rc_gpio_set_value_mmap(int pin, int state);
EXTERNC int rc_gpio_get_value_mmap(int pin);


/*******************************************************************************
* PWM
*
* These functions provide a general interface to all 3 PWM subsystems, each of
* which have two available channels A and B. PWM subsystems 1 and 2 are used for
* controlling the 4 motors on the Robotics Cape, however they may be controlled
* by the user directly instead of using the motor API. PWM subsystem 0 channels
* A and B can be accessed on the UART1 header if set up with the Pinmux API to 
* do so. The user may have exclusive use of that subsystem.
*
* @ int rc_pwm_init(int ss, int frequency)
*
* Configures subsystem 0, 1, or 2 to operate at a particular frequency. This may
* be called at runtime to change the pwm frequency without stopping the motors
* or pwm signal. Returns 0 on success or -1 on failure.
*
* @ int rc_pwm_close(int ss){
*
* Unexports a subsystem to put it into low-power state. Not necessary for the
* the user to call during normal program operation. This is mostly for internal
* use and cleanup.
*
* @ int rc_pwm_set_duty(int ss, char ch, float duty)
*
* Updates the duty cycle through the file system userspace driver. subsystem ss
* must be 0,1,or 2 and channel 'ch' must be A or B. Duty cycle must be bounded
* between 0.0f (off) and 1.0f(full on). Returns 0 on success or -1 on failure.
*
* @ int rc_pwm_set_duty_ns(int ss, char ch, int duty_ns)
*
* like rc_pwm_set_duty() but takes a pulse width in nanoseconds which must range
* from 0 (off) to the number of nanoseconds in a single cycle as determined
* by the freqency specified when calling rc_pwm_init(). The default PWM
* frequency of the motors is 25kz corresponding to a maximum pulse width of
* 40,000ns. However, this function will likely only be used by the user if they
* have set a custom PWM frequency for a more specific purpose. Returns 0 on
* success or -1 on failure.
*
* @ int rc_pwm_set_duty_mmap(int ss, char ch, float duty)
*
* This is the fastest way to set the pwm duty cycle and is used internally by
* the rc_set_motor() function but is also available to the user. This is done
* with direct memory access from userspace to the pwm subsystem. It's use is
* identical to rc_pwm_set_duty where subsystem ss must be 0,1, or 2 where
* 1 and 2 are used by the motor H bridges. Channel 'ch' must be 'A' or 'B' and
* duty must be from 0.0f to 1.0f. The subsystem must be intialized with
* rc_pwm_init() before use. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_pwm_init(int ss, int frequency);
int rc_pwm_close(int ss);
int rc_pwm_set_duty(int ss, char ch, float duty);
int rc_pwm_set_duty_ns(int ss, char ch, int duty_ns);
EXTERNC int rc_pwm_set_duty_mmap(int ss, char ch, float duty);

/*******************************************************************************
* time
*
* @ void rc_nanosleep(uint64_t ns)
* 
* A wrapper for the normal UNIX nanosleep function which takes a number of
* nanoseconds instead of a timeval struct. This also handles restarting
* nanosleep with the remaining time in the event that nanosleep is interrupted
* by a signal. There is no upper limit on the time requested.
*
* @ void rc_usleep(uint64_t ns)
* 
* The traditional usleep function, however common, is deprecated in linux as it
* uses SIGALARM which interferes with alarm and timer functions. This uses the
* new POSIX standard nanosleep to accomplish the same thing which further
* supports sleeping for lengths longer than 1 second. This also handles
* restarting nanosleep with the remaining time in the event that nanosleep is 
* interrupted by a signal. There is no upper limit on the time requested.
*
* @ uint64_t rc_timespec_to_micros(timespec ts)
* 
* Returns a number of microseconds corresponding to a timespec struct.
* Useful because timespec structs are annoying.
*
* @ uint64_t rc_timespec_to_millis(timespec ts)
* 
* Returns a number of milliseconds corresponding to a timespec struct.
* Useful because timespec structs are annoying.
*
* @ uint64_t rc_timeval_to_micros(timeval tv)
* 
* Returns a number of microseconds corresponding to a timespec struct.
* Useful because timeval structs are annoying.
*
* @ uint64_t rc_timeval_to_millis(timeval ts)
* 
* Returns a number of milliseconds corresponding to a timespec struct.
* Useful because timeval structs are annoying.
*
* @ uint64_t rc_nanos_since_epoch()
* 
* Returns the number of nanoseconds since epoch using system CLOCK_REALTIME
* This function itself takes about 1100ns to complete at 1ghz under ideal
* circumstances.
*
* @ uint64_t rc_nanos_since_boot()
* 
* Returns the number of nanoseconds since system boot using CLOCK_MONOTONIC
* This function itself takes about 1100ns to complete at 1ghz under ideal
* circumstances.
*
* @ uint64_t rc_nanos_thread_time()
* 
* Returns the number of nanoseconds from when when the calling thread was
* started in CPU time. This time only increments when the processor is working
* on the calling thread and not when the thread is sleeping. This is usually for
* timing how long blocks of user-code take to execute. This function itself
* takes about 2100ns to complete at 1ghz under ideal circumstances.
*
* @ timespec rc_timespec_diff(timespec start, timespec end)
* 
* Returns the time difference between two timespec structs as another timespec.
* Convenient for use with nanosleep() function and accurately timed loops.
* Unlike timespec_sub defined in time.h, rc_timespec_diff does not care which
* came first, A or B. A positive difference in time is always returned.
*
* @ int rc_timespec_add(timespec* start, double seconds)
* 
* Adds an amount of time in seconds to a timespec struct. The time added is a
* floating point value to make respresenting fractions of a second easier.
* the timespec is passed as a pointer so it can be modified in place.
* Seconds may be negative.
*******************************************************************************/
void rc_nanosleep(uint64_t ns);
EXTERNC void rc_usleep(unsigned int us);
uint64_t rc_timespec_to_micros(timespec ts);
uint64_t rc_timespec_to_millis(timespec ts);
uint64_t rc_timeval_to_micros(timeval tv);
uint64_t rc_timeval_to_millis(timeval tv);
uint64_t rc_nanos_since_epoch();
uint64_t rc_nanos_since_boot();
uint64_t rc_nanos_thread_time();
timespec rc_timespec_diff(timespec A, timespec B);
void rc_timespec_add(timespec* start, double seconds);

/*******************************************************************************
* Other Functions
*
* This is a collection of miscellaneous useful functions that are part of the
* robotics cape library. These do not necessarily interact with hardware.
*
* @ void rc_null_func()
*
* A simple function that just returns. This exists so function pointers can be 
* set to do nothing such as button and imu interrupt handlers.
*
* @ float rc_get_random_float()
* @ double rc_get_random_double()
*
* Returns a random single or double prevision point value between -1 and 1. 
* These are here because the rand() function from stdlib.h only returns and
* integer. These are highly optimized routines that use bitwise operation
* instead of floating point division.
*
* @ int rc_saturate_float(float* val, float min, float max)
* @ int rc_saturate_double(double* val, double min, double max)
*
* Modifies val to be bounded between between min and max. Returns 1 if 
* saturation occurred, 0 if val was already in bound, and -1 if min was falsely
* larger than max.
*
* @ char *rc_byte_to_binary(char x)
* 
* This returns a string (char*) of '1' and '0' representing a character.
* For example, print "00101010" with printf(rc_byte_to_binary(42));
*
* @ int rc_suppress_stdout(int (*func)(void))
*
* Executes a functiton func with all outputs to stdout suppressed. func must
* take no arguments and must return an integer. Adapt this source to your
* liking if you need to pass arguments. For example, if you have a function
* int foo(), call it with supressed output to stdout as follows:
* int ret = rc_suppress_stdout(&foo);
*
* @ int rc_suppress_stderr(int (*func)(void))
* 
* executes a functiton func with all outputs to stderr suppressed. func must
* take no arguments and must return an integer. Adapt this source to your
* liking if you need to pass arguments. For example, if you have a function
* int foo(), call it with supressed output to stderr as follows:
* int ret = rc_suppress_stderr(&foo);
*
* @ rc_continue_or_quit()
*
* This is a blocking function which returns 1 if the user presses ENTER.
* it returns 0 on any other keypress. If ctrl-C is pressed it will
* additionally set the global state to EXITITING and return -1. 
* This is a useful function for checking if the user wishes to continue with a 
* process or quit.
*
* @ float rc_version_float()
*
* Returns a floating-point representation of the roboticscape library version
* for easy comparison.
*
* @ const char* rc_version_string()
*
* Returns a string of the roboticscape package version for printing.
*******************************************************************************/
void rc_null_func();
float rc_get_random_float();
double rc_get_random_double();
int rc_saturate_float(float* val, float min, float max);
int rc_saturate_double(double* val, double min, double max);
char* rc_byte_to_binary(unsigned char x);
int rc_suppress_stdout(int (*func)(void));
int rc_suppress_stderr(int (*func)(void));
int rc_continue_or_quit();
float rc_version_float();
const char* rc_version_string();

/*******************************************************************************
* Linear Algebra Types
*
* The Vector and Matrix types here are used throughout the rest of the Robotics
* Cape library behind the scenes, but are also available to the user along with
* a collection of hardware-accelerated linear algebra functions. The premise
* of both types is that a small rc_vector_t and rc_matrix_t struct contains
* information about type's size and a pointer to where dynamically allocated
* memory exists that stores the actual data for the vector or matrix. Use
* rc_alloc_vector and rc_alloc_matrix to dynamically allocate memory for each
* new vector or matrix. Then use rc_free_vector and rc_free_matrix to free the
* memory when you are done using it. See the remaining vector, matrix, and
* linear algebra functions for more details.
*******************************************************************************/
// vector type
typedef struct rc_vector_t{
	int len;
	float* d;
	int initialized;
} rc_vector_t;

// matrix type
typedef struct rc_matrix_t{
	int rows;
	int cols;
	float** d;
	int initialized;
} rc_matrix_t;

/*******************************************************************************
* Vectors
*
* @ int rc_alloc_vector(rc_vector_t* v, int length)
*
* Allocates memory for vector v to have specified length. If v is initially the
* right length then nothing is done and the data in v is preserved. If v is
* uninitialized or of the wrong length then any existing memory is freed and new
* memory is allocated, helping to prevent accidental memory leaks. The contents 
* of the new vector is not guaranteed to be anything in particular.
* Returns 0 if successful, otherwise returns -1. Will only be unsuccessful if 
* length is invalid or there is insufficient memory available.
*
* @ int rc_free_vector(rc_vector_t* v)
*
* Frees the memory allocated for vector v and importantly sets the length and
* initialized flag of the rc_vector_t struct to 0 to indicate to other functions
* that v no longer points to allocated memory and cannot be used until more
* memory is allocated such as with rc_alloc_vector or rc_vector_zeros.
* Returns 0 on success. Will only fail and return -1 if it is passed a NULL
* pointer.
*
* @ rc_vector_t rc_empty_vector()
*
* Returns an rc_vector_t with no allocated memory and the initialized flag set
* to 0. This is useful for initializing vectors when they are declared since
* local variables declared in a function without global variable scope in C are
* not guaranteed to be zeroed out which can lead to bad memory pointers and 
* segfaults if not handled carefully. We recommend initializing all
* vectors with this function before using rc_alloc_matrix or any other function.
*
* @ int rc_vector_zeros(rc_vector_t* v, int length)
*
* Resizes vector v and allocates memory for a vector with specified length.
* The new memory is pre-filled with zeros. Any existing memory allocated for v
* is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on error.
*
* @ int rc_vector_ones(rc_vector_t* v, int length)
*
* Resizes vector v and allocates memory for a vector with specified length.
* The new memory is pre-filled with floating-point ones. Any existing memory
* allocated for v is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on error.
*
* @ int rc_random_vector(rc_vector_t* v, int length)
*
* Resizes vector v and allocates memory for a vector with specified length.
* The new memory is pre-filled with random floating-point values between -1.0f
* and 1.0f. Any existing memory allocated for v is freed if necessary to avoid 
* memory leaks.
* Returns 0 on success or -1 on error.
*
* @ int rc_vector_fibonnaci(rc_vector_t* v, int length)
*
* Creates a vector of specified length populated with the fibonnaci sequence.
* Returns 0 on success or -1 on error.
*
* @ int rc_vector_from_array(rc_vector_t* v, float* ptr, int length)
*
* Sometimes you will have a normal C-array of floats and wish to convert to 
* rc_vector_t format for use with the other linear algebra functions.
* This function duplicates the contents of an array of floats into vector v and
* ensures v is sized correctly. Existing data in v (if any) is freed and lost.
* Returns 0 on success or -1 on failure.
*
* @ int rc_duplicate_vector(rc_vector_t a, rc_vector_t* b)
*
* Allocates memory for a duplicate of vector a and copies the contents into
* the new vector b. Simply making a copy of the rc_vector_t struct is not
* sufficient as the rc_vector_t struct simply contains a pointer to the memory
* allocated to contain the contents of the vector. rc_duplicate_vector sets b
* to be a new rc_vector_t with a pointer to freshly-allocated memory.
* Returns 0 on success or -1 on error.
*
* @ int rc_set_vector_entry(rc_vector_t* v, int pos, float val)
*
* Sets the entry of vector 'v' in position 'pos' to 'val' where the position is
* zero-indexed. In practice this is never used as it is much easier for the user
* to set values directly with this code:
*
* v.d[pos]=val;
*
* However, we provide this function for completeness. It is not strictly
* necessary for v to be provided as a pointer as a copy of the struct v
* would also contain the correct pointer to the original vector's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function. 
* Returns 0 on success or -1 on error.
*
* @ float rc_get_vector_entry(rc_vector_t v, int pos)
*
* Returns the entry of vector 'v' in position 'pos' where the position is
* zero-indexed. Returns -1.0f on failure and prints an error message to stderr.
* In practice this is never used as it is much easier for the user to read
* values directly with this code:
*
* val = v.d[pos];
*
* However, we provide this function for completeness. It also provides sanity
* checks to avoid possible segfaults.
*
* @ int rc_print_vector(rc_vector_t v)
*
* Prints to stdout the contents of vector v in one line. This is not advisable
* for extremely long vectors but serves for quickly debugging or printing 
* results. It prints 4 decimal places with padding for a sign. We recommend 
* rc_print_vector_sci() for very small or very large numbers where scientific
* notation would be more appropriate. Returns 0 on success or -1 on failure.
*
* @ int rc_print_vector_sci(rc_vector_t v)
*
* Prints to stdout the contents of vector v in one line. This is not advisable
* for extremely long vectors but serves for quickly debugging or printing 
*
* @ int rc_vector_times_scalar(rc_vector_t* v, float s)
*
* Multiplies every entry in vector v by scalar s. It is not strictly
* necessary for v to be provided as a pointer since a copy of the struct v
* would also contain the correct pointer to the original vector's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function.
* Returns 0 on success or -1 on failure.
*
* @ float rc_vector_norm(rc_vector_t v, float p)
*
* Just like the matlab norm(v,p) function, returns the vector norm defined by
* sum(abs(v)^p)^(1/p), where p is any positive real value. Most common norms
* are the 1 norm which gives the sum of absolute values of the vector and the
* 2-norm which is the square root of sum of squares.
* for infinity and -infinity norms see vector_max and vector_min
*
* @ int rc_vector_max(rc_vector_t v)
*
* Returns the index of the maximum value in v or -1 on failure. The value 
* contained in the returned index is the equivalent to the infinity norm. If the
* max value occurs multiple times then the first instance is returned.
*
* @ int rc_vector_min(rc_vector_t v)
*
* Returns the index of the minimum value in v or -1 on failure. The value 
* contained in the returned index is the equivalent to the minus-infinity norm.
* If the min value occurs multiple times then the first instance is returned.
*
* @ float rc_std_dev(rc_vector_t v)
*
* Returns the standard deviation of the values in a vector or -1.0f on failure.
*
* @ float rc_vector_mean(rc_vector_t v)
*
* Returns the mean (average) of all values in vector v or -1.0f on error.
*
* @ int rc_vector_projection(rc_vector_t v, rc_vector_t e, rc_vector_t* p)
*
* Populates vector p with the projection of vector v onto e.
* Returns 0 on success, otherwise -1.
*
* @ float rc_vector_dot_product(rc_vector_t v1, rc_vector_t v2)
*
* Returns the dot product of two equal-length vectors or floating-point -1.0f
* on error.
*
* @ int rc_vector_outer_product(rc_vector_t v1, rc_vector_t v2, rc_matrix_t* A)
* 
* Computes v1 times v2 where v1 is a column vector and v2 is a row vector.
* Output is a matrix with same rows as v1 and same columns as v2.
* Returns 0 on success, otherwise -1.
*
* @ int rc_vector_cross_product(rc_vector_t v1, rc_vector_t v2, rc_vector_t* p)
*
* Computes the cross-product of two vectors, each of length 3. The result is
* placed in vector p and and existing memory used by p is freed and lost.
* Returns 0 on success, otherwise -1.
*
* @ int rc_vector_sum(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s)
*
* Populates vector s with the sum of vectors v1 and v2. Any existing memory
* allocated for s is freed and lost, new memory is allocated if necessary.
* Returns 0 on success, otherwise -1.
*
* @ int rc_vector_sum_inplace(rc_vector_t* v1, rc_vector_t v2)
*
* Adds vector v2 to v1 and leaves the result in v1. The original contents of v1
* are lost and v2 is left untouched.
* Returns 0 on success, otherwise -1.
*******************************************************************************/
int   rc_alloc_vector(rc_vector_t* v, int length);
int   rc_free_vector(rc_vector_t* v);
rc_vector_t rc_empty_vector();
int   rc_vector_zeros(rc_vector_t* v, int length);
int   rc_vector_ones(rc_vector_t* v, int length);
int   rc_random_vector(rc_vector_t* v, int length);
int   rc_vector_fibonnaci(rc_vector_t* v, int length);
int   rc_vector_from_array(rc_vector_t* v, float* ptr, int length);
int   rc_duplicate_vector(rc_vector_t a, rc_vector_t* b);
int   rc_set_vector_entry(rc_vector_t* v, int pos, float val);
float rc_get_vector_entry(rc_vector_t v, int pos);
int   rc_print_vector(rc_vector_t v);
int   rc_print_vector_sci(rc_vector_t v);
int   rc_vector_times_scalar(rc_vector_t* v, float s);
float rc_vector_norm(rc_vector_t v, float p);
int   rc_vector_max(rc_vector_t v);
int   rc_vector_min(rc_vector_t v);
float rc_std_dev(rc_vector_t v);
float rc_vector_mean(rc_vector_t v);
int   rc_vector_projection(rc_vector_t v, rc_vector_t e, rc_vector_t* p);
float rc_vector_dot_product(rc_vector_t v1, rc_vector_t v2);
int   rc_vector_outer_product(rc_vector_t v1, rc_vector_t v2, rc_matrix_t* A);
int   rc_vector_cross_product(rc_vector_t v1, rc_vector_t v2, rc_vector_t* p);
int   rc_vector_sum(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s);
int   rc_vector_sum_inplace(rc_vector_t* v1, rc_vector_t v2);



/*******************************************************************************
* Matrix
*
* @ int rc_alloc_matrix(rc_matrix_t* A, int rows, int cols)
*
* Allocates memory for matrix A to have new dimensions given by arguments rows 
* and cols. If A is initially the right size, nothing is done and the data in A
* is preserved. If A is uninitialized or of the wrong size then any existing
* memory is freed and new memory is allocated, helping to prevent accidental
* memory leaks. The contents of the new matrix is not guaranteed to be anything
* in particular.
* Returns 0 on success, otherwise -1. Will only be unsuccessful if 
* rows&cols are invalid or there is insufficient memory available.
*
* @ int rc_free_matrix(rc_matrix_t* A)
*
* Frees the memory allocated for a matrix A and importantly sets the dimensions
* and initialized flag of the rc_matrix_t struct to 0 to indicate to other
* functions that A no longer points to allocated memory and cannot be used until
* more memory is allocated such as with rc_alloc_matrix or rc_matrix_zeros.
* Returns 0 on success. Will only fail and return -1 if it is passed a NULL
* pointer.
*
* @ rc_matrix_t rc_empty_matrix()
*
* Returns an rc_matrix_t with no allocated memory and the initialized flag set
* to 0. This is useful for initializing rc_matrix_t structs when they are
* declared since local variables declared in a function without global variable
* scope in C are not guaranteed to be zeroed out which can lead to bad memory 
* pointers and segfaults if not handled carefully. We recommend initializing all
* matrices with this before using rc_alloc_matrix or any other function.
*
* @ int rc_matrix_zeros(rc_matrix_t* A, int rows, int cols)
*
* Resizes matrix A and allocates memory for a matrix with specified rows &
* columns. The new memory is pre-filled with zeros. Any existing memory 
* allocated for A is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on error.
*
* @ int rc_identity_matrix(rc_matrix_t* A, int dim)
*
* Resizes A to be a square identity matrix with dimensions dim-by-dim. Any
* existing memory allocated for A is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ int rc_random_matrix(rc_matrix_t* A, int rows, int cols)
*
* Resizes A to be a matrix with the specified number of rows and columns and
* populates the new memory with random numbers evenly distributed between -1.0
* and 1.0. Any existing memory allocated for A is freed if necessary to avoid
* memory leaks. Returns 0 on success or -1 on failure.
*
* @ int rc_diag_matrix(rc_matrix_t* A, rc_vector_t v)
*
* Resizes A to be a square matrix with the same number of rows and columns as 
* vector v's length. The diagonal entries of A are then populated with the
* contents of v and the off-diagonal entries are set to 0. The original contents
* of A are freed to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ int rc_duplicate_matrix(rc_matrix_t A, rc_matrix_t* B)
*
* Makes a duplicate of the data from matrix A and places into matrix B. If B is
* already the right size then its contents are overwritten. If B is unallocated
* or is of the wrong size then the memory is freed if necessary and new memory
* is allocated to hold the duplicate of A.
* Returns 0 on success or -1 on error.
*
* @ int rc_set_matrix_entry(rc_matrix_t* A, int row, int col, float val)
*
* Sets the specified single entry of matrix A to 'val' where the position is
* zero-indexed at the top-left corner. In practice this is never used as it is
* much easier for the user to set values directly with this code:
*
* A.d[row][col]=val;
*
* However, we provide this function for completeness. It is not strictly
* necessary for A to be provided as a pointer since a copy of the struct A
* would also contain the correct pointer to the original matrix's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function. Returns 0 on success or -1 on error.
*
* @ float rc_get_matrix_entry(rc_matrix_t A, int row, int col)
*
* Returns the specified single entry of matrix 'A' in position 'pos' where the
* position is zero-indexed. Returns -1.0f on failure and prints an error message
* to stderr. In practice this is never used as it is much easier for the user to
* read values directly with this code:
*
* val = A.d[row][col];
*
* However, we provide this function for completeness. It also provides sanity
* checks to avoid possible segfaults.
*
* @ int rc_print_matrix(rc_matrix_t A)
*
* Prints the contents of matrix A to stdout in decimal notation with 4 decimal
* places. Not recommended for very large matrices as rows will typically
* linewrap if the terminal window is not wide enough.
*
* @ void rc_print_matrix_sci(rc_matrix_t A)
*
* Prints the contents of matrix A to stdout in scientific notation with 4
* significant figures. Not recommended for very large matrices as rows will 
* typically linewrap if the terminal window is not wide enough.
*
* @ int rc_matrix_times_scalar(rc_matrix_t* A, float s)
*
* Multiplies every entry in A by scalar value s. It is not strictly
* necessary for A to be provided as a pointer since a copy of the struct A
* would also contain the correct pointer to the original matrix's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function. Returns 0 on success or -1 on failure.
*
* @ int rc_multiply_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C)
*
* Multiplies A*B=C. C is resized and its original contents are freed if 
* necessary to avoid memory leaks. Returns 0 on success or -1 on failure.
*
* @ int rc_left_multiply_matrix_inplace(rc_matrix_t A, rc_matrix_t* B)
*
* Multiplies A*B and puts the result back in the place of B. B is resized and
* its original contents are freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ int rc_right_multiply_matrix_inplace(rc_matrix_t* A, rc_matrix_t B)
*
* Multiplies A*B and puts the result back in the place of A. A is resized and
* its original contents are freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ int rc_add_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C)
*
* Resizes matrix C and places the sum A+B in C. The original contents of C are
* safely freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ int rc_add_matrices_inplace(rc_matrix_t* A, rc_matrix_t B)
*
* Adds matrix B to A and places the result in A so the original contents of A
* are lost. Use rc_add_matrices if you wish to keep the contents of both matrix
* A and B. Returns 0 on success or -1 on failure.
*
* @ int rc_matrix_transpose(rc_matrix_t A, rc_matrix_t* T)
*
* Resizes matrix T to hold the transposed contents of A and leaves A untouched.
* Returns 0 on success or -1 on failure.
*
* @ int rc_matrix_transpose_inplace(rc_matrix_t* A)
*
* Transposes matrix A in place. Use as an alternative to rc_matrix_transpose
* if you no longer have need for the original contents of matrix A.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int   rc_alloc_matrix(rc_matrix_t* A, int rows, int cols);
int   rc_free_matrix(rc_matrix_t* A);
rc_matrix_t rc_empty_matrix();
int   rc_matrix_zeros(rc_matrix_t* A, int rows, int cols);
int   rc_identity_matrix(rc_matrix_t* A, int dim);
int   rc_random_matrix(rc_matrix_t* A, int rows, int cols);
int   rc_diag_matrix(rc_matrix_t* A, rc_vector_t v);
int   rc_duplicate_matrix(rc_matrix_t A, rc_matrix_t* B);
int   rc_set_matrix_entry(rc_matrix_t* A, int row, int col, float val);
float rc_get_matrix_entry(rc_matrix_t A, int row, int col);
int   rc_print_matrix(rc_matrix_t A);
void  rc_print_matrix_sci(rc_matrix_t A);
int   rc_matrix_times_scalar(rc_matrix_t* A, float s);
int   rc_multiply_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C);
int   rc_left_multiply_matrix_inplace(rc_matrix_t A, rc_matrix_t* B);
int   rc_right_multiply_matrix_inplace(rc_matrix_t* A, rc_matrix_t B);
int   rc_add_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C);
int   rc_add_matrices_inplace(rc_matrix_t* A, rc_matrix_t B);
int   rc_matrix_transpose(rc_matrix_t A, rc_matrix_t* T);
int   rc_matrix_transpose_inplace(rc_matrix_t* A);

/*******************************************************************************
* Linear Algebra
*
* @ int rc_matrix_times_col_vec(rc_matrix_t A, rc_vector_t v, rc_vector_t* c)
*
* Multiplies matrix A times column vector v and places the result in column
* vector c. Any existing data in c is freed if necessary and c is resized
* appropriately. Vectors v and c are interpreted as column vectors, but nowhere
* in their definitions are they actually specified as one or the other.
* Returns 0 on success and -1 on failure.
*
* @ int rc_row_vec_times_matrix(rc_vector_t v, rc_matrix_t A, rc_vector_t* c)
*
* Multiplies row vector v times matrix A and places the result in row
* vector c. Any existing data in c is freed if necessary and c is resized
* appropriately. Vectors v and c are interpreted as row vectors, but nowhere
* in their definitions are they actually specified as one or the other.
* Returns 0 on success and -1 on failure.
*
* @ float rc_matrix_determinant(rc_matrix_t A)
*
* Returns the determinant of square matrix A or -1.0f on failure.
*
* @ int rc_lup_decomp(rc_matrix_t A, rc_matrix_t* L, rc_matrix_t* U, rc_matrix_t* P)
*
* Performs LUP decomposition on matrix A with partial pivoting and places the
* result in matrices L,U,&P. Matrix A remains untouched and the original
* contents of LUP (if any) are freed and LUP are resized appropriately.
* Returns 0 on success or -1 on failure.
*
* @ int rc_qr_decomp(rc_matrix_t A, rc_matrix_t* Q, rc_matrix_t* R)
*
* Uses householder reflection method to find the QR decomposition of A.
* Returns 0 on success or -1 on failure.
*
* @ int rc_invert_matrix(rc_matrix_t A, rc_matrix_t* Ainv)
*
* Inverts Matrix A via LUP decomposition method and places the result in matrix
* Ainv. Any existing memory allocated for Ainv is freed if necessary and its
* contents are overwritten. Returns 0 on success or -1 on failure such as if
* matrix A is not invertible.
*
* @ int rc_invert_matrix_inplace(rc_matrix_t* A)
*
* Inverts Matrix A in place. The original contents of A are lost.
* Returns 0 on success or -1 on failure such as if A is not invertible.
*
* @ int rc_lin_system_solve(rc_matrix_t A, rc_vector_t b, rc_vector_t* x)
*
* Solves Ax=b for given matrix A and vector b. Places the result in vector x.
* existing contents of x are freed and new memory is allocated if necessary.
* Thank you to Henry Guennadi Levkin for open sourcing this routine, it's
* adapted here for RC use and includes better detection of unsolvable systems.
* Returns 0 on success or -1 on failure.
* 
* @ int rc_lin_system_solve_qr(rc_matrix_t A, rc_vector_t b, rc_vector_t* x)
*
* Finds a least-squares solution to the system Ax=b for non-square A using QR
* decomposition method and places the solution in x. 
* Returns 0 on success or -1 on failure.
*
* @ int rc_fit_ellipsoid(rc_matrix_t pts, rc_vector_t* ctr, rc_vector_t* lens)
*
* Fits an ellipsoid to a set of points in 3D space. The principle axes of the
* fitted ellipsoid align with the global coordinate system. Therefore there are
* 6 degrees of freedom defining the ellipsoid: the x,y,z coordinates of the
* centroid and the lengths from the centroid to the surface in each of the 3
* directions. 
*
* rc_matrix_t 'pts' is a tall matrix with 3 columns and at least 6 rows.
* Each row must contain the x,y&z components of each individual point to be fit.
* If only 6 rows are provided, the resulting ellipsoid will be an exact fit.
* Otherwise the result is a least-squares fit to the over-defined dataset.
*
* The final x,y,z position of the centroid will be placed in vector 'ctr' and
* the lengths or radius from the centroid to the surface along each axis will
* be placed in the vector 'lens'
*
* Returns 0 on success or -1 on failure. 
*******************************************************************************/
int   rc_matrix_times_col_vec(rc_matrix_t A, rc_vector_t v, rc_vector_t* c);
int   rc_row_vec_times_matrix(rc_vector_t v, rc_matrix_t A, rc_vector_t* c);
float rc_matrix_determinant(rc_matrix_t A);
int   rc_lup_decomp(rc_matrix_t A, rc_matrix_t* L, rc_matrix_t* U, rc_matrix_t* P);
int   rc_qr_decomp(rc_matrix_t A, rc_matrix_t* Q, rc_matrix_t* R);
int   rc_invert_matrix(rc_matrix_t A, rc_matrix_t* Ainv);
int   rc_invert_matrix_inplace(rc_matrix_t* A);
int   rc_lin_system_solve(rc_matrix_t A, rc_vector_t b, rc_vector_t* x);
int   rc_lin_system_solve_qr(rc_matrix_t A, rc_vector_t b, rc_vector_t* x);
int   rc_fit_ellipsoid(rc_matrix_t pts, rc_vector_t* ctr, rc_vector_t* lens);


/*******************************************************************************
* polynomial Manipulation
*
* We represent polynomials as a vector of coefficients with the highest power
* term on the left at vector index 0. The following polynomial manipulation
* functions are designed to behave like their counterparts in the Numerical
* Renaissance codebase.
*
* @ int rc_print_poly(rc_vector_t v)
*
* Like rc_print_vector, but assumes the contents represent a polynomial and
* prints the coefficients with trailing powers of x for easier reading. This
* relies on your terminal supporting unicode UTF-8.
*
* @ int rc_poly_conv(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
*
* Convolutes the polynomials a&b and places the result in vector c. This finds
* the coefficients of the polynomials resulting from multiply a*b. The original
* contents of c are freed and new memory is allocated if necessary.
* returns 0 on success or -1 on failure.
*
* @ int rc_poly_power(rc_vector_t a, int n, rc_vector_t* b)
*
* Raises a polynomial a to itself n times where n is greater than or equal to 0.
* Places the result in vector b, any existing memory allocated for b is freed
* and its contents are lost. Returns 0 on success and -1 on failure.
*
* @ int rc_poly_add(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
*
* Add two polynomials a&b with right justification and place the result in c.
* Any existing memory allocated for c is freed and its contents are lost.
* Returns 0 on success and -1 on failure.
*
* @ int rc_poly_add_inplace(rc_vector_t* a, rc_vector_t b)
*
* Adds polynomials b&a with right justification. The result is placed in vector
* a and a's original contents are lost. More memory is allocated for a if
* necessary. Returns 0 on success or -1 on failure.
*
* @ int rc_poly_subtract(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
*
* Subtracts two polynomials a-b with right justification and place the result in
* c. Any existing memory allocated for c is freed and its contents are lost.
* Returns 0 on success and -1 on failure.
*
* @ int rc_poly_subtract_inplace(rc_vector_t* a, rc_vector_t b)
*
* Subtracts b from a with right justification. a stays in place and new memory 
* is allocated only if b is longer than a.
*
* @ int rc_poly_differentiate(rc_vector_t a, int d, rc_vector_t* b)
*
* Calculates the dth derivative of the polynomial a and places the result in 
* vector b. Returns 0 on success or -1 on failure.
*
* @ int rc_poly_divide(rc_vector_t n, rc_vector_t d, rc_vector_t* div, rc_vector_t* rem)
*
* Divides denominator d into numerator n. The remainder is placed into vector
* rem and the divisor is placed into vector div. Returns 0 on success or -1
* on failure.
*
* @ int rc_poly_butter(int N, float wc, rc_vector_t* b)
*
* Calculates vector of coefficients for continuous-time Butterworth polynomial
* of order N and cutoff wc (rad/s) and places them in vector b.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_print_poly(rc_vector_t v);
int rc_poly_conv(rc_vector_t a, rc_vector_t b, rc_vector_t* c);
int rc_poly_power(rc_vector_t a, int n, rc_vector_t* b);
int rc_poly_add(rc_vector_t a, rc_vector_t b, rc_vector_t* c);
int rc_poly_add_inplace(rc_vector_t* a, rc_vector_t b);
int rc_poly_subtract(rc_vector_t a, rc_vector_t b, rc_vector_t* c);
int rc_poly_subtract_inplace(rc_vector_t* a, rc_vector_t b);
int rc_poly_differentiate(rc_vector_t a, int d, rc_vector_t* b);
int rc_poly_divide(rc_vector_t n, rc_vector_t d, rc_vector_t* div, rc_vector_t* rem);
int rc_poly_butter(int N, float wc, rc_vector_t* b);

/*******************************************************************************
* Quaternion Math
*
* @ float rc_quaternion_norm(rc_vector_t q)
*
* Returns the length of a quaternion vector by finding its 2-norm.
* Prints an error message and returns -1.0f on error.
*
* @ float rc_quaternion_norm_array(float q[4])
*
* Returns the length of a quaternion vector by finding its 2-norm.
* Prints an error message and returns -1.0f on error.
*
* @ int rc_normalize_quaternion(rc_vector_t* q)
*
* Normalizes a quaternion in-place to have length 1.0. Returns 0 on success.
* Returns -1 if the quaternion is uninitialized or has 0 length.
*
* @ int rc_normalize_quaternion_array(float q[4])
*
* Same as normalize_quaternion but performs the action on an array instead of
* a rc_vector_t type. 
*
* @ int rc_quaternion_to_tb(rc_vector_t q, rc_vector_t* tb)
*
* Populates vector tb with 321 Tait Bryan angles in array order XYZ with
* operation order 321(yaw-Z, pitch-Y, roll-x). If tb is already allocated and of
* length 3 then the new values are written in place, otherwise any existing 
* memory is freed and a new vector of length 3 is allocated for tb. 
* Returns 0 on success or -1 on failure.
*
* @ void rc_quaternion_to_tb_array(float q[4], float tb[3])
*
* Same as rc_quaternion_to_tb but takes arrays instead.
*
* @ int rc_tb_to_quaternion(rc_vector_t tb, rc_vector_t* q)
*
* Populates quaternion vector q with the quaternion corresponding to the 
* tait-bryan pitch-roll-yaw values in vector tb. If q is already of length 4
* then old contents are simply overwritten. Otherwise q'd existing memory is
* freed and new memory is allocated to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ void rc_tb_to_quaternion_array(float tb[3], float q[4])
*
* Like rc_tb_to_quaternion but takes arrays as arguments.
*
* @ int rc_quaternion_conjugate(rc_vector_t q, rc_vector_t* c)
*
* Populates quaternion vector c with the conjugate of quaternion q where the 3 
* imaginary parts ijk are multiplied by -1. If c is already of length 4 then the
* old values are overwritten. Otherwise the old memory in c is freed and new
* memory is allocated to help prevent memory leaks. 
* Returns 0 on success or -1 on failure.
*
* @ int rc_quaternion_conjugate_inplace(rc_vector_t* q)
*
* Conjugates quaternion q by multiplying the 3 imaginary parts ijk by -1.
* Returns 0 on success or -1 on failure.
*
* @ void rc_quaternion_conjugate_array(float q[4], float c[4])
*
* Populates quaternion vector c with the conjugate of quaternion q where the 3 
* imaginary parts ijk are multiplied by -1.
* Returns 0 on success or -1 on failure.
*
* @ void rc_quaternion_conjugate_array_inplace(float q[4])
*
* Conjugates quaternion q by multiplying the 3 imaginary parts ijk by -1.
* Returns 0 on success or -1 on failure.
*
* @ int rc_quaternion_imaginary_part(rc_vector_t q, rc_vector_t* img)
*
* Populates vector i with the imaginary components ijk of of quaternion vector
* q. If img is already of length 3 then its original contents are overwritten.
* Otherwise the original allocated memory is freed and new memory is allocated.
* Returns 0 on success or -1 on failure.
*
* @ int rc_quaternion_multiply(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
*
* Calculates the quaternion Hamilton product ab=c and places the result in
* vector argument c. If c is already of length 4 then the old values are 
* overwritten. Otherwise the old memory in c is freed and new memory is
* allocated to help prevent memory leaks. 
* Returns 0 on success or -1 on failure.
*
* @ void rc_quaternion_multiply_array(float a[4], float b[4], float c[4])
*
* Calculates the quaternion Hamilton product ab=c and places the result in c
*
* @ int rc_rotate_quaternion(rc_vector_t* p, rc_vector_t q)
*
* Rotates the quaternion p by quaternion q with the operation p'=qpq* 
* Returns 0 on success or -1 on failure.
*
* @ void rc_rotate_quaternion_array(float p[4], float q[4])
*
* Rotates the quaternion p by quaternion q with the operation p'=qpq* 
*
* @ int rc_quaternion_rotate_vector(rc_vector_t* v, rc_vector_t q)
*
* Rotate a 3D vector v in-place about the origin by quaternion q by converting
* v to a quaternion and performing the operation p'=qpq* 
* Returns 0 on success or -1 on failure.
*
* @ void rc_quaternion_rotate_vector_array(float v[3], float q[4])
*
* Rotate a 3D vector v in-place about the origin by quaternion q by converting
* v to a quaternion and performing the operation p'=qpq* 
*
* @ int rc_quaternion_to_rotation_matrix(rc_vector_t q, rc_matrix_t* m)
*
* Populates m with a 3x3 rotation matrix which would perform the equivalent
* rotation as quaternion q when multiplied by a 3D vector. If m is already
* 3x3 then its contents are overwritten, otherwise its existing memory is freed
* and new memory is allocated.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
float rc_quaternion_norm(rc_vector_t q);
float rc_quaternion_norm_array(float q[4]);
int   rc_normalize_quaternion(rc_vector_t* q);
int   rc_normalize_quaternion_array(float q[4]);
int   rc_quaternion_to_tb(rc_vector_t q, rc_vector_t* tb);
void  rc_quaternion_to_tb_array(float q[4], float tb[3]);
int   rc_tb_to_quaternion(rc_vector_t tb, rc_vector_t* q);
void  rc_tb_to_quaternion_array(float tb[3], float q[4]);
int   rc_quaternion_conjugate(rc_vector_t q, rc_vector_t* c);
int   rc_quaternion_conjugate_inplace(rc_vector_t* q);
void  rc_quaternion_conjugate_array(float q[4], float c[4]);
void  rc_quaternion_conjugate_array_inplace(float q[4]);
int   rc_quaternion_imaginary_part(rc_vector_t q, rc_vector_t* img);
int   rc_quaternion_multiply(rc_vector_t a, rc_vector_t b, rc_vector_t* c);
void  rc_quaternion_multiply_array(float a[4], float b[4], float c[4]);
int   rc_rotate_quaternion(rc_vector_t* p, rc_vector_t q);
void  rc_rotate_quaternion_array(float p[4], float q[4]);
int   rc_quaternion_rotate_vector(rc_vector_t* v, rc_vector_t q);
void  rc_quaternion_rotate_vector_array(float v[3], float q[4]);
int   rc_quaternion_to_rotation_matrix(rc_vector_t q, rc_matrix_t* m);

/*******************************************************************************
* Ring Buffer
*
* Ring buffers are FIFO (first in first out) buffers of fixed length which
* efficiently boot out the oldest value when full. They are particularly well
* suited for storing the last n values in a discrete time filter.
*
* The user creates their own instance of a buffer and passes a pointer to the
* these ring_buf functions to perform normal operations. 
*
* @ int rc_alloc_ringbuf(rc_ringbuf_t* buf, int size)
*
* Allocates memory for a ring buffer and initializes an rc_ringbuf_t struct.
* If ring buffer b is already the right size then it is left untouched.
* Otherwise any existing memory allocated for buf is free'd to avoid memory
* leaks and new memory is allocated. Returns 0 on success or -1 on failure.
*
* @ rc_ringbuf_t rc_empty_ringbuf()
*
* Returns an rc_ringbuf_t struct which is completely zero'd out with no memory
* allocated for it. This is useful for declaring new ring buffers since structs
* declared inside of functions are not necessarily zero'd out which can cause
* the struct to contain problematic contents leading to segfaults. New ring
* buffers should be initialized with this before calling rc_alloc_ringbuf.
*
* @ int rc_free_ringbuf(rc_ringbuf_t* buf)
*
* Frees the memory allocated for buffer buf. Also set the initialized flag to 0
* so other functions don't try to access unallocated memory.
* Returns 0 on success or -1 on failure.
*
* @ int rc_reset_ringbuf(rc_ringbuf_t* buf)
*
* Sets all values in the buffer to 0 and sets the buffer index back to 0.
* Returns 0 on success or -1 on failure.
*
* @ int rc_insert_new_ringbuf_value(rc_ringbuf_t* buf, float val)
* 
* Puts a new float into the ring buffer and updates the index accordingly.
* If the buffer was full then the oldest value in the buffer is automatically
* removed. Returns 0 on success or -1 on failure.
*
* @ float rc_get_ringbuf_value(rc_ringbuf_t* buf, int pos)
*
* Returns the float which is 'pos' steps behind the last value added to the 
* buffer. If 'position' is given as 0 then the most recent value is returned. 
* Position 'pos' obviously can't be larger than the buffer size minus 1.
* Prints an error message and return -1.0f on error.
*
* @ float rc_std_dev_ringbuf(rc_ringbuf_t buf)
*
* Returns the standard deviation of the values in the ring buffer.
*******************************************************************************/
typedef struct rc_ringbuf_t {
	float* d;
	int size;
	int index;
	int initialized;
} rc_ringbuf_t;

int   rc_alloc_ringbuf(rc_ringbuf_t* buf, int size);
rc_ringbuf_t rc_empty_ringbuf();
int   rc_reset_ringbuf(rc_ringbuf_t* buf);
int   rc_free_ringbuf(rc_ringbuf_t* buf);
int   rc_insert_new_ringbuf_value(rc_ringbuf_t* buf, float val);
float rc_get_ringbuf_value(rc_ringbuf_t* buf, int position);
float rc_std_dev_ringbuf(rc_ringbuf_t buf);

/*******************************************************************************
* Discrete SISO Filters
*
* This is a collection of functions for generating and implementing discrete 
* SISO filters for arbitrary transfer functions. 
*
* @ int rc_alloc_filter(rc_filter_t* f, rc_vector_t num, rc_vector_t den, float dt)
*
* Allocate memory for a discrete-time filter & populates it with the transfer
* function coefficients provided in vectors num and den. The memory in num and
* den is duplicated so those vectors can be reused or freed after allocating a 
* filter without fear of disturbing the function of the filter. Argument dt is
* the timestep in seconds at which the user expects to operate the filter.
* The length of demonimator den must be at least as large as numerator num to
* avoid describing an improper transfer function. If rc_filter_t pointer f
* points to an existing filter then the old filter's contents are freed safely
* to avoid memory leaks. We suggest initializing filter f with rc_empty_filter
* before calling this function if it is not a global variable to ensure it does
* not accidentally contain invlaid contents such as null pointers. The filter's
* order is derived from the length of the denominator polynomial.
* Returns 0 on success or -1 on failure.
*
* @ int rc_alloc_filter_from_arrays(rc_filter_t* f,int order,float dt,float* num,float* den)
*
* Like rc_alloc_filter(), but takes arrays for the numerator and denominator
* coefficients instead of vectors. Arrays num and denmust be the same length
* (order+1) like a semi-proper transfer function. Proper transfer functions with
* relative degree >=1 can still be used but the numerator must be filled with
* leading zeros. This function will throw a segmentation fault if your arrays
* are not both of length order+1. It is safer to use the rc_alloc_filter.
* Returns 0 on success or -1 on failure.
*
* @ int rc_free_filter(rc_filter_t* f)
*
* Frees the memory allocated by a filter's buffers and coefficient vectors. Also
* resets all filter properties back to 0. Returns 0 on success or -1 on failure.
*
* @ rc_filter_t rc_empty_filter()
*
* This is a very important function. If your d_filter_t struct is not a global
* variable, then its initial contents cannot be guaranteed to be anything in
* particular. Therefore it could contain problematic contents which could
* interfere with functions in this library. Therefore, you should always
* initialize your filters with rc_empty_filter before using with any other
* function in this library such as rc_alloc_filter. This serves the same
* purpose as rc_empty_matrix, rc_empty_vector, and rc_empty_ringbuf.
*
* @ int rc_print_filter(rc_filter_t f)
*
* Prints the transfer function and other statistic of a filter to the screen.
* only works on filters up to order 9
*
* @ float rc_march_filter(rc_filter_t* f, float new_input)
*
* March a filter forward one step with new input provided as an argument.
* Returns the new output which could also be accessed with filter.newest_output
* If saturation or soft-start are enabled then the output will automatically be
* bound appropriately. The steps counter is incremented by one and internal
* ring buffers are updated accordingly. Once a filter is created, this is
* typically the only function required afterwards.
*
* @ int rc_reset_filter(rc_filter_t* f)
*
* Resets all previous inputs and outputs to 0 and resets the step counter
* and saturation flag. This is sufficient to start the filter again as if it
* were just created. Returns 0 on success or -1 on failure.
*
* @ int rc_enable_saturation(rc_filter_t* f, float min, float max)
*
* If saturation is enabled for a specified filter, the filter will automatically
* bound the output between min and max. You may ignore this function if you wish
* the filter to run unbounded.
*
* @ int rc_did_filter_saturate(rc_filter_t* f)
*
* Returns 1 if the filter saturated the last time step. Returns 0 otherwise.
* This information could also be retrieved by looking at the 'sat_flag' value
* in the filter struct.
*
* @ int enable_soft_start(rc_filter_t* filter, float seconds)
*
* Enables soft start functionality where the output bound is gradually opened
* linearly from 0 to the normal saturation range. This occurs over the time 
* specified from argument 'seconds' from when the filter is first created or 
* reset. Saturation must already be enabled for this to work. This assumes that
* the user does indeed call rc_march_filter at roughly the same time interval
* as the 'dt' variable in the filter struct which is set at creation time.
* The soft-start property is maintained through a call to rc_reset_filter
* so the filter will soft-start again after each reset. This feature should only
* really be used for feedback controllers to prevent jerky starts.
* The saturation flag will not be set during this period as the output is
* usually expected to be bounded and we don't want to falsely trigger alarms
* or saturation counters. Returns 0 on success or -1 on failure.
*
* @ float rc_previous_filter_input(rc_filter_t* f, int steps)
*
* Returns the input 'steps' back in time. Steps = 0 returns most recent input.
* 'steps' must be between 0 and order inclusively as those are the
* only steps retained in memory for normal filter operation. To record values
* further back in time we suggest creating your own rc_ringbuf_t ring buffer.
* Returns -1.0f and prints an error message if there is an issue.
*
* @ float rc_previous_filter_output(rc_filter_t* f, int steps)
*
* Returns the input 'steps' back in time. Steps = 0 returns most recent input.
* 'steps' must be between 0 and order inclusively as those are the
* only steps retained in memory for normal filter operation. To record values
* further back in time we suggest creating your own rc_ringbuf_t ring buffer.
* Returns -1.0f and prints an error message if there is an issue.
*
* @ float rc_newest_filter_output(rc_filter_t* f)
*
* Returns the most recent output from the filter. Alternatively the user could
* access the 'newest_output' component of the rc_filter_t struct. Returns -1.0f
* and prints an error message if there is an issue.
*
* @ float rc_newest_filter_input(rc_filter_t* f)
*
* Returns the most recent input to the filter. Alternatively the user could
* access the 'newest_input' component of the rc_filter_t struct. Returns -1.0f
* and prints an error message if there is an issue.
*
* @ int rc_prefill_filter_inputs(rc_filter_t* f, float in)
*
* Fills all previous inputs to the filter as if they had been equal to 'in'
* Most useful when starting high-pass filters to prevent unwanted jumps in the
* output when starting with non-zero input.
* Returns 0 on success or -1 on failure.
*
* @ int prefill_filter_outputs(rc_filter_t* f, float out)
*
* Fills all previous outputs of the filter as if they had been equal to 'out'
* Most useful when starting low-pass filters to prevent unwanted settling time
* when starting with non-zero input. Returns 0 on success or -1 on failure.
*
* @ int rc_multiply_filters(rc_filter_t f1, rc_filter_t f2, rc_filter_t* f3)
*
* Creates a new filter f3 by multiplying f1*f2. The contents of f3 are freed
* safely if necessary and new memory is allocated to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*
* @ int rc_c2d_tustin(rc_filter_t* f,rc_vector_t num,rc_vector_t den,float dt,float w)
* 
* Creates a discrete time filter with similar dynamics to a provided continuous
* time transfer function using tustin's approximation with prewarping about a
* frequency of interest 'w' in radians per second.
*
* arguments:
* rc_vector_t num: 	continuous time numerator coefficients
* rc_vector_t den: 	continuous time denominator coefficients
* float dt:			desired timestep of discrete filter
* float w:			prewarping frequency in rad/s
*
* Any existing memory allocated for f is freed is necessary to prevent memory
* leaks. Returns 0 on success or -1 on failure.
*
* @ int rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant)
*
* Creates a first order low pass filter. Any existing memory allocated for f is 
* freed safely to avoid memory leaks and new memory is allocated for the new 
* filter. dt is in units of seconds and time_constant is the number of seconds 
* it takes to rise to 63.4% of a steady-state input. This can be used alongside
* rc_first_order_highpass to make a complementary filter pair.
* Returns 0 on success or -1 on failure.
*
* @ int rc_first_order_highpass(rc_filter_t* f, float dt, float time_constant)
*
* Creates a first order high pass filter. Any existing memory allocated for f is 
* freed safely to avoid memory leaks and new memory is allocated for the new 
* filter. dt is in units of seconds and time_constant is the number of seconds 
* it takes to decay by 63.4% of a steady-state input. This can be used alongside
* rc_first_order_lowpass to make a complementary filter pair.
* Returns 0 on success or -1 on failure.
*
* @ int rc_butterworth_lowpass(rc_filter_t* f, int order, float dt, float wc)
*
* Creates a Butterworth low pass filter of specified order and cutoff frequency
* wc in rad/s. The user must also specify the discrete filter's timestep dt
* in seconds. Any existing memory allocated for f is freed safely to avoid
* memory leaks and new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*
* @ int rc_butterworth_highpass(rc_filter_t* f, int order, float dt, float wc)
*
* Creates a Butterworth high pass filter of specified order and cutoff frequency
* wc in rad/s. The user must also specify the discrete filter's timestep dt
* in seconds. Any existing memory allocated for f is freed safely to avoid
* memory leaks and new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*
* @ int rc_moving_average(rc_filter_t* f, int samples, int dt)
*
* Makes a FIR moving average filter that averages over 'samples' which must be
* greater than or equal to 2 otherwise no averaging would be performed. Any
* existing memory allocated for f is freed safely to avoid memory leaks and new
* memory is allocated for the new filter. Returns 0 on success or -1 on failure.
*
* @ int rc_integrator(rc_filter_t *f, float dt)
*
* Creates a first order integrator. Like most functions here, the dynamics are
* only accurate if the filter is called with a timestep corresponding to dt.
* Any existing memory allocated for f is freed safely to avoid memory leaks and
* new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*
* @ int rc_double_integrator(rc_filter_t* f, float dt)
*
* Creates a second order double integrator. Like most functions here, the 
* dynamics are only accurate if the filter is called with a timestep
* corresponding to dt. Any existing memory allocated for f is freed safely to
* avoid memory leaks and new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*
* @ int rc_pid_filter(rc_filter_t* f,float kp,float ki,float kd,float Tf,float dt)
*
* Creates a discrete-time implementation of a parallel PID controller with 
* high-frequency rolloff. This is equivalent to the Matlab function: 
* C = pid(Kp,Ki,Kd,Tf,Ts)
*
* We cannot implement a pure differentiator with a discrete transfer function
* so this filter has high frequency rolloff with time constant Tf. Smaller Tf
* results in less rolloff, but Tf must be greater than dt/2 for stability.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
typedef struct rc_filter_t{
	// transfer function properties
	int order;			// transfer function order
	float dt;			// timestep in seconds
	float gain;			// gain usually 1.0
	rc_vector_t num;	// numerator coefficients 
	rc_vector_t den;	// denominator coefficients 
	// saturation settings
	int sat_en;			// set to 1 by enable_saturation()
	float sat_min;		// lower saturation limit
	float sat_max;		// upper saturation limit
	int sat_flag;		// 1 if saturated on the last step
	// soft start settings
	int ss_en;			// set to 1 by enbale_soft_start()
	float ss_steps;		// steps before full output allowed
	// dynamically allocated ring buffers
	rc_ringbuf_t in_buf;
	rc_ringbuf_t out_buf;
	// newest input and output for quick reference
	float newest_input;	// shortcut for the most recent input
	float newest_output;// shortcut for the most recent output
	// other
	uint64_t step;		// steps since last reset
	int initialized;	// initialization flag
} rc_filter_t;

int   rc_alloc_filter(rc_filter_t* f, rc_vector_t num, rc_vector_t den, float dt);
int   rc_alloc_filter_from_arrays(rc_filter_t* f,int order,float dt,float* num,float* den);
int   rc_free_filter(rc_filter_t* f);
rc_filter_t rc_empty_filter();
int   rc_print_filter(rc_filter_t f);
float rc_march_filter(rc_filter_t* f, float new_input);
int   rc_reset_filter(rc_filter_t* f);
int   rc_enable_saturation(rc_filter_t* f, float min, float max);
int   rc_did_filter_saturate(rc_filter_t* f);
int   rc_enable_soft_start(rc_filter_t* f, float seconds);
float rc_previous_filter_input(rc_filter_t* f, int steps);
float rc_previous_filter_output(rc_filter_t* f, int steps);
float rc_newest_filter_output(rc_filter_t* f);
float rc_newest_filter_input(rc_filter_t* f);
int   rc_prefill_filter_inputs(rc_filter_t* f, float in);
int   rc_prefill_filter_outputs(rc_filter_t* f, float out);
int   rc_multiply_filters(rc_filter_t f1, rc_filter_t f2, rc_filter_t* f3);
int   rc_c2d_tustin(rc_filter_t* f,rc_vector_t num,rc_vector_t den,float dt,float w);
int   rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant);
int   rc_first_order_highpass(rc_filter_t* f, float dt, float time_constant);
int   rc_butterworth_lowpass(rc_filter_t* f, int order, float dt, float wc);
int   rc_butterworth_highpass(rc_filter_t* f, int order, float dt, float wc);
int   rc_moving_average(rc_filter_t* f, int samples, int dt);
int   rc_integrator(rc_filter_t *f, float dt);
int   rc_double_integrator(rc_filter_t* f, float dt);
int   rc_pid_filter(rc_filter_t* f,float kp,float ki,float kd,float Tf,float dt);



#endif //ROBOTICS_CAPE


