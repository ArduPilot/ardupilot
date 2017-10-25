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

EXTERNC int rc_initialize();	// call at the beginning of main()
EXTERNC int rc_cleanup();		// call at the end of main()
EXTERNC int rc_kill();	// not usually necessary, use kill_robot example instead
EXTERNC void rc_enable_signal_handler();


#define HIGH 1
#define LOW  0


// GPIO
EXTERNC int initialize_mmap_gpio();

#define MMAP_OFFSET (0x44C00000)
#define MMAP_SIZE   (0x481AEFFF-MMAP_OFFSET)




/* GPIO Memory Registers */
#define GPIO_REGISTER_SIZE (4)

#define GPIO0   (0x44E07000)
#define GPIO1   (0x4804C000)
#define GPIO2   (0x481AC000)
#define GPIO3   (0x481AE000)

#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT   (0x194)
#define GPIO_OE           (0x134)
#define GPIO_DATAOUT      (0x13C)
#define GPIO_DATAIN       (0x138)

#define TRUE 1
#define FALSE 0


#define INPUT    ((unsigned char)(1))
#define OUTPUT   ((unsigned char)(0))
#define PULLUP   ((unsigned char)(1))
#define PULLDOWN ((unsigned char)(0))
#define PULL_DISABLED ((unsigned char)(2))

/*********************************
* clock control registers
*********************************/
#ifndef CM_PER
  #define CM_PER 0x44E00000 //base of Clock Module Peripheral control
  #define CM_PER_PAGE_SIZE 1024 //1kb
#endif


#define CM_PER_GPIO1_CLKCTRL 0xAC
#define CM_PER_GPIO2_CLKCTRL 0xB0
#define CM_PER_GPIO3_CLKCTRL 0xB4

/* Clock Module Memory Registers */
#define CM_WKUP 0x44E00400

#define MODULEMODE_DISABLED 0x0
#define MODULEMODE_ENABLE   0x2

#define CM_WKUP_ADC_TSC_CLKCTRL 0xBC




typedef enum rc_state_t {
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
} rc_state_t;

rc_state_t rc_get_state();
EXTERNC int rc_set_state(rc_state_t new_state);
int rc_print_state();


EXTERNC int rc_enable_motors();
EXTERNC int rc_disable_motors();
EXTERNC int rc_set_motor(int motor, float duty);
EXTERNC int rc_set_motor_all(float duty);
EXTERNC int rc_set_motor_free_spin(int motor);
EXTERNC int rc_set_motor_free_spin_all();
EXTERNC int rc_set_motor_brake(int motor);
EXTERNC int rc_set_motor_brake_all();



#define PID_FILE "/var/run/robotics_cape.pid"


#define MOTOR_CHANNELS  4
#define DEFAULT_PWM_FREQ 25000

#define IMU_INTERRUPT_PIN 117  //gpio3.21 P9.25

//// gpio output pins
#define MDIR1A_BLUE 64  //gpio2.0 pin T13
#define MDIR1B      31  //gpio0.31  P9.13
#define MDIR2A      48  //gpio1.16  P9.15
#define MDIR2B_BLUE 10  //gpio0.10  P8_31
#define MDIR4A      70  //gpio2.6   P8.45
#define MDIR4B      71  //gpio2.7   P8.46
#define MDIR3B      72  //gpio2.8   P8.43
#define MDIR3A      73  //gpio2.9   P8.44
#define MOT_STBY    20  //gpio0.20  P9.41

//// BB Blue GPIO OUT
#define BLUE_GP0_PIN_4 49 // gpio 1_17 pin P9.23

// Battery Indicator LEDs
#define BATT_LED_1  27 // P8.17
#define BATT_LED_2  65 // P8.18
#define BATT_LED_2_BLUE 11 // different on BB Blue
#define BATT_LED_3  61 // P8.26
#define BATT_LED_4  26 // P8.14

#define POLL_TIMEOUT 100 /* 0.1 seconds */
#define INTERRUPT_PIN 117  //gpio3.21 P9.25



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

/*******************************************************************************
* GPIO
*******************************************************************************/
#define HIGH 1
#define LOW 0

typedef enum rc_pin_direction_t{
	INPUT_PIN,
	OUTPUT_PIN
}rc_pin_direction_t;


int rc_pwm_init(int ss, int frequency);
int rc_pwm_close(int ss);
int rc_pwm_set_duty(int ss, char ch, float duty);
int rc_pwm_set_duty_ns(int ss, char ch, int duty_ns);
EXTERNC int rc_pwm_set_duty_mmap(int ss, char ch, float duty);

void rc_nanosleep(uint64_t ns);
EXTERNC void rc_usleep(unsigned int us);


EXTERNC int initialize_motors();
EXTERNC int configure_gpio_pins();


#endif //ROBOTICS_CAPE


