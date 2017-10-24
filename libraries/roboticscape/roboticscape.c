/*******************************************************************************
* robticscape.c
* 
* This is one of many c-files that are used to build libroboticscape.so
* however it contains the majority of the core components.
*******************************************************************************/

//#include "../APMrover2/Rover.h"
//#include "../../APMrover2/defines.h"
//#include <AP_HAL/AP_HAL.h>

#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h> 			// for system()
#include "roboticscape.h"
#include "rc_defs.h"
#include "gpio/rc_gpio_setup.h"
#include "mmap/rc_mmap_gpio_adc.h"	// used for fast gpio functions
#include "mmap/rc_mmap_pwmss.h"		// used for fast pwm functions
#include "other/rc_pru.h"
#include "gpio/rc_buttons.h"
#include "pwm/rc_motors.h"



#define CAPE_NAME	"RoboticsCape"
#define MAX_BUF		512

/*******************************************************************************
* Global Variables
*******************************************************************************/
// global roboticscape state
enum rc_state_t rc_state = UNINITIALIZED;



/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();
void shutdown_signal_handler(int signo);


/*******************************************************************************
* int rc_initialize()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/
int rc_initialize(){
	FILE *fd; 
	rc_bb_model_t model;

	// ensure root privaleges until we sort out udev rules
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: Robotics Cape library must be run as root\n");
		return -1;
	}

	// check if another project was using resources
	// kill that process cleanly with sigint if so
	#ifdef DEBUG
		printf("checking for existing PID_FILE\n");
	#endif
	rc_kill();

	// whitelist blue, black, and black wireless only when RC device tree is in use
	model = rc_get_bb_model();
	if(model!=BB_BLACK_RC && model!=BB_BLACK_W_RC && model!=BB_BLUE){
		// also check uEnv.txt in case using older device tree
		if(system("grep -q roboticscape /boot/uEnv.txt")!=0){
			//fprintf(stderr,"WARNING: RoboticsCape library should only be run on BB Blue, Black, and Black wireless when the 			roboticscape device tree is in use.\n");
			printf("WARNING: RoboticsCape library should only be run on BB Blue, Black, and Black wireless when the 				roboticscape device tree is in use.\n");
		//	fprintf(stderr,"If you are on a BB Black or Black Wireless, please execute \"configure_robotics_dt.sh\" and reboot to 				enable the device tree\n");
		}
	}

	// start state as Uninitialized
	rc_set_state(UNINITIALIZED);
	
	// Start Signal Handler
	#ifdef DEBUG
	printf("Initializing exit signal handler\n");
	#endif
	rc_enable_signal_handler();


	// initialize pinmux
	#ifdef DEBUG
	printf("Initializing: PINMUX\n");
	#endif
	rc_set_default_pinmux();

	// initialize gpio pins
	#ifdef DEBUG
	printf("Initializing: GPIO\n");
	#endif
	if(configure_gpio_pins()<0){
		printf("ERROR: failed to configure GPIO\n");
		return -1;
	}

	// now use mmap for fast gpio
	#ifdef DEBUG
	printf("Initializing: MMAP GPIO\n");
	#endif
	if(initialize_mmap_gpio()){
		printf("mmap_gpio_adc.c failed to initialize gpio\n");
		return -1;
	}

	// now adc
	#ifdef DEBUG
	printf("Initializing: ADC\n");
	#endif
	if(initialize_mmap_adc()){
		//fprintf(stderr,"mmap_gpio_adc.c failed to initialize adc\n");
		printf(stderr,"mmap_gpio_adc.c failed to initialize adc\n");
		return -1;
	}

	// eQep encoder counters
//	#ifdef DEBUG
	printf("Initializing: eQEP\n");
//	#endif
	// this also zero's out the encoder counters
	if(init_eqep(0)){
		printf(stderr,"WARNING: failed to initialize eQEP0\n");
	}
	if(init_eqep(1)){
		printf(stderr,"WARNING: failed to initialize eQEP1\n");
	}
	if(init_eqep(2)){
		printf(stderr,"WARNING: failed to initialize eQEP2\n");
	}

	// motors
//	#ifdef DEBUG
	printf("Initializing: Motors\n");
//	#endif
	if(initialize_motors()){
		printf(stderr,"WARNING: Failed to initialize motors\n");
	}

	//set up function pointers for button press events
	#ifdef DEBUG
	printf("Initializing: Buttons\n");
	#endif
	if(initialize_button_handlers()<0){
		//fprintf(stderr,"ERROR: failed to start button threads\n");
		printf(stderr,"ERROR: failed to start button threads\n");
		return -1;
	}
/*
	// start PRU
	#ifdef DEBUG
	printf("Initializing: PRU\n");
	#endif
	initialize_pru();
*/
	// create new pid file with process id
	#ifdef DEBUG
		printf("opening PID_FILE\n");
	#endif
	fd = fopen(PID_FILE, "ab+");
	if (fd == NULL) {
		//fprintf(stderr,"error opening PID_FILE for writing\n");
		printf(stderr,"error opening PID_FILE for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);

	// Print current PID
	#ifdef DEBUG
	//printf("Process ID: %d\n", (int)current_pid); 
	printf("Process ID: %d\n", (int)current_pid); 
 	#endif

	// wait to let threads start up
	rc_usleep(10000);

	return 0;
}

/*******************************************************************************
*	int rc_cleanup()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
*******************************************************************************/
int rc_cleanup(){
	// just in case the user forgot, set state to exiting
	rc_set_state(EXITING);

	// announce we are starting cleanup process
	printf("\nExiting Cleanly\n");
	
	#ifdef DEBUG
	printf("waiting for button handlers to join\n");
	#endif
	wait_for_button_handlers_to_join();

	#ifdef DEBUG
	printf("turning off GPIOs & PWM\n");
	#endif
	rc_set_led(GREEN,LOW);
	rc_set_led(RED,LOW);

	#ifdef DEBUG
	printf("Turning off motors\n");
	#endif
	rc_disable_motors();

	#ifdef DEBUG
	printf("Turning off SPI slaves\n");
	#endif
	rc_manual_deselect_spi_slave(1);
	rc_manual_deselect_spi_slave(2);

	#ifdef DEBUG
	printf("Turning off servo power rail\n");
	#endif
	rc_disable_servo_power_rail();
	
	#ifdef DEBUG
	printf("Stopping dsm service\n");
	#endif
	rc_stop_dsm_service();	
	
	#ifdef DEBUG
	printf("Deleting PID file\n");
	#endif
	FILE* fd;
	// clean up the pid_file if it still exists
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}
	#ifdef DEBUG
	printf("end of cleanup_cape\n");
	#endif
	return 0;
}

/*******************************************************************************
* @ rc_state_t rc_get_state()
*
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
rc_state_t rc_get_state(){
	return rc_state;
}

/*******************************************************************************
* @ int rc_set_state(rc_state_t new_state)
*
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
int rc_set_state(rc_state_t new_state){
	rc_state = new_state;
	return 0;
}

/*******************************************************************************
* @ int rc_print_state()
* 
* Prints the textual name of the state to the current state to the screen.
*******************************************************************************/
int rc_print_state(){
	switch(rc_state){
	case UNINITIALIZED:
		printf("UNINITIALIZED");
		break;
	case PAUSED:
		printf("PAUSED");
		break;
	case RUNNING:
		printf("RUNNING");
		break;
	case EXITING:
		printf("EXITING");
		break;
	default:
		fprintf(stderr,"ERROR: invalid state\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* @ int rc_set_led(rc_led_t led, int state)
* 
* turn on or off the green or red LED on robotics cape
* if state is 0, turn led off, otherwise on.
* we suggest using the names HIGH or LOW
*******************************************************************************/
int rc_set_led(rc_led_t led, int state){
	int val;
	if(state) val = HIGH;
	else val = LOW;
	
	switch(led){
	case GREEN:
		return rc_gpio_set_value_mmap(GRN_LED, val);
		break;
	case RED:
		return rc_gpio_set_value_mmap(RED_LED, val);
		break;
	default:
		fprintf(stderr,"LED must be GREEN or RED\n");
		break;
	}
	return -1;
}

/*******************************************************************************
* int rc_get_led(rc_led_t led)
* 
* returns the state of the green or red LED on robotics cape
* state is LOW(0), or HIGH(1)
*******************************************************************************/
int rc_get_led(rc_led_t led){
	switch(led){
	case GREEN:
		return rc_gpio_get_value(GRN_LED);
	case RED:
		return rc_gpio_get_value(RED_LED);
	default:
		fprintf(stderr,"LED must be GREEN or RED\n");
	}
	return -1;
}

/*******************************************************************************
* rc_blink_led(rc_led_t led, float hz, float period)
*	
* Flash an LED at a set frequency for a finite period of time.
* This is a blocking call and only returns after flashing.
*******************************************************************************/
int rc_blink_led(rc_led_t led, float hz, float period){
	const int delay_us = 1000000.0/(2.0*hz); 
	const int blinks = period*2.0*hz;
	int i;
	int toggle = 0;
	
	for(i=0;i<blinks;i++){
		toggle = !toggle;
		if(rc_get_state()==EXITING) break;
		rc_set_led(led,toggle);
		// wait for next blink
		rc_usleep(delay_us);
	}
	
	rc_set_led(led, 0); // make sure it is left off
	return 0;
}

/*******************************************************************************
* int rc_get_encoder_pos(int ch)
* 
* returns the encoder counter position
*******************************************************************************/
int rc_get_encoder_pos(int ch){
	if(ch<1 || ch>4){
		fprintf(stderr,"Encoder Channel must be from 1 to 4\n");
		return -1;
	}
	// 4th channel is counted by the PRU not eQEP
	if(ch==4) return get_pru_encoder_pos();
	// first 3 channels counted by eQEP
	return  read_eqep(ch-1);
}

/*******************************************************************************
* int rc_set_encoder_pos(int ch, int val)
* 
* sets the encoder counter position
*******************************************************************************/
int rc_set_encoder_pos(int ch, int val){
	if(ch<1 || ch>4){
		fprintf(stderr,"Encoder Channel must be from 1 to 4\n");
		return -1;
	}
	// 4th channel is counted by the PRU not eQEP
	if(ch==4) return set_pru_encoder_pos(val);
	// else write to eQEP
	return write_eqep(ch-1, val);
}

/*******************************************************************************
* float rc_battery_voltage()
* 
* returns the LiPo battery voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
float rc_battery_voltage(){
	float v = (rc_adc_volt(LIPO_ADC_CH)*V_DIV_RATIO)+LIPO_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}

/*******************************************************************************
* float rc_dc_jack_voltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
float rc_dc_jack_voltage(){
	float v = (rc_adc_volt(DC_JACK_ADC_CH)*V_DIV_RATIO)+DC_JACK_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}

/*******************************************************************************
* int rc_adc_raw(int ch)
*
* returns the raw adc reading
*******************************************************************************/
int rc_adc_raw(int ch){
	if(ch<0 || ch>6){
		fprintf(stderr,"ERROR: analog pin must be in 0-6\n");
		return -1;
	}
	return mmap_adc_read_raw((uint8_t)ch);
}

/*******************************************************************************
* float rc_adc_volt(int ch)
* 
* returns an actual voltage for an adc channel
*******************************************************************************/
float rc_adc_volt(int ch){
	if(ch<0 || ch>6){
		fprintf(stderr,"ERROR: analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = mmap_adc_read_raw((uint8_t)ch);
	return raw_adc * 1.8 / 4095.0;
}

/*******************************************************************************
* int rc_enable_servo_power_rail()
* 
* Turns on the 6V power regulator to the servo power rail.
*******************************************************************************/
int rc_enable_servo_power_rail(){
	return rc_gpio_set_value_mmap(SERVO_PWR, HIGH);
}

/*******************************************************************************
* int rc_disable_servo_power_rail()
* 
* Turns off the 6V power regulator to the servo power rail.
*******************************************************************************/
int rc_disable_servo_power_rail(){
	return rc_gpio_set_value_mmap(SERVO_PWR, LOW);
}

/*******************************************************************************
* shutdown_signal_handler(int signo)
*
* catch Ctrl-C signal and change system state to EXITING
* all threads should watch for rc_get_state()==EXITING and shut down cleanly
*******************************************************************************/
void shutdown_signal_handler(int signo){
	switch(signo){
	case SIGINT: // normal ctrl-c shutdown interrupt
		rc_set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
		break;
	case SIGTERM: // catchable terminate signal
		rc_set_state(EXITING);
		printf("\nreceived SIGTERM\n");
		break;
	case SIGHUP: // terminal closed or disconnected, carry on anyway
		break;
	default:
		break;
	}
	return;
}

/*******************************************************************************
* @ int rc_kill()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the rc_kill example program from the command line to close whatever
* program is running in the background.
*
* return values: 
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*******************************************************************************/
int rc_kill(){
	FILE* fd;
	int old_pid, i;
	// start by checking if a pid file exists
	if(access(PID_FILE, F_OK ) != 0){
		// PID file missing
		return 0;
	}
	// attempt to open PID file
	// if the file didn't open, no project is runnning in the background
	// so return 0
	fd = fopen(PID_FILE, "r");
	if(fd==NULL) return 0;
	// try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(PID_FILE);
		return -2;
	}
	// check if it's our own pid, if so return 0
	if(old_pid == (int)getpid()) return 0;
	// now see if the process for the read pid is still running
	if(getpgid(old_pid) < 0){
		// process not running, remove the pid file
		remove(PID_FILE);
		return 0;
	}
	// process must be running, attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<30; i++){
		if(getpgid(old_pid) >= 0) rc_usleep(100000);
		else{ // succcess, it shut down properly
			remove(PID_FILE);
			return 1; 
		}
	}
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);
	rc_usleep(500000);
	// delete the old PID file if it was left over
	remove(PID_FILE);
	// return -1 indicating the program had to be killed
	return -1;
}

/*******************************************************************************
* @ void rc_disable_signal_handler(
*
* Disables the built-in signal handler. Use only if you want to implement your
* own signal handler. Make sure your handler sets rc_state to EXITING or calls
* cleanup_cape on shutdown to ensure roboticscape library threads close
* cleanly.
*******************************************************************************/
void rc_disable_signal_handler(){
	signal(SIGINT, SIG_DFL);
	signal(SIGKILL, SIG_DFL);
	signal(SIGHUP, SIG_DFL);
	return;
}

/*******************************************************************************
* @ void rc_enable_signal_handler(
*
* enables the built-in signal handler if it was disabled before. The built-in 
* signal handler is enabled in rc_initialize()
*******************************************************************************/
void rc_enable_signal_handler(){
	signal(SIGINT, shutdown_signal_handler);
	signal(SIGTERM, shutdown_signal_handler);
	signal(SIGHUP, shutdown_signal_handler);
	return;
}

