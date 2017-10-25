
#include <signal.h>
#include "roboticscape.h"
#include "mmap/rc_mmap_pwmss.h"		// used for fast pwm functions
//#include "rc_motors.h"
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include "rc_pwm_userspace_defs.h"
#include "preprocessor_macros.h"
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


// global variables
int mdir1a, mdir2b; // variable gpio pin assignments
int motors_initialized = 0;
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64
#define MAXBUF 64

// variables
int duty_fd[6];   // pointers to duty cycle file descriptor
int period_ns[3];   //one period (frequency) per subsystem
char simple_pwm_initialized[3] = {0,0,0};
int ver; // pwm driver version, 0 or 1. automatically detected

/*******************************************************************************
* Global Variables
*******************************************************************************/
// global roboticscape state
enum rc_state_t rc_state = UNINITIALIZED;


volatile uint32_t *map; // pointer to /dev/mem
int mapped = 0; // boolean to check if mem mapped
int gpio_initialized = 0;
int adc_initialized = 0;

/*******************************************************************************
*   Shared Map Function
*******************************************************************************/
// map /dev/mem if it hasn't been done already
int init_mmap() {
  if(mapped){
    return 0;
  }
  int fd = open("/dev/mem", O_RDWR);
  errno=0;
  if(unlikely(fd==-1)){
    printf("Unable to open /dev/mem\n");
    if(unlikely(errno==EPERM)) printf("Insufficient privileges\n");
    return -1;
  }
  map = (uint32_t*)mmap(NULL, MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,\
                              fd, MMAP_OFFSET);
  if(map == MAP_FAILED) {
    close(fd);
    printf("Unable to map /dev/mem\n");
    return -1;
  }
  mapped = TRUE;
  return 0;
}

int initialize_mmap_gpio(){
  // return immediately if gpio already initialized
  if(gpio_initialized){
    return 0;
  }
  // check the mmap poitner is sorted
  if (init_mmap()){
    return -1;
  }

  // now we must enable clock signal to the gpio subsystems
  // first open the clock register to see if the PWMSS has clock enabled
  int dev_mem;
  dev_mem = open("/dev/mem", O_RDWR);
  if(dev_mem == -1) {
    printf("Unable to open /dev/mem\n");
    return -1;
  }
  volatile char *cm_per_base;
  cm_per_base=mmap(0,CM_PER_PAGE_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,\
                                dev_mem,CM_PER);
  if(cm_per_base == (void *) -1) {
    printf("Unable to mmap cm_per\n");
    return -1;
  }

  #ifdef DEBUG
  printf("turning on cm-per module_enable bits for gpio 1,2,3\n");
  #endif
  *(uint16_t*)(cm_per_base + CM_PER_GPIO1_CLKCTRL) |= MODULEMODE_ENABLE;
  *(uint16_t*)(cm_per_base + CM_PER_GPIO2_CLKCTRL) |= MODULEMODE_ENABLE;
  *(uint16_t*)(cm_per_base + CM_PER_GPIO3_CLKCTRL) |= MODULEMODE_ENABLE;

  #ifdef DEBUG
  printf("new cm_per_gpio3_clkctrl: %d\n", *(uint16_t*)(cm_per_base+CM_PER_GPIO3_CLKCTRL));
  #endif
  close(dev_mem);
  int fd, len;
  char buf[32];

  fd = open("/sys/class/gpio/export", O_WRONLY);
  if (fd == -1) {
    printf("/sys/class/gpio/export doesn't exist\n");
    return -1;
  }
  len = snprintf(buf, sizeof(buf), "%d", 113);
  write(fd, buf, len);
  close(fd);

  gpio_initialized=1;
  return 0;
}

// write HIGH or LOW to a pin
// pinMUX must already be configured for output
int rc_gpio_set_value_mmap(int pin, int state) {
  if(initialize_mmap_gpio()){
    return -1;
  }
  if(pin<0 || pin>128){
    printf("invalid gpio pin\n");
    return -1;
  }
  int bank = pin/32;
  int id = pin - bank*32;
  int bank_offset;
  switch(bank){
    case 0:
      bank_offset=GPIO0;
      break;
    case 1:
      bank_offset=GPIO1;
      break;
    case 2:
      bank_offset=GPIO2;
      break;
    case 3:
      bank_offset=GPIO3;
      break;
    default:
      return -1;
  }
  //map[(bank_offset-MMAP_OFFSET+GPIO_OE)/4] &= ~(1<<p.bank_id);
  if(state) map[(bank_offset-MMAP_OFFSET+GPIO_DATAOUT)/4] |= (1<<id);
  else map[(bank_offset-MMAP_OFFSET+GPIO_DATAOUT)/4] &= ~(1<<id);
  return 0;
}

// returns 1 or 0 for HIGH or LOW
// pinMUX must already be configured for input
int rc_gpio_get_value_mmap(int pin) {
  if(initialize_mmap_gpio()){
    return -1;
  }
  if(pin<0 || pin>128){
    printf("invalid gpio pin\n");
    return -1;
  }
  int bank = pin/32;
  int id = pin - bank*32;
  int bank_offset;
  switch(bank){
    case 0:
      bank_offset=GPIO0;
      break;
    case 1:
      bank_offset=GPIO1;
      break;
    case 2:
      bank_offset=GPIO2;
      break;
    case 3:
      bank_offset=GPIO3;
      break;
    default:
      return -1;
  }
  return (map[(bank_offset-MMAP_OFFSET+GPIO_DATAIN)/4] & (1<<id))>>id;
}



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
* int rc_get_encoder_pos(int ch)
* 
* returns the encoder counter position
*******************************************************************************/
int rc_get_encoder_pos(int ch){
	if(ch<1 || ch>4){
		fprintf(stderr,"Encoder Channel must be from 1 to 4\n");
		return -1;
	}
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
	return write_eqep(ch-1, val);
}


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
* int initialize_motors()
*
* set up gpio assignments, pwm channels, and make sure motors are left off.
* GPIO exporting must be done previously with simple_init_gpio()
*******************************************************************************/
int rc_pwm_init(int ss, int frequency){
  int export_fd;
  int periodA_fd; // pointers to frequency file descriptor
  int periodB_fd;
  int enableA_fd;  // run (enable) file pointers
  int enableB_fd;
  int polarityA_fd;
  int polarityB_fd;
  char buf[MAXBUF];
  int len;

  if(ss<0 || ss>2){
    printf("PWM subsystem must be between 0 and 2\n");
    return -1;
  }

  // check driver is loaded
  if(access(pwm_export_path[0][ss], F_OK ) == 0) ver=0;
  else if(access(pwm_export_path[1][ss], F_OK ) == 0) ver=1;
  else{
    fprintf(stderr,"ERROR: ti-pwm driver not loaded for pwm subsystem %d\n", ss);
    return -1;
  }

  // open export file for that subsystem
  export_fd = open(pwm_export_path[ver][ss], O_WRONLY);
  if(unlikely(export_fd<0)){
    fprintf(stderr,"ERROR in rc_pwm_init, can't open pwm export file for writing\n");
    return -1;
  }

  // export just the A channel for that subsystem and check that it worked
  write(export_fd, "0", 1);
  if(unlikely(access(pwm_chA_enable_path[ver][ss], F_OK ) != 0)){
    fprintf(stderr,"ERROR: export failed for hrpwm%d channel A\n", ss);
    return -1;
  }

  // set up file descriptors for A channel
  enableA_fd = open(pwm_chA_enable_path[ver][ss], O_WRONLY);
  periodA_fd = open(pwm_chA_period_path[ver][ss], O_WRONLY);
  duty_fd[(2*ss)] = open(pwm_chA_duty_path[ver][ss], O_WRONLY);
  polarityA_fd = open(pwm_chA_polarity_path[ver][ss], O_WRONLY);


  // disable A channel and set polarity before period
  write(enableA_fd, "0", 1);
  write(duty_fd[(2*ss)], "0", 1); // set duty cycle to 0
  write(polarityA_fd, "0", 1); // set the polarity

  // set the period in nanoseconds
  period_ns[ss] = 1000000000/frequency;
  len = snprintf(buf, sizeof(buf), "%d", period_ns[ss]);
  write(periodA_fd, buf, len);


  // now we can set up the 'B' channel since the period has been set
  // the driver will not let you change the period when both are exported

  // export the B channel and check that it worked
  write(export_fd, "1", 1);
  if(unlikely(access(pwm_chB_enable_path[ver][ss], F_OK )!=0)){
    fprintf(stderr,"ERROR: export failed for hrpwm%d channel B\n", ss);
    return -1;
  }

  // set up file descriptors for B channel
  enableB_fd = open(pwm_chB_enable_path[ver][ss], O_WRONLY);
  periodB_fd = open(pwm_chB_period_path[ver][ss], O_WRONLY);
  duty_fd[(2*ss)+1] = open(pwm_chB_duty_path[ver][ss], O_WRONLY);
  polarityB_fd = open(pwm_chB_polarity_path[ver][ss], O_WRONLY);

  // disable and set polarity before period
  write(enableB_fd, "0", 1);
  write(polarityB_fd, "0", 1);
  write(duty_fd[(2*ss)+1], "0", 1);

  // set the period to match the A channel
  len = snprintf(buf, sizeof(buf), "%d", period_ns[ss]);
  write(periodB_fd, buf, len);

  // enable A&B channels
  write(enableA_fd, "1", 1);
  write(enableB_fd, "1", 1);

  // close all the files
  close(export_fd);
  close(enableA_fd);
  close(enableB_fd);
  close(periodA_fd);
  close(periodB_fd);
  close(polarityA_fd);
  close(polarityB_fd);

  // everything successful
  simple_pwm_initialized[ss] = 1;
  return 0;
}

/*******************************************************************************
* int rc_pwm_close(int ss){
*
* Unexports a subsystem to put it into low-power state. Not necessary for the
* the user to call during normal program operation. This is mostly for internal
* use and cleanup.
*******************************************************************************/
int rc_pwm_close(int ss){
  int fd;

  // sanity check
  if(unlikely(ss<0 || ss>2)){
    fprintf(stderr,"ERROR in rc_pwm_close, subsystem must be between 0 and 2\n");
    return -1;
  }

  // attempt both driver versions
  if(access(pwm_unexport_path[0][ss], F_OK ) == 0) ver=0;
  else if(access(pwm_unexport_path[1][ss], F_OK ) == 0) ver=1;
  else{
    fprintf(stderr,"ERROR in rc_pwm_close: ti-pwm driver not loaded for pwm subsystem %d\n", ss);
    return -1;
  }

  // open the unexport file for that subsystem
  fd = open(pwm_unexport_path[ver][ss], O_WRONLY);
  if(unlikely(fd < 0)){
    fprintf(stderr,"ERROR in rc_pwm_close: can't open pwm unexport file for hrpwm%d\n", ss);
    return -1;
  }

  // write 0 and 1 to the file to unexport both channels
  write(fd, "0", 1);
  write(fd, "1", 1);

  close(fd);
  simple_pwm_initialized[ss] = 0;
  return 0;

}


/****************************************************************
 * rc_gpio_export
 ****************************************************************/
int rc_gpio_export(unsigned int gpio){
  int fd, len;
  char buf[MAX_BUF];

  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0) {
    perror("gpio/export");
    return fd;
  }
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);

  return 0;
}

/****************************************************************
 * rc_gpio_unexport
 ****************************************************************/
int rc_gpio_unexport(unsigned int gpio){
  int fd, len;
  char buf[MAX_BUF];

  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0) {
    perror("gpio/export");
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  return 0;
}

/****************************************************************
 * rc_gpio_set_dir
 ****************************************************************/
int rc_gpio_set_dir(int gpio, rc_pin_direction_t out_flag){
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%i/direction", gpio);
  fd = open(buf, O_WRONLY);
  //printf("%d\n", gpio);
  if (fd < 0) {
    perror("gpio/direction");
    return fd;
  }

  if (out_flag == OUTPUT_PIN)
    write(fd, "out", 4);
  else
    write(fd, "in", 3);

  close(fd);
  return 0;
}

/****************************************************************
 * rc_gpio_set_value
 ****************************************************************/
int rc_gpio_set_value(unsigned int gpio, int value){
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    perror("gpio/set-value");
    return fd;
  }

  if(value)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);

  close(fd);
  return 0;
}

/****************************************************************
 * rc_gpio_get_value
 ****************************************************************/
int rc_gpio_get_value(unsigned int gpio){
  int fd, ret;
  char buf[MAX_BUF];
  char ch;

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

  fd = open(buf, O_RDONLY);
  if (fd < 0) {
    perror("gpio/get-value");
    return fd;
  }

  read(fd, &ch, 1);

  if (ch != '0') ret = 1;
  else ret = 0;

  close(fd);
  return ret;
}
/****************************************************************
 * rc_gpio_fd_open
 ****************************************************************/

int rc_gpio_fd_open(unsigned int gpio)
{
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

  fd = open(buf, O_RDONLY | O_NONBLOCK );
  if (fd < 0) {
    perror("gpio/fd_open");
  }
  return fd;
}

/****************************************************************
 * rc_gpio_fd_close
 ****************************************************************/

int rc_gpio_fd_close(int fd){
  return close(fd);
}


int setup_output_pin(int pin, int val){

  if(rc_gpio_export(pin)){
    fprintf(stderr,"ERROR: Failed to export gpio pin %d\n", pin);
    return -1;
  }
  if(rc_gpio_set_dir(pin, OUTPUT_PIN)){
    fprintf(stderr,"ERROR: Failed to set gpio pin %d as output\n", pin);
    return -1;
  }
  if(rc_gpio_set_value(pin, val)){
    fprintf(stderr, "ERROR: Failed to set gpio pin %d value\n", pin);
    return -1;
  }
  return 0;
}

int setup_input_pin(int pin){

  if(rc_gpio_export(pin)){
    fprintf(stderr,"ERROR: Failed to export gpio pin %d\n", pin);
    return -1;
  }
  if(rc_gpio_set_dir(pin, INPUT_PIN)){
    fprintf(stderr,"ERROR: Failed to set gpio pin %d as output\n", pin);
    return -1;
  }
  return 0;
}


int configure_gpio_pins(){
  int mdir1a, mdir2b;
  int ret = 0;

    mdir1a = MDIR1A_BLUE;
    mdir2b = MDIR2B_BLUE;

  int old_stderr;
  FILE  *null_out;
  // change stdout to null for this operation as the prussdrv.so
  // functions are noisy
  old_stderr = dup(STDERR_FILENO);
  fflush(stderr);
  null_out = fopen("/dev/null", "w");
  dup2(fileno(null_out), STDERR_FILENO);

  // put back stdout
  fflush(stderr);
  fclose(null_out);
  dup2(old_stderr,STDERR_FILENO);
  close(old_stderr);


  // MOTOR Direction and Standby pins
  ret |= setup_output_pin(mdir1a, LOW);
  ret |= setup_output_pin(MDIR1B, LOW);
  ret |= setup_output_pin(MDIR2A, LOW);
  ret |= setup_output_pin(mdir2b, LOW);
  ret |= setup_output_pin(MDIR3A, LOW);
  ret |= setup_output_pin(MDIR3B, LOW);
  ret |= setup_output_pin(MDIR4A, LOW);
  ret |= setup_output_pin(MDIR4B, LOW);
  ret |= setup_output_pin(MOT_STBY, LOW);

  // IMU
  ret |= setup_input_pin(IMU_INTERRUPT_PIN);

  if(ret){
    printf("WARNING: Failed to configure all gpio pins\n");
    return -1;
  }

  return 0;
}




/*******************************************************************************
* int rc_pwm_set_duty(int ss, char ch, float duty)
*
* Updates the duty cycle through the file system userspace driver. subsystem ss
* must be 0,1,or 2 and channel 'ch' must be A or B. Duty cycle must be bounded
* between 0.0f (off) and 1.0f(full on). Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_pwm_set_duty(int ss, char ch, float duty){
  // start with sanity checks
  if(unlikely(duty>1.0f || duty<0.0f)){
    fprintf(stderr,"ERROR in rc_pwm_set_duty: duty must be between 0.0 & 1.0\n");
    return -1;
  }

  // set the duty
  int duty_ns = duty*period_ns[ss];
  return rc_pwm_set_duty_ns(ss, ch, duty_ns);
}

/*******************************************************************************
* int rc_pwm_set_duty_ns(int ss, char ch, int duty_ns)
*
* like rc_pwm_set_duty() but takes a pulse width in nanoseconds which must range
* from 0 (off) to the number of nanoseconds in a single cycle as determined
* by the freqency specified when calling rc_pwm_init(). The default PWM
* frequency of the motors is 25kz corresponding to a maximum pulse width of
* 40,000ns. However, this function will likely only be used by the user if they
* have set a custom PWM frequency for a more specific purpose. Returns 0 on
* success or -1 on failure.
*******************************************************************************/
int rc_pwm_set_duty_ns(int ss, char ch, int duty_ns){
  int len;
  char buf[MAXBUF];
  // start with sanity checks
  if(unlikely(ss<0 || ss>2)){
    fprintf(stderr,"ERROR in rc_pwm_set_duty_ns, PWM subsystem must be between 0 and 2\n");
    return -1;
  }
  // initialize subsystem if not already
  if(simple_pwm_initialized[ss]==0){
    printf("initializing PWMSS%d with default PWM frequency %dhz\n", ss, DEFAULT_PWM_FREQ);
    rc_pwm_init(ss, DEFAULT_PWM_FREQ);
  }
  // boundary check
  if(unlikely(duty_ns>period_ns[ss] || duty_ns<0)){
    fprintf(stderr,"ERROR in rc_pwm_set_duty_ns, duty must be between 0 & %d for current frequency\n", period_ns[ss]);
    return -1;
  }

  // set the duty
  len = snprintf(buf, sizeof(buf), "%d", duty_ns);
  switch(ch){
  case 'A':
    write(duty_fd[(2*ss)], buf, len);
    break;
  case 'B':
    write(duty_fd[(2*ss)+1], buf, len);
    break;
  default:
    printf("pwm channel must be 'A' or 'B'\n");
    return -1;
  }

  return 0;

}
int initialize_motors(){

    mdir1a = MDIR1A_BLUE;
    mdir2b = MDIR2B_BLUE;

  #ifdef DEBUG
  printf("Initializing: PWM\n");
  #endif
  if(rc_pwm_init(1,DEFAULT_PWM_FREQ)){
    printf("ERROR: failed to initialize hrpwm1\n");
    return -1;
  }
  if(rc_pwm_init(2,DEFAULT_PWM_FREQ)){
    printf("ERROR: failed to initialize PWMSS 2\n");
    return -1;
  }

  motors_initialized = 1;
  rc_disable_motors();
  return 0;
}


/*******************************************************************************
* rc_enable_motors()
*
* turns on the standby pin to enable the h-bridge ICs
* returns 0 on success
*******************************************************************************/
int rc_enable_motors(){
  if(motors_initialized==0){
    printf("ERROR: trying to enable motors before they have been initialized\n");
    return -1;
  }
  rc_set_motor_free_spin_all();
  return rc_gpio_set_value_mmap(MOT_STBY, HIGH);
}

/*******************************************************************************
* int rc_disable_motors()
*
* turns off the standby pin to disable the h-bridge ICs
* and disables PWM output signals, returns 0 on success
*******************************************************************************/
int rc_disable_motors(){
  if(motors_initialized==0){
    printf("ERROR: trying to disable motors before they have been initialized\n");
    return -1;
  }
  rc_gpio_set_value_mmap(MOT_STBY, LOW);
  rc_set_motor_free_spin_all();
  return 0;
}

/*******************************************************************************
* int rc_set_motor(int motor, float duty)
*
* set a motor direction and power
* motor is from 1 to 4, duty is from -1.0 to +1.0
*******************************************************************************/
int rc_set_motor(int motor, float duty){
  uint8_t a,b;
  if(motors_initialized==0){
    printf("ERROR: trying to rc_set_motor before they have been initialized\n");
    return -1;
  }
  //check that the duty cycle is within +-1
  if (duty>1.0){
    duty = 1.0;
  }
  else if(duty<-1.0){
    duty=-1.0;
  }
  //switch the direction pins to H-bridge
  if (duty>=0){
    a=HIGH;
    b=LOW;
  }
  else{
    a=LOW;
    b=HIGH;
    duty=-duty;
  }

  // set gpio direction outputs & duty
  switch(motor){
    case 1:
      rc_gpio_set_value_mmap(mdir1a, a);
      rc_gpio_set_value_mmap(MDIR1B, b);
      rc_pwm_set_duty_mmap(1, 'A', duty);
      break;
    case 2:
      rc_gpio_set_value_mmap(MDIR2A, b);
      rc_gpio_set_value_mmap(mdir2b, a);
      rc_pwm_set_duty_mmap(1, 'B', duty);
      break;
    case 3:
      rc_gpio_set_value_mmap(MDIR3A, b);
      rc_gpio_set_value_mmap(MDIR3B, a);
      rc_pwm_set_duty_mmap(2, 'A', duty);
      break;
    case 4:
      rc_gpio_set_value_mmap(MDIR4A, a);
      rc_gpio_set_value_mmap(MDIR4B, b);
      rc_pwm_set_duty_mmap(2, 'B', duty);
      break;
    default:
      printf("enter a motor value between 1 and 4\n");
      return -1;
  }
  return 0;
}

/*******************************************************************************
* int rc_set_motor_all(float duty)
*
* applies the same duty cycle argument to all 4 motors
*******************************************************************************/
int rc_set_motor_all(float duty){
  int i;
  if(motors_initialized==0){
    printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
    return -1;
  }
  for(i=1;i<=MOTOR_CHANNELS; i++){
    rc_set_motor(i, duty);
  }
  return 0;
}

/*******************************************************************************
* int rc_set_motor_free_spin(int motor)
*
* This puts one or all motor outputs in high-impedance state which lets the
* motor spin freely as if it wasn't connected to anything.
*******************************************************************************/
int rc_set_motor_free_spin(int motor){
  if(motors_initialized==0){
    printf("ERROR: trying to rc_set_motor_free_spin before they have been initialized\n");
    return -1;
  }
  // set gpio direction outputs & duty
  switch(motor){
    case 1:
      rc_gpio_set_value_mmap(mdir1a, 0);
      rc_gpio_set_value_mmap(MDIR1B, 0);
      rc_pwm_set_duty_mmap(1, 'A', 0.0);
      break;
    case 2:
      rc_gpio_set_value_mmap(MDIR2A, 0);
      rc_gpio_set_value_mmap(mdir2b, 0);
      rc_pwm_set_duty_mmap(1, 'B', 0.0);
      break;
    case 3:
      rc_gpio_set_value_mmap(MDIR3A, 0);
      rc_gpio_set_value_mmap(MDIR3B, 0);
      rc_pwm_set_duty_mmap(2, 'A', 0.0);
      break;
    case 4:
      rc_gpio_set_value_mmap(MDIR4A, 0);
      rc_gpio_set_value_mmap(MDIR4B, 0);
      rc_pwm_set_duty_mmap(2, 'B', 0.0);
      break;
    default:
      printf("enter a motor value between 1 and 4\n");
      return -1;
  }
  return 0;
}

/*******************************************************************************
* @ int rc_set_motor_free_spin_all()
*******************************************************************************/
int rc_set_motor_free_spin_all(){
  int i;
  if(motors_initialized==0){
    printf("ERROR: trying to rc_set_motor_free_spin_all before they have been initialized\n");
    return -1;
  }
  for(i=1;i<=MOTOR_CHANNELS; i++){
    rc_set_motor_free_spin(i);
  }
  return 0;
}

/*******************************************************************************
* @ void rc_nanosleep(uint64_t ns)
*
* A wrapper for the normal UNIX nanosleep function which takes a number of
* nanoseconds instead of a timeval struct. This also handles restarting
* nanosleep with the remaining time in the event that nanosleep is interrupted
* by a signal. There is no upper limit on the time requested.
*******************************************************************************/
void rc_nanosleep(uint64_t ns){
  struct timespec req,rem;
  req.tv_sec = ns/1000000000;
  req.tv_nsec = ns%1000000000;
  // loop untill nanosleep sets an error or finishes successfully
  errno=0; // reset errno to avoid false detection
  while(nanosleep(&req, &rem) && errno==EINTR){
    req.tv_sec = rem.tv_sec;
    req.tv_nsec = rem.tv_nsec;
  }
  return;
}

/*******************************************************************************
* @ void rc_usleep(uint64_t ns)
*
* The traditional usleep function, however common, is deprecated in linux as it
* uses SIGALARM which interferes with alarm and timer functions. This uses the
* new POSIX standard nanosleep to accomplish the same thing which further
* supports sleeping for lengths longer than 1 second. This also handles
* restarting nanosleep with the remaining time in the event that nanosleep is
* interrupted by a signal. There is no upper limit on the time requested.
*******************************************************************************/
void rc_usleep(unsigned int us){
  rc_nanosleep(us*1000);
  return;
}

