/*******************************************************************************
* rc_pwm.c
*
* These functions provide a general interface to all 3 PWM subsystems, each of
* which have two available channels A and B. PWM subsystems 1 and 2 are used for
* controlling the 4 motors on the Robotics Cape, however they may be controlled
* by the user directly instead of using the motor API. PWM subsystem 0 channels
* A and B can be accessed on the UART1 header if set up with the Pinmux API to 
* do so. The user may have exclusive use of that subsystem.
*******************************************************************************/
#include "roboticscape/roboticscape.h"
#include "rc_pwm_userspace_defs.h"
#include "roboticscape/preprocessor_macros.h"
#include "roboticscape/rc_defs.h"
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>


#define MAXBUF 64

// variables
int duty_fd[6]; 	// pointers to duty cycle file descriptor
int period_ns[3]; 	//one period (frequency) per subsystem
char simple_pwm_initialized[3] = {0,0,0};
int ver; // pwm driver version, 0 or 1. automatically detected

/*******************************************************************************
* int rc_pwm_init(int ss, int frequency)
*
* Configures subsystem 0, 1, or 2 to operate at a particular frequency. This may
* be called at runtime to change the pwm frequency without stopping the motors
* or pwm signal. Returns 0 on success or -1 on failure.
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

