/*******************************************************************************
* rc_mmap_pwmss.c
*******************************************************************************/

#include "rc_mmap_pwmss.h"
#include "rc_tipwmss.h"
#include "roboticscape/preprocessor_macros.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>

volatile char *cm_per_base;
int cm_per_mapped=0;
volatile char *pwm_base[3]; // pwm subsystem pointers for eQEP
int pwmss_mapped[3] = {0,0,0}; // to record which subsystems have been mapped
int eqep_initialized[3] = {0,0,0};
int pwm_initialized[3] = {0,0,0};

/********************************************
*  PWMSS Mapping
*********************************************/
// maps the base of each PWM subsystem into an array
// this is used by eQEP and PWM
// returns immediately if this has already been done 
int map_pwmss(int ss){
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	//return 0 if it's already been mapped.
	if(pwmss_mapped[ss]){
		return 0;
	}
	
	//open /dev/mem file pointer for mmap
	#ifdef DEBUG
		printf("opening /dev/mem\n");
	#endif
	int dev_mem = open("/dev/mem", O_RDWR | O_SYNC);
	errno=0;
	if (unlikely(dev_mem ==-1)){
	  perror("in map_pwmss(),Could not open /dev/mem");
	  return -1;
	}
	
	// first open the clock register to see if the PWMSS has clock enabled
	if(!cm_per_mapped){
		#ifdef DEBUG
		printf("mapping CM_PER\n");
		#endif
		cm_per_base=mmap(0,CM_PER_PAGE_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,CM_PER);
		if(cm_per_base == (void *) -1) {
			printf("Unable to mmap cm_per\n");
			return -1;
		}
		cm_per_mapped = 1;
	}
	
	// if this subsystem hasn't already been mapped, 
	// then we probably need to enable clock signal to it in cm_per
	uint32_t cm_per_clkctrl;
	switch(ss){
	case 0:
		cm_per_clkctrl = CM_PER_EPWMSS0_CLKCTRL;
		break;
	case 1:
		cm_per_clkctrl = CM_PER_EPWMSS1_CLKCTRL;
		break;
	case 2:
		cm_per_clkctrl = CM_PER_EPWMSS2_CLKCTRL;
		break;
	default:
		return -1;
	}
	
	*(uint16_t*)(cm_per_base + cm_per_clkctrl) |= MODULEMODE_ENABLE;
	#ifdef DEBUG
	printf("new clkctrl%d: %d\n", ss, *(uint16_t*)(cm_per_base + cm_per_clkctrl));
	#endif

	
	// now map the appropriate subsystem base address
	#ifdef DEBUG
		printf("calling mmap() for base %d\n", ss);
	#endif
	switch(ss){
	case 0:
		pwm_base[0] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS0_BASE);
		break;
	case 1:
		pwm_base[1] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS1_BASE);
		break;
	case 2:
		pwm_base[2] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS2_BASE);
		break;
	default:
		printf("invalid ss\n");
		return -1;
	}
	#ifdef DEBUG
		printf("finished mapping for base %d\n", ss);
	#endif
	
	if(pwm_base[ss] == (void *) -1) {
		printf("Unable to mmap pwm \n");
		return -1;
	}
	pwmss_mapped[ss]=1;
	
	// enable clock from PWMSS
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= 0x010;
	
	close(dev_mem);
	#ifdef DEBUG
		printf("closed /dev/mem\n");
	#endif
	return 0;
}

/********************************************
*  eQEP
*********************************************/

// init_eqep takes care of sanity checks and returns quickly
// if nothing is to be initialized.
int init_eqep(int ss){
	// range sanity check
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	// see if eQEP already got initialized
	if(eqep_initialized[ss]){
		return 0;
	}

	// check ti-eqep driver is up
	switch(ss){
	case 0:
		if(access("/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep", F_OK ) != 0){
			printf("ERROR: ti-eqep driver not loaded for eqep0\n");
			return -1;
		}
		break;
	case 1:
		if(access("/sys/devices/platform/ocp/48302000.epwmss/48302180.eqep", F_OK ) != 0){
			printf("ERROR: ti-eqep driver not loaded for eqep1\n");
			return -1;
		}
		break;
	case 2:
		if(access("/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep", F_OK ) != 0){
			printf("ERROR: ti-eqep driver not loaded for eqep2\n");
			return -1;
		}
		break;
	default:
		break;
	}


	// make sure the subsystem is mapped
	if(map_pwmss(ss)){
		printf("failed to map PWMSS %d\n", ss);
		return -1;
	}
	#ifdef DEBUG
		printf("setting eqep ctrl registers\n");
	#endif
	//turn off clock to eqep
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) &= ~PWMSS_EQEPCLK_EN;
	// Write the decoder control settings
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QDECCTL) = 0;
	// set maximum position to two's compliment of -1, aka UINT_MAX
	*(uint32_t*)(pwm_base[ss]+EQEP_OFFSET+QPOSMAX)=-1;
	// Enable interrupt
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QEINT) = UTOF;
	// set unit period register
	*(uint32_t*)(pwm_base[ss]+EQEP_OFFSET+QUPRD)=0x5F5E100;
	// enable counter in control register
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QEPCTL) = PHEN|IEL0|SWI|UTE|QCLM;
	//enable clock from PWMSS
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= PWMSS_EQEPCLK_EN;
	
	// Test eqep by resetting position
	#ifdef DEBUG
		printf("testing eQEP write\n");
	#endif
	*(uint32_t*)(pwm_base[ss] + EQEP_OFFSET +QPOSCNT) = 0;
	#ifdef DEBUG
		printf("successfully tested eQEP write\n");
	#endif
	eqep_initialized[ss] = 1;
	return 0;
}

// read a value from eQEP counter
int read_eqep(int ch){
	if(init_eqep(ch)) return -1;
	return  *(int*)(pwm_base[ch] + EQEP_OFFSET +QPOSCNT);
}

// write a value to the eQEP counter
int write_eqep(int ch, int val){
	if(init_eqep(ch)) return -1;
	*(int*)(pwm_base[ch] + EQEP_OFFSET +QPOSCNT) = val;
	return 0;
}

/*******************************************************************************
* int rc_pwm_set_duty_mmap(int ss, char ch, float duty)
*
* This is the fastest way to set the pwm duty cycle and is used internally by
* the rc_set_motor() function but is also available to the user. This is done
* with direct memory access from userspace to the pwm subsystem. It's use is
* identical to rc_pwm_set_duty where subsystem ss must be 0,1, or 2 where
* 1 and 2 are used by the motor H bridges. Channel 'ch' must be 'A' or 'B' and
* duty must be from 0.0f to 1.0f. The subsystem must be intialized with
* rc_pwm_init() before use. Returns 0 on success or -1 on failure.
*******************************************************************************/

// set duty cycle for either channel A or B in a given subsystem
// input channel is a character 'A' or 'B'
int rc_pwm_set_duty_mmap(int ss, char ch, float duty){
	// make sure the subsystem is mapped
	if(unlikely(map_pwmss(ss))){
		fprintf(stderr,"ERROR in rc_pwm_set_duty_mmap,failed to map PWMSS %d\n", ss);
		return -1;
	}
	//sanity check duty
	if(unlikely(duty>1.0f||duty<0.0f)){
		fprintf(stderr,"ERROR in rc_pwm_set_duty_mmap, duty must be between 0.0f & 1.0f\n");
		return -1;
	}
	
	// duty ranges from 0 to TBPRD+1 for 0-100% PWM duty
	uint16_t period = *(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBPRD);
	uint16_t new_duty = (uint16_t)lroundf(duty * (period+1));
	
	#ifdef DEBUG
		printf("period : %d\n", period);
		printf("new_duty : %d\n", new_duty);
	#endif
	
	// change appropriate compare register
	switch(ch){
	case 'A':
		*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPA) = new_duty;
		break;
	case 'B':
		*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPB) = new_duty;
		break;
	default:
		fprintf(stderr,"ERROR in rc_pwm_set_duty_mmap, pwm channel must be 'A' or 'B'\n");
		return -1;
	}
	
	return 0;
}
