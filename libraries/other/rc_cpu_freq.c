/*******************************************************************************
* rc_cpu_freq.c
*
* A collection of functions for adjusting the cpu frequency on the beaglebone.
* This is part of the robotics cape library but is not dependent on any other
* component so these functions could be used independently.
*******************************************************************************/

#include "roboticscape/roboticscape.h"
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

#define GOVERNOR_PATH  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
#define SETSPEED_PATH  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed"
#define CURFREQ_PATH   "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"


/*******************************************************************************
* int rc_set_cpu_freq(rc_cpu_freq_t freq)
*
* Sets the CPU frequency to either a fixed value or to onedemand automatic
* scaling mode. Returns 0 on success, -1 on failure.
*******************************************************************************/
int rc_set_cpu_freq(rc_cpu_freq_t freq){
	FILE* gov_fd  = fopen(GOVERNOR_PATH,  "w");
	if (gov_fd == NULL) {
		fprintf(stderr,"ERROR in rc_set_cpu_freq, cpu governor driver not available in this kernel\n");
		return -1;
	}
	
	// in automatic frequency mode, use the ondemand (default) governor
	if(freq==FREQ_ONDEMAND){
		fprintf(gov_fd, "ondemand");
		fflush(gov_fd);
		fclose(gov_fd);
		return 0;
	}
	
	// for a specific desired frequency, use userspace governor
	FILE* freq_fd = fopen(SETSPEED_PATH, "w");
	if (freq_fd == NULL) {
		fprintf(stderr,"ERROR in rc_set_cpu_freq, can't open CPU set frequency path\n");
		fclose(gov_fd);
		return -1;
	}
	fprintf(gov_fd, "userspace");
	fflush(gov_fd);
	fclose(gov_fd);
	
	switch(freq){
	case FREQ_300MHZ:
		fprintf(freq_fd, "300000");
		break;
	case FREQ_600MHZ:
		fprintf(freq_fd, "600000");
		break;
	case FREQ_800MHZ:
		fprintf(freq_fd, "800000");
		break;
	case FREQ_1000MHZ:
		fprintf(freq_fd, "1000000");
		break;
	default:
		printf("ERROR: unknown cpu frequency\n");
		fclose(freq_fd);
		return -1;
	}
	fflush(freq_fd);
	fclose(freq_fd);
	return 0;
}

/*******************************************************************************
* rc_cpu_freq_t rc_get_cpu_freq()
*
* Returns the current clock speed of the Beaglebone's Sitara processor in the
* form of the provided enumerated type. It will never return the FREQ_ONDEMAND
* value as the intention of this function is to see the clock speed as set by
* either the user or the ondemand governor itself.
*******************************************************************************/
rc_cpu_freq_t rc_get_cpu_freq(){
	int freq;
	FILE* freq_fd = fopen(CURFREQ_PATH, "r");
	if (freq_fd == NULL) {
		fprintf(stderr,"ERROR in rc_get_cpu_freq, cpu governor driver not available in this kernel\n");
		return -1;
	}
	if(fscanf(freq_fd,"%d",&freq)<0){
		fprintf(stderr,"ERROR in rc_get_cpu_freq, failed to read from CPU current frequency path\n");
		return -1;
	}
	fclose(freq_fd);
	switch(freq){
	case 300000:
		return FREQ_300MHZ;
	case 600000:
		return FREQ_600MHZ;
	case 800000:
		return FREQ_800MHZ;
	case 1000000:
		return FREQ_1000MHZ;
	default:
		fprintf(stderr,"ERROR in rc_get_cpu_freq, unknown cpu frequency detected\n");
	}
	return -1;
}

/*******************************************************************************
* int rc_print_cpu_freq()
*
* Prints the current frequency to the screen. For example "300MHZ".
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_print_cpu_freq(){
	int freq;
	FILE* freq_fd = fopen(CURFREQ_PATH, "r");
	if (freq_fd == NULL) {
		fprintf(stderr,"ERROR in rc_print_cpu_freq, cpu governor driver not available in this kernel\n");
		return -1;
	}
	if(fscanf(freq_fd,"%d",&freq)<0){
		fprintf(stderr,"ERROR in rc_print_cpu_freq, failed to read frequency path\n");
		return -1;
	}
	fclose(freq_fd);
	printf("%dmhz", freq/1000);
	return 0;
}



