/*******************************************************************************
* rc_mmap_gpio_adc.c
*******************************************************************************/

#include "rc_mmap_gpio_adc.h"
#include "rc_mmap_gpio_adc_defs.h"
#include "roboticscape/preprocessor_macros.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


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

/*******************************************************************************
*   GPIO
*******************************************************************************/

// initialize GPIO
// by default gpio subsystems 0,1,&2 have clock signals on boot
// this function exports a pin from the last subsystem (113) so the
// gpio driver enables the clock signal so the rest of the gpio
// functions here work.
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


/*********************************************
*   ADC
*********************************************/

// Initialize the Analog-Digital Converter
// each channel is set up in software one-shot mode for general purpose reading
// internal averaging set to 8 samples to reduce noise
int initialize_mmap_adc() {
	if(adc_initialized){
		return 0;
	}
	if (init_mmap()){
		return -1;
	}
	
	// enable the CM_WKUP_ADC_TSC_CLKCTRL with CM_WKUP_MODUELEMODE_ENABLE
	#ifdef DEBUG
	printf("Enabling Clock to ADC_TSC\n"); 
	#endif
	map[(CM_WKUP+CM_WKUP_ADC_TSC_CLKCTRL-MMAP_OFFSET)/4] |= MODULEMODE_ENABLE;

	// waiting for adc clock module to initialize
	#ifdef DEBUG
	printf("Checking clock signal has started\n"); 
	#endif
	while(!(map[(CM_WKUP+CM_WKUP_ADC_TSC_CLKCTRL-MMAP_OFFSET)/4] & \
														MODULEMODE_ENABLE)) {
		usleep(10);
		#ifdef DEBUG
		printf("Waiting for CM_WKUP_ADC_TSC_CLKCTRL to enable with MODULEMODE_ENABLE\n"); 
		#endif
		
	}
	
	// disable adc
	#ifdef DEBUG
	printf("Temporarily Disabling ADC\n"); 
	#endif
	map[(ADC_CTRL-MMAP_OFFSET)/4] &= !0x01;
	
	// make sure STEPCONFIG write protect is off
	map[(ADC_CTRL-MMAP_OFFSET)/4] |= ADC_STEPCONFIG_WRITE_PROTECT_OFF;

	// set up each ADCSTEPCONFIG for each ain pin
	map[(ADCSTEPCONFIG1-MMAP_OFFSET)/4] = 0x00<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY1-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG2-MMAP_OFFSET)/4] = 0x01<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY2-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG3-MMAP_OFFSET)/4] = 0x02<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY3-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG4-MMAP_OFFSET)/4] = 0x03<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY4-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG5-MMAP_OFFSET)/4] = 0x04<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY5-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG6-MMAP_OFFSET)/4] = 0x05<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY6-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG7-MMAP_OFFSET)/4] = 0x06<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY7-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG8-MMAP_OFFSET)/4] = 0x07<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY8-MMAP_OFFSET)/4]  = 0<<24;
	
	// enable the ADC
	map[(ADC_CTRL-MMAP_OFFSET)/4] |= 0x01;
		
	// clear the FIFO buffer
	int output;
	while(map[(FIFO0COUNT-MMAP_OFFSET)/4] & FIFO_COUNT_MASK){
		output =  map[(ADC_FIFO0DATA-MMAP_OFFSET)/4] & ADC_FIFO_MASK;
	}
	// just suppress the warning about output not being used
	if(output){}
	
	adc_initialized = 1;
	return 0;
}


// Read in from an analog pin with oneshot mode
int mmap_adc_read_raw(int ch) {
		  
	// clear the FIFO buffer just in case it's not empty
	int output;
	while(map[(FIFO0COUNT-MMAP_OFFSET)/4] & FIFO_COUNT_MASK){
		output =  map[(ADC_FIFO0DATA-MMAP_OFFSET)/4] & ADC_FIFO_MASK;
	}
		
	// enable step for the right pin
	map[(ADC_STEPENABLE-MMAP_OFFSET)/4] |= (0x01<<(ch+1));
	
	// wait for sample to appear in the FIFO buffer
	while(!(map[(FIFO0COUNT-MMAP_OFFSET)/4] & FIFO_COUNT_MASK)){}
	
	// return the the FIFO0 data register
	output =  map[(ADC_FIFO0DATA-MMAP_OFFSET)/4] & ADC_FIFO_MASK;
	
	return output;
}

