/*******************************************************************************
* rc_gpio_setup.c
* functions for initial setup of gpio pins. Not for use by the user.
*******************************************************************************/
#include <stdio.h>
#include "roboticscape/rc_defs.h"
#include "roboticscape/roboticscape.h"
#include <stdio.h>
#include <stdlib.h> // for system()
#include <unistd.h>	
#include <string.h> // for strcat()


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

	// Blue-only setup
	if(rc_get_bb_model()==BB_BLUE){
		mdir1a = MDIR1A_BLUE;
		mdir2b = MDIR2B_BLUE;
		// ret |= setup_output_pin(BLUE_SPI_PIN_6_SS1, HIGH);
		// ret |= setup_output_pin(BLUE_SPI_PIN_6_SS2, HIGH);
		// ret |= setup_input_pin(BLUE_GP0_PIN_3);
		// ret |= setup_input_pin(BLUE_GP0_PIN_4);
		// ret |= setup_input_pin(BLUE_GP0_PIN_5);
		// ret |= setup_input_pin(BLUE_GP0_PIN_6);
		// ret |= setup_input_pin(BLUE_GP1_PIN_3);
		// ret |= setup_input_pin(BLUE_GP1_PIN_4);
	}
	// Cape-Only stuff
	else{
		mdir1a = MDIR1A;
		mdir2b = MDIR2B;
		// ret |= setup_output_pin(CAPE_SPI_PIN_6_SS1, HIGH);
		// ret |= setup_output_pin(CAPE_SPI_PIN_6_SS2, HIGH);
	}

	// Shared Pins

	// LEDs
	// don't return error on these guys, might be controlled by kernel
	// but mmap will still work. shut up errors too
	int old_stderr;
	FILE  *null_out;
	// change stdout to null for this operation as the prussdrv.so
	// functions are noisy
	old_stderr = dup(STDERR_FILENO);
	fflush(stderr);
	null_out = fopen("/dev/null", "w");
	dup2(fileno(null_out), STDERR_FILENO);
	// configure the pins
	setup_output_pin(RED_LED, LOW);
	setup_output_pin(GRN_LED, LOW);
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
	
	// DSM
	ret |= setup_output_pin(DSM_PIN, LOW);
	
	// servo power
	ret |= setup_output_pin(SERVO_PWR, LOW);

	// buttons
	ret |= setup_input_pin(MODE_BTN); 
	ret |= setup_input_pin(PAUSE_BTN);
	rc_gpio_set_edge(MODE_BTN, EDGE_BOTH);
	rc_gpio_set_edge(PAUSE_BTN, EDGE_BOTH);

	// IMU
	ret |= setup_input_pin(IMU_INTERRUPT_PIN);

	// UART1, GPS, and SPI pins
	// ret |= setup_input_pin(GPS_HEADER_PIN_3); 
	// ret |= setup_input_pin(GPS_HEADER_PIN_4);
	// ret |= setup_input_pin(UART1_HEADER_PIN_3); 
	// ret |= setup_input_pin(UART1_HEADER_PIN_4);
	// ret |= setup_input_pin(SPI_HEADER_PIN_3);
	// ret |= setup_input_pin(SPI_HEADER_PIN_4);
	// ret |= setup_input_pin(SPI_HEADER_PIN_5);


	if(ret){
		printf("WARNING: Failed to configure all gpio pins\n");
		return -1;
	}

	return 0;
}
