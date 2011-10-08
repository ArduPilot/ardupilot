#ifdef __AVR_ATmega1280__
/*
	APM2_RC.cpp - Radio Control Library for Ardupilot Mega. Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

	RC Input : PPM signal on IC4 pin
	RC Output : 11 Servo outputs (standard 20ms frame)

	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch, pwm) : Output value to servos (range : 900 - 2100us) ch = 0..10
		InputCh(ch) : Read a channel input value.	ch = 0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
								 Automatically resets when we call InputCh to read channels
		
*/

#include "APM2_RC.h"

#define REVERSE 3050

// Variable definition for Input Capture interrupt
volatile uint16_t 	ICR4_old;
volatile uint8_t 	PPM_Counter = 0;
volatile uint16_t 	raw[8] = {1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200};

// Constructors ////////////////////////////////////////////////////////////////
APM2_RC::APM2_RC()
{
	_direction_mask = 255;	// move to super class

}

void 
APM2_RC::init()
{
	// Init PWM Timer 1
	pinMode(11, OUTPUT); //		 (PB5 / OC1A)
	pinMode(12, OUTPUT); // OUT2 (PB6 / OC1B)
	pinMode(13, OUTPUT); // OUT3 (PB7 / OC1C)

	// Timer 3
	pinMode(2, OUTPUT); // OUT7 (PE4 / OC3B)
	pinMode(3, OUTPUT); // OUT6 (PE5 / OC3C)
	pinMode(4, OUTPUT); //		 (PE3 / OC3A)

	// Timer 5
	pinMode(44, OUTPUT); // OUT1 (PL5 / OC5C)
	pinMode(45, OUTPUT); // OUT0 (PL4 / OC5B)
	pinMode(46, OUTPUT); //		 (PL3 / OC5A)

	// Init PPM input and PWM Timer 4
	pinMode(49, INPUT);	// ICP4 pin (PL0) (PPM input)
	pinMode(7, OUTPUT);	 // OUT5 (PH4 / OC4B)
	pinMode(8, OUTPUT);	 // OUT4 (PH5 / OC4C)

	//Remember the registers not declared here remains zero by default... 
	TCCR1A =((1 << WGM11) | (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1)); // Please read page 131 of DataSheet, we are changing the registers settings of WGM11, COM1B1, COM1A1 to 1 thats all... 
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
	OCR1A = 3000; // PB5, none
	//OCR1B = 3000; // PB6, OUT2
	//OCR1C = 3000; // PB7 OUT3
	ICR1 = 40000; // 50hz freq...Datasheet says	(system_freq / prescaler) / target frequency. So (16000000hz / 8) / 50hz = 40000,

	// Init PWM Timer 3
	TCCR3A =((1 << WGM31) | (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1));
	TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); 
	OCR3A = 3000; // PE3, NONE
	//OCR3B = 3000; // PE4, OUT7
	//OCR3C = 3000; // PE5, OUT6
	ICR3 = 40000; // 50hz freq

	
	// Init PWM Timer 5
	TCCR5A =((1 << WGM51) | (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1)); 
	TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51);
	OCR5A = 3000; // PL3, 
	//OCR5B = 3000; // PL4, OUT0
	//OCR5C = 3000; // PL5, OUT1
	ICR5 = 40000; // 50hz freq

			
	// Init PPM input and PWM Timer 4
	TCCR4A = ((1 << WGM40) | (1 << WGM41) | (1 << COM4C1) | (1 << COM4B1) | (1 << COM4A1));	
	TCCR4B = ((1 << WGM43) | (1 << WGM42) | (1 << CS41) | (1 << ICES4));
	OCR4A = 40000; // /50hz freq.
	//OCR4B = 3000; // PH4, OUT5
	//OCR4C = 3000; // PH5, OUT4
 
	//TCCR4B |=(1<<ICES4); //Changing edge detector (rising edge). 
	//TCCR4B &=(~(1<<ICES4)); //Changing edge detector. (falling edge)
	TIMSK4 |= (1 << ICIE4); // Enable Input Capture interrupt. Timer interrupt mask

	// trim out the radio
	for(int c = 0; c < 50; c++){
		delay(20);
		read();
	}
	
	trim();
	
	for(int y = 0; y < 8; y++) { 
		set_ch_pwm(y, radio_trim[y]);
	}
}

void APM2_RC::read()
{
	//Serial.print("ch1 in ");
	//Serial.print(raw[CH1],DEC);
	
	// reverse any incoming PWM if needed
	for(int y = 0; y < 8; y++) { 
		if((_direction_mask & (1 << y)) == 0)
			radio_in[y] = REVERSE - raw[y];
		else
			radio_in[y] = raw[y];
	}

	//Serial.print("\tch1 in ");
	//Serial.print(radio_in[CH1],DEC);

	if(_mix_mode == 1){
		// elevons
		int16_t ailerons = (float)(radio_in[CH1] - radio_trim[CH1]);
		int16_t elevator = (float)(radio_in[CH2] - radio_trim[CH2]) * .7;

		//Serial.print("\tailerons ");
		//Serial.print(ailerons,DEC);

		//Serial.print("\tradio_trim ");
		//Serial.print(radio_trim[CH1],DEC);

		radio_in[CH1] = (elevator - ailerons); // left
		radio_in[CH2] = (elevator + ailerons); // right		

		radio_in[CH1] += radio_trim[CH1];
		radio_in[CH2] += radio_trim[CH2];
		
		//Serial.print("\tch1 in ");
		//Serial.print(radio_in[CH1],DEC);

		//Serial.print("\tch1 trim ");
		//Serial.print(radio_trim[CH1],DEC);

		//Serial.print("radio_in[CH1] ");
		//Serial.print(radio_in[CH1],DEC);
		//Serial.print(" \tradio_in[CH2] ");
		//Serial.println(radio_in[CH2],DEC);
	}

	// output servos
	for (uint8_t i = 0; i < 8; i++){
		if (i == 3) continue;
		if(radio_in[i] >= radio_trim[i])
			servo_in[i] = (float)(radio_in[i] - radio_min[i]) / (float)(radio_max[i] - radio_min[i]) * 100.0;
		else
			servo_in[i] = (float)(radio_in[i] - radio_trim[i]) / (float)(radio_trim[i] - radio_min[i]) * 100.0;
	}
	servo_in[CH3] = (float)(radio_in[CH3] - radio_min[CH3]) / (float)(radio_max[CH3] - radio_min[CH3]) * 100.0;
	servo_in[CH3] = constrain(servo_out[CH3], 0, 100);	
}

void
APM2_RC::output()
{
	uint16_t out;
	for (uint8_t i = 0; i < 8; i++){
		if (i == 3) continue;
		if(radio_in[i] >= radio_trim[i])
			out = ((servo_in[i] * (radio_max[i] - radio_trim[i])) / 100) + radio_trim[i];
		else
			out = ((servo_in[i] * (radio_max[i] - radio_trim[i])) / 100) + radio_trim[i];
		set_ch_pwm(i, out);
	}

	out = (servo_out[CH3] * (float)(radio_max[CH3] - radio_min[CH3])) / 100.0;
	out += radio_min[CH3];
	set_ch_pwm(CH3, out);
}

void 
APM2_RC::trim()
{
	uint8_t temp = _mix_mode;
	_mix_mode = 0;
	read();
	_mix_mode = temp;
	
	// Store the trim values
	// ---------------------
	for (int y = 0; y < 8; y++) 
		radio_trim[y] = radio_in[y];
}
void
APM2_RC::twitch_servos(uint8_t times)
{
	// todo
}
void
APM2_RC::set_ch_pwm(uint8_t ch, uint16_t pwm)
{	
	//pwm = constrain(pwm, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
	
	switch(ch){
		case 0:
			//Serial.print("\tpwm out ");
			//Serial.print(pwm,DEC);

			if((_direction_mask & 1) == 0 )
				pwm = REVERSE - pwm;

			//Serial.print("\tpwm out ");
			//Serial.println(pwm,DEC);
			
			OCR5B = pwm << 1;
			break;	// ch0
			
		case 1:
			if((_direction_mask & 2) == 0 )
				pwm = REVERSE - pwm;
			OCR5C = pwm << 1;
			break;	// ch0
			
		case 2:
			if((_direction_mask & 4) == 0 )
				pwm = REVERSE - pwm;
			OCR1B = pwm << 1;
			break;	// ch0
			
		case 3:
			if((_direction_mask & 8) == 0 )
				pwm = REVERSE - pwm;
			OCR1C = pwm << 1;
			break;	// ch0
			
		case 4:
			if((_direction_mask & 16) == 0 )
				pwm = REVERSE - pwm;
			OCR4C = pwm << 1;
			break;	// ch0
			
		case 5:
			if((_direction_mask & 32) == 0 )
				pwm = REVERSE - pwm;
			OCR4B = pwm << 1;
			break;	// ch0
			
		case 6:
			if((_direction_mask & 64) == 0 )
				pwm = REVERSE - pwm;
			OCR3C = pwm << 1;
			break;	// ch0
			
		case 7:
			if((_direction_mask & 128) == 0 )
				pwm = REVERSE - pwm;
			OCR3B = pwm << 1;
			break;	// ch0

		case 8:
			OCR5A = pwm << 1;
			break;	// ch0

		case 9:
			OCR1A = pwm << 1;
			break;	// ch0

		case 10:
			OCR3A = pwm << 1;
			break;	// ch0
	} 
}

/****************************************************
	 Input Capture Interrupt ICP4 => PPM signal read
 ****************************************************/
ISR(TIMER4_CAPT_vect)	
{
	uint16_t pulse;
	uint16_t pulse_width;
	
	pulse = ICR4;
	if (pulse < ICR4_old){			 				// Take care of the overflow of Timer4 (TOP = 40000)
		pulse_width = (pulse + 40000) - ICR4_old;	// Calculating pulse 
	}else{
		pulse_width = pulse - ICR4_old;				// Calculating pulse 
	}
	ICR4_old = pulse;
	
	
	if (pulse_width > 8000){  						// SYNC pulse
		PPM_Counter = 0;
	} else {
		//PPM_Counter &= 0x07;						// For safety only (limit PPM_Counter to 7)
		raw[PPM_Counter++] = pulse_width >> 1;		// Saving pulse. 
	}
}



// InstantPWM implementation
// This function forces the PWM output (reset PWM) on Out0 and Out1 (Timer5). For quadcopters use
void APM2_RC::force_out_0_1(void)
{
	if (TCNT5 > 5000)	 // We take care that there are not a pulse in the output
		TCNT5 = 39990;	 // This forces the PWM output to reset in 5us (10 counts of 0.5us). The counter resets at 40000
}
// This function forces the PWM output (reset PWM) on Out2 and Out3 (Timer1). For quadcopters use
void APM2_RC::force_out_2_3(void)
{
	if (TCNT1 > 5000)
		TCNT1 = 39990;
}

// This function forces the PWM output (reset PWM) on Out6 and Out7 (Timer3). For quadcopters use
void APM2_RC::force_out_6_7(void)
{
	if (TCNT3 > 5000)
		TCNT3 = 39990;
}
#endif
