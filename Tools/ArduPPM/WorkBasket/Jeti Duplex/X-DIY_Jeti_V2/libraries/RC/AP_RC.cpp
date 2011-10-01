/*
	AP_RC.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include "AP_RC.h"
#include <avr/interrupt.h>
#define REVERSE 3050

// Variable definition for interrupt
volatile uint16_t timer1count	= 0;
volatile uint16_t timer2count	= 0;
volatile uint16_t timer3count	= 0;
volatile uint16_t timer4count   = 0;

volatile int16_t timer1diff	= 1500 * 2;
volatile int16_t timer2diff	= 1500 * 2;
volatile int16_t timer3diff	= 1100 * 2;
volatile int16_t timer4diff	= 1500 * 2;

//volatile uint16_t raw[8];

#define CH1_READ 1
#define CH2_READ 2
#define CH3_READ 4
#define CH4_READ 8

volatile int8_t 	_rc_ch_read 	= 0;
volatile uint8_t 	_timer_ovf_a	= 0;
volatile uint8_t 	_timer_ovf_b	= 0;
volatile uint8_t 	_timer_ovf		= 0;


AP_RC::AP_RC()
{
	_direction_mask = 255;	// move to super class
	pinMode(2,INPUT);	// PD2 - INT0 		- CH 1 in
	pinMode(3,INPUT);	// PD3 - INT1 		- CH 2 in
	pinMode(11,INPUT); 	// PB3 - MOSI/OC2	- CH 3 in
	pinMode(13,INPUT); 	// PB5 - SCK		- CH 4 in

	pinMode(10,OUTPUT);	// PB2 - OC1B		- CH 1 out
	pinMode(8, OUTPUT); // PB0 - AIN1		- CH 3 out
	pinMode(9, OUTPUT);	// PB1 - OC1A		- CH 2 out
	DDRC |= B00010000;	// PC4 - 			- CH 4 out
}

void
AP_RC::init()
{
	// enable pin change interrupt 2 - PCINT23..16
	PCICR = _BV(PCIE2);
	// enable pin change interrupt 0 -  PCINT7..0
	PCICR |= _BV(PCIE0);
	// enable in change interrupt on PB5 (digital pin 13)
	PCMSK0 = _BV(PCINT3) | _BV(PCINT5);
	// enable pin change interrupt on PD2,PD3 (digital pin 2,3)
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19);
	
	// Timer 1
	TCCR1A = ((1 << WGM11) | (1 << COM1B1) | (1 << COM1A1));
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	// Loop value
	ICR1 = 40000;

	// Throttle;
	// Setting up the Timer 2 - 8 bit timer
	TCCR2A 	= 0x0; 			// Normal Mode						
	TCCR2B 	= _BV(CS21) |_BV(CS20); //prescaler 32, at 16mhz (32/16) = 2, the counter will increment 1 every 2us
	
	// trim out the radio
	for(int c = 0; c < 50; c++){
		delay(20);
		read();
	}
	
	trim();

	for(int y = 0; y < 4; y++) { 
		set_ch_pwm(y, radio_trim[y]);
	}
	
	// enable throttle and Ch4 output
	TIMSK1 |= _BV(ICIE1); 	// Timer / Counter1, Input Capture Interrupt Enable // PB0 - output throttle
	TIMSK2 	= _BV(TOIE1) | _BV(OCIE2A) | _BV(OCIE2B);	// Timer / Counter2 Compare Match A	
}

void 
AP_RC::read()
{	
	if((_direction_mask & 1) == 0 )
		timer1diff = REVERSE - timer1diff;
	if((_direction_mask & 2) == 0 )
		timer2diff = REVERSE - timer2diff;
	if((_direction_mask & 4) == 0 )
		timer3diff = REVERSE - timer3diff;
	if((_direction_mask & 8) == 0 )
		timer4diff = REVERSE - timer4diff;
		
	if(_mix_mode == 1){
		// elevons
		int16_t ailerons = (timer1diff - radio_trim[CH1]) * .3;
		int16_t elevator = (timer2diff - radio_trim[CH2]) * .7;
		
		radio_in[CH1] = (elevator - ailerons); // left
		radio_in[CH2] = (elevator + ailerons); // right		

		radio_in[CH1] += radio_trim[CH1];
		radio_in[CH2] += radio_trim[CH2];

		//Serial.print("radio_in[CH1] ");
		//Serial.print(radio_in[CH1],DEC);
		//Serial.print(" \tradio_in[CH2] ");
		//Serial.println(radio_in[CH2],DEC);
		
	}else{
		// normal
		radio_in[CH1] = timer1diff;
		radio_in[CH2] = timer2diff;
	}

	radio_in[CH3] = (float)radio_in[CH3] * .9 + timer3diff * .1;
	radio_in[CH4] = timer4diff;
	
	check_throttle_failsafe(radio_in[CH3]);

	// output servos
	for (uint8_t i = 0; i < 4; i++){
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
AP_RC::output()
{
	uint16_t out;
	for (uint8_t i = 0; i < 4; i++){
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
AP_RC::trim()
{
	uint8_t temp = _mix_mode;
	_mix_mode = 0;
	read();
	_mix_mode = temp;
	
	radio_trim[CH1] = radio_in[CH1];
	radio_trim[CH2] = radio_in[CH2];
	radio_trim[CH3] = radio_in[CH3];
	radio_trim[CH4] = radio_in[CH4];
	
	//Serial.print("trim ");
	//Serial.println(radio_trim[CH1], DEC);
}

void
AP_RC::twitch_servos(uint8_t times)
{
	while (times > 0){
		set_ch_pwm(CH1, radio_trim[CH1] + 100);
		set_ch_pwm(CH2, radio_trim[CH2] + 100);
		delay(400);
		set_ch_pwm(CH1, radio_trim[CH1] - 100);
		set_ch_pwm(CH2, radio_trim[CH2] - 100);
		delay(200);
		set_ch_pwm(CH1, radio_trim[CH1]);
		set_ch_pwm(CH2, radio_trim[CH2]);
		delay(30);
		times--;
	}
}

void
AP_RC::set_ch_pwm(uint8_t ch, uint16_t pwm)
{
	switch(ch){
		case CH1:
			if((_direction_mask & 1) == 0 )
				pwm = REVERSE - pwm;
			pwm <<= 1;
			OCR1A = pwm;
			break;

		case CH2:
			if((_direction_mask & 2) == 0 )
				pwm = REVERSE - pwm;
			pwm <<= 1;
			OCR1B = pwm;
			break;

		case CH3:
			if((_direction_mask & 4) == 0 )
				pwm = REVERSE - pwm;
			// Jason's fancy 2µs hack
			_timer_out 			= pwm % 512; 
			_timer_ovf_a 		= pwm / 512;
			_timer_out >>= 1;
			OCR2A = _timer_out;
			break;

		case CH4:
			if((_direction_mask & 8) == 0 )
				pwm = REVERSE - pwm;
			_timer_out 			= pwm % 512; 
			_timer_ovf_b 		= pwm / 512;
			_timer_out >>= 1;
			OCR2B = _timer_out;
			break;
	} 
}



// radio PWM input timers
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	if(PIND & B00000100){ 		// ch 1 (pin 2) is high
		if ((_rc_ch_read & CH1_READ) != CH1_READ){
			_rc_ch_read |= CH1_READ;
			timer1count = cnt;
		}
	}else if ((_rc_ch_read & CH1_READ) == CH1_READ){	// ch 1 (pin 2) is Low, and we were reading
		_rc_ch_read &= B11111110;
		if (cnt < timer1count)	 // Timer1 reset during the read of this pulse
			 timer1diff = (cnt + 40000 - timer1count) >> 1;		// Timer1 TOP = 40000
		else
			timer1diff = (cnt - timer1count) >> 1;
	}
	
	if(PIND & B00001000){ 		// ch 2 (pin 3) is high
		if ((_rc_ch_read & CH2_READ) != CH2_READ){
			_rc_ch_read |= CH2_READ;
			timer2count = cnt;
		}
	}else if ((_rc_ch_read & CH2_READ) == CH2_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111101;
		if (cnt < timer2count)	 // Timer1 reset during the read of this pulse
			 timer2diff = (cnt + 40000 - timer2count) >> 1;		// Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer2count) >> 1;
	}
}

ISR(PCINT0_vect)
{
	int cnt = TCNT1;
	if(PINB & 8){	// pin 11
		if ((_rc_ch_read & CH3_READ) != CH3_READ){
			_rc_ch_read |= CH3_READ;
			timer3count = cnt;
		}
	}else if ((_rc_ch_read & CH3_READ) == CH3_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111011;
		if (cnt < timer3count)	 // Timer1 reset during the read of this pulse
			timer3diff = (cnt + 40000 - timer3count) >> 1;		// Timer1 TOP = 40000
		else
			timer3diff = (cnt - timer3count) >> 1;
	}

	if(PINB & 32){	// pin 13
		if ((_rc_ch_read & CH4_READ) != CH4_READ){
			_rc_ch_read |= CH4_READ;
			timer4count = cnt;
		}
	}else if ((_rc_ch_read & CH4_READ) == CH4_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11110111;
		if (cnt < timer4count)	// Timer1 reset during the read of this pulse
			timer4diff = (cnt + 40000 - timer4count) >> 1;		// Timer1 TOP = 40000
		else
			timer4diff = (cnt - timer4count) >> 1;
	}
}



// Throttle Timer Interrupt
// ------------------------
ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{
	//This is a timer 1 interrupts, executed every 20us 
	PORTB |= B00000001; //Putting the pin high!
	PORTC |= B00010000; //Putting the pin high!	
	TCNT2 = 0; //restarting the counter of timer 2
	_timer_ovf = 0;
}

ISR(TIMER2_OVF_vect)
{
	_timer_ovf++;
}

ISR(TIMER2_COMPA_vect) // Timer/Counter2 Compare Match A
{
	if(_timer_ovf == _timer_ovf_a){
		PORTB &= B11111110; //Putting the pin low
	}
}

ISR(TIMER2_COMPB_vect) // Timer/Counter2 Compare Match B Rudder Servo
{
	if(_timer_ovf == _timer_ovf_b){
		PORTC &= B11101111; //Putting the pin low!
	}
} 

