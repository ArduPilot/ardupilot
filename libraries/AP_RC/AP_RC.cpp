/*
	AP_Radio.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include "AP_RC.h"

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define THROTTLE_PIN 13
#include <avr/interrupt.h>

#define CH3TRIM 1100

// Variable definition for interrupt
volatile uint16_t timer1count	= 0;
volatile uint16_t timer2count	= 0;
volatile uint16_t timer3count	= 0;
volatile uint16_t timer4count   = 0;

volatile uint16_t timer1diff	= 1500 * 2;
volatile uint16_t timer2diff	= 1500 * 2;
volatile uint16_t timer3diff	= 1100 * 2;
volatile uint16_t timer4diff	= 1500 * 2;


#define CH1_READ 1
#define CH2_READ 2
#define CH3_READ 4
#define CH4_READ 8

volatile int8_t 	_rc_ch_read = 0;
volatile uint8_t 	_timer_ovf_a	= 0;
volatile uint8_t 	_timer_ovf_b	= 0;
volatile uint8_t 	_timer_ovf		= 0;
//volatile uint16_t ap_rc_input[4];


AP_RC::AP_RC()
{
	pinMode(11,INPUT); 	// PB3 - MOSI/OC2	- Throttle in
	pinMode(13,INPUT); 	// PB5 - SCK		- Rudder in
	pinMode(8, OUTPUT); // PB0 - AIN1		- OUTPUT THROTTLE
	pinMode(9, OUTPUT);	// PB1 - OC1A		- Elevator PWM out
	pinMode(10,OUTPUT);	// PB2 - OC1B		- Aileron PWM out
	// set Analog out 4 to output
	DDRC |= B00010000;	
}

void
AP_RC::read_pwm()
{
	input[CH1] = timer1diff;
	input[CH2] = timer2diff;
	input[CH3] = timer3diff;
	input[CH4] = timer4diff;
}

void AP_RC::init(int trims[])
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
	// apply initial values
	set_ch_pwm(CH1, trims[CH1]);
	set_ch_pwm(CH2, trims[CH2]);
	ICR1 = 40000;

	// Throttle;
	// Setting up the Timer 2 - 8 bit timer
	TCCR2A 	= 0x0; 			// Normal Mode						
	TCCR2B 	= _BV(CS21) |_BV(CS20); //prescaler 32, at 16mhz (32/16) = 2, the counter will increment 1 every 2us
	// apply initial values
	//OCR2A = (trims[CH3]-1000) / 4;
	//OCR2B = trims[CH4] / 4; // center the rudder
	set_ch_pwm(CH3, trims[CH3]);
	set_ch_pwm(CH4, trims[CH4]);

	TIMSK1 |= _BV(ICIE1); 	// Timer / Counter1, Input Capture Interrupt Enable // PB0 - output throttle
	TIMSK2 	= _BV(TOIE1) | _BV(OCIE2A) | _BV(OCIE2B);	// Timer / Counter2 Compare Match A	

}

void
AP_RC::set_ch_pwm(uint8_t ch, uint16_t pwm)
{	
	switch(ch){
		case CH1:
			pwm <<= 1;
			OCR1A = pwm;
			break;

		case CH2:
			pwm <<= 1;
			OCR1B = pwm;
			break;

		case CH3:
			// Jason's fancy 2µs hack
			_timer_out 			= pwm % 512; 
			_timer_ovf_a 		= pwm / 512;
			_timer_out >>= 1;
			OCR2A = _timer_out;
			break;

		case CH4:
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
			 timer1diff = (cnt + 40000 - timer1count);		// Timer1 TOP = 40000
		else
			timer1diff = (cnt - timer1count);
		timer1diff >>= 1;
	}
	
	if(PIND & B00001000){ 		// ch 2 (pin 3) is high
		if ((_rc_ch_read & CH2_READ) != CH2_READ){
			_rc_ch_read |= CH2_READ;
			timer2count = cnt;
		}
	}else if ((_rc_ch_read & CH2_READ) == CH2_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111101;
		if (cnt < timer2count)	 // Timer1 reset during the read of this pulse
			 timer2diff = (cnt + 40000 - timer2count);		// Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer2count);
		timer2diff >>= 1;
	}
}

ISR(PCINT0_vect)
{
	int cnt = TCNT1;
	
#if THROTTLE_PIN == 11
	if(PINB & 8){	// pin 11
#else
	if(PINB & 32){	// pin 13
#endif
		if ((_rc_ch_read & CH3_READ) != CH3_READ){
			_rc_ch_read |= CH3_READ;
			timer3count = cnt;
		}
	}else if ((_rc_ch_read & CH3_READ) == CH3_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111011;
		if (cnt < timer3count)	 // Timer1 reset during the read of this pulse
			timer3diff = (cnt + 40000 - timer3count);		// Timer1 TOP = 40000
		else
			timer3diff = (cnt - timer3count);
		timer3diff >>= 1;

	}

#if THROTTLE_PIN == 11
	if(PINB & 32){	// pin 13
#else
	if(PINB & 8){	// pin 11
#endif
		if ((_rc_ch_read & CH4_READ) != CH4_READ){
			_rc_ch_read |= CH4_READ;
			timer4count = cnt;
		}
	}else if ((_rc_ch_read & CH4_READ) == CH4_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11110111;
		if (cnt < timer4count)	// Timer1 reset during the read of this pulse
			timer4diff = (cnt + 40000 - timer4count);		// Timer1 TOP = 40000
		else
			timer4diff = (cnt - timer4count);
		timer4diff >>= 1;
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

