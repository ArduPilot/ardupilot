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
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

// Variable definition for interrupt
volatile uint16_t timer1count   = 0;
volatile uint16_t timer2count   = 0;
volatile uint16_t timer3count   = 0;
volatile uint16_t timer4count   = 0;

volatile int16_t timer1diff     = 1500 * 2;
volatile int16_t timer2diff     = 1500 * 2;
volatile int16_t timer3diff     = 1100 * 2;
volatile int16_t timer4diff     = 1500 * 2;

//volatile uint16_t raw[8];
#define CH1_READ 1
#define CH2_READ 2
#define CH3_READ 4
#define CH4_READ 8

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3

volatile int8_t		_rc_ch_read;
volatile uint16_t	_timer_out;
volatile uint16_t	_timer_ovf_a;
volatile uint16_t	_timer_ovf_b;
volatile uint16_t	_timer_ovf;


AP_RC::AP_RC()
{
	pinMode(2,INPUT);       // PD2 - INT0		- CH 1 in
	pinMode(3,INPUT);       // PD3 - INT1		- CH 2 in
	pinMode(11,INPUT);      // PB3 - MOSI/OC2	- CH 3 in
	pinMode(13,INPUT);      // PB5 - SCK		- CH 4 in

	pinMode(10,OUTPUT);     // PB2 - OC1B		- CH 1 out
	pinMode(8, OUTPUT); 	// PB0 - AIN1		- CH 3 out
	pinMode(9, OUTPUT);     // PB1 - OC1A		- CH 2 out
	DDRC |= B00010000;      // PC4 -			- CH 4 out
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
	TCCR2A  = 0x0;		  // Normal Mode
	TCCR2B  = _BV(CS21) |_BV(CS20); //prescaler 32, at 16mhz (32/16) = 2, the counter will increment 1 every 2us

	// enable throttle and Ch4 output
	TIMSK1 |= _BV(ICIE1);   // Timer / Counter1, Input Capture Interrupt Enable // PB0 - output throttle
	TIMSK2  = _BV(TOIE1) | _BV(OCIE2A) | _BV(OCIE2B);       // Timer / Counter2 Compare Match A

	/*
	set_ch_pwm(0, 1500);
	set_ch_pwm(1, 1500);
	set_ch_pwm(2, 1500);
	set_ch_pwm(3, 1500);
	*/
}

uint16_t
AP_RC::input_ch(uint8_t ch)
{
	switch(ch){
		case CH_1:
			return timer1diff;
			break;

		case CH_2:
			return timer2diff;
			break;

		case CH_3:
			return timer3diff;
			break;

		case CH_4:
			return timer4diff;
			break;
	}
}

void
AP_RC::output_ch_pwm(uint8_t ch, uint16_t pwm)
{
	switch(ch){
		case CH_1:
			pwm <<= 1;								// multiplies by 2
			OCR1A = pwm;
			break;

		case CH_2:
			pwm <<= 1;
			OCR1B = pwm;							// multiplies by 2
			break;

		case CH_3:
			_timer_out				= pwm % 512;
			_timer_ovf_a			= pwm / 512;
			_timer_out >>= 1;						// divides by 2
			//OCR2A = _timer_out;
			if(OCR2A != _timer_out)
				OCR2A = _timer_out;

			break;

		case CH_4:
			_timer_out				= pwm % 512;
			_timer_ovf_b			= pwm / 512;
			_timer_out >>= 1;						// divides by 2
			//OCR2B = _timer_out;
			if(OCR2B != _timer_out)
				OCR2B = _timer_out;

			break;
	}
}



// radio PWM input timers
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	if(PIND & B00000100){	   // ch 1 (pin 2) is high
		if ((_rc_ch_read & CH1_READ) != CH1_READ){
			_rc_ch_read |= CH1_READ;
			timer1count = cnt;
		}
	}else if ((_rc_ch_read & CH1_READ) == CH1_READ){	// ch 1 (pin 2) is Low, and we were reading
		_rc_ch_read &= B11111110;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
			 timer1diff = (cnt + 40000 - timer1count) >> 1;	 // Timer1 TOP = 40000
		else
			timer1diff = (cnt - timer1count) >> 1;
	}

	if(PIND & B00001000){	   // ch 2 (pin 3) is high
		if ((_rc_ch_read & CH2_READ) != CH2_READ){
			_rc_ch_read |= CH2_READ;
			timer2count = cnt;
		}
	}else if ((_rc_ch_read & CH2_READ) == CH2_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111101;
		if (cnt < timer2count)   // Timer1 reset during the read of this pulse
			 timer2diff = (cnt + 40000 - timer2count) >> 1;	 // Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer2count) >> 1;
	}
}

ISR(PCINT0_vect)
{
	int cnt = TCNT1;
	if(PINB & 8){   // pin 11
		if ((_rc_ch_read & CH3_READ) != CH3_READ){
			_rc_ch_read |= CH3_READ;
			timer3count = cnt;
		}
	}else if ((_rc_ch_read & CH3_READ) == CH3_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111011;
		if (cnt < timer3count)   // Timer1 reset during the read of this pulse
			timer3diff = (cnt + 40000 - timer3count) >> 1;	  // Timer1 TOP = 40000
		else
			timer3diff = (cnt - timer3count) >> 1;
	}

	if(PINB & 32){  // pin 13
		if ((_rc_ch_read & CH4_READ) != CH4_READ){
			_rc_ch_read |= CH4_READ;
			timer4count = cnt;
		}
	}else if ((_rc_ch_read & CH4_READ) == CH4_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11110111;
		if (cnt < timer4count)  // Timer1 reset during the read of this pulse
			timer4diff = (cnt + 40000 - timer4count) >> 1;	  // Timer1 TOP = 40000
		else
			timer4diff = (cnt - timer4count) >> 1;
	}
}



// Throttle Timer Interrupt
// ------------------------
ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{
	//This is a timer 1 interrupts, executed every 20us
	PORTB 		|= B00000001; 	//Putting the pin high!
	PORTC 		|= B00010000; 	//Putting the pin high!
	TCNT2 		= 0; 			//restarting the counter of timer 2
	_timer_ovf 	= 0;
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