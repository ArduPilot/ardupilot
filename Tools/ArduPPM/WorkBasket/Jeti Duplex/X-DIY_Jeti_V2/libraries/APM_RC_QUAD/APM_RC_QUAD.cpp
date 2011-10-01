/*
	APM_RC_QUAD.cpp - Radio Control Library for Ardupilot Mega. Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	This is a special version of the library for use in Quadcopters
	Because we have modified servo outputs 0..3 to have a higher rate (300Hz)

	RC Input : PPM signal on IC4 pin
	RC Output : 8 servo Outputs, 
	           OUT0..OUT3 : 300Hz Servo output (for Quad ESC´s)
			   OUT4..OUT7 : standard 50Hz servo outputs

	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
		InputCh(ch) : Read a channel input value.  ch=0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
		             Automatically resets when we call InputCh to read channels
		
*/
#include "APM_RC_QUAD.h"

#include <avr/interrupt.h>
#include "WProgram.h"

// Variable definition for Input Capture interrupt
volatile unsigned int ICR4_old;
volatile unsigned char PPM_Counter=0;
volatile unsigned int PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile unsigned char radio_status=0;

/****************************************************
   Input Capture Interrupt ICP4 => PPM signal read
 ****************************************************/
ISR(TIMER4_CAPT_vect)  
{
  unsigned int Pulse;
  unsigned int Pulse_Width;
  
  Pulse=ICR4;
  if (Pulse<ICR4_old)     // Take care of the overflow of Timer4 (TOP=40000)
    Pulse_Width=(Pulse + 40000)-ICR4_old;  //Calculating pulse 
  else
    Pulse_Width=Pulse-ICR4_old;            //Calculating pulse 
  if (Pulse_Width>8000)   // SYNC pulse?
    PPM_Counter=0;
  else
    {
    PPM_Counter &= 0x07;  // For safety only (limit PPM_Counter to 7)
    PWM_RAW[PPM_Counter++]=Pulse_Width;  //Saving pulse. 
    if (PPM_Counter >= NUM_CHANNELS)
      radio_status = 1;
    }
  ICR4_old = Pulse;
}


// Constructors ////////////////////////////////////////////////////////////////

APM_RC_QUAD_Class::APM_RC_QUAD_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void APM_RC_QUAD_Class::Init(void)
{
  // Init PWM Timer 1
  //pinMode(11,OUTPUT); //     (PB5/OC1A)
  pinMode(12,OUTPUT); //OUT2 (PB6/OC1B)
  pinMode(13,OUTPUT); //OUT3 (PB7/OC1C)

  //Remember the registers not declared here remains zero by default... 
  TCCR1A =((1<<WGM11)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
  //OCR1A = 3000; //PB5, none
  OCR1B = 3000; //PB6, OUT2
  OCR1C = 3000; //PB7  OUT3
  ICR1 = 6600; //300hz freq...

  // Init PWM Timer 3
  pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
  pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
  //pinMode(4,OUTPUT); //     (PE3/OC3A)
  TCCR3A =((1<<WGM31)|(1<<COM3B1)|(1<<COM3C1));
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
  //OCR3A = 3000; //PE3, NONE
  OCR3B = 3000; //PE4, OUT7
  OCR3C = 3000; //PE5, OUT6
  ICR3 = 40000; //50hz freq (standard servos)

  // Init PWM Timer 5
  pinMode(44,OUTPUT);  //OUT1 (PL5/OC5C)
  pinMode(45,OUTPUT);  //OUT0 (PL4/OC5B)
  //pinMode(46,OUTPUT);  //     (PL3/OC5A)
  
  TCCR5A =((1<<WGM51)|(1<<COM5B1)|(1<<COM5C1)); 
  TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
  //OCR5A = 3000; //PL3, 
  OCR5B = 3000; //PL4, OUT0
  OCR5C = 3000; //PL5, OUT1
  ICR5 = 6600; //300hz freq

  // Init PPM input and PWM Timer 4
  pinMode(49, INPUT);  // ICP4 pin (PL0) (PPM input)
  pinMode(7,OUTPUT);   //OUT5 (PH4/OC4B)
  pinMode(8,OUTPUT);   //OUT4 (PH5/OC4C)
      
  TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));  
  //Prescaler set to 8, that give us a resolution of 0.5us
  // Input Capture rising edge
  TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4));
  
  OCR4A = 40000; ///50hz freq. (standard servos)
  OCR4B = 3000; //PH4, OUT5
  OCR4C = 3000; //PH5, OUT4
 
  //TCCR4B |=(1<<ICES4); //Changing edge detector (rising edge). 
  //TCCR4B &=(~(1<<ICES4)); //Changing edge detector. (falling edge)
  TIMSK4 |= (1<<ICIE4); // Enable Input Capture interrupt. Timer interrupt mask
}

void APM_RC_QUAD_Class::OutputCh(unsigned char ch, int pwm)
{
  pwm=constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  pwm<<=1;   // pwm*2;
 
 switch(ch)
  {
    case 0:  OCR5B=pwm; break;  //ch0
    case 1:  OCR5C=pwm; break;  //ch1
    case 2:  OCR1B=pwm; break;  //ch2
    case 3:  OCR1C=pwm; break;  //ch3
    case 4:  OCR4C=pwm; break;  //ch4
    case 5:  OCR4B=pwm; break;  //ch5
    case 6:  OCR3C=pwm; break;  //ch6
    case 7:  OCR3B=pwm; break;  //ch7
  } 
}

int APM_RC_QUAD_Class::InputCh(unsigned char ch)
{
  int result;
  int result2;
  
  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
  result =  PWM_RAW[ch]>>1;  // Because timer runs at 0.5us we need to do value/2
  result2 =  PWM_RAW[ch]>>1;
  if (result != result2)
    result =  PWM_RAW[ch]>>1;   // if the results are different we make a third reading (this should be fine)
  
  // Limit values to a valid range
  result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  radio_status=0; // Radio channel read
  return(result);
}

unsigned char APM_RC_QUAD_Class::GetState(void)
{
  return(radio_status);
}

// make one instance for the user to use
APM_RC_QUAD_Class APM_RC_QUAD;