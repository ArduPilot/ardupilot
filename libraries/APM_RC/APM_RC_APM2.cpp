/*
	APM_RC_APM2.cpp - Radio Control Library for Ardupilot Mega 2.0. Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	RC Input : PPM signal on IC4 pin
	RC Output : 11 Servo outputs (standard 20ms frame)

	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
		InputCh(ch) : Read a channel input value.  ch=0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
		             Automatically resets when we call InputCh to read channels

*/
#include "APM_RC_APM2.h"

#include "WProgram.h"

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint16_t APM_RC_APM2::_PWM_RAW[NUM_CHANNELS] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile uint8_t APM_RC_APM2::_radio_status=0;

/****************************************************
   Input Capture Interrupt ICP5 => PPM signal read
 ****************************************************/
void APM_RC_APM2::_timer5_capt_cb(void)
{
  static uint16_t prev_icr;
  static uint8_t frame_idx;
  uint16_t icr;
  uint16_t pwidth;

  icr = ICR5;
  // Calculate pulse width assuming timer overflow TOP = 40000
  if ( icr < prev_icr ) {
    pwidth = ( icr + 40000 ) - prev_icr;
  } else {
    pwidth = icr - prev_icr;
  }

  // Was it a sync pulse? If so, reset frame.
  if ( pwidth > 8000 ) {
    frame_idx=0;
  } else {
    // Save pulse into _PWM_RAW array.
    if ( frame_idx < NUM_CHANNELS ) {
      _PWM_RAW[ frame_idx++ ] = pwidth;
      // If this is the last pulse in a frame, set _radio_status.
      if (frame_idx >= NUM_CHANNELS) {
        _radio_status = 1;
      }
    }
  }
  // Save icr for next call.
  prev_icr = icr;
}


// Constructors ////////////////////////////////////////////////////////////////

APM_RC_APM2::APM_RC_APM2()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void APM_RC_APM2::Init( Arduino_Mega_ISR_Registry * isr_reg )
{
  // --------------------- TIMER1: OUT1 and OUT2 -----------------------
  pinMode(12,OUTPUT); // OUT1 (PB6/OC1B)
  pinMode(11,OUTPUT); // OUT2 (PB5/OC1A)

  // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
  // COM1A and COM1B enabled, set to low level on match.
  // CS11: prescale by 8 => 0.5us tick
  TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1));
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = 40000; // 0.5us tick => 50hz freq
  OCR1A = 0xFFFF; // Init OCR registers to nil output signal
  OCR1B = 0xFFFF;

  // --------------- TIMER4: OUT3, OUT4, and OUT5 ---------------------
  pinMode(8,OUTPUT); // OUT3 (PH5/OC4C)
  pinMode(7,OUTPUT); // OUT4 (PH4/OC4B)
  pinMode(6,OUTPUT); // OUT5 (PH3/OC4A)

  // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR4.
  // COM4A, 4B, 4C enabled, set to low level on match.
  // CS41: prescale by 8 => 0.5us tick
  TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1));
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  OCR4A = 0xFFFF; // Init OCR registers to nil output signal
  OCR4B = 0xFFFF;
  OCR4C = 0xFFFF;
  ICR4 = 40000; // 0.5us tick => 50hz freq

  //--------------- TIMER3: OUT6, OUT7, and OUT8 ----------------------
  pinMode(3,OUTPUT); // OUT6 (PE5/OC3C)
  pinMode(2,OUTPUT); // OUT7 (PE4/OC3B)
  pinMode(5,OUTPUT); // OUT8 (PE3/OC3A)

  // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR3
  // COM3A, 3B, 3C enabled, set to low level on match
  // CS31: prescale by 8 => 0.5us tick
  TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  OCR3A = 0xFFFF; // Init OCR registers to nil output signal
  OCR3B = 0xFFFF;
  OCR3C = 0xFFFF;
  ICR3 = 40000; // 0.5us tick => 50hz freq

  //--------------- TIMER5: PPM INPUT ---------------------------------
  // Init PPM input on Timer 5
  pinMode(48, INPUT);  // PPM Input (PL1/ICP5)

  // WGM: 1 1 1 1. Fast PWM, TOP is OCR5A
  // COM all disabled.
  // CS51: prescale by 8 => 0.5us tick
  // ICES5: Input Capture on rising edge
  TCCR5A =((1<<WGM50)|(1<<WGM51));
  // Input Capture rising edge
  TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5));
  OCR5A = 40000; // 0.5us tick => 50hz freq. The input capture routine
                 // assumes this 40000 for TOP.

  isr_reg->register_signal( ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb );
  // Enable Input Capture interrupt
  TIMSK5 |= (1<<ICIE5);
}

void APM_RC_APM2::OutputCh(unsigned char ch, uint16_t pwm)
{
  pwm=constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  pwm<<=1;   // pwm*2;

 switch(ch)
  {
    case 0:  OCR1B=pwm; break;  // out1
    case 1:  OCR1A=pwm; break;  // out2
    case 2:  OCR4C=pwm; break;  // out3
    case 3:  OCR4B=pwm; break;  // out4
    case 4:  OCR4A=pwm; break;  // out5
    case 5:  OCR3C=pwm; break;  // out6
    case 6:  OCR3B=pwm; break;  // out7
    case 7:  OCR3A=pwm; break;  // out8
  }
}

uint16_t APM_RC_APM2::InputCh(unsigned char ch)
{
  uint16_t result;
  uint16_t result2;

	if (_HIL_override[ch] != 0) {
		return _HIL_override[ch];
	}

  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
  result =  _PWM_RAW[ch]>>1;  // Because timer runs at 0.5us we need to do value/2
  result2 = _PWM_RAW[ch]>>1;
  if (result != result2)
    result =  _PWM_RAW[ch]>>1;   // if the results are different we make a third reading (this should be fine)

  // Limit values to a valid range
  result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  _radio_status=0; // Radio channel read
  return(result);
}

unsigned char APM_RC_APM2::GetState(void)
{
  return(_radio_status);
}

// InstantPWM is not implemented!

void APM_RC_APM2::Force_Out(void) { }
void APM_RC_APM2::Force_Out0_Out1(void) { }
void APM_RC_APM2::Force_Out2_Out3(void) { }
void APM_RC_APM2::Force_Out6_Out7(void) { }

/* ---------------- OUTPUT SPEED CONTROL ------------------ */
// Output rate options:
#define OUTPUT_SPEED_50HZ 0
#define OUTPUT_SPEED_200HZ 1
#define OUTPUT_SPEED_400HZ 2

void APM_RC_APM2::SetFastOutputChannels(uint32_t chmask)
{
    if ((chmask & ( MSK_CH_1 | MSK_CH_2 )) != 0)
        _set_speed_ch1_ch2(OUTPUT_SPEED_400HZ);

    if ((chmask & ( MSK_CH_3 | MSK_CH_4 | MSK_CH_5 )) != 0)
        _set_speed_ch3_ch4_ch5(OUTPUT_SPEED_400HZ);

    if ((chmask & ( MSK_CH_6 | MSK_CH_7 | MSK_CH_8 )) != 0)
        _set_speed_ch6_ch7_ch8(OUTPUT_SPEED_400HZ);
}

void APM_RC_APM2::_set_speed_ch1_ch2(uint8_t speed)
{
  switch(speed) {
  case OUTPUT_SPEED_400HZ:
    ICR1 = 5000;
    break;
  case OUTPUT_SPEED_200HZ:
    ICR1 = 10000;
    break;
  case OUTPUT_SPEED_50HZ:
  default:
    ICR1 = 40000;
    break;
  }
}

void APM_RC_APM2::_set_speed_ch3_ch4_ch5(uint8_t speed)
{
  switch(speed) {
  case OUTPUT_SPEED_400HZ:
    ICR4 = 5000;
    break;
  case OUTPUT_SPEED_200HZ:
    ICR4 = 10000;
    break;
  case OUTPUT_SPEED_50HZ:
  default:
    ICR4 = 40000;
    break;
  }

}

void APM_RC_APM2::_set_speed_ch6_ch7_ch8(uint8_t speed)
{
  switch(speed) {
  case OUTPUT_SPEED_400HZ:
    ICR3 = 5000;
    break;
  case OUTPUT_SPEED_200HZ:
    ICR3 = 10000;
    break;
  case OUTPUT_SPEED_50HZ:
  default:
    ICR3 = 40000;
    break;
  }

}

// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_RC_APM2::setHIL(int16_t v[NUM_CHANNELS])
{
	uint8_t sum = 0;
	for (unsigned char i=0; i<NUM_CHANNELS; i++) {
		if (v[i] != -1) {
			_HIL_override[i] = v[i];
		}
		if (_HIL_override[i] != 0) {
			sum++;
		}
	}
		if (sum == 0) {
			return 0;
		} else {
			return 1;
		}
	_radio_status = 1;
}

void APM_RC_APM2::clearOverride(void)
{
	for (unsigned char i=0; i<NUM_CHANNELS; i++) {
		_HIL_override[i] = 0;
	}
}

#endif
