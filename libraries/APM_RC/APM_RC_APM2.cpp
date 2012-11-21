/*
 *       APM_RC_APM2.cpp - Radio Control Library for Ardupilot Mega 2.0. Arduino
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       RC Input : PPM signal on IC4 pin
 *       RC Output : 11 Servo outputs (standard 20ms frame)
 *
 *       Methods:
 *               Init() : Initialization of interrupts an Timers
 *               OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
 *               InputCh(ch) : Read a channel input value.  ch=0..7
 *               GetState() : Returns the state of the input. 1 => New radio frame to process
 *                            Automatically resets when we call InputCh to read channels
 *
 */
#include "APM_RC_APM2.h"

#include <avr/interrupt.h>
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(DESKTOP_BUILD)
 # error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint16_t APM_RC_APM2::_PWM_RAW[NUM_CHANNELS] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile uint8_t APM_RC_APM2::_radio_status=0;

/****************************************************
*   Input Capture Interrupt ICP5 => PPM signal read
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
        // pass through values if at least a minimum number of channels received
        if( frame_idx >= MIN_CHANNELS ) {
            _radio_status = 1;
            _ppm_count++;
        }
        frame_idx=0;
    } else {
        // Save pulse into _PWM_RAW array.
        if ( frame_idx < NUM_CHANNELS ) {
            _PWM_RAW[ frame_idx++ ] = pwidth;
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
    digitalWrite(12,HIGH);  // pulling high before changing to output avoids a momentary drop of the pin to low because the ESCs have a pull-down resistor it seems
    digitalWrite(11,HIGH);
    pinMode(12,OUTPUT); // OUT1 (PB6/OC1B)
    pinMode(11,OUTPUT); // OUT2 (PB5/OC1A)
    digitalWrite(12,HIGH);  // pulling high before changing to output avoids a momentary drop of the pin to low because the ESCs have a pull-down resistor it seems
    digitalWrite(11,HIGH);

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
    // CS11: prescale by 8 => 0.5us tick
    TCCR1A =((1<<WGM11));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = 40000; // 0.5us tick => 50hz freq
    OCR1A = 0xFFFF; // Init OCR registers to nil output signal
    OCR1B = 0xFFFF;

    // --------------- TIMER4: OUT3, OUT4, and OUT5 ---------------------
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(6,HIGH);
    pinMode(8,OUTPUT); // OUT3 (PH5/OC4C)
    pinMode(7,OUTPUT); // OUT4 (PH4/OC4B)
    pinMode(6,OUTPUT); // OUT5 (PH3/OC4A)
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(6,HIGH);

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR4.
    // CS41: prescale by 8 => 0.5us tick
    TCCR4A =((1<<WGM41));
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    OCR4A = 0xFFFF; // Init OCR registers to nil output signal
    OCR4B = 0xFFFF;
    OCR4C = 0xFFFF;
    ICR4 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER3: OUT6, OUT7, and OUT8 ----------------------
    digitalWrite(3,HIGH);
    digitalWrite(2,HIGH);
    digitalWrite(5,HIGH);
    pinMode(3,OUTPUT); // OUT6 (PE5/OC3C)
    pinMode(2,OUTPUT); // OUT7 (PE4/OC3B)
    pinMode(5,OUTPUT); // OUT8 (PE3/OC3A)
    digitalWrite(3,HIGH);
    digitalWrite(2,HIGH);
    digitalWrite(5,HIGH);

    // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR3
    // CS31: prescale by 8 => 0.5us tick
    TCCR3A =((1<<WGM31));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    OCR3A = 0xFFFF; // Init OCR registers to nil output signal
    OCR3B = 0xFFFF;
    OCR3C = 0xFFFF;
    ICR3 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER5: PPM INPUT, OUT10, and OUT11 ---------------
    // Init PPM input on Timer 5
    pinMode(48, INPUT); // PPM Input (PL1/ICP5)

    digitalWrite(45,HIGH);
    digitalWrite(44,HIGH);
    pinMode(45, OUTPUT); // OUT10 (PL4/OC5B)
    pinMode(44, OUTPUT); // OUT11 (PL5/OC5C)
    digitalWrite(45,HIGH);
    digitalWrite(44,HIGH);

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
    pwm<<=1; // pwm*2;

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
    case 9:  OCR5B=pwm; break;  // out10
    case 10: OCR5C=pwm; break;  // out11
    }
}

uint16_t APM_RC_APM2::OutputCh_current(uint8_t ch)
{
    uint16_t pwm=0;
    switch(ch) {
    case 0:  pwm=OCR1B; break;      // out1
    case 1:  pwm=OCR1A; break;      // out2
    case 2:  pwm=OCR4C; break;      // out3
    case 3:  pwm=OCR4B; break;      // out4
    case 4:  pwm=OCR4A; break;      // out5
    case 5:  pwm=OCR3C; break;      // out6
    case 6:  pwm=OCR3B; break;      // out7
    case 7:  pwm=OCR3A; break;      // out8
    case 9:  pwm=OCR5B; break;      // out10
    case 10: pwm=OCR5C; break;      // out11
    }
    return pwm>>1;
}

void APM_RC_APM2::enable_out(uint8_t ch)
{
    switch(ch) {
    case 0: TCCR1A |= (1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A |= (1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A |= (1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A |= (1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A |= (1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A |= (1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A |= (1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A |= (1<<COM3A1); break; // CH_8 : OC3A
    case 9: TCCR5A |= (1<<COM5B1); break; // CH_10 : OC5B
    case 10: TCCR5A |= (1<<COM5C1); break; // CH_11 : OC5C
    }
}

void APM_RC_APM2::disable_out(uint8_t ch)
{
    switch(ch) {
    case 0: TCCR1A &= ~(1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A &= ~(1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A &= ~(1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A &= ~(1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A &= ~(1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A &= ~(1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A &= ~(1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A &= ~(1<<COM3A1); break; // CH_8 : OC3A
    case 9: TCCR5A &= ~(1<<COM5B1); break; // CH_10 : OC5B
    case 10: TCCR5A &= ~(1<<COM5C1); break; // CH_11 : OC5C
    }
}

uint16_t APM_RC_APM2::InputCh(unsigned char ch)
{
    uint16_t result;

    if (_HIL_override[ch] != 0) {
        return _HIL_override[ch];
    }

    // We need to block ICP5 interrupts during the read of 16 bit PWM values
    uint8_t _timsk5 = TIMSK5;
    TIMSK5 &= ~(1<<ICIE5);

    // value
    result = _PWM_RAW[ch];

    // Enable ICP5 interrupt if previously active
    TIMSK5 = _timsk5;
    
    // Because timer runs at 0.5us we need to do value/2
    result >>= 1;

    // Limit values to a valid range
    result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
    _radio_status = 0;     // Radio channel read
    return result;
}

unsigned char APM_RC_APM2::GetState(void)
{
    return(_radio_status);
}

// InstantPWM is not implemented!

void APM_RC_APM2::Force_Out(void) {
}
void APM_RC_APM2::Force_Out0_Out1(void) {
}
void APM_RC_APM2::Force_Out2_Out3(void) {
}
void APM_RC_APM2::Force_Out6_Out7(void) {
}

/* ---------------- OUTPUT SPEED CONTROL ------------------ */

void APM_RC_APM2::SetFastOutputChannels(uint32_t chmask, uint16_t speed_hz)
{
    uint16_t icr = _map_speed(speed_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2))) != 0) {
        ICR1 = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) | _BV(CH_5))) != 0) {
        ICR4 = icr;
    }

    if ((chmask & ( _BV(CH_6) | _BV(CH_7) | _BV(CH_8))) != 0) {
        ICR3 = icr;
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
    _radio_status = 1;
    _last_update = millis();
    if (sum == 0) {
        return 0;
    } else {
        return 1;
    }
}

void APM_RC_APM2::clearOverride(void)
{
    for (unsigned char i=0; i<NUM_CHANNELS; i++) {
        _HIL_override[i] = 0;
    }
}

#endif
