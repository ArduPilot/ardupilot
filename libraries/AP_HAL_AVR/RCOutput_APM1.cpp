#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1

#include <avr/interrupt.h>

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_AVR.h"
#include "RCOutput.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* No init argument required */
void APM1RCOutput::init(void* machtnichts) {
    // --------------------- TIMER1: CH_3, CH_4, and CH_10 ---------------
    hal.gpio->pinMode(11,HAL_GPIO_OUTPUT); // CH_10 (PB5/OC1A)
    hal.gpio->pinMode(12,HAL_GPIO_OUTPUT); // CH_3 (PB6/OC1B)
    hal.gpio->pinMode(13,HAL_GPIO_OUTPUT); // CH_4 (PB7/OC1C)

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
    // CS11: prescale by 8 => 0.5us tick
    TCCR1A =((1<<WGM11));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = 40000; // 0.5us tick => 50hz freq
    OCR1A = 0xFFFF; // Init OCR registers to nil output signal
    OCR1B = 0xFFFF;
    OCR1C = 0xFFFF;
    
    //--------------- TIMER3: CH_7, CH_8, and CH_11 ---------------------
    hal.gpio->pinMode(5,HAL_GPIO_OUTPUT); // CH_11 (PE3/OC3A)
    hal.gpio->pinMode(2,HAL_GPIO_OUTPUT); // CH_8 (PE4/OC3B)
    hal.gpio->pinMode(3,HAL_GPIO_OUTPUT); // CH_7 (PE5/OC3C)

    // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR3
    // CS31: prescale by 8 => 0.5us tick
    TCCR3A =((1<<WGM31));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    OCR3A = 0xFFFF; // Init OCR registers to nil output signal
    OCR3B = 0xFFFF;
    OCR3C = 0xFFFF;
    ICR3 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER4: CH_6 and CH_5  ----------------------------
    // NB TIMER4 is shared with PPM input from RCInput_APM1.cpp
    // The TIMER4 registers are assumed to be setup already.
    hal.gpio->pinMode(7,HAL_GPIO_OUTPUT); // CH_5 (PH4/OC4B)
    hal.gpio->pinMode(8,HAL_GPIO_OUTPUT); // CH_6 (PH5/OC4C)


    //--------------- TIMER5: CH_1, CH_2 and CH_9 -----------------------
    hal.gpio->pinMode(46, HAL_GPIO_OUTPUT); // CH_9 (PL3/OC5A)
    hal.gpio->pinMode(45, HAL_GPIO_OUTPUT); // CH_1 (PL4/OC5B)
    hal.gpio->pinMode(44, HAL_GPIO_OUTPUT); // CH_2 (PL5/OC5C)
    
    // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR5
    // CS51: prescale by 8 => 0.5us tick
    TCCR5A =((1<<WGM51));
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    OCR5A = 0xFFFF; // Init OCR registers to nil output signal
    OCR5B = 0xFFFF;
    OCR5C = 0xFFFF;
    ICR5 = 40000; // 0.5us tick => 50hz freq
}

/* Output freq (1/period) control */
void APM1RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    uint16_t icr = _timer_period(freq_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2) | _BV(CH_9))) != 0) {
        ICR5 = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) | _BV(CH_10))) != 0) {
        ICR1 = icr;
    }

    if ((chmask & ( _BV(CH_7) | _BV(CH_8) | _BV(CH_11))) != 0) {
        ICR3 = icr;
    }
    /* No change permitted for CH_5 and CH_6 - that ICR register is
     * shared with the input capture for RCInput */
}

uint16_t APM1RCOutput::get_freq(uint8_t ch) {
    uint16_t icr;
    switch (ch) {
        case CH_3:
        case CH_4:
        case CH_10:
            icr = ICR1;
            break;
        /* CH_5 and CH_6 share TIMER4 with input capture.
         * The period is specified in OCR4A rather than the ICR. */
        case CH_5:
        case CH_6:
            icr = OCR4A;
            break; 
        case CH_7:
        case CH_8:
        case CH_11:
            icr = ICR3;
            break;
        case CH_1:
        case CH_2:
        case CH_9:
            icr = ICR5;
            break;
        default:
            return 0;
    }
    /* transform to period by inverse of _time_period(icr). */
    return (2000000UL / icr);
}

/* Output active/highZ control, either by single channel at a time
 * or a mask of channels */
void APM1RCOutput::enable_ch(uint8_t ch) {
    switch(ch) {
    case 0:  TCCR5A |= (1<<COM5B1); break;  // CH_1 : OC5B
    case 1:  TCCR5A |= (1<<COM5C1); break;  // CH_2 : OC5C
    case 2:  TCCR1A |= (1<<COM1B1); break;  // CH_3 : OC1B
    case 3:  TCCR1A |= (1<<COM1C1); break;  // CH_4 : OC1C
    case 4:  TCCR4A |= (1<<COM4C1); break;  // CH_5 : OC4C
    case 5:  TCCR4A |= (1<<COM4B1); break;  // CH_6 : OC4B
    case 6:  TCCR3A |= (1<<COM3C1); break;  // CH_7 : OC3C
    case 7:  TCCR3A |= (1<<COM3B1); break;  // CH_8 : OC3B
    case 8:  TCCR5A |= (1<<COM5A1); break;  // CH_9 : OC5A
    case 9:  TCCR1A |= (1<<COM1A1); break;  // CH_10: OC1A
    case 10: TCCR3A |= (1<<COM3A1); break;  // CH_11: OC3A
    }
}

void APM1RCOutput::disable_ch(uint8_t ch) {
    switch(ch) {
    case 0:  TCCR5A &= ~(1<<COM5B1); break;  // CH_1 : OC5B
    case 1:  TCCR5A &= ~(1<<COM5C1); break;  // CH_2 : OC5C
    case 2:  TCCR1A &= ~(1<<COM1B1); break;  // CH_3 : OC1B
    case 3:  TCCR1A &= ~(1<<COM1C1); break;  // CH_4 : OC1C
    case 4:  TCCR4A &= ~(1<<COM4C1); break;  // CH_5 : OC4C
    case 5:  TCCR4A &= ~(1<<COM4B1); break;  // CH_6 : OC4B
    case 6:  TCCR3A &= ~(1<<COM3C1); break;  // CH_7 : OC3C
    case 7:  TCCR3A &= ~(1<<COM3B1); break;  // CH_8 : OC3B
    case 8:  TCCR5A &= ~(1<<COM5A1); break;  // CH_9 : OC5A
    case 9:  TCCR1A &= ~(1<<COM1A1); break;  // CH_10: OC1A
    case 10: TCCR3A &= ~(1<<COM3A1); break;  // CH_11: OC3A
    }
}

/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_OUTPUT_MAX_PULSEWIDTH) return RC_OUTPUT_MAX_PULSEWIDTH;
    if (p < RC_OUTPUT_MIN_PULSEWIDTH) return RC_OUTPUT_MIN_PULSEWIDTH;
    return p;
}

/* Output, either single channel or bulk array of channels */
void APM1RCOutput::write(uint8_t ch, uint16_t period_us) {
    /* constrain, then scale from 1us resolution (input units)
     * to 0.5us (timer units) */
    uint16_t pwm = constrain_period(period_us) << 1;
    switch(ch)
    {
    case 0:  OCR5B=pwm; break;  //ch1
    case 1:  OCR5C=pwm; break;  //ch2
    case 2:  OCR1B=pwm; break;  //ch3
    case 3:  OCR1C=pwm; break;  //ch4
    case 4:  OCR4C=pwm; break;  //ch5
    case 5:  OCR4B=pwm; break;  //ch6
    case 6:  OCR3C=pwm; break;  //ch7
    case 7:  OCR3B=pwm; break;  //ch8
    case 8:  OCR5A=pwm; break;  //ch9,  PL3
    case 9:  OCR1A=pwm; break;  //ch10, PB5
    case 10: OCR3A=pwm; break;  //ch11, PE3
    }
}

/* Read back current output state, as either single channel or
 * array of channels. */
uint16_t APM1RCOutput::read(uint8_t ch) {
    uint16_t pwm=0;
    switch(ch) {
    case 0:  pwm=OCR5B; break;      //ch1
    case 1:  pwm=OCR5C; break;      //ch2
    case 2:  pwm=OCR1B; break;      //ch3
    case 3:  pwm=OCR1C; break;      //ch4
    case 4:  pwm=OCR4C; break;      //ch5
    case 5:  pwm=OCR4B; break;      //ch6
    case 6:  pwm=OCR3C; break;      //ch7
    case 7:  pwm=OCR3B; break;      //ch8
    case 8:  pwm=OCR5A; break;      //ch9,  PL3
    case 9:  pwm=OCR1A; break;      //ch10, PB5
    case 10: pwm=OCR3A; break;      //ch11, PE3
    }
    /* scale from 0.5us resolution (timer units) to 1us units */
    return pwm>>1;
}

void APM1RCOutput::read(uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

uint16_t APM1RCOutput::_timer_period(uint16_t speed_hz) {
    return 2000000UL / speed_hz;
}
#endif
