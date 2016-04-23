
#include "RCOutput.h"

extern const AP_HAL::HAL& hal;
using namespace REVOMINI;

static int analogOutPin[REVOMINI_MAX_OUTPUT_CHANNELS];


static inline long map(long value, long fromStart, long fromEnd,
                long toStart, long toEnd) {
    return (value - fromStart) * (toEnd - toStart) / (fromEnd - fromStart) +
        toStart;
}

void REVOMINIRCOutput::InitDefaultPWM(void)
{
	output_channel_ch1=46; //Timer3/3
	output_channel_ch2=45; //Timer3/4
	output_channel_ch3=50; //Timer9/2
	output_channel_ch4=49; //Timer2/2
	output_channel_ch5=48; //Timer5/2
	output_channel_ch6=47; //Timer5/1
}

void REVOMINIRCOutput::init()
{
    InitDefaultPWM();

    InitPWM();
}


void REVOMINIRCOutput::InitPWM()
{
  analogOutPin[MOTORID1] = output_channel_ch1;
  analogOutPin[MOTORID2] = output_channel_ch2;
  analogOutPin[MOTORID3] = output_channel_ch3;
  analogOutPin[MOTORID4] = output_channel_ch4;
  analogOutPin[MOTORID5] = output_channel_ch5;
  analogOutPin[MOTORID6] = output_channel_ch6;
//  analogOutPin[MOTORID7] = output_channel_ch7;
//  analogOutPin[MOTORID8] = output_channel_ch8;

  for(int8_t i = MOTORID1; i <= MOTORID6; i++)
      {
      hal.gpio->pinMode(analogOutPin[i],PWM);
      }
}

#define _BV(bit) (1 << (bit))

void REVOMINIRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
    {
    uint32_t icr = _timer_period(freq_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2) )) != 0) {
	TIM3->ARR = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) |  _BV(CH_5) | _BV(CH_6))) != 0) {
	TIM2->ARR = icr;
    }

}

uint16_t REVOMINIRCOutput::get_freq(uint8_t ch) {
    uint32_t icr;
    switch (ch) {
    case CH_1:
    case CH_2:
        icr = (TIMER3->regs)->ARR;
        break;
    case CH_3:
    case CH_4:
    case CH_5:
    case CH_6:
        icr = (TIMER2->regs)->ARR;
        break;
    default:
        return 0;
    }
    /* transform to period by inverse of _time_period(icr). */
    return (uint16_t)(2000000UL / icr);
}

void REVOMINIRCOutput::enable_ch(uint8_t ch)
{}

void REVOMINIRCOutput::enable_mask(uint32_t chmask)
{}

void REVOMINIRCOutput::disable_ch(uint8_t ch)
{}

void REVOMINIRCOutput::disable_mask(uint32_t chmask)
{}

/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void REVOMINIRCOutput::write(uint8_t ch, uint16_t period_us)
{
    uint16_t pwm = constrain_period(period_us) << 1;


    uint8_t pin = analogOutPin[ch];
    timer_dev *dev = PIN_MAP[pin].timer_device;

    if (pin >= BOARD_NR_GPIO_PINS || dev == NULL || dev->type == TIMER_BASIC)
	{
	return;
	}

    timer_set_compare(dev, PIN_MAP[pin].timer_channel, pwm);
    TIM_Cmd(dev->regs, ENABLE);
}


void REVOMINIRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t REVOMINIRCOutput::read(uint8_t ch)
{

    uint16_t pin = analogOutPin[ch];
    timer_dev *dev = PIN_MAP[pin].timer_device;
    if (pin >= BOARD_NR_GPIO_PINS || dev == NULL || dev->type == TIMER_BASIC)
	{
	return RC_INPUT_MIN_PULSEWIDTH;
	}

    uint16_t pwm;
    pwm =     timer_get_compare(dev, PIN_MAP[pin].timer_channel);
    return pwm >> 1;
}

void REVOMINIRCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

uint32_t REVOMINIRCOutput::_timer_period(uint16_t speed_hz) {
    return (uint32_t)(2000000UL / speed_hz);
}
