/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/

#pragma GCC optimize ("O2")

#include "gpio_hal.h"
#include <boards.h>

#include <exti.h>

#include "GPIO.h"
#include "Scheduler.h"
#include "RCOutput.h"



using namespace F4Light;

void GPIO::_pinMode(uint8_t pin, uint8_t mode)
{
    gpio_pin_mode outputMode;
    bool pwm = false;

    switch(mode) { // modes defined to be compatible so no transcode required
    case OUTPUT:
    case OUTPUT_OPEN_DRAIN:
    case INPUT:
//    case INPUT_FLOATING: synonim and cause doubled 'case'
    case INPUT_ANALOG:
    case INPUT_PULLUP:
    case INPUT_PULLDOWN:
        outputMode = (gpio_pin_mode)mode;
        break;

    case PWM:
        outputMode = GPIO_AF_OUTPUT_PP;
        pwm = true;
        break;

    case PWM_OPEN_DRAIN:
        outputMode = GPIO_AF_OUTPUT_OD;
        pwm = true;
        break;

    default:
        assert_param(0);
        return;
    }


    const stm32_pin_info &p = PIN_MAP[pin];

    const gpio_dev* dev =     p.gpio_device;
    uint8_t bit =             p.gpio_bit;
    const timer_dev * timer = p.timer_device;

    gpio_set_mode(dev, bit, outputMode);

    if (pwm && timer != NULL) {    
        gpio_set_speed(dev, bit, GPIO_speed_25MHz);  // cleanflight sets 2MHz
	gpio_set_af_mode(dev, bit, timer->af);
	timer_set_mode(timer, p.timer_channel, TIMER_PWM); // init in setupTimers()
    } else {
        gpio_set_af_mode(dev, bit, 0); // reset
    }
}


void GPIO::pinMode(uint8_t pin, uint8_t output){

    if ((pin >= BOARD_NR_GPIO_PINS))   return;

    _pinMode(pin, output);
}


uint8_t GPIO::read(uint8_t pin) {
    if (pin >= BOARD_NR_GPIO_PINS)     return 0;

    return _read(pin);
}


void GPIO::write(uint8_t pin, uint8_t value) {
    if ((pin >= BOARD_NR_GPIO_PINS))   return;

#ifdef BUZZER_PWM_HZ // passive buzzer

// AP_Notify Buzzer.cpp don't supports passive buzzer so we need a small hack
    if(pin == BOARD_BUZZER_PIN){
        if(value == HAL_BUZZER_ON){
            const stm32_pin_info &p = PIN_MAP[pin];
            const timer_dev *dev = p.timer_device;
            if(dev->state->freq==0) {
                configTimeBase(dev, 0,  BUZZER_PWM_HZ * 10); // it should be personal timer
            }
            _pinMode(pin, PWM);
            uint32_t n = RCOutput::_timer_period(BUZZER_PWM_HZ, dev);
            timer_set_reload(dev, n);
            timer_set_compare(dev, p.timer_channel, n/2);
            return;
        } else {
            _pinMode(pin, OUTPUT); // to disable, just change mode
        }
    }
#endif

    _write(pin, value);
}


void GPIO::toggle(uint8_t pin)
{
    if ((pin >= BOARD_NR_GPIO_PINS))  return;
    
    _toggle(pin);
}


/* Interrupt interface: */
bool GPIO::_attach_interrupt(uint8_t pin, Handler p, uint8_t mode, uint8_t priority)
{
    if ( (pin >= BOARD_NR_GPIO_PINS) || !p) return false;

    const stm32_pin_info &pp = PIN_MAP[pin];
    
    exti_attach_interrupt_pri((afio_exti_num)(pp.gpio_bit),
                           gpio_exti_port(pp.gpio_device),
                           p, exti_out_mode((ExtIntTriggerMode)mode),
                           priority);

    return true;
}

void GPIO::detach_interrupt(uint8_t pin)
{
    if ( pin >= BOARD_NR_GPIO_PINS) return;

    exti_detach_interrupt((afio_exti_num)(PIN_MAP[pin].gpio_bit));
}



/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin) {

    if ((pin >= BOARD_NR_GPIO_PINS)) return NULL;

    return  get_channel(pin); 
}


void DigitalSource::mode(uint8_t md)
{
    gpio_pin_mode outputMode;

    switch(md) {
    case OUTPUT:
    case OUTPUT_OPEN_DRAIN:
    case INPUT:
//    case INPUT_FLOATING:
    case INPUT_ANALOG:
    case INPUT_PULLUP:
    case INPUT_PULLDOWN:
        outputMode = (gpio_pin_mode)md;
        break;

    // no PWM via this interface!
    default:
        assert_param(0);
        return;
    }

    gpio_set_mode( _device, _bit, outputMode);
    gpio_set_speed(_device, _bit, GPIO_speed_100MHz); // to use as CS
}

void digitalWrite(uint8_t pin, uint8_t value) { F4Light::GPIO::_write(pin, value); }
uint8_t digitalRead(uint8_t pin) { return F4Light::GPIO::_read(pin); }

void digitalToggle(uint8_t pin) { return F4Light::GPIO::_toggle(pin); }
