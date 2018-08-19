/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/


#pragma GCC optimize ("O2")

#include "AP_HAL_F4Light.h"
#include "RCOutput.h"
#include <AP_Param_Helper/AP_Param_Helper.h>
#include <4way/serial_4way.h>

#include "GPIO.h"

using namespace F4Light;


// only one!
//#define  DEBUG_PWM 5 // motor 6
//#define DEBUG_INT 5


#define F4Light_OUT_CHANNELS 6 // motor's channels enabled by default


#ifndef SERVO_PIN_5
 #define SERVO_PIN_5 ((uint8_t)-1)
#endif

#ifndef SERVO_PIN_6
 #define SERVO_PIN_6 ((uint8_t)-1)
#endif

#ifndef SERVO_PIN_7
 #define SERVO_PIN_7 ((uint8_t)-1)
#endif

#ifndef SERVO_PIN_8
 #define SERVO_PIN_8 ((uint8_t)-1)
#endif

#ifndef SERVO_PIN_9
 #define SERVO_PIN_9 ((uint8_t)-1)
#endif

#ifndef SERVO_PIN_10
 #define SERVO_PIN_10 ((uint8_t)-1)
#endif

#ifndef SERVO_PIN_11
 #define SERVO_PIN_11 ((uint8_t)-1)
#endif



// #if FRAME_CONFIG == QUAD_FRAME // this is only QUAD layouts

// ArduCopter
static const uint8_t output_channels_arducopter[]= {  // pin assignment
    SERVO_PIN_1, //Timer3/3  - 1
    SERVO_PIN_2, //Timer3/4  - 2
    SERVO_PIN_3, //Timer2/3  - 3
    SERVO_PIN_4, //Timer2/2  - 4
    SERVO_PIN_5, 
    SERVO_PIN_6,

    // servo channels on input port
    SERVO_PIN_7, // PB15  CH2_IN - PPM2 
    SERVO_PIN_8, // PC6  CH3_IN UART6
    SERVO_PIN_9, // PC7  CH4_IN UART6
    SERVO_PIN_10, // PC8  CH5_IN i2c
    SERVO_PIN_11, // PC9  CH6_IN i2c
};


// mimics foreign layouts

// OpenPilot
static const uint8_t output_channels_openpilot[]= {  // pin assignment
    SERVO_PIN_2, //Timer3/4  - 2
    SERVO_PIN_4, //Timer2/2  - 4
    SERVO_PIN_1, //Timer3/3  - 1
    SERVO_PIN_3, //Timer2/3  - 3
    SERVO_PIN_5, //Timer2/1
    SERVO_PIN_6, //Timer2/0

    // servo channels on input port
    SERVO_PIN_7, // PB15  CH2_IN - PPM2 
    SERVO_PIN_8, // PC6  CH3_IN UART6
    SERVO_PIN_9, // PC7  CH4_IN UART6
    SERVO_PIN_10, // PC8  CH5_IN i2c
    SERVO_PIN_11, // PC9  CH6_IN i2c
};


// Cleanflight
static const uint8_t output_channels_cleanflight[]= {  // pin assignment
    SERVO_PIN_2, //Timer3/4  - 2
    SERVO_PIN_3, //Timer2/3  - 3
    SERVO_PIN_4, //Timer2/2  - 4
    SERVO_PIN_1, //Timer3/3  - 1
    SERVO_PIN_5, //Timer2/1
    SERVO_PIN_6, //Timer2/0

    // servo channels on input port
    SERVO_PIN_7, // PB15  CH2_IN - PPM2 
    SERVO_PIN_8, // PC6  CH3_IN UART6
    SERVO_PIN_9, // PC7  CH4_IN UART6
    SERVO_PIN_10, // PC8  CH5_IN i2c
    SERVO_PIN_11, // PC9  CH6_IN i2c
};

// Arducopter, shifted 2 pins right to use up to 2 servos
static const uint8_t output_channels_servo[]= {  // pin assignment
    SERVO_PIN_3, //Timer2/3  - 1
    SERVO_PIN_4, //Timer2/2  - 2
    SERVO_PIN_5, //Timer2/1  - 3
    SERVO_PIN_6, //Timer2/0  - 4
    SERVO_PIN_1, //Timer3/3    servo1
    SERVO_PIN_2, //Timer3/4    servo2

    // servo channels on input port
    SERVO_PIN_7, // PB15  CH2_IN - PPM2 
    SERVO_PIN_8, // PC6  CH3_IN UART6
    SERVO_PIN_9, // PC7  CH4_IN UART6
    SERVO_PIN_10, // PC8  CH5_IN i2c
    SERVO_PIN_11, // PC9  CH6_IN i2c
};



static const uint8_t * const revo_motor_map[]={
    output_channels_arducopter,
    output_channels_servo,
    output_channels_openpilot,
    output_channels_cleanflight,
};


// #endif

static const uint8_t *output_channels = output_channels_openpilot;  // current pin assignment

enum     BOARD_PWM_MODES RCOutput::_mode = BOARD_PWM_NORMAL;
bool     RCOutput::_once_mode = false;

uint16_t RCOutput::_period[F4Light_MAX_OUTPUT_CHANNELS] IN_CCM;
uint16_t RCOutput::_freq[F4Light_MAX_OUTPUT_CHANNELS] IN_CCM;
uint8_t  RCOutput::_initialized[F4Light_MAX_OUTPUT_CHANNELS] IN_CCM;
uint16_t RCOutput::_enabled_channels=0;
bool     RCOutput::_sbus_enabled=0;
bool     RCOutput::_corked=0;
uint8_t  RCOutput::_used_channels=0;

uint8_t RCOutput::_servo_mask=0;

uint32_t RCOutput::_timer2_preload;
uint16_t RCOutput::_timer3_preload;

uint8_t  RCOutput::_pwm_type=0;

const timer_dev* RCOutput::out_timers[16]  IN_CCM;
uint8_t          RCOutput::num_out_timers  IN_CCM;


#define PWM_TIMER_KHZ          2000  // 1000 in cleanflight
#define ONESHOT125_TIMER_KHZ   8000  // 8000 in cleanflight
#define ONESHOT42_TIMER_KHZ   28000  // 24000 in cleanflight
#define PWM_BRUSHED_TIMER_KHZ 16000  // 8000 in cleanflight

#define _BV(bit) (1U << (bit))

void RCOutput::init()
{
    memset(&_period[0], 0, sizeof(_period));
    memset(&_initialized[0], 0, sizeof(_initialized));

    _used_channels=0;
    
}


void RCOutput::do_4way_if(AP_HAL::UARTDriver* uart) {
    esc4wayInit(output_channels, F4Light_OUT_CHANNELS);
    esc4wayProcess(uart);
}


void RCOutput::lateInit(){ // 2nd stage with loaded parameters

    uint8_t map = hal_param_helper->_motor_layout;
    _servo_mask = hal_param_helper->_servo_mask;
    _pwm_type   = hal_param_helper->_pwm_type;
    
    if(map >= ARRAY_SIZE(revo_motor_map)) return; // don't initialize if parameter is wrong
    output_channels = revo_motor_map[map];
    
    InitPWM();
}

void RCOutput::InitPWM()
{
    for(uint8_t i = 0; i < F4Light_MAX_OUTPUT_CHANNELS; i++) {
        _freq[i] = 50;
    }
    fill_timers();
    _set_output_mode(MODE_PWM_NORMAL); // init timers
}



// not from _freq to take channel dependency
uint16_t RCOutput::get_freq(uint8_t ch) {
    if(ch >= F4Light_MAX_OUTPUT_CHANNELS) return 0;
    
    const timer_dev *dev = PIN_MAP[output_channels[ch]].timer_device;

    /* transform to period by inverse of _time_period(icr) */
    return (uint16_t)(dev->state->freq / timer_get_reload(dev));
}

// fill array of used timers
void RCOutput::fill_timers(){
    memset(out_timers, 0, sizeof(out_timers)); // clear it first
    num_out_timers=0;

    for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
        if (!(_enabled_channels & _BV(ch))) continue;      // not enabled

        const timer_dev *tim = PIN_MAP[output_channels[ch]].timer_device;

        bool add=true;
        for(uint8_t i =0; i<num_out_timers;i++){
            if(out_timers[i]==tim){
                add=false;
                break;
            }
        }
        if(add) out_timers[num_out_timers++] = tim;
    }
}


void RCOutput::_set_output_mode(enum RCOutput::output_mode mode) {
    
    uint32_t period=0;
    uint32_t freq;

    _once_mode=false;
    

    switch(_pwm_type) {
    case 0:
    default:
        switch(mode){
        case MODE_PWM_NORMAL:
            _mode=BOARD_PWM_NORMAL;
            break;

        case MODE_PWM_BRUSHED:
            _mode=BOARD_PWM_BRUSHED;
            break;    
        
        default:
        case MODE_PWM_ONESHOT:
            _mode=BOARD_PWM_ONESHOT;
            break;
        }  
        break;
          
    case 1:
        _mode=BOARD_PWM_ONESHOT;
        break;
        
    case 2:
        _mode=BOARD_PWM_ONESHOT125;
        break;
    
    case 3:
        _mode=BOARD_PWM_ONESHOT42;
        break;

    case 4:
        _mode=BOARD_PWM_PWM125;
        break;
    }
    
// TODO: remove hardwiring timers
// TODO: we should change mode only for channels with freq > 50Hz

    switch(_mode){

    case BOARD_PWM_NORMAL:
    default:
// output uses timers 2 & 3 so let init them for PWM mode
        period    = ((PWM_TIMER_KHZ*1000UL) / 50); // 50Hz by default - will be corrected late per-channel in init_channel()
        freq      = PWM_TIMER_KHZ;        // 2MHz 0.5us ticks - for 50..490Hz PWM
        break;

    case BOARD_PWM_ONESHOT: // same as PWM but with manual restarting
// output uses timers 2 & 3 so let init them for PWM mode
        period    = (uint32_t)-1; // max possible
        freq      = PWM_TIMER_KHZ;       // 2MHz 0.5us ticks - for 50..490Hz PWM
        _once_mode=true;
        break;

    case BOARD_PWM_ONESHOT125:
        period    = (uint32_t)-1; // max possible
        freq      = ONESHOT125_TIMER_KHZ;       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
//    at 16Mhz, period with 65536 will be 244Hz so even 16-bit timer will never overflows at 500Hz loop
        _once_mode=true;
        break;

   case BOARD_PWM_PWM125: // like Oneshot125 but PWM so not once
        period    = (uint32_t)-1; // max possible
        freq      = ONESHOT125_TIMER_KHZ;       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
//    at 16Mhz, period with 65536 will be 244Hz so even 16-bit timer will never overflows at 500Hz loop
        break;

    case BOARD_PWM_ONESHOT42:
        period    = (uint32_t)-1; // max possible
        freq      = ONESHOT42_TIMER_KHZ;       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
//    at 28Mhz, period with 65536 will be 427Hz so even 16-bit timer should not overflows at 500Hz loop, but very close to
        _once_mode=true;
        break;

    case BOARD_PWM_BRUSHED: 
                     // dev    period   freq, kHz
        period = 1000;
        freq   = PWM_BRUSHED_TIMER_KHZ;       // 16MHz  - 0..1 in 1000 steps
        break;
    }


#if 1
// correct code should init all timers used for outputs
        
        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
            const timer_dev *tim = PIN_MAP[output_channels[ch]].timer_device;
            tim->state->update=false;  // reset flag first
        }

        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
            if (!(_enabled_channels & _BV(ch))) continue;      // not enabled

            if(_freq[ch]>50){
                const timer_dev *tim = PIN_MAP[output_channels[ch]].timer_device;    
                tim->state->update=true; // set flag for update for needed timer
            }
        }

        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
            uint8_t pin = output_channels[ch];
    
            const timer_dev *tim = PIN_MAP[pin].timer_device;
            if(tim->state->update) {
                configTimeBase(tim, period,  freq);
                tim->state->update = false;    // only once
                if(_mode == BOARD_PWM_BRUSHED) tim->state->freq_scale=1;
            }
        }

#else // raw and dirty way

                             // dev    period   freq, kHz                             
    configTimeBase(TIMER2, period,  freq);       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
    configTimeBase(TIMER3, period,  freq);       // 16MHz 62.5ns ticks 

    if(_mode == BOARD_PWM_BRUSHED) {
        TIMER2->state->freq_scale=1;
        TIMER3->state->freq_scale=1;
    }

#endif

    init_channels();

#if 1 
    for(uint8_t i =0; i<num_out_timers;i++){
        timer_resume(out_timers[i]);
    }
#else
    timer_resume(TIMER2);
    timer_resume(TIMER3);
#endif
}

void RCOutput::_set_pin_output_mode(uint8_t ch) {
    
    uint32_t period=0;
    uint32_t freq;

    switch(_mode){

    case BOARD_PWM_NORMAL:
    default:
// output uses timers 2 & 3 so let init them for PWM mode
        period    = ((PWM_TIMER_KHZ*1000UL) / 50); // 50Hz by default - will be corrected late per-channel in init_channel()
        freq      = PWM_TIMER_KHZ;        // 2MHz 0.5us ticks - for 50..490Hz PWM
        break;

    case BOARD_PWM_ONESHOT: // same as PWM but with manual restarting
// output uses timers 2 & 3 so let init them for PWM mode
        period    = (uint32_t)-1; // max possible
        freq      = PWM_TIMER_KHZ;       // 2MHz 0.5us ticks - for 50..490Hz PWM
        break;

    case BOARD_PWM_ONESHOT125:
        period    = (uint32_t)-1; // max possible
        freq      = ONESHOT125_TIMER_KHZ;       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
//    at 16Mhz, period with 65536 will be 244Hz so even 16-bit timer will never overflows at 500Hz loop
        break;

   case BOARD_PWM_PWM125: // like Oneshot125 but PWM so not once
        period    = (uint32_t)-1; // max possible
        freq      = ONESHOT125_TIMER_KHZ;       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
//    at 16Mhz, period with 65536 will be 244Hz so even 16-bit timer will never overflows at 500Hz loop
        break;

    case BOARD_PWM_ONESHOT42:
        period    = (uint32_t)-1; // max possible
        freq      = ONESHOT42_TIMER_KHZ;       // 16MHz 62.5ns ticks - for 125uS..490Hz OneShot125
//    at 28Mhz, period with 65536 will be 427Hz so even 16-bit timer should not overflows at 500Hz loop, but very close to
        break;

    case BOARD_PWM_BRUSHED: 
                     // dev    period   freq, kHz
        period = 1000;
        freq   = PWM_BRUSHED_TIMER_KHZ;       // 16MHz  - 0..1 in 1000 steps
        break;
    }


    uint8_t pin = output_channels[ch];
    
    const timer_dev *tim = PIN_MAP[pin].timer_device;
    configTimeBase(tim, period,  freq);
    if(_mode == BOARD_PWM_BRUSHED) tim->state->freq_scale=1;
}




bool RCOutput::is_servo_enabled(uint8_t ch){
    if(ch>=F4Light_OUT_CHANNELS){ // servos
        uint8_t sn = ch - F4Light_OUT_CHANNELS;
        if(!(_servo_mask & (1<<sn)) ) return false;
    }

    return true;
}

// for Oneshot125 
// [1000;2000] => [125;250]
// so frequency of timers should be 8 times more - 16MHz, but timers on 84MHz can give only 16.8MHz

// channels 1&2, 3&4&5&6 can has a different rates
void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    uint32_t mask=1;
    
    uint16_t freq = freq_hz;
    
    for(uint8_t i=0; i< F4Light_MAX_OUTPUT_CHANNELS; i++) { // кто последний тот и папа
        if(chmask & mask) {
            if(!(_enabled_channels & mask) ) return;      // not enabled

// for true one-shot        if(_once_mode && freq_hz>50) continue; // no frequency in OneShoot modes
            _freq[i] = freq_hz;
            
            if(_once_mode && freq_hz>50) freq = freq_hz / 2; // divide frequency by 2 in OneShoot modes
            const uint8_t pin = output_channels[i];
            const timer_dev *dev = PIN_MAP[pin].timer_device;
            timer_set_reload(dev,  _timer_period(freq, dev)); 
        }
        mask <<= 1;
    }
}

void RCOutput::init_channel(uint8_t ch){
    if(ch>=F4Light_MAX_OUTPUT_CHANNELS) return;

    uint8_t pin = output_channels[ch];
    if (pin >= BOARD_NR_GPIO_PINS) return;

    const stm32_pin_info &p = PIN_MAP[pin];
    const timer_dev *dev = p.timer_device;

    timer_set_mode(   dev, p.timer_channel, TIMER_PWM);

    uint16_t freq = _freq[ch];
    if(_once_mode && freq>50) freq/=2;
    timer_set_reload(dev,  _timer_period(freq, dev));
    if(_once_mode) {
        timer_set_compare(dev, p.timer_channel, 0); // to prevent outputs in case of timer overflow
    }
}


void RCOutput::init_channels(){
    for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
        init_channel(ch);
    }
}

/* constrain pwm to be between min and max pulsewidth */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void RCOutput::set_pwm(uint8_t ch, uint16_t pwm){
    
    if(ch>=F4Light_MAX_OUTPUT_CHANNELS) return;

    if (!(_enabled_channels & _BV(ch))) return;      // not enabled

    if(!is_servo_enabled(ch)) return; // disabled servo

    uint8_t pin = output_channels[ch];
    if (pin >= BOARD_NR_GPIO_PINS) return;

    switch(_mode){
    case BOARD_PWM_BRUSHED:
        pwm -= 1000; // move from 1000..2000 to 0..1000
        break;

    case BOARD_PWM_ONESHOT42: // works at single freq
    case BOARD_PWM_ONESHOT125:
        break;

    default:
        pwm <<= 1; // frequency of timers 2MHz
        break;
    }

    const stm32_pin_info &p = PIN_MAP[pin];
    const timer_dev *dev = p.timer_device;

    pwm *= dev->state->freq_scale; // take into account the inaccuracy of setting the timer frequency for small prescalers
    timer_set_compare(dev, p.timer_channel, pwm); 
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if(ch>=F4Light_MAX_OUTPUT_CHANNELS) return;

    if(_used_channels<ch) _used_channels=ch+1;
    
    uint16_t pwm = constrain_period(period_us);

    if(_period[ch]==pwm) return; // already so

    _period[ch]=pwm;
    
    if(_corked) return;

    set_pwm(ch, pwm);
}



void RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    if(ch>=F4Light_MAX_OUTPUT_CHANNELS) return;

    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    if(ch>=F4Light_MAX_OUTPUT_CHANNELS) return RC_INPUT_MIN_PULSEWIDTH;

    return _period[ch];
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{// here we don't need to limit channel count - all unsupported will be read as RC_INPUT_MIN_PULSEWIDT
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (ch >= F4Light_MAX_OUTPUT_CHANNELS) 
        return;

    if(_enabled_channels & (1U<<ch) ) return; // already OK

    if(!is_servo_enabled(ch)) return; // disabled servo
            
    _enabled_channels |= (1U<<ch);
    if (_period[ch] == PWM_IGNORE_THIS_CHANNEL) {
        _period[ch] = 0;
    }
    
    if(!_initialized[ch]) {
    
        uint8_t pin = output_channels[ch];

        _set_pin_output_mode(ch);
        GPIO::_pinMode(pin, PWM);
        init_channel(ch);
        
        const timer_dev *dev = PIN_MAP[pin].timer_device;

        timer_resume(dev);
    
        _initialized[ch]=true;
    }

    fill_timers(); // re-calculate list of used timers
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (ch >= F4Light_MAX_OUTPUT_CHANNELS) {
        return;
    }

    if(!is_servo_enabled(ch)) return; // disabled servo

    _enabled_channels &= ~(1U<<ch);
    _period[ch] = PWM_IGNORE_THIS_CHANNEL;
    

    uint8_t pin = output_channels[ch];
    if (pin >= BOARD_NR_GPIO_PINS) return;

    GPIO::_pinMode(pin, OUTPUT);
    GPIO::_write(pin, 0);

    fill_timers(); // re-calculate list of used timers
}


void RCOutput::push()
{
#ifdef DEBUG_PWM
    uint8_t spin = output_channels[DEBUG_PWM]; // motor 6 as strobe
    GPIO::_pinMode(spin, OUTPUT);
    GPIO::_write(spin, 1);
#endif
    
    for (uint16_t ch = 0; ch < _used_channels; ch++) {
        set_pwm(ch, _period[ch]);
    }

    if(_once_mode){   // generate timer's update on ALL used pins, but only once per timer

#if 1
        for(uint8_t i =0; i<num_out_timers;i++){
            timer_generate_update(out_timers[i]);
        }

        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
            const stm32_pin_info &p = PIN_MAP[output_channels[ch]];
            timer_set_compare(p.timer_device, p.timer_channel, 0); // to prevent outputs  in case of timer overflows
        }

#else

        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {    
            const timer_dev *tim = PIN_MAP[output_channels[ch]].timer_device;
            tim->state->update=false;  // reset flag first
        }

        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
            if (!(_enabled_channels & _BV(ch))) continue;      // not enabled
            const timer_dev *tim = PIN_MAP[output_channels[ch]].timer_device;
    
            tim->state->update=true; // set flag for update for needed timer
        }

        for (uint16_t ch = 0; ch < F4Light_OUT_CHANNELS; ch++) {
            const timer_dev *tim = PIN_MAP[output_channels[ch]].timer_device;
            if(tim->state->update) {
                timer_generate_update(tim);
                tim->state->update = false;    // only once
            }
        }
#endif
    }

    _corked = false;
        
#ifdef DEBUG_PWM
    GPIO::_write(spin, 0);
#endif
}

