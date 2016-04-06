#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>


// Constructors ////////////////////////////////////////////////////////////////
using namespace AP_HAL;
using namespace REVOMINI;


extern const AP_HAL::HAL& hal;

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
// PATCH FOR FAILSAFE AND FRSKY
#define MINOFFWIDTH 1000
#define MAXOFFWIDTH 22000

#define MINCHECK 900
#define MAXCHECK 2100

/* private variables to communicate with input capture isr */
volatile uint16_t REVOMINIRCInput::_pulse_capt[REVOMINI_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint32_t REVOMINIRCInput::_last_pulse[REVOMINI_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint8_t  REVOMINIRCInput::_valid_channels = 0;
volatile uint64_t REVOMINIRCInput::_timestamp_last_signal;

volatile unsigned char radio_status_rc = 0;
volatile unsigned char sync = 0;
volatile unsigned int currentChannel = 0;

unsigned int uiRcErrCnt1 = 0;
unsigned int uiRcErrCnt2 = 0;
unsigned int uiRcErrCnt3 = 0;

typedef struct
    {
    byte edge;
    unsigned long riseTime;
    unsigned long fallTime;
    unsigned int lastGoodWidth;
    } tPinTimingData;
volatile static tPinTimingData pinData[8];

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void REVOMINIRCInput::rxIntPPMSUM(uint8_t state, uint16_t value)
    {
    static uint8_t  channel_ctr;

    if (value >= 4000) // Frame synchronization
	{
	    if( channel_ctr >= REVOMINI_RC_INPUT_MIN_CHANNELS ) {
		_valid_channels = channel_ctr;
	    }
	    channel_ctr = 0;
	}
    else
	{
        if (channel_ctr < REVOMINI_RC_INPUT_NUM_CHANNELS) {
    	    _timestamp_last_signal =  systick_uptime();
            _pulse_capt[channel_ctr] = value;
            _last_pulse[channel_ctr] = systick_uptime();

            channel_ctr++;
            if (channel_ctr == REVOMINI_RC_INPUT_NUM_CHANNELS) {
                _valid_channels = REVOMINI_RC_INPUT_NUM_CHANNELS;
            }
        }

	}
    }


void REVOMINIRCInput::InitDefaultPPM(char board)
    {
	// REVOMINI
	input_channel_ch1 = 4;  // PB14 T12/1
	input_channel_ch2 = 5;  // PB15 T12/2
	input_channel_ch3 = 12; // PC6 T8/1
	input_channel_ch4 = 13; // PC7 T8/2
	input_channel_ch5 = 14; // PC8 T8/3
	input_channel_ch6 = 15; // PC9 T8/4
    }


REVOMINIRCInput::REVOMINIRCInput()
    {
    }

void REVOMINIRCInput::init()
    {

    /*initial check for pin2-pin3 bridge. If detected switch to PPMSUM  */
    //default to standard PPM

/*  _iboard = 2;
    uint8_t channel3_status = 0;
    uint8_t pin2, pin3;
    //input pin 2
    pin2 = 5;
    //input pin 3
    pin3 = 12;

    //set pin2 as output and pin 3 as input
    hal.gpio->pinMode(pin2, OUTPUT);
    hal.gpio->pinMode(pin3, INPUT);

    //default pin3 to 0
    hal.gpio->write(pin3, 0);
    hal.scheduler->delay(1);

    //write 1 to pin 2 and read pin3
    hal.gpio->write(pin2, 1);
    hal.scheduler->delay(1);
    //if pin3 is 1 increment counter
    if (hal.gpio->read(pin3) == 1)
	channel3_status++;

    //write 0 to pin 2 and read pin3
    hal.gpio->write(pin2, 0);
    hal.scheduler->delay(1);
    //if pin3 is 0 increment counter
    if (hal.gpio->read(pin3) == 0)
	channel3_status++;

    //write 1 to pin 2 and read pin3
    hal.gpio->write(pin2, 1);
    hal.scheduler->delay(1);
    //if pin3 is 1 increment counter
    if (hal.gpio->read(pin3) == 1)
	channel3_status++;

    //if counter is 3 then we are in PPMSUM
    if (channel3_status == 3)
	_iboard = 11;
*/

    _iboard = 11;

    if (_iboard < 10) //PWM
	{
	for (byte channel = 0; channel < 8; channel++)
	    pinData[channel].edge = FALLING_EDGE;
	// Init Radio In
	hal.console->println("Init Default PPM");
	pwmInit(false);
	}
    else //PPMSUM
	{
	// Init Radio In
	hal.console->println("Init Default PPMSUM");
	attachPWMCaptureCallback(rxIntPPMSUM);
	pwmInit(true);
	}

    clear_overrides();
    }

uint8_t REVOMINIRCInput::valid_channels()
    {
    if(_iboard < 10)
	return 1;
    else
	return _valid_channels;

    }

bool REVOMINIRCInput::new_input()
{
    noInterrupts();
    bool valid = _timestamp_last_signal != _last_read || _override_valid;
    interrupts();
    return valid;
}

uint8_t REVOMINIRCInput::num_channels()
{
/*    noInterrupts();
    uint8_t n = _rcin.channel_count;
    interrupts(); */
    return _valid_channels;
;
}

uint16_t REVOMINIRCInput::read(uint8_t ch)
    {
    uint16_t data;
    uint32_t pulse;

    noInterrupts();
    _last_read = _timestamp_last_signal;
    
    _override_valid = false;
    if (_iboard < 10)
	{
	//data = rcPinValue[ch];
	data = pwmRead(ch);
	}
    else
	{
	data = _pulse_capt[ch];
	pulse = _last_pulse[ch];
	}
    interrupts();

    /* Check for override */
    uint16_t over = _override[ch];

    if((_iboard >= 10) && (ch == 2) && (systick_uptime() - pulse > 50))
            data = 900;

    return (over == 0) ? data : over;
    }

uint8_t REVOMINIRCInput::read(uint16_t* periods, uint8_t len)
    {
    noInterrupts();
    for (uint8_t i = 0; i < len; i++)
	{
	    if (_iboard < 10)
		periods[i] = pwmRead(i);
	    else{
		if ( i == 2 && (systick_uptime() - _last_pulse[i] > 50) )
		   periods[i] = 900;
	    else
		periods[i] = _pulse_capt[i];
	}

	    if (_override[i] != 0)
		periods[i] = _override[i];
	}
    interrupts();

    return len;
    }



bool REVOMINIRCInput::set_overrides(int16_t *overrides, uint8_t len)
    {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
    }

bool REVOMINIRCInput::set_override(uint8_t channel, int16_t override)
    {
    if (override < 0) return false; /* -1: no change. */
    if (channel < 8) {
        _override[channel] = override;
        if (override != 0) {
    	    _override_valid = true;
            return true;
        }
    }
    return false;
    }

void REVOMINIRCInput::clear_overrides()
    {
    for (int i = 0; i < 8; i++) {
	set_override(i, 0);
    }
    }

