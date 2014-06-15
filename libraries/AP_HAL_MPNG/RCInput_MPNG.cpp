#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_MPNG)

#include <avr/io.h>
#include <avr/interrupt.h>

#include <AP_HAL.h>
#include <AP_HAL_MPNG.h>
#include "RCInput.h"
#include "utility/ISRRegistry.h"
using namespace AP_HAL;
using namespace MPNG;

extern const HAL& hal;

// PPM_SUM(CPPM) or PWM Signal processing
//#define SERIAL_PPM SERIAL_PPM_ENABLED
/*
	SERIAL_PPM_DISABLED				// Separated channel signal (PWM) on A8-A15 pins
	SERIAL_PPM_ENABLED				// For all boards, PPM_SUM pin is A8
	SERIAL_PPM_ENABLED_PL1		// Use for RCTIMER CRIUS AIOP Pro v2 ONLY, connect your receiver into PPM SUM pin
*/   
#ifndef SERIAL_PPM
# define SERIAL_PPM SERIAL_PPM_DISABLED
#endif

// Uncomment line below in order to use not Standard channel mapping
//#define RC_MAPPING RC_MAP_STANDARD
/*
	RC_MAP_STANDARD 1
	RC_MAP_GRAUPNER 2
	RC_MAP_HITEC 3
	RC_MAP_MULTIWII 4
	RC_MAP_JR 5
*/

#ifndef RC_MAPPING
# define RC_MAPPING RC_MAP_MULTIWII
#endif

#if RC_MAPPING == RC_MAP_STANDARD
	static uint8_t pinRcChannel[8] = {0, 1, 2, 3, 4, 5, 6, 7}; // ROLL,PITCH,THROTTLE,YAW,MODE,AUX2,CAMPITCH,CAMROLL
#elif RC_MAPPING == RC_MAP_GRAUPNER
	static uint8_t pinRcChannel[8] = {1, 3, 2, 0, 4, 5, 6, 7}; // PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL
#elif RC_MAPPING == RC_MAP_HITEC
	static uint8_t pinRcChannel[8] = {1, 0, 2, 3, 4, 5, 6, 7}; // PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
#elif RC_MAPPING == RC_MAP_MULTIWII
	static uint8_t pinRcChannel[8] = {1, 2, 0, 3, 4, 5, 6, 7}; // ROLL,THROTTLE,PITCH,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
#elif RC_MAPPING == RC_MAP_JR
	static uint8_t pinRcChannel[8] = {1, 2, 0, 3, 5, 6, 4, 7}; // FLAPS:MODE, GEAR:SAVE TRIMM = apm ch7
#else
# error Wrong RC_MAPPING
#endif

// PPM_SUM filtering
#define FILTER FILTER_DISABLED
/*
	FILTER_DISABLED
	FILTER_AVERAGE
	FILTER_JITTER
*/

#define JITTER_THRESHOLD 4
#define AVARAGE_FACTOR 1 		//changes the influance of the current meassured value to the final filter value -
								//bigger means less influance
								//min.value 1, resonable [1,2], uneven is faster [1]
#if (AVARAGE_FACTOR < 1)
# error Wrong AVARAGE_FACTOR selected. Minimum value 1
#endif

#define MIN_PULSEWIDTH 1800 // 900
#define MAX_PULSEWIDTH 4200 // 2100
#define MIN_PPM_SYNCHWIDTH  5000 //2500


/* private variables to communicate with input capture isr */
volatile uint16_t MPNGRCInput::_pulse_capt[AVR_RC_INPUT_NUM_CHANNELS] = {0};  
volatile uint8_t  MPNGRCInput::_valid_channels = 0;
volatile uint16_t MPNGRCInput::rcPinValueRAW[AVR_RC_INPUT_NUM_CHANNELS]; // Default RC values
	
typedef void (*ISRFuncPtr)(void);
static volatile ISRFuncPtr FireISRRoutine = 0; 

ISR(PCINT2_vect) {
	if (FireISRRoutine)
		FireISRRoutine();
}

/* ISR for PPM SUM decoder on PL1 pin (only CRIUS v2 board) */
void MPNGRCInput::_ppmsum_PL1_isr(void) {
    static uint16_t icr5_prev;
    static uint8_t  channel_ctr;

    const uint16_t icr5_current = ICR5;
    uint16_t pulse_width = icr5_current - icr5_prev;;
	
    if (pulse_width > MIN_PPM_SYNCHWIDTH) {
        // sync pulse detected.  Pass through values if at least a minimum number of channels received
        if( channel_ctr >= AVR_RC_INPUT_MIN_CHANNELS ) {
            _valid_channels = channel_ctr;
        }
        channel_ctr = 0;
    } else {
        if (channel_ctr < AVR_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = pulse_width;
            channel_ctr++;
            if (channel_ctr == AVR_RC_INPUT_NUM_CHANNELS) {
                _valid_channels = AVR_RC_INPUT_NUM_CHANNELS;
            }
        }
    }
    icr5_prev = icr5_current;
}

/* ISR for PPM SUM decoder on A8 pin */
void MPNGRCInput::_ppmsum_A8_isr(void)
{ 
	uint16_t curr_time;
	uint16_t period_time;
	uint8_t mask;
	uint8_t pin = 1;
	static uint16_t last_time;
	static uint8_t PCintLast;
	static uint8_t curr_ch_number;
	static bool GotFirstSynch;

	curr_time = TCNT5;         // 0.5us resolution
	pin = PINK;               // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

	// Rising edge detection
	if (mask & 1) { // Ensure pin A8(PPM_SUM) changed
		if (pin & 1) { // Rising edge?

//			hal.gpio->write(46,1);
//			hal.gpio->write(46,0);
	
			// it should be guaranteed to wrap around - do not need to check. (unsigned values)
			period_time = curr_time-last_time;
			last_time = curr_time; // Save edge time
				
			// Process channel pulse
			// Good widths?
			if ((period_time < MAX_PULSEWIDTH) && (period_time > MIN_PULSEWIDTH) && (GotFirstSynch)) {
				if (curr_ch_number < AVR_RC_INPUT_NUM_CHANNELS) {
					#if FILTER == FILTER_DISABLED
						rcPinValueRAW[curr_ch_number] = period_time;
					#elif FILTER == FILTER_AVERAGE 
						rcPinValueRAW[curr_ch_number]=((AVARAGE_FACTOR*rcPinValueRAW[curr_ch_number])+period_time)/(AVARAGE_FACTOR+1);
					#elif FILTER == FILTER_JITTER 
						if (abs(rcPinValueRAW[curr_ch_number]-period_time) > JITTER_THRESHOLD)
							rcPinValueRAW[curr_ch_number] = period_time;
					#endif
	
				}
				// Count always even if we will get more then NUM_CHANNELS >> fault detection.
				curr_ch_number++;
	
				if (curr_ch_number > AVR_RC_INPUT_NUM_CHANNELS) {
					GotFirstSynch = false;						//reset decoder
				}
			}

			// Process First SYNCH
			// We skip first frame, because we can start from middle, so first frame can be invalid
			else if ((period_time > MIN_PPM_SYNCHWIDTH) && (!GotFirstSynch))
			{
				GotFirstSynch = true;
				curr_ch_number=0;
			}

			// Process any other SYNCH
			// it's SYNCH
			else if ((period_time > MIN_PPM_SYNCHWIDTH))
			{
				// Check for minimal channels and copy data to further process
				if (curr_ch_number >= AVR_RC_INPUT_MIN_CHANNELS){
					for (uint8_t i=0; i < AVR_RC_INPUT_NUM_CHANNELS; i++) 		// store channels
					{
						_pulse_capt[i] = rcPinValueRAW[i];
					}
					_valid_channels = AVR_RC_INPUT_NUM_CHANNELS;
				}
				else{
					_valid_channels = 0;
				}
				curr_ch_number=0;								// always rest on synch
			}
	
			// Process FAILURE - start from beginning ....
			// that's bad - we do not want to be here at any time ....
			else {
				curr_ch_number=0;
				_valid_channels = 0;
				GotFirstSynch = false;						//reset decoder
			}
		}
	}
}

volatile uint16_t MPNGRCInput::edgeTime[8]; // Save edge time for every channel in order to calculate pulse width

void MPNGRCInput::_pwm_A8_A15_isr(void)
{ //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
	uint8_t mask;
	uint8_t pin;
	uint16_t cTime,dTime;
	static uint8_t PCintLast;

	cTime = TCNT5;         // from sonar
	pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

	// generic split PPM  
	// mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
	// chan = pin sequence of the port. chan begins at D2 and ends at D7
	
	if (mask & 1<<0) {
		if (!(pin & 1<<0)) {
			dTime = (cTime-edgeTime[0]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[0] = dTime;
		} else edgeTime[0] = cTime;
	}
	if (mask & 1<<1) {
		if (!(pin & 1<<1)) {
			dTime = (cTime-edgeTime[1]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[1] = dTime;
		} else edgeTime[1] = cTime;
	}
	if (mask & 1<<2) {
		if (!(pin & 1<<2)) {
			dTime = (cTime-edgeTime[2]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[2] = dTime;
		} else edgeTime[2] = cTime;
	}
	if (mask & 1<<3) {
		if (!(pin & 1<<3)) {
			dTime = (cTime-edgeTime[3]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[3] = dTime;
		} else edgeTime[3] = cTime;
	}
	if (mask & 1<<4) {
		if (!(pin & 1<<4)) {
			dTime = (cTime-edgeTime[4]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[4] = dTime;
		} else edgeTime[4] = cTime;
	}
	if (mask & 1<<5) {
		if (!(pin & 1<<5)) {
			dTime = (cTime-edgeTime[5]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[5] = dTime;
		} else edgeTime[5] = cTime;
	}
	if (mask & 1<<6) {
		if (!(pin & 1<<6)) {
			dTime = (cTime-edgeTime[6]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[6] = dTime;
		} else edgeTime[6] = cTime;
	}
	if (mask & 1<<7) {
		if (!(pin & 1<<7)) {
			dTime = (cTime-edgeTime[7]); if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) _pulse_capt[7] = dTime;
		} else edgeTime[7] = cTime;
	}
	
	// If we got pulse on throttle pin, report success  
	if (mask & 1<<pinRcChannel[2]) {
		_valid_channels = AVR_RC_INPUT_NUM_CHANNELS;
	}
}

void MPNGRCInput::init(void* _isrregistry) {
	/* initialize overrides */
	clear_overrides();

	DDRK = 0;  // Set PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
//	hal.gpio->pinMode(46, GPIO_OUTPUT); // ICP5 pin (PL1) (PPM input) CRIUS v2
//	hal.gpio->write(46,0);

	//Timer5 already configured in Scheduler
	//TCCR5A = 0; //standard mode with overflow at A and OC B and C interrupts
	//TCCR5B = (1<<CS11); //Prescaler set to 8, resolution of 0.5us

#if SERIAL_PPM == SERIAL_PPM_DISABLED
		FireISRRoutine = _pwm_A8_A15_isr;
		PORTK = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
		PCMSK2 = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); // enable interrupts on A8-A15 pins;
		PCICR |= (1 << PCIE2); // PCINT2 Interrupt enable
#elif SERIAL_PPM == SERIAL_PPM_ENABLED
		FireISRRoutine = _ppmsum_A8_isr;
		PORTK = (1<<PCINT16); //enable internal pull up on the SERIAL SUM pin A8
		PCMSK2 |= (1 << PCINT16); // Enable int for pin A8(PCINT16)
		PCICR |= (1 << PCIE2); // PCINT2 Interrupt enable
#elif SERIAL_PPM == SERIAL_PPM_ENABLED_PL1
		FireISRRoutine = 0;
		hal.gpio->pinMode(48, GPIO_INPUT); // ICP5 pin (PL1) (PPM input) CRIUS v2
		ISRRegistry* isrregistry = (ISRRegistry*) _isrregistry;
		isrregistry->register_signal(ISR_REGISTRY_TIMER5_CAPT, _ppmsum_PL1_isr);
		TCCR5B |= (1<<ICES5); // Enable input capture on rising edge 
		TIMSK5 |= (1<<ICIE5); // Enable input capture interrupt. Timer interrupt mask  
		PCMSK2 = 0;	// Disable INT for pin A8-A15
#else
#error You must check SERIAL_PPM mode, something wrong
#endif
}

uint8_t MPNGRCInput::valid_channels() { return _valid_channels; }

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

uint16_t MPNGRCInput::read(uint8_t ch) {
    /* constrain ch */
    if (ch >= AVR_RC_INPUT_NUM_CHANNELS) return 0;
    /* grab channel from isr's memory in critical section*/
    cli();
    uint16_t capt = _pulse_capt[pinRcChannel[ch]];
    sei();
    _valid_channels = 0;
    /* scale _pulse_capt from 0.5us units to 1us units. */
    uint16_t pulse = constrain_pulse(capt >> 1);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t MPNGRCInput::read(uint16_t* periods, uint8_t len) {
    /* constrain len */
    if (len > AVR_RC_INPUT_NUM_CHANNELS) { len = AVR_RC_INPUT_NUM_CHANNELS; }
    /* grab channels from isr's memory in critical section */
    cli();
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = _pulse_capt[pinRcChannel[i]];
    }
    sei();
    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (uint8_t i = 0; i < len; i++) {
        /* scale _pulse_capt from 0.5us units to 1us units. */
        periods[i] = constrain_pulse(periods[i] >> 1);
        /* check for override */
        if (_override[i] != 0) {
            periods[i] = _override[i];
        }
    }
    uint8_t v = _valid_channels;
    _valid_channels = 0;
    return v;
}

bool MPNGRCInput::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool MPNGRCInput::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return false; /* -1: no change. */
    if (channel < AVR_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            _valid_channels = 1;
            return true;
        }
    }
    return false;
}

void MPNGRCInput::clear_overrides() {
    for (uint8_t i = 0; i < AVR_RC_INPUT_NUM_CHANNELS; i++) {
        _override[i] = 0;
    }
}

#endif
