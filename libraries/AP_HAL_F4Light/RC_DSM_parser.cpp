/*
(c) 2017 night_ghost@ykoctpa.ru

*/

#pragma GCC optimize ("O2")

#include <AP_HAL/HAL.h>

#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>
#include <AP_HAL/utility/dsm.h>
#include <AP_HAL/utility/sumd.h>
#include "sbus.h"
#include "GPIO.h"
#include "ring_buffer_pulse.h"

#include "RC_DSM_parser.h"

using namespace F4Light;


extern const AP_HAL::HAL& hal;


#if defined(BOARD_DSM_USART)
UARTDriver DSM_parser::uartSDriver(BOARD_DSM_USART); // UART connected to DSM pin
#endif



void DSM_parser::init(uint8_t ch)  {
#if defined(BOARD_SPEKTRUM_RX_PIN)

    memset((void *)&_val[0],   0, sizeof(_val));
    memset((void *)&dsm,       0, sizeof(dsm));
    
    _last_signal=0;
    _last_change =0;

    uint32_t sig = board_get_rtc_register(RTC_DSM_BIND_REG);
    if( (sig & ~DSM_BIND_SIGN_MASK) == DSM_BIND_SIGNATURE) {
        board_set_rtc_register(0, RTC_DSM_BIND_REG);
        _rc_bind(sig & DSM_BIND_SIGN_MASK);
    }
    
    GPIO::_pinMode(BOARD_SPEKTRUM_RX_PIN, INPUT_PULLUP);
 #ifdef BOARD_SPEKTRUM_PWR_PIN
    GPIO::_pinMode(BOARD_SPEKTRUM_PWR_PIN, OUTPUT);
    GPIO::_write(BOARD_SPEKTRUM_PWR_PIN, BOARD_SPEKTRUM_PWR_ON);
 #endif

    _ioc = Scheduler::register_io_completion(FUNCTOR_BIND_MEMBER(&DSM_parser::_io_completion, void));

    uartSDriver.end(); // just for case
    
    // initialize DSM UART
    uartSDriver.begin(115200);
    Revo_handler h = { .mp = FUNCTOR_BIND_MEMBER(&DSM_parser::add_dsm_uart_input, void) };
    uartSDriver.setCallback(h.h);
}

/*
  add some bytes of input in DSM serial stream format, coping with partial packets - UART input callback
 */
void DSM_parser::add_dsm_uart_input() {
    if(_ioc) {
        Scheduler::do_io_completion(_ioc);
    }
}

void DSM_parser::_io_completion(){
    
    while(uartSDriver.available()){
        
        // at least 1 byte we have
        const uint8_t dsm_frame_size = sizeof(dsm.frame);

        uint32_t now = AP_HAL::millis();    
        if (now - dsm.last_input_ms > 5) {
            // resync based on time
            dsm.partial_frame_count = 0;
        }
        dsm.last_input_ms = now;
    
        if (dsm.partial_frame_count + 1 > dsm_frame_size) {
            return; // we can't add bytes to buffer
        }
        

        char c= uartSDriver.read();
        
        if(state != S_DSM) { // try to decode  SUMD data
            uint16_t values[F4Light_RC_INPUT_NUM_CHANNELS];
            uint8_t rssi;
            uint8_t rx_count;
            uint16_t channel_count;

            if (sumd_decode(c, &rssi, &rx_count, &channel_count, values, F4Light_RC_INPUT_NUM_CHANNELS) == 0) {
                if (channel_count > F4Light_RC_INPUT_NUM_CHANNELS) {
                    continue;
                }
                state=S_SUMD;
                for (uint8_t i=0; i<channel_count; i++) {
                    if (values[i] != 0) {
                        if(_val[i] != values[i]) _last_change = systick_uptime();
                        _val[i] = values[i];
                    }
                }
                _channels = channel_count + 1;
                _last_signal = systick_uptime();
                _val[channel_count] = rssi;        // say about RSSI in last channel
            }
        }

        if(state!=S_SUMD) {
            dsm.frame[dsm.partial_frame_count] = c;
            dsm.partial_frame_count += 1;

    	    if (dsm.partial_frame_count == dsm_frame_size) {
                dsm.partial_frame_count = 0;
                uint16_t values[F4Light_RC_INPUT_NUM_CHANNELS] {};
                uint16_t num_values=0;
                if (dsm_decode(AP_HAL::micros64(), dsm.frame, values, &num_values, 16) &&
                    num_values >= 5 && num_values <F4Light_RC_INPUT_NUM_CHANNELS) {
                    state=S_DSM;
                    for (uint8_t i=0; i<num_values; i++) {
                        if (values[i] != 0) {
                            if(_val[i] != values[i]) _last_change = systick_uptime();
                            _val[i] = values[i];
                        }
                    }
                /*
                  the apparent number of channels can change on DSM,
                  as they are spread across multiple frames. We just
                  use the max num_values we get
                 */
                    uint8_t nc = num_values+1;
                    if (nc > _channels) {
                        _channels = nc;
                    }
/*
    DSM frame from datasheet:
typedef stuct {
 UINT8 fades; 
 UINT8 system;
 UINT16 servo[7];
} INT_REMOTE_STR;

so we get RSSI from 1st byte and store it to last channel
*/
                    _val[_channels-1] = dsm.frame[0];
            
                    _last_signal = systick_uptime();
                }
            }
        }
    }
#endif // defined(BOARD_SPEKTRUM_RX_PIN)
}



#ifdef BOARD_SPEKTRUM_RX_PIN
void DSM_parser::_rc_bind(uint16_t dsmMode){
/*
To put a receiver into bind mode, within 200ms of power application the host device needs to issue a
series of falling pulses. The number of pulses issued selects which bind types will be accepted from
the transmitter. Selecting the 11ms mode automatically allows either 11ms or 22ms protocols to be
used. Selecting DSMX will automatically allow DSM2 to be used if the transmitter is unable to
support DSMX. For this reason, we recommend that 11ms DSMX be used (9 (“internal”) or 10
(“external”) pulses).
DSMX Bind Modes:
Pulses Mode Protocol Type
7     Internal  DSMx 22ms    
8     External  DSMx 22ms
9     Internal  DSMx 11ms
10    External  DSMx 11ms

see https://github.com/SpektrumFPV/SpektrumDocumentation/blob/master/Telemetry/Remote%20Receiver%20Interfacing.pdf

*/    


    Scheduler::_delay(72);

    for (int i = 0; i < dsmMode; i++) {                         /*Pulse RX pin a number of times*/
	Scheduler::_delay_microseconds(120);
	GPIO::_write(BOARD_SPEKTRUM_RX_PIN, 0);
	Scheduler::_delay_microseconds(120);
	GPIO::_write(BOARD_SPEKTRUM_RX_PIN, 1);
    }

    Scheduler::_delay(50);
}



bool DSM_parser::bind(int dsmMode) const {
#ifdef BOARD_SPEKTRUM_PWR_PIN
    uartSDriver.end();

    GPIO::_write(BOARD_SPEKTRUM_PWR_PIN, BOARD_SPEKTRUM_PWR_OFF); /* power down DSM satellite */

    Scheduler::_delay(500);


    GPIO::_write(BOARD_SPEKTRUM_PWR_PIN, BOARD_SPEKTRUM_PWR_ON);     /* power up DSM satellite*/

    GPIO::_pinMode(BOARD_SPEKTRUM_RX_PIN, OUTPUT);           /*Set UART RX pin to active output mode*/

    _rc_bind(dsmMode);

    uartSDriver.begin(115200);                                  	/*Restore USART RX pin to RS232 receive mode*/

#else
    // store request to bing in BACKUP RAM
    board_set_rtc_register(DSM_BIND_SIGNATURE | ( dsmMode & DSM_BIND_SIGN_MASK), RTC_DSM_BIND_REG);
     
#endif
    return true;
}

#endif // BOARD_SPEKTRUM_RX_PIN
