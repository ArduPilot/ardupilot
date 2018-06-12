/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/
#pragma GCC optimize ("O2")

#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>
#include <AP_HAL/utility/dsm.h>
#include <AP_HAL/utility/sumd.h>
#include "sbus.h"
#include "GPIO.h"
#include "ring_buffer_pulse.h"

#include "RC_PPM_parser.h"

#include "UARTDriver.h"
#include "UART_PPM.h"

using namespace F4Light;


extern const AP_HAL::HAL& hal;


void PPM_parser::init(uint8_t ch){
    memset((void *)&_val[0], 0, sizeof(_val));
    _last_signal=0;
    _last_change=0;
    _channels=0;
    
    channel_ctr=0;
    
    _ch = ch + 1;

    last_pulse = {0,0};

    _ioc = Scheduler::register_io_completion(FUNCTOR_BIND_MEMBER(&PPM_parser::parse_pulses, void));
    // TODO Panic on IOC not allocated ?

    // callback is called on each edge so must be as fast as possible
    Revo_handler h = { .mp = FUNCTOR_BIND_MEMBER(&PPM_parser::start_ioc, void) };
    pwm_setHandler(h.h, _ch-1);
    
    sbus_state[0].mode=BOARD_RC_SBUS;
    sbus_state[1].mode=BOARD_RC_SBUS_NI;
}


void PPM_parser::start_ioc(void){ 
    Scheduler::do_io_completion(_ioc);
}

void PPM_parser::parse_pulses(void){
    if(_ch==0) return; // not initialized

    Pulse p;
#if 0 // [ statistics to tune memory usage
    uint16_t np = getPPM_count(_ch);
    if(np>RCInput::max_num_pulses) RCInput::max_num_pulses=np;
#endif //]

    while( getPPM_Pulse(&p, _ch-1)){
        rxIntRC(last_pulse.length, p.length, p.state);
        last_pulse = p;
    }
}


void PPM_parser::rxIntRC(uint16_t last_value, uint16_t value, bool state)
{

    if(state) { // was 1 so falling
        if(_rc_mode==BOARD_RC_NONE){
            _process_ppmsum_pulse( (last_value + value) >>1 ); // process PPM only if no protocols detected
        }

        if((_rc_mode & ~BOARD_RC_SBUS_NI) == 0){
            // test for non-inverted SBUS in 2nd memory structures
            _process_sbus_pulse(value>>1, last_value>>1, sbus_state[1]);  // was 1 so now is length of 1, last is a length of 0
        }

    } else { // was 0 so rising

        if((_rc_mode & ~BOARD_RC_SBUS) == 0){
            // try treat as SBUS (inverted)
            // SBUS protocols detection occures on the beginning of start bit of next frame
            _process_sbus_pulse(value>>1, last_value>>1,  sbus_state[0]); // was 0 so now is length of 0, last is a length of 1
        }


        if((_rc_mode & ~(BOARD_RC_DSM | BOARD_RC_SUMD)) == 0){
            // try treat as DSM or SUMD. Detection occures on the end of stop bit
            _process_dsm_pulse(value>>1, last_value>>1);
        }

    }
}


bool PPM_parser::_process_ppmsum_pulse(uint16_t value)
{
    if (value >= 2700) { // Frame synchronization
	if( channel_ctr >= F4Light_RC_INPUT_MIN_CHANNELS ) {
	    _channels = channel_ctr;
	}
	channel_ctr = 0;
	_got_ppm=true;

        return true;	    
    } else if(value > 700 && value < 2300) {
        if (channel_ctr < F4Light_RC_INPUT_NUM_CHANNELS) {
    	    _last_signal =  systick_uptime();
    	    if(_val[channel_ctr] != value) _last_change = _last_signal;
            _val[channel_ctr] = value;

            channel_ctr++;
            if (channel_ctr >= F4Light_RC_INPUT_NUM_CHANNELS) {
                _channels = F4Light_RC_INPUT_NUM_CHANNELS;
            }
        }
        return true;
    } else { // try another protocols
        return false;
    }
}





/*
  process a SBUS input pulse of the given width
  
  pulses are captured on each edges and SBUS parser called on rising edge - beginning of start bit
*/

void PPM_parser::_process_sbus_pulse(uint16_t width_s0, uint16_t width_s1, F4Light::PPM_parser::SbusState &state)
{
    // convert to bit widths, allowing for up to 4usec error, assuming 100000 bps - inverted
    uint16_t bits_s0 = (width_s0+4) / 10;
    uint16_t bits_s1 = (width_s1+4) / 10;
    
    uint8_t byte_ofs = state.bit_ofs/12;
    uint8_t bit_ofs  = state.bit_ofs%12;
    uint16_t nlow;

    if (bits_s1 == 0 || bits_s0 == 0) {        // invalid data
        goto reset;
    }

    if (bits_s1+bit_ofs > 10) { // invalid data as last two bits must be stop bits
        goto reset;
    }
        

    // pull in the high bits
    state.bytes[byte_ofs] |= ((1U<<bits_s1)-1) << bit_ofs;
    state.bit_ofs += bits_s1;
    bit_ofs       += bits_s1;

    // pull in the low bits
    nlow = bits_s0;          // length of low bits
    if (nlow + bit_ofs > 12) { // goes over byte boundary?
        nlow = 12 - bit_ofs;   // remaining part of byte
    }
    bits_s0 -= nlow;  // zero bit residual
    state.bit_ofs += nlow; // fill by zeros till byte end

    if (state.bit_ofs == 25*12 && bits_s0 > 12) { // all frame got and was gap
        // we have a full frame
        uint8_t bytes[25];
        uint16_t i;

        for (i=0; i<25; i++) {
            // get inverted data
            uint16_t v = ~state.bytes[i];
    
            if ((v & 1) != 0) {        // check start bit
                goto reset;
            }
            
            if ((v & 0xC00) != 0xC00) {// check stop bits
                goto reset;
            }
            // check parity
            uint8_t parity = 0, j;
            for (j=1; j<=8; j++) {
                parity ^= (v & (1U<<j))?1:0;
            }
            if (parity != (v&0x200)>>9) {
                goto reset;
            }
            bytes[i] = ((v>>1) & 0xFF);
        }

        uint16_t values[F4Light_RC_INPUT_NUM_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe=false, sbus_frame_drop=false;


        if (sbus_decode(bytes, values, &num_values,
                        &sbus_failsafe, &sbus_frame_drop,
                        F4Light_RC_INPUT_NUM_CHANNELS) &&
            num_values >= F4Light_RC_INPUT_MIN_CHANNELS) 
        {

            for (i=0; i<num_values; i++) {
                if(_val[i] != values[i]) _last_change = systick_uptime();
                _val[i] = values[i];
            }
            _channels = num_values;
            
            _rc_mode = state.mode; // lock input mode, SBUS has a parity and other checks so false positive is unreal
            
            if (!sbus_failsafe) {
                _got_dsm = true;
                _last_signal = systick_uptime();
            }
        }
        goto reset_ok;
    } else if (bits_s0 > 12) { // Was inter-frame gap but not full frame 
        goto reset;
    }
    return;
reset:

reset_ok:
    state.bit_ofs=0;
    memset(&state.bytes, 0, sizeof(state.bytes));
}



/*
  process a DSM satellite input pulse of the given width

  pulses are captured on each edges and DSM parser called on falling edge - eg. beginning of start bit 
*/

void PPM_parser::_process_dsm_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1uSec error, assuming 115200 bps
    uint16_t bits_s0 = ((width_s0+4)*(uint32_t)115200) / 1000000;
    uint16_t bits_s1 = ((width_s1+4)*(uint32_t)115200) / 1000000;
    uint8_t bit_ofs, byte_ofs;
    uint16_t nbits;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;

    if(byte_ofs > 15) {
        // invalid data
        goto reset;
    }

    // pull in the high bits
    nbits = bits_s0;
    if (nbits+bit_ofs > 10) {
        nbits = 10 - bit_ofs;
    }
    dsm_state.bytes[byte_ofs] |= ((1U<<nbits)-1) << bit_ofs;
    dsm_state.bit_ofs += nbits;
    bit_ofs += nbits;


    if (bits_s0 - nbits > 10) {
        if (dsm_state.bit_ofs == 16*10) {
            // we have a full frame
            uint8_t bytes[16];
            uint8_t i;
            for (i=0; i<16; i++) {
                // get raw data
                uint16_t v = dsm_state.bytes[i];

                // check start bit
                if ((v & 1) != 0) {
                    goto reset;
                }
                // check stop bits
                if ((v & 0x200) != 0x200) {
                    goto reset;
                }
                uint8_t bt= ((v>>1) & 0xFF);
                bytes[i] = bt;
                
                if(_rc_mode != BOARD_RC_DSM) { 
                // try to decode  SUMD data on each byte, decoder butters frame itself.
                    uint16_t values[F4Light_RC_INPUT_NUM_CHANNELS];
                    uint8_t rssi;
                    uint8_t rx_count;
                    uint16_t channel_count;

                    if (sumd_decode(bt, &rssi, &rx_count, &channel_count, values, F4Light_RC_INPUT_NUM_CHANNELS) == 0) {
                        if (channel_count > F4Light_RC_INPUT_NUM_CHANNELS) {
                            continue;
                        }
                        _rc_mode = BOARD_RC_SUMD;
                        for (uint8_t j=0; j<channel_count; j++) {
                            if (values[j] != 0) {
                                if(_val[j] != values[j]) _last_change = systick_uptime();
                                _val[j] = values[j];
                            }
                        }
                        _channels = channel_count;
                        _last_signal = systick_uptime();
//                        _rssi = rssi;
                    }
                }
                
                if(_rc_mode == BOARD_RC_NONE) { // if protocol not decoded
                    UART_PPM::putch(bt, _ch);    // push received bytes to memory queue to get via fake UARTs
                }
            }
            if(_rc_mode != BOARD_RC_SUMD) { // try to decode buffer as DSM on full frame
                uint16_t values[F4Light_RC_INPUT_NUM_CHANNELS];
                uint16_t num_values=0;
                if (dsm_decode(AP_HAL::micros64(), bytes, values, &num_values, F4Light_RC_INPUT_NUM_CHANNELS) &&
                    num_values >= F4Light_RC_INPUT_MIN_CHANNELS) {

                    _rc_mode = BOARD_RC_DSM; // lock input mode, DSM has a checksum so false positive is unreal

                    for (i=0; i<num_values; i++) {
                        if(_val[i] != values[i]) _last_change = systick_uptime();
                        _val[i] = values[i];
                    }
                    
                    uint32_t nc=num_values+1;
                    if(nc>_channels) 
                        _channels = nc;
                    _val[_channels-1]=bytes[0]; // rssi
                    _got_dsm = true;
                    _last_signal = systick_uptime();
                }
            }
        }
        memset(&dsm_state, 0, sizeof(dsm_state));
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs  = dsm_state.bit_ofs%10;

    if (bits_s1+bit_ofs > 10) {
        // invalid data
        goto reset;
    }

    // pull in the low bits
    dsm_state.bit_ofs += bits_s1;
    return;
reset:
    memset(&dsm_state, 0, sizeof(dsm_state));
}


