/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/

#include <AP_HAL/HAL.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>
#include <AP_HAL/utility/dsm.h>
#include "sbus.h"
#include "GPIO.h"
#include "ring_buffer_pulse.h"

#include "RC_PPM_parser.h"

#ifdef BOARD_SPEKTRUM_RX_PIN
 #include "RC_DSM_parser.h"
#endif
#ifdef BOARD_SBUS_UART
 #include "RC_SBUS_parser.h"
#endif


// Constructors ////////////////////////////////////////////////////////////////
using namespace F4Light;


/*
    DSM satellite connection
        1   2   3   4
pins    *   *   *   *   *   *   *
use    gnd vcc 26  103 xxx xxx xxx
DSM    GND     rx  en

*/


extern const AP_HAL::HAL& hal;
/*
input_channels:
    4,  // PB14 T12/1 - PPM
    5,  // PB15 T12/2 - PPM2
    12, // PC6  T8/1  - 6_tx 
    13, // PC7  T8/2  - 6_rx 
    14, // PC8  T8/3  - Soft_scl / soft_TX
    15, // PC9  T8/4  - Soft_sda / soft_RX
*/

_parser * RCInput::parsers[MAX_RC_PARSERS] IN_CCM;  // individual parsers on each PPM pin and DSM/SBUS USART
uint8_t   RCInput::num_parsers IN_CCM;


uint8_t   RCInput::_valid_channels IN_CCM; //  = 0;
uint64_t  RCInput::_last_read IN_CCM; // = 0;


bool RCInput::is_PPM IN_CCM;

uint8_t RCInput::_last_read_from IN_CCM;

uint16_t RCInput::max_num_pulses IN_CCM;

bool RCInput::fs_flag IN_CCM;
bool RCInput::aibao_fs_flag  IN_CCM;

bool RCInput::rc_failsafe_enabled IN_CCM;

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}




RCInput::RCInput()
{   }

void RCInput::init() {
    caddr_t ptr;

/* OPLINK AIR port pinout
1       2       3       4       5       6       7
               PD2     PA15                
gnd    +5      26      103                     
used as
for DSM:       rx      pow
for RFM        int     cs

*/

    is_PPM=true;
    _last_read_from=0;
    max_num_pulses=0;

    pwmInit(is_PPM); // PPM sum mode

    uint8_t pp=0;

    ptr = sbrk_ccm(sizeof(PPM_parser)); // allocate memory in CCM
    parsers[pp++] = new(ptr) PPM_parser;
    
    ptr = sbrk_ccm(sizeof(PPM_parser)); // allocate memory in CCM
    parsers[pp++] = new(ptr) PPM_parser;

#ifdef BOARD_SPEKTRUM_RX_PIN
    ptr = sbrk_ccm(sizeof(DSM_parser)); // allocate memory in CCM
    parsers[pp++] =new(ptr) DSM_parser; 
#endif
#ifdef BOARD_NRF_CS_PIN
    ptr = sbrk_ccm(sizeof(NRF_parser)); // allocate memory in CCM
    parsers[pp++] =new(ptr) NRF_parser;
#endif
#ifdef BOARD_SBUS_UART
    ptr = sbrk_ccm(sizeof(SBUS_parser)); // allocate memory in CCM
    parsers[pp++] =new(ptr) SBUS_parser;
#endif

    num_parsers = pp; // counter

    for(uint8_t i=0; i<num_parsers;i++) {
        parsers[i]->init(i);
    }

}


void RCInput::late_init(uint8_t b) {

    for(uint8_t i=0; i<num_parsers;i++) {
        parsers[i]->late_init(b);
    }
    
    if(b & BOARD_RC_FAILSAFE) rc_failsafe_enabled=true;
}

// we can have 4 individual sources of data - internal DSM from UART5, SBUS from UART1 and 2 PPM parsers
bool RCInput::new_input()
{
    uint8_t inp=hal_param_helper->_rc_input;
    if(inp &&  inp < num_parsers+1){
        inp-=1;
        
        return parsers[inp]->get_last_signal() > _last_read;
    }

    for(uint8_t i=0; i<num_parsers;i++) {
        if(parsers[i]->get_last_signal() >_last_read) return true;
    }
    
    return false;
}


uint8_t RCInput::num_channels()
{
    return _valid_channels;
}


uint16_t RCInput::last_4=0;

//#define LOST_TIME 50 // this is wrong! Any packet lost and viola... 
#define LOST_TIME 500
#define FRAME_TIME 50 // time between packets

uint16_t RCInput::_read_ppm(uint8_t ch,uint8_t n){
    const _parser *p = parsers[n];
    _valid_channels = p->get_valid_channels();
    return            p->get_val(ch);
}



uint16_t RCInput::read(uint8_t ch)
{
    uint16_t data=0;
    uint64_t pulse=0;
    uint64_t last=0;

    if(ch>=F4Light_RC_INPUT_NUM_CHANNELS) return 0;

    uint64_t now=systick_uptime();
    uint8_t got=0;


    uint32_t dead_time = hal_param_helper->_rc_fs * 1000UL;  // time in seconds
    
    uint8_t inp=hal_param_helper->_rc_input;
    if(inp &&  inp < num_parsers+1 ){
        inp-=1;

        const _parser *p = parsers[inp];
        pulse           = p->get_last_signal();
        last            = p->get_last_change();
        data            = p->get_val(ch);
        _valid_channels = p->get_valid_channels();
        got = inp+1;

    } else if(now - _last_read > FRAME_TIME) { // seems that we loose 1 frame on current RC receiver so should select new one
        uint32_t best_t=(uint32_t) -1;
        
        for(uint8_t i=0; i<num_parsers;i++) {
            
            const _parser *p = parsers[i];
            uint64_t pt = p->get_last_signal();
            uint64_t lt = p->get_last_change();
            
            uint32_t dt = now-pt; // time from signal
            
            if( pt >_last_read &&  // data is newer than last
                dt<best_t &&          // and most recent
                ((now - lt ) < dead_time || !rc_failsafe_enabled)) // and time from last change less than DEAD_TIME
            {
                best_t = dt;
                data            = p->get_val(ch);
                _valid_channels = p->get_valid_channels();                
                pulse = pt;
                last  = lt;
                got   = i+1;
            }
        }
        // now we have a most recent data
    }
    
    if(got)  {
        _last_read_from = got;
        _last_read      = pulse;
    } else {
        if(_last_read_from) {   //      read from the last channel
            uint8_t n = _last_read_from-1;
            const _parser *p = parsers[n];
            pulse           = p->get_last_signal();
            last            = p->get_last_change();
            data            = p->get_val(ch);
            _valid_channels = p->get_valid_channels();
            _last_read = pulse;
                        
        } else { // no data at all

            if( ch == 2) data = 899; // to know the source
            else         data = 1000;
        }
    }

    if( ch == 4) {
        last_4 = data;
    }

    if(ch == 2) { // throttle
        if( (now-pulse) > LOST_TIME ||   // last pulse is very old
            ((now-last) > dead_time && rc_failsafe_enabled) ) // pulses OK but last change is very old
        {
            data = 900;
            
            if(!fs_flag) {
                fs_flag=true;
#ifdef DEBUG_BUILD
                printf("\n failsafe! now=%lld last pulse=%lld last change=%lld\n",now, pulse, last);
#endif
            }
        } else {
            fs_flag=false;
        }

        if(hal_param_helper->_aibao_fs) {
/*
 Receiver-DEVO-RX719-for-Walkera-Aibao
 failsafe: mode below 1000 and throttle at 1500
*/            
            if(last_4 < 990 && data >1300 && data < 1700){
                if(!aibao_fs_flag){
                    aibao_fs_flag=true;
#ifdef DEBUG_BUILD
                    printf("\nAibao failsafe! ch4=%d ch2=%d\n",last_4, data);
#endif
                }
                data = 901; // to know the source
            } else {
                aibao_fs_flag=false;            
            }
        }

    }
    return data;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{

    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }

    return _valid_channels;
}

bool RCInput::rc_bind(int dsmMode){
#ifdef BOARD_SPEKTRUM_RX_PIN
    return parsers[2]->bind(dsmMode); // only DSM
#else
    return false;
#endif

}

