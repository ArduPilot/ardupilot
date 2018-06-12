#pragma once 

#include "RC_parser.h"
#include "RCInput.h"



// helper class with PPM/SBUS parsers to localize all internal data
class F4Light::PPM_parser :  public F4Light::_parser {
public:
    PPM_parser()
        : _ch(0)
        , last_pulse({0,0})
        , _got_ppm(false)
        , _got_dsm(false)
        , _was_ppm(false)
        , _was_dsm(false)
        , _rc_mode(BOARD_RC_NONE)
     {}

    void init(uint8_t ch);

    struct SbusState {
        uint16_t bytes[25]; // including start bit, parity and stop bits
        uint16_t bit_ofs;
        BOARD_RC_MODE mode;
    };

protected:
    void parse_pulses(void);
    void start_ioc(void);

private:
    void rxIntRC(uint16_t value0, uint16_t value1, bool state);

    bool _process_ppmsum_pulse(uint16_t value);
    void _process_dsm_pulse(uint16_t width_s0, uint16_t width_s1);
    void _process_sbus_pulse(uint16_t width_s0, uint16_t width_s1, struct PPM_parser::SbusState &state);


    void add_dsm_input();  // add some DSM  input bytes, for RCInput over a PPMSUM line
    void add_sbus_input(); // add some SBUS input bytes, for RCInput over a PPMSUM line

    uint8_t _ch;

    Pulse last_pulse;
    uint8_t channel_ctr;

    bool _got_ppm;
    bool _got_dsm;

    bool _was_ppm; 
    bool _was_dsm;

    // state of add_dsm_input
    struct DSM {
        uint8_t frame[16];
        uint8_t partial_frame_count;
        uint64_t last_input_ms;
    } dsm;

    // state of add_sbus_input
    struct SBUS {
        uint8_t frame[26];
        uint8_t partial_frame_count;
        uint32_t last_input_ms;
    } sbus;
    
    
    // state of SBUS bit decoder
    struct SbusState sbus_state[2];

    // state of DSM bit decoder
    struct DSM_State {
        uint16_t bytes[16]; // including start bit and stop bit
        uint16_t bit_ofs;
    } dsm_state;

    enum BOARD_RC_MODE _rc_mode;    

    uint8_t _ioc;
};

