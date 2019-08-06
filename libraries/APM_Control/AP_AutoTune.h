#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>

class AP_AutoTune {
public:
    struct ATGains {
        AP_Float tau;
        AP_Float P;
        AP_Float I;
        AP_Float D;
        AP_Float FF;
        AP_Int16 rmax;
        AP_Int16 imax;
    };

    enum ATType {
        AUTOTUNE_ROLL  = 0,
        AUTOTUNE_PITCH = 1
    };

    struct PACKED log_ATRP {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        uint8_t  type;
        uint8_t  state;
        int16_t  servo;
        float    demanded;
        float    achieved;
        float    P;
    };


    // constructor
    AP_AutoTune(ATGains &_gains, ATType type, const AP_Vehicle::FixedWing &parms);

    // called when autotune mode is entered
    void start(void);

    // called to stop autotune and restore gains when user leaves
    // autotune
    void stop(void);

    // update called whenever autotune mode is active. This is
    // typically at 50Hz
    void update(float desired_rate, float achieved_rate, float servo_out);

    // are we running?
    bool running:1;
    
private:
    // the current gains
    ATGains &current;

    // what type of autotune is this
    ATType type;

	const AP_Vehicle::FixedWing &aparm;

    // did we saturate surfaces?
    bool saturated_surfaces:1;

    // values to restore if we leave autotune mode
    ATGains restore; 

    // values we last saved
    ATGains last_save; 

    // values to save on the next save event
    ATGains next_save;

    // time when we last saved
    uint32_t last_save_ms = 0;

    // the demanded/achieved state
    enum ATState {DEMAND_UNSATURATED,
                  DEMAND_UNDER_POS, 
                  DEMAND_OVER_POS,
                  DEMAND_UNDER_NEG,
                  DEMAND_OVER_NEG} state = DEMAND_UNSATURATED;

    // when we entered the current state
    uint32_t state_enter_ms = 0;

    void check_save(void);
    void check_state_exit(uint32_t state_time_ms);
    void save_gains(const ATGains &v);

    void write_log(float servo, float demanded, float achieved);

    void log_param_change(float v, const char *suffix);
    void save_float_if_changed(AP_Float &v, float value, const char *suffix);
    void save_int16_if_changed(AP_Int16 &v, int16_t value, const char *suffix);
};
