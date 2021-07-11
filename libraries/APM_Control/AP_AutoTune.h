#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AC_PID/AC_PID.h>

class AP_AutoTune {
public:
    struct ATGains {
        AP_Float tau;
        AP_Int16 rmax_pos;
        AP_Int16 rmax_neg;
        float FF, P, I, D, IMAX;
        float flt_T, flt_E, flt_D;
    };

    enum ATType {
        AUTOTUNE_ROLL  = 0,
        AUTOTUNE_PITCH = 1
    };

    struct PACKED log_ATRP {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        uint8_t type;
        uint8_t state;
        float actuator;
        float P_slew;
        float D_slew;
        float FF_single;
        float FF;
        float P;
        float I;
        float D;
        uint8_t action;
        float rmax;
        float tau;
    };


    // constructor
    AP_AutoTune(ATGains &_gains, ATType type, const AP_Vehicle::FixedWing &parms, AC_PID &rpid);

    // called when autotune mode is entered
    void start(void);

    // called to stop autotune and restore gains when user leaves
    // autotune
    void stop(void);

    // update called whenever autotune mode is active. This is
    // called at the main loop rate
    void update(AP_Logger::PID_Info &pid_info, float scaler, float angle_err_deg);

    // are we running?
    bool running;
    
private:
    // the current gains
    ATGains &current;
    AC_PID &rpid;

    // what type of autotune is this
    ATType type;

	const AP_Vehicle::FixedWing &aparm;

    // values to restore if we leave autotune mode
    ATGains restore; 

    // values we last saved
    ATGains last_save; 

    // values to save on the next save event
    ATGains next_save;

    // time when we last saved
    uint32_t last_save_ms;

    // last logging time
    uint32_t last_log_ms;

    // the demanded/achieved state
    enum class ATState {IDLE,
                        DEMAND_POS,
                        DEMAND_NEG};
    ATState state;

    // the demanded/achieved state
    enum class Action {NONE,
                       LOW_RATE,
                       SHORT,
                       RAISE_PD,
                       LOWER_PD,
                       IDLE_LOWER_PD,
                       RAISE_D,
                       RAISE_P,
                       LOWER_D,
                       LOWER_P};
    Action action;

    // when we entered the current state
    uint32_t state_enter_ms;

    void check_save(void);
    void check_state_exit(uint32_t state_time_ms);
    void save_gains(const ATGains &v);

    void save_float_if_changed(AP_Float &v, float value);
    void save_int16_if_changed(AP_Int16 &v, int16_t value);
    void state_change(ATState newstate);

    // get gains with PID components
    ATGains get_gains(const ATGains &current);
    void set_gains(const ATGains &v);

    // update rmax and tau towards target
    void update_rmax();

    // 5 point mode filter for FF estimate
    ModeFilterFloat_Size5 ff_filter;

    LowPassFilterFloat actuator_filter;
    LowPassFilterFloat rate_filter;
    LowPassFilterFloat target_filter;

    // separate slew limiters for P and D
    float slew_limit_max, slew_limit_tau;
    SlewLimiter slew_limiter_P{slew_limit_max, slew_limit_tau};
    SlewLimiter slew_limiter_D{slew_limit_max, slew_limit_tau};

    float max_actuator;
    float min_actuator;
    float max_rate;
    float min_rate;
    float max_target;
    float min_target;
    float max_P;
    float max_D;
    float min_Dmod;
    float max_Dmod;
    float max_SRate_P;
    float max_SRate_D;
    float FF_single;
    uint16_t ff_count;
    float dt;
    float D_limit;
    float P_limit;
    uint32_t D_set_ms;
    uint32_t P_set_ms;
    uint8_t done_count;
};
