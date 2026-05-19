#pragma once

// VTOL assistance in a forward flight mode

class QuadPlane;
class VTOL_Assist {
public:
    VTOL_Assist(QuadPlane& _quadplane):quadplane(_quadplane) {};

    // check for assistance needed
    bool should_assist(float aspeed, bool have_airspeed);

    // Assistance not needed, reset any state
    void reset();

    // speed below which quad assistance is given
    AP_Float speed;

    // angular error at which quad assistance is given
    AP_Int8 angle;

    // altitude to trigger assistance
    AP_Int16 alt;

    // Time hysteresis for triggering of assistance
    AP_Float delay;

    // special options
    AP_Int16 options;

    // assist options
    enum class OPTION {
        FW_FORCE_DISABLED=(1U<<0),
        SPIN_DISABLED=(1U<<1),
    };
    
    bool option_is_set(OPTION option) const {
        return (options.get() & int32_t(option)) != 0;
    }
    
    // State from pilot
    enum class STATE {
        ASSIST_DISABLED,
        ASSIST_ENABLED,
        FORCE_ENABLED,
    };
    void set_state(STATE _state) { state = _state; }

    // Logging getters for assist types
    bool in_force_assist() const { return force_assist; }
    bool in_speed_assist() const { return speed_assist; }
    bool in_alt_assist() const { return alt_error.is_active(); }
    bool in_angle_assist() const { return angle_error.is_active(); }

    // check if we are in VTOL recovery
    bool check_VTOL_recovery(void);

    // output rudder and elevator for spin recovery
    void output_spin_recovery(void);
    
private:

    // Default to enabled
    STATE state = STATE::ASSIST_ENABLED;

    class Assist_Hysteresis {
    public:
        // Reset state
        void reset();

        // Update state, return true when first triggered
        bool update(const bool trigger, const uint32_t &now_ms, const uint32_t &trigger_delay_ms, const uint32_t &clear_delay_ms);

        // Return true if the output is active
        bool is_active() const { return active; }

    private:
        uint32_t start_ms;
        uint32_t last_ms;
        bool active;
    };
    Assist_Hysteresis angle_error;
    Assist_Hysteresis alt_error;

    // Force and speed assist have no hysteresis
    bool force_assist;
    bool speed_assist;

    // Reference to access quadplane
    QuadPlane& quadplane;
};
