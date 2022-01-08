#include <AP_Param/AP_Param.h>

// weather vane class
class AC_WeatherVane {
    public:

        // Constructor
        AC_WeatherVane(void);

        CLASS_NO_COPY(AC_WeatherVane);

        // Calculate and return the yaw rate to weathervane the vehicle
        float get_yaw_rate_cds(const float roll_cdeg, const float pitch_cdeg, const float rate_limit);

        // Returns true if the vehicle is in a condition whereby weathervaning is allowed
        // pilot_yaw can be an angle or a rate or rcin from yaw channel.  It just needs to represent a pilot's request to yaw the vehicle
        bool should_weathervane(const int16_t pilot_yaw, const float hgt);

        // Function to reset all flags and set values. Invoked whenever the weather vaning process is interrupted
        void reset(void);

        // allow/disallow weather vaning from other means than by the parameter
        void allow_weathervaning(bool allow) { allowed = allow; }

        static AC_WeatherVane* get_singleton(void) { return _singleton; }

        static const struct AP_Param::GroupInfo var_info[];

    private:

        // Different options for the direction that vehicle will turn into wind
        enum class Direction {
            OFF = 0,
            NOSE_IN = 1, // Only nose into wind
            NOSE_OR_TAIL_IN = 2, // Nose in or tail into wind, which ever is closest
            SIDE_IN = 3, // Side into wind for copter tailsitters
        };

        // Returns the set direction, handling the variable cast to type Direction
        Direction get_direction() const { return (Direction)_direction.get(); }

        // Paramaters
        AP_Int8 _direction;
        AP_Float _gain;
        AP_Float _min_dz_ang_deg;
        AP_Float _min_height;
        AP_Float _max_vel_xy;
        AP_Float _max_vel_z;

        uint32_t last_pilot_input_ms;
        float last_output;
        bool active_msg_sent;
        uint32_t first_activate_ms;

        // Init to true here to avoid a race between init of RC_channel and weathervane
        bool allowed = true;

        static AC_WeatherVane *_singleton;
};

namespace AP {
    AC_WeatherVane *weathervane();
};
