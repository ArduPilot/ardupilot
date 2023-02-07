#include <AP_Param/AP_Param.h>

// weather vane class
class AC_WeatherVane {
    public:

        // Constructor
        AC_WeatherVane(void);

        CLASS_NO_COPY(AC_WeatherVane);

        // Calculate and return the yaw output to weathervane the vehicle
        bool get_yaw_out(float &yaw_output, const int16_t pilot_yaw, const float hgt, const float roll_cdeg, const float pitch_cdeg, const bool is_takeoff, const bool is_landing);

        // Function to reset all flags and set values. Invoked whenever the weather vaning process is interrupted
        void reset(void);

        // allow/disallow weather vaning from other means than by the parameter
        void allow_weathervaning(bool allow) { allowed = allow; }

        static const struct AP_Param::GroupInfo var_info[];

    private:

        // Different options for the direction that vehicle will turn into wind
        enum class Direction {
            OFF = 0,
            NOSE_IN = 1, // Only nose into wind
            NOSE_OR_TAIL_IN = 2, // Nose in or tail into wind, which ever is closest
            SIDE_IN = 3, // Side into wind for copter tailsitters
            TAIL_IN = 4, // backwards, for tailsitters, makes it easier to descend
        };

        enum class Options {
            PITCH_ENABLE = (1<<0),
        };
    
        // Paramaters
        AP_Int8 _direction;
        AP_Float _gain;
        AP_Float _min_dz_ang_deg;
        AP_Float _min_height;
        AP_Float _max_vel_xy;
        AP_Float _max_vel_z;
        AP_Int8 _landing_direction;
        AP_Int8 _takeoff_direction;
        AP_Int16 _options;

        float last_output;
        bool active_msg_sent;
        uint32_t first_activate_ms;
        uint32_t last_check_ms;

        // Init to true here to avoid a race between init of RC_channel and weathervane
        bool allowed = true;
};
