#if !(HAL_MINIMIZE_FEATURES && APM_BUILD_TYPE(APM_BUILD_ArduCopter))

#include <AP_Param/AP_Param.h>
#include <AP_InertialNav/AP_InertialNav.h>

// weather vane class
class AC_WeatherVane {
    public:

        // Constructor
        AC_WeatherVane(const AP_InertialNav& inav);

        // Do not allow copies
        AC_WeatherVane(const AC_WeatherVane &other) = delete;
        AC_WeatherVane &operator=(const AC_WeatherVane&) = delete;

        // Calculate and return the yaw rate to weathervane the vehicle
        float get_yaw_rate_cds(const int16_t roll_cdeg, const int16_t pitch_cdeg);

        // Returns true if the vehicle is in a condition whereby weathervaning is allowed
        bool should_weathervane(const int16_t pilot_yaw, const int16_t roll_cdeg, const int16_t pitch_cdeg);

        // Use to relax weathervaning on landing.  Must be persistently called before calls to get_weathervane_yaw_rate_cds().
        void set_relax(bool relax) { should_relax = relax; }

        static const struct AP_Param::GroupInfo var_info[];

    private:

        // References to other libraries
        const AP_InertialNav& _inav;

        // Different options for the direction that vehicle will turn into wind
        enum class Direction {
            OFF = 0,
            NOSE_IN = 1, // Only nose into wind
            NOSE_OR_TAIL_IN = 2, // Nose in or tail into wind, which ever is closest
            SIDE_IN = 3, // Side into wind for copter tailsitters
        };

        // Returns the set direction, handling the variable cast to type Direction
        Direction get_direction() const { return (Direction)_direction.get(); }

        // Function to reset all flags and set values. Invoked whenever the weather vaning process is interupted
        void reset(void);

        /*
          Check if vehicle is above the minimum height to weather vane.
          Returns true if above the min set height.
          Uses terrain height if it is enabled.
        */
        bool below_min_height(void);

        // Paramaters
        AP_Int8 _direction;
        AP_Float _gain;
        AP_Float _min_dz_ang_deg;
        AP_Int32 _min_height;
        AP_Float _max_vel_xy;
        AP_Float _max_vel_z;

        uint32_t last_pilot_input_ms;
        float last_output;
        bool should_relax;
        bool active_msg_sent;
        uint32_t first_activate_ms;
};
#endif
