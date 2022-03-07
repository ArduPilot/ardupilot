#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <AC_PID/AC_PID.h>

class AP_ADRC : public AC_PID {
    public:
       // Constructor for ADRC
        AP_ADRC(float B0, float dt);

        //virtual ~AP_ADRC() = default;

        CLASS_NO_COPY(AP_ADRC);

        //  update_all - set target and measured inputs to ADRC controller and calculate outputs
        //  target and error are filtered
        float update_all(float target, float measurement, bool limit = false) override;

        // Set time step in seconds
        void set_dt(float dt);

        // Reset ESO
        void reset_eso(float measurement);

        // Reset filter
        void reset_filter(){
            _flags.reset_filter = true;
        }

        const AP_Logger::PID_Info& get_pid_info(void) const {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADRC PIDINFO USED.");
            return _pid_info; }

        // parameter var table
        static const struct AP_Param::GroupInfo var_info[];
    private:

        float fal(float e, float alpha, float delta);

        float sign(float x);

        // parameters
        AP_Float _wc;          // Response bandwidth in rad/s
        AP_Float _wo;          // State estimation bandwidth in rad/s
        AP_Float _b0;          // Control gain
        AP_Float _limit;
        AP_Float _delta;
        AP_Int8  _order;

       // flags
        struct ar_adrc_flags {
            bool reset_filter :1; // true when input filter should be reset during next call to set_input
        } _flags;

        // internal varibales
        float _dt;                // timestep in seconds

        // ESO interal variables
        float _z1;
        float _z2;
        float _z3;

        AP_Logger::PID_Info _pid_info;

};
