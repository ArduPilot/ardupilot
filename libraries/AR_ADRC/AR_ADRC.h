#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>

class AR_ADRC{
    public:
       // Constructor for ADRC
        AR_ADRC();
        virtual ~AR_ADRC() = default;

        CLASS_NO_COPY(AR_ADRC);

        //  update_all - set target and measured inputs to ADRC controller and calculate outputs
        //  target and error are filtered
        float update_all(float target,float measurement);

        // Set time step in seconds
        void set_dt(float dt);

        // Reset ESO
        void reset_eso(float measurement);

        // Reset filter
        void reset_filter(){
            flags_.reset_filter_ = true;
        }

        const AP_Logger::PID_Info& get_debug_info(void) const { return _debug_info; }

        // parameter var table
         static const struct AP_Param::GroupInfo var_info[];

    private:

        float AR_ADRC::fal(float e, float alpha, float delta);
        
        // parameters
        AP_Float wc_;          // Response bandwidth in rad/s
        AP_Float wo_;          // State estimation bandwidth in rad/s
        AP_Float b0_;          // Control gain
        AP_Float limit_;
        AP_Float delta_;
        AP_Int8  order_;
  
       // flags
        struct ar_adrc_flags {
            bool reset_filter_ :1; // true when input filter should be reset during next call to set_input
        } flags_;

        // internal varibales
        float dt_;                // timestep in seconds

        // ESO interal variables
        float z1_;
        float z2_;
        float z3_;
     
        AP_Logger::PID_Info _debug_info;
  
};