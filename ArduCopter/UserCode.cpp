#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    tmrs_motor_power_monitor.init();
    tmrs_tether_power_monitor.init();
    tmrs_payload_controller.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    /**
     * TMRS sensors
     */
    uint8_t n = gcs().num_gcs();
    for(uint8_t i = 0; i < n; i++) {
        if(gcs().chan(i).initialised) {
            tmrs_motor_power_monitor.send_mavlink_tmrs_motor_status(gcs().chan(i).get_chan());
            tmrs_tether_power_monitor.send_mavlink_tether_power_status(gcs().chan(i).get_chan());
            tmrs_payload_controller.send_mavlink_tmrs_payload_status(gcs().chan(i).get_chan());
        }
    }
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
