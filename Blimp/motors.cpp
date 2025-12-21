#include "Blimp.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Blimp::motors_output()
{
    // output any servo channels
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();

    // cork now, so that all channel outputs happen at once
    srv.cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // send output signals to motors
    motors->output();

    // push all channels
    srv.push();
}
