#include "Rover.h"

// update mission including starting or stopping commands. called by scheduler at 10Hz
void Rover::update_mission(void)
{
    if (control_mode == &mode_auto) {
        if (ahrs.home_is_set() && mode_auto.mission.num_commands() > 1) {
            mode_auto.mission.update();
        }
    }
}
