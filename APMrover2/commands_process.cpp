/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

// called by update navigation at 10Hz
// --------------------
void Rover::update_commands(void)
{
    if(control_mode == AUTO) {
        if (home_is_set != HOME_UNSET && mission.num_commands() > 1) {
            mission.update();
        }
    }
}
