#pragma once

#include <AP_AuxFunc/AP_AuxFunc.h>

#include "mode.h"

class AP_AuxFunc_Rover : public AP_AuxFunc
{

public:

protected:

    bool init_function(Function function, SwitchPos) override;
    bool do_function(Function function, SwitchPos) override;

private:

    void do_function_change_mode(Mode &mode, const SwitchPos pos);

    void add_waypoint_for_current_loc();

    void do_function_sailboat_motor_3pos(const SwitchPos pos);
};
