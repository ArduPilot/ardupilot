#pragma once

#include <AP_AuxFunc/AP_AuxFunc.h>

class AP_AuxFunc_Plane : public AP_AuxFunc
{

public:

protected:

    bool init_function(Function function, SwitchPos pos) override;
    bool do_function(Function function, SwitchPos) override;

private:

    void do_function_change_mode(Mode::Number number, SwitchPos pos);

    void do_function_q_assist_state(SwitchPos pos);

    void do_function_crow_mode(SwitchPos pos);

    void do_function_soaring_3pos(SwitchPos pos);

    void do_function_flare(SwitchPos pos);

    void do_function_mission_reset(const SwitchPos pos) override;

};
