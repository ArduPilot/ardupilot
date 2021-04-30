#pragma once

#include <AP_AuxFunc/AP_AuxFunc.h>
#include "mode.h"

class AP_AuxFunc_Copter : public AP_AuxFunc
{

public:

protected:

    bool init_function(Function ch_option, SwitchPos pos) override;
    bool do_function(Function ch_option, SwitchPos pos) override;

private:

    void do_function_armdisarm(const SwitchPos pos) override;
    void do_function_change_mode(const Mode::Number mode, const SwitchPos pos);
    void do_function_change_air_mode(const SwitchPos pos);

};
