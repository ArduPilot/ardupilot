#pragma once

#include <AP_AuxFunc/AP_AuxFunc.h>

class AP_AuxFunc_Blimp : public AP_AuxFunc
{

public:

protected:

    bool init_function(AP_AuxFunc::Function function, SwitchPos) override;
    bool do_function(AP_AuxFunc::Function function, SwitchPos) override;

private:

    void do_function_change_mode(const Mode::Number mode, const SwitchPos pos);
    void do_function_change_air_mode(const SwitchPos pos);

};
