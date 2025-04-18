#pragma once

#if AP_PERIPH_BATTERY_INFO_ENABLED

class BattInfo {
public:
    friend class AP_Periph_FW;
    BattInfo(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int32 num_cycles;
    AP_Float num_hours;
};

#endif // AP_PERIPH_BATTERY_INFO_ENABLED

