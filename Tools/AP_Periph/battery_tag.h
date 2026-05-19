#pragma once

#if AP_PERIPH_BATTERY_TAG_ENABLED

class BatteryTag {
public:
    friend class AP_Periph_FW;
    BatteryTag(void);

    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

private:
    uint32_t last_update_ms;
    uint32_t last_bcast_ms;

    AP_Int32 num_cycles;
    AP_Int32 serial_num;
    AP_Int32 capacity_mAh;
    AP_Int32 first_use_min;
    AP_Int32 last_use_min;
    AP_Float armed_hours;
    AP_Float cycle_min;

    uint32_t arm_start_ms;
    bool was_armed;
};

#endif // AP_PERIPH_BATTERY_TAG_ENABLED

