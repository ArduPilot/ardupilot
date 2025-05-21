#pragma once

#if AP_PERIPH_BATTERY_BALANCE_ENABLED

class BattBalance {
public:
    friend class AP_Periph_FW;
    BattBalance(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 num_cells;
    AP_Int8 id;
    AP_Int8 cell1_pin;
    AP_Float rate;
    uint32_t last_send_ms;

    AP_HAL::AnalogSource **cells;
    uint8_t cells_allocated;
};

#endif // AP_PERIPH_BATTERY_BALANCE_ENABLED

