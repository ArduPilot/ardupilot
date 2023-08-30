#pragma once

#ifdef HAL_PERIPH_ENABLE_RCIN

class Parameters_RCIN {
public:
    Parameters_RCIN(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int32 rcin_protocols;
    AP_Int8 rcin_rate_hz;
    AP_Int8 rcin1_port;
    AP_Int16 rcin1_port_options;
};

#endif
