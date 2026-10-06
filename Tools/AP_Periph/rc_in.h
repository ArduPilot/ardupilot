#pragma once

#include "elrs.h"

#if AP_PERIPH_RCIN_ENABLED || AP_PERIPH_ELRS_ENABLED

class Parameters_RCIN {
public:
    Parameters_RCIN(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 rcin_rate_hz;
#if AP_PERIPH_RCIN_ENABLED
    AP_Int32 rcin_protocols;
    AP_Int8 rcin1_port;
    AP_Int16 rcin1_port_options;
#endif
#if AP_PERIPH_ELRS_ENABLED
    Parameters_ELRS elrs;
#endif
};

#endif
