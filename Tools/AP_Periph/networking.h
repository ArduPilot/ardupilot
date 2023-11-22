#pragma once

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_NETWORKING

#ifndef HAL_PERIPH_NETWORK_NUM_PASSTHRU
#define HAL_PERIPH_NETWORK_NUM_PASSTHRU 2
#endif

class Networking_Periph {
public:
    Networking_Periph() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

private:

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    class Passthru {
    public:
        friend class Networking_Periph;

        CLASS_NO_COPY(Passthru);

        Passthru() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        void init();
        void update();
        
        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 enabled;
        AP_Int8 ep1;
        AP_Int8 ep2;
        AP_Int32 baud1;
        AP_Int32 baud2;
        AP_Int32 options1;
        AP_Int32 options2;

        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
    } passthru[HAL_PERIPH_NETWORK_NUM_PASSTHRU];
#endif // HAL_PERIPH_NETWORK_NUM_PASSTHRU

    AP_Networking networking_lib;
};

#endif // HAL_PERIPH_ENABLE_BATTERY_BALANCE

