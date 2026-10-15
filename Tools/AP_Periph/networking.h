#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if AP_PERIPH_NETWORKING_ENABLED

#include <AP_Networking/AP_Networking.h>

#ifndef HAL_PERIPH_NETWORK_NUM_PASSTHRU
#define HAL_PERIPH_NETWORK_NUM_PASSTHRU 2
#endif

#ifndef AP_PERIPH_NET_PPP_PORT_DEFAULT
#define AP_PERIPH_NET_PPP_PORT_DEFAULT -1
#endif

#ifndef AP_PERIPH_NET_PPP_BAUD_DEFAULT
#define AP_PERIPH_NET_PPP_BAUD_DEFAULT 12500000
#endif

#ifndef HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
#define HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS 0
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
        // configure a serial port for passthru
        void configure_serial_port(AP_HAL::UARTDriver *port, uint32_t baud, uint32_t options, int8_t parity, int8_t stop_bits);

        AP_Int8 enabled;
        AP_Int8 ep1;
        AP_Int8 ep2;
        AP_Int32 baud1;
        AP_Int32 baud2;
        AP_Int32 options1;
        AP_Int32 options2;
#if HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
        AP_Int8 parity1;
        AP_Int8 stop_bits1;
        AP_Int8 parity2;
        AP_Int8 stop_bits2;
#endif

        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
    } passthru[HAL_PERIPH_NETWORK_NUM_PASSTHRU];
#endif // HAL_PERIPH_NETWORK_NUM_PASSTHRU

    AP_Networking networking_lib;
    bool got_addresses;

#if AP_NETWORKING_BACKEND_PPP
    AP_Int8 ppp_port;
    AP_Int32 ppp_baud;
#endif
};

#endif // AP_PERIPH_NETWORKING_ENABLED
