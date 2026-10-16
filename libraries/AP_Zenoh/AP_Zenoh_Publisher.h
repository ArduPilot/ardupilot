#pragma once

#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include <zenoh-pico.h>

class AP_Zenoh_Publisher
{
public:
    bool publish(const char *data);
    bool publish(const uint8_t *data, size_t len);
    bool valid() const
    {
        return _valid;
    }

private:
    z_owned_publisher_t _pub;
    bool _valid{false};

    friend class AP_Zenoh;
};

#endif // AP_ZENOH_ENABLED
