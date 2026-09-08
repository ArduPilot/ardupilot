#pragma once

#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <zenoh-pico.h>

#include "AP_Zenoh_Publisher.h"
#include "AP_Zenoh_Subscriber.h"

struct AP_Zenoh_ConfigItem {
    uint8_t key;
    const char *value;
};

class AP_Zenoh
{
public:
    AP_Zenoh(const AP_Zenoh_ConfigItem *config_items, uint8_t config_count);

    CLASS_NO_COPY(AP_Zenoh);

    static AP_Zenoh *get_singleton()
    {
        return _singleton;
    }

    bool init();

    bool declare_publisher(const char *key_expr, AP_Zenoh_Publisher &pub);
    bool declare_subscriber(const char *key_expr,
                            AP_Zenoh_Subscriber &sub,
                            AP_Zenoh_Subscriber::callback_t callback);

    static constexpr const char *msg_prefix = "Zenoh";

private:
    static AP_Zenoh *_singleton;

    const AP_Zenoh_ConfigItem *_config_items;
    uint8_t _config_count;

    z_owned_session_t _session;
    bool _initialized;
};

#endif // AP_ZENOH_ENABLED
