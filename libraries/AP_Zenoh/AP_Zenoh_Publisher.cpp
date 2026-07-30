#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include "AP_Zenoh_Publisher.h"

bool AP_Zenoh_Publisher::publish(const char *data)
{
    if (!_valid) {
        return false;
    }
    z_owned_bytes_t payload;
    z_bytes_copy_from_str(&payload, data);
    return z_publisher_put(z_loan(_pub), z_move(payload), NULL) == 0;
}

bool AP_Zenoh_Publisher::publish(const uint8_t *data, size_t len)
{
    if (!_valid) {
        return false;
    }
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf(&payload, data, len);
    return z_publisher_put(z_loan(_pub), z_move(payload), NULL) == 0;
}

#endif // AP_ZENOH_ENABLED
