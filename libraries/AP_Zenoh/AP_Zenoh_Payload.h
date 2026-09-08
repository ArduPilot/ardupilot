#pragma once

#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include <zenoh-pico.h>

class AP_Zenoh_Payload
{
public:
    AP_Zenoh_Payload(z_loaned_sample_t *sample) : _sample(sample) {}

    const uint8_t *data() const;
    size_t len() const;

    size_t key_expression(char *buf, size_t buf_len) const;
    z_sample_kind_t kind() const;

    bool has_timestamp() const;
    uint64_t timestamp_ntp64() const;

    bool has_encoding() const;
    size_t encoding(char *buf, size_t buf_len) const;

    bool has_schema() const;
    size_t schema(char *buf, size_t buf_len) const;

private:
    z_loaned_sample_t *_sample;
};

#endif // AP_ZENOH_ENABLED
