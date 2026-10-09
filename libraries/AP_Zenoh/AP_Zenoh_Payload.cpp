#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include "AP_Zenoh_Payload.h"
#include <stdio.h>
#include <string.h>

const uint8_t *AP_Zenoh_Payload::data() const
{
    const z_loaned_bytes_t *payload = z_sample_payload(_sample);
    z_bytes_slice_iterator_t iter = z_bytes_get_slice_iterator(payload);
    z_view_slice_t slice;
    if (z_bytes_slice_iterator_next(&iter, &slice)) {
        return z_slice_data(z_loan(slice));
    }
    return nullptr;
}

size_t AP_Zenoh_Payload::len() const
{
    const z_loaned_bytes_t *payload = z_sample_payload(_sample);
    z_bytes_slice_iterator_t iter = z_bytes_get_slice_iterator(payload);
    z_view_slice_t slice;
    if (z_bytes_slice_iterator_next(&iter, &slice)) {
        return z_slice_len(z_loan(slice));
    }
    return 0;
}

size_t AP_Zenoh_Payload::key_expression(char *buf, size_t buf_len) const
{
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(_sample), &keystr);
    return snprintf(buf, buf_len, "%s", z_string_data(z_loan(keystr)));
}

z_sample_kind_t AP_Zenoh_Payload::kind() const
{
    return z_sample_kind(_sample);
}

bool AP_Zenoh_Payload::has_timestamp() const
{
    return z_sample_timestamp(_sample) != nullptr;
}

uint64_t AP_Zenoh_Payload::timestamp_ntp64() const
{
    const z_timestamp_t *ts = z_sample_timestamp(_sample);
    if (ts == nullptr) {
        return 0;
    }
    return z_timestamp_ntp64_time(ts);
}

bool AP_Zenoh_Payload::has_encoding() const
{
    const z_loaned_encoding_t *enc = z_sample_encoding(_sample);
    return enc != nullptr;
}

size_t AP_Zenoh_Payload::encoding(char *buf, size_t buf_len) const
{
    if (buf_len == 0) {
        return 0;
    }
    const z_loaned_encoding_t *enc = z_sample_encoding(_sample);
    z_owned_string_t enc_str;
    if (z_encoding_to_string(enc, &enc_str) == 0) {
        size_t ret = snprintf(buf, buf_len, "%s",
                              z_string_data(z_loan(enc_str)));
        z_drop(z_move(enc_str));
        return ret;
    }
    if (buf_len > 0) {
        buf[0] = '\0';
    }
    return 0;
}

bool AP_Zenoh_Payload::has_schema() const
{
    const z_loaned_encoding_t *enc = z_sample_encoding(_sample);
    return _z_string_check(&enc->schema);
}

size_t AP_Zenoh_Payload::schema(char *buf, size_t buf_len) const
{
    if (buf_len == 0) {
        return 0;
    }
    const z_loaned_encoding_t *enc = z_sample_encoding(_sample);
    const size_t slen = _z_string_len(&enc->schema);
    if (slen == 0) {
        buf[0] = '\0';
        return 0;
    }
    const char *sdata = _z_string_data(&enc->schema);
    const size_t copy_len = (slen < buf_len - 1) ? slen : (buf_len - 1);
    memcpy(buf, sdata, copy_len);
    buf[copy_len] = '\0';
    return copy_len;
}

#endif // AP_ZENOH_ENABLED
