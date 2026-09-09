#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InternalError/AP_InternalError.h>

#include "msp.h"
#include "msp_sbuf.h"

#include <string.h>

#if HAL_MSP_ENABLED

uint8_t* MSP::sbuf_ptr(sbuf_t *buf)
{
    return buf->ptr;
}

uint16_t MSP::sbuf_bytes_remaining(const sbuf_t *buf)
{
    return buf->end - buf->ptr;
}

bool MSP::sbuf_check_bounds(const sbuf_t *buf, const uint8_t len)
{
    if (sbuf_bytes_remaining(buf) < len) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    return true;
}

void MSP::sbuf_switch_to_reader(sbuf_t *buf, uint8_t *base)
{
    buf->end = buf->ptr;
    buf->ptr = base;
}

void MSP::sbuf_write_data(sbuf_t *dst, const void *data, int len)
{
    if (!sbuf_check_bounds(dst, len)) {
        return;
    }
    memcpy(dst->ptr, data, len);
    dst->ptr += len;
}

uint8_t MSP::sbuf_read_u8(sbuf_t *src)
{
    if (!sbuf_check_bounds(src, 1)) {
        return 0;
    }
    return *src->ptr++;
}

uint16_t MSP::sbuf_read_u16(sbuf_t *src)
{
    if (!sbuf_check_bounds(src, 2)) {
        return 0;
    }
    uint16_t val = le16toh_ptr(src->ptr);
    src->ptr += 2;
    return val;
}

#endif //HAL_MSP_ENABLED