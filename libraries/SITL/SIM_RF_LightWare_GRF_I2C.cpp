#include "SIM_config.h"

#if AP_SIM_RF_LIGHTWARE_GRF_I2C_ENABLED

#include "SIM_RF_LightWare_GRF_I2C.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <string.h>

using namespace SITL;

#define GRF_SIM_REG_PRODUCT_NAME     0
#define GRF_SIM_REG_DISTANCE_OUTPUT  27
#define GRF_SIM_REG_DISTANCE_DATA_CM 44
#define GRF_SIM_REG_UPDATE_RATE      74
#define GRF_SIM_PRODUCT_NAME_LEN 16
#define GRF_SIM_U32_LEN          4
#define GRF_SIM_DISTANCE_LEN     8
#define GRF_SIM_MIN_RANGE_M   0.2f
#define GRF_SIM_MAX_RANGE_M   500.0f
#define GRF_SIM_STRENGTH_DB   100

// Copy src into dst (capped at dst_max) and zero-fill any leftover bytes.
// Returns the number of bytes copied from src.
static uint16_t emit_padded(uint8_t *dst, uint16_t dst_max,
                            const uint8_t *src, uint16_t src_len)
{
    const uint16_t copy = src_len < dst_max ? src_len : dst_max;
    memcpy(dst, src, copy);
    if (copy < dst_max) {
        memset(dst + copy, 0, dst_max - copy);
    }
    return copy;
}

void LightWareGRF_I2C::update(const class Aircraft &aircraft)
{
    range_m = aircraft.rangefinder_range();
}

uint8_t LightWareGRF_I2C::apply_write(const uint8_t *buf, uint16_t len)
{
    if (len == 0) {
        return last_register;
    }
    const uint8_t reg = buf[0];
    last_register = reg;

    // Just a register select, nothing to apply.
    if (len == 1) {
        return reg;
    }

    // Remaining bytes are the data for that register.
    const uint8_t *data = &buf[1];
    const uint16_t data_len = len - 1U;
    switch (reg) {
    case GRF_SIM_REG_UPDATE_RATE:
        if (data_len >= GRF_SIM_U32_LEN) {
            const uint32_t rate = le32toh_ptr(data);
            update_rate_hz = rate > 0 ? (uint8_t)rate : 1;
        }
        break;
    case GRF_SIM_REG_DISTANCE_OUTPUT:
        if (data_len >= GRF_SIM_U32_LEN) {
            distance_output_mask = le32toh_ptr(data);
        }
        break;
    default:
        break;
    }
    return reg;
}

uint16_t LightWareGRF_I2C::read_register(uint8_t reg, uint8_t *buf, uint16_t buf_max)
{
    switch (reg) {
    case GRF_SIM_REG_PRODUCT_NAME: {
        uint8_t name[GRF_SIM_PRODUCT_NAME_LEN] = {};
        memcpy(name, "GRF250", 6);
        return emit_padded(buf, buf_max, name, sizeof(name));
    }
    case GRF_SIM_REG_UPDATE_RATE: {
        uint8_t p[GRF_SIM_U32_LEN];
        put_le32_ptr(p, update_rate_hz);
        return emit_padded(buf, buf_max, p, sizeof(p));
    }
    case GRF_SIM_REG_DISTANCE_OUTPUT: {
        uint8_t p[GRF_SIM_U32_LEN];
        put_le32_ptr(p, distance_output_mask);
        return emit_padded(buf, buf_max, p, sizeof(p));
    }
    case GRF_SIM_REG_DISTANCE_DATA_CM: {
        // Sensor reports distance in 0.1 m steps; driver scales back up to cm.
        float dist_m = range_m;
        uint32_t strength = GRF_SIM_STRENGTH_DB;
        if (dist_m < GRF_SIM_MIN_RANGE_M) { dist_m = GRF_SIM_MIN_RANGE_M; strength = 0; }
        if (dist_m > GRF_SIM_MAX_RANGE_M) { dist_m = GRF_SIM_MAX_RANGE_M; strength = 0; }

        const uint32_t raw = (uint32_t)(dist_m * 100.0f) / 10U;
        uint8_t p[GRF_SIM_DISTANCE_LEN];
        put_le32_ptr(&p[0], raw);
        put_le32_ptr(&p[4], strength);
        return emit_padded(buf, buf_max, p, sizeof(p));
    }
    default:
        memset(buf, 0, buf_max);
        return buf_max;
    }
}

int LightWareGRF_I2C::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2 &&
        data->msgs[0].flags == I2C_RDWR &&
        data->msgs[1].flags == I2C_M_RD) {
        const uint8_t reg = apply_write(data->msgs[0].buf, data->msgs[0].len);
        read_register(reg, data->msgs[1].buf, data->msgs[1].len);
        return 0;
    }

    // Write-only: register select or a config write.
    if (data->nmsgs == 1 && data->msgs[0].flags == I2C_RDWR) {
        apply_write(data->msgs[0].buf, data->msgs[0].len);
        return 0;
    }

    // Read-only: respond based on the last register we were pointed at.
    if (data->nmsgs == 1 && data->msgs[0].flags == I2C_M_RD) {
        read_register(last_register, data->msgs[0].buf, data->msgs[0].len);
        return 0;
    }

    return -1;
}

#endif  // AP_SIM_RF_LIGHTWARE_GRF_I2C_ENABLED
