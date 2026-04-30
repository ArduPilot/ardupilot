#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED

#include "AP_RangeFinder_LightWare_GRF_I2C.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define GRF_I2C_PRODUCT_NAME_PAYLOAD_LEN 16
#define GRF_I2C_U32_PAYLOAD_LEN          4
#define GRF_I2C_DISTANCE_PAYLOAD_LEN     8
#define GRF_I2C_MAX_WRITE_PAYLOAD_LEN    16
#define GRF_I2C_HANDSHAKE_ATTEMPTS       4
#define GRF_I2C_HANDSHAKE_RETRY_MS       30
#define GRF_I2C_POLL_HZ                  20

const AP_Param::GroupInfo AP_RangeFinder_LightWare_GRF_I2C::var_info[] = {
    // @Group: GRF_
    // @Path: AP_RangeFinder_LightWare_GRF_Common.cpp
    AP_SUBGROUPINFO(_common, "GRF_", 12, AP_RangeFinder_LightWare_GRF_I2C, AP_RangeFinder_LightWare_GRF_Common),

    AP_GROUPEND
};

AP_RangeFinder_LightWare_GRF_I2C::AP_RangeFinder_LightWare_GRF_I2C(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params,
    AP_HAL::I2CDevice &_dev)
    : AP_RangeFinder_Backend_I2C(_state, _params, _dev)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// Write a command
bool AP_RangeFinder_LightWare_GRF_I2C::write_command(MessageID msgid,
                                                    const uint8_t *payload,
                                                    uint16_t payload_len)
{
    uint8_t tx[1 + GRF_I2C_MAX_WRITE_PAYLOAD_LEN];
    if (1U + payload_len > sizeof(tx)) {
        return false;
    }
    tx[0] = (uint8_t)msgid;
    if (payload_len > 0 && payload != nullptr) {
        memcpy(&tx[1], payload, payload_len);
    }
    return dev.transfer(tx, 1U + payload_len, nullptr, 0);
}

// Read a register and hand back the raw bytes.
bool AP_RangeFinder_LightWare_GRF_I2C::read_payload(MessageID msgid,
                                                   uint8_t *payload_buf,
                                                   uint16_t payload_len)
{
    const uint8_t reg = (uint8_t)msgid;
    return dev.transfer(&reg, 1, payload_buf, payload_len);
}

bool AP_RangeFinder_LightWare_GRF_I2C::init()
{
    WITH_SEMAPHORE(dev.get_semaphore());
    dev.set_retries(2);

    // Ask the sensor for its product name. The reply starts with "GRF" if
    // we've found a GRF-family rangefinder. Retry a few times in case the
    // sensor is still waking up.
    uint8_t name[GRF_I2C_PRODUCT_NAME_PAYLOAD_LEN];
    bool detected = false;
    for (uint8_t attempt = 0; attempt < GRF_I2C_HANDSHAKE_ATTEMPTS && !detected; attempt++) {
        if (!read_payload(MessageID::PRODUCT_NAME, name, sizeof(name)) ||
            !AP_RangeFinder_LightWare_GRF_Common::matches_product_name(name, sizeof(name))) {
            hal.scheduler->delay(GRF_I2C_HANDSHAKE_RETRY_MS);
            continue;
        }
        detected = true;
    }
    if (!detected) {
        return false;
    }
    name[sizeof(name) - 1] = '\0';
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LightWare %s detected (I2C)", (const char*)name);

    // Configuration happens inside the timer so we don't hold up boot.
    _config_step = ConfigStep::WRITE_UPDATE_RATE;
    _config_retry = 0;

    dev.register_periodic_callback(1000000UL / GRF_I2C_POLL_HZ,
        FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWare_GRF_I2C::timer, void));

    return true;
}

// Run one tick of the config FSM: each call does exactly one I2C op.
// Each WRITE_* step is followed by a VERIFY_* step that reads the register
// back; mismatches retry the write a few times then give up and move on.
void AP_RangeFinder_LightWare_GRF_I2C::run_config_step()
{
    uint8_t payload[GRF_I2C_U32_PAYLOAD_LEN];
    uint8_t rx[GRF_I2C_U32_PAYLOAD_LEN];

    switch (_config_step) {
    case ConfigStep::WRITE_UPDATE_RATE:
        put_le32_ptr(payload, (uint32_t)_common.update_rate);
        (void)write_command(MessageID::UPDATE_RATE, payload, sizeof(payload));
        _config_step = ConfigStep::VERIFY_UPDATE_RATE;
        return;

    case ConfigStep::VERIFY_UPDATE_RATE:
        put_le32_ptr(payload, (uint32_t)_common.update_rate);
        if (!read_payload(MessageID::UPDATE_RATE, rx, sizeof(rx)) ||
            memcmp(rx, payload, sizeof(rx)) != 0) {
            if (++_config_retry < 3) {
                _config_step = ConfigStep::WRITE_UPDATE_RATE;
                return;
            }
        }
        _config_retry = 0;
        _config_step = ConfigStep::WRITE_DISTANCE_OUTPUT;
        return;

    case ConfigStep::WRITE_DISTANCE_OUTPUT:
        put_le32_ptr(payload, _common.build_distance_output_bitmask());
        (void)write_command(MessageID::DISTANCE_OUTPUT, payload, sizeof(payload));
        _config_step = ConfigStep::VERIFY_DISTANCE_OUTPUT;
        return;

    case ConfigStep::VERIFY_DISTANCE_OUTPUT:
        put_le32_ptr(payload, _common.build_distance_output_bitmask());
        if (!read_payload(MessageID::DISTANCE_OUTPUT, rx, sizeof(rx)) ||
            memcmp(rx, payload, sizeof(rx)) != 0) {
            if (++_config_retry < 3) {
                _config_step = ConfigStep::WRITE_DISTANCE_OUTPUT;
                return;
            }
        }
        _config_retry = 0;
        _config_step = ConfigStep::DONE;
        return;

    case ConfigStep::DONE:
        return;
    }
}

void AP_RangeFinder_LightWare_GRF_I2C::timer()
{
    // Finish boot-time config before we start polling for distance readings.
    // The gap between timer calls gives the sensor time to apply each write.
    if (_config_step != ConfigStep::DONE) {
        run_config_step();
        return;
    }

    uint8_t rx[GRF_I2C_DISTANCE_PAYLOAD_LEN];
    if (!read_payload(MessageID::DISTANCE_DATA_CM, rx, sizeof(rx))) {
        return;
    }

    float dist_m;
    if (!_common.parse_distance_cm_payload(rx, sizeof(rx), dist_m)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _accum.sum_m += dist_m;
    _accum.count++;
}

void AP_RangeFinder_LightWare_GRF_I2C::update()
{
    WITH_SEMAPHORE(_sem);

    if (_accum.count > 0) {
        state.distance_m = _accum.sum_m / _accum.count;
        state.last_reading_ms = AP_HAL::millis();
        _accum.sum_m = 0;
        _accum.count = 0;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 500) {
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED
