#include "AP_RangeFinder_LightWare_GRF_Common.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED || AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/AP_Common.h>

const AP_Param::GroupInfo AP_RangeFinder_LightWare_GRF_Common::var_info[] = {
    // @Param: RET
    // @DisplayName: LightWare GRF Distance Return Type
    // @Description: Selects which single return to use.
    // @Values: 0:FirstRaw,1:FirstFiltered,2:LastRaw,3:LastFiltered
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RET", 1, AP_RangeFinder_LightWare_GRF_Common, return_selection, (uint8_t)GRF_ReturnSelection::FIRST_RAW),

    // @Param: ST
    // @DisplayName: LightWare GRF Minimum Return Strength
    // @Description: Minimum acceptable return signal strength in dB. Returns weaker than this will be ignored. Set to 0 to disable filtering.
    // @Range: 0 255
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ST", 2, AP_RangeFinder_LightWare_GRF_Common, minimum_return_strength, 0),

    // @Param: RATE
    // @DisplayName: LightWare GRF Update Rate
    // @Description: The update rate of the sensor in Hz.
    // @Range: 1 50
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("RATE", 3, AP_RangeFinder_LightWare_GRF_Common, update_rate, GRF_DEFAULT_RATE_HZ),

    AP_GROUPEND
};

AP_RangeFinder_LightWare_GRF_Common::AP_RangeFinder_LightWare_GRF_Common()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_RangeFinder_LightWare_GRF_Common::matches_product_name(const uint8_t *buf, uint16_t len)
{
    // Must be at least "GRFXXX\0" = 7 bytes
    if (len < 7) {
        return false;
    }

    // Compare the first 3 bytes with "GRF"
    return strncmp((const char*)buf, "GRF", 3) == 0;
}

uint32_t AP_RangeFinder_LightWare_GRF_Common::build_distance_output_bitmask() const
{
    // Two bits out of the six below are set: one picks the distance value
    // we want, the other asks for the matching strength reading alongside.
    //   0: first raw   1: first filtered  2: first strength
    //   3: last raw    4: last filtered   5: last strength
    uint8_t data_bit = 1;      // default: first filtered
    uint8_t strength_bit = 2;  // default: first strength
    switch (GRF_ReturnSelection(return_selection.get())) {
    case GRF_ReturnSelection::FIRST_RAW:
        data_bit = 0;
        break;
    case GRF_ReturnSelection::LAST_RAW:
        data_bit = 3;
        strength_bit = 5;
        break;
    case GRF_ReturnSelection::LAST_FILTERED:
        data_bit = 4;
        strength_bit = 5;
        break;
    case GRF_ReturnSelection::FIRST_FILTERED:
        break;
    }
    return (1U << data_bit) | (1U << strength_bit);
}

bool AP_RangeFinder_LightWare_GRF_Common::parse_distance_cm_payload(const uint8_t *payload,
                                                                    uint16_t len,
                                                                    float &distance_m) const
{
    if (len < 8) {
        return false;
    }

    // Sensor reports distance in 0.1 m steps, so scale up to cm before use.
    const uint32_t distance_cm = le32toh_ptr(&payload[0]) * 10;
    const uint32_t strength_db = le32toh_ptr(&payload[4]);

    if (distance_cm == 0 || distance_cm > GRF_MAX_DISTANCE_CM) {
        return false;
    }

    if (minimum_return_strength > 0 && strength_db < (uint32_t)minimum_return_strength) {
        return false;
    }

    distance_m = distance_cm * 0.01f;
    return true;
}

#endif  // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED || AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED
