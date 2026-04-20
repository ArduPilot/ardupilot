#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED || AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED

#include <AP_Param/AP_Param.h>

#define GRF_MAX_DISTANCE_CM   50000
#define GRF_DEFAULT_RATE_HZ   50

// Shared parameters and helpers for the LightWare GRF-family rangefinders
// (GRF-250, GRF-500). Both the serial and I2C backends embed one of these
// so the GRF_* params and common logic live in exactly one place.
class AP_RangeFinder_LightWare_GRF_Common {
public:
    AP_RangeFinder_LightWare_GRF_Common();

    static const struct AP_Param::GroupInfo var_info[];

    // Supported command/message IDs by the GRF LightWare rangefinders. We don't use all of them yet.
    enum class MessageID : uint8_t {
        PRODUCT_NAME           = 0,
        HARDWARE_VERSION       = 1,
        FIRMWARE_VERSION       = 2,
        SERIAL_NUMBER          = 3,
        SAVE_PARAMETERS        = 12,
        RESET                  = 14,
        DISTANCE_OUTPUT        = 27,
        STREAM                 = 30,
        DISTANCE_DATA_CM       = 44,
        DISTANCE_DATA_MM       = 45,
        LASER_FIRING           = 50,
        TEMPERATURE            = 55,
        AUTO_EXPOSURE          = 70,
        UPDATE_RATE            = 74,
        ALARM_STATUS           = 76,
        RETURN_MODE            = 77,
        LOST_SIGNAL_COUNTER    = 78,
        MEDIAN_FILTER_ENABLE   = 86,
        MEDIAN_FILTER_SIZE     = 87,
        SMOOTHING_FILTER_ENABLE= 88,
        SMOOTHING_FACTOR       = 89,
        BAUD_RATE              = 91,
        I2C_ADDRESS            = 92,
        ROLLING_AVERAGE_ENABLE = 93,
        ROLLING_AVERAGE_SIZE   = 94,
        SLEEP_COMMAND          = 98,
        LED_STATE              = 110,
        ZERO_OFFSET            = 114
    };

    // Distance return modes
    enum class GRF_ReturnSelection : uint8_t {
        FIRST_RAW       = 0,
        FIRST_FILTERED  = 1,
        LAST_RAW        = 2,
        LAST_FILTERED   = 3
    };

    // Initialization configuration steps
    enum class ConfigStep : uint8_t {
        HANDSHAKE,
        UPDATE_RATE,
        DISTANCE_OUTPUT,
        STREAM,
        DONE
    };

    AP_Int8 return_selection; // first or last return, filtered or unfiltered
    AP_Int8 minimum_return_strength; // minimum acceptable signal strength in db
    AP_Int8 update_rate; // update rate in Hz

    // Checks if PRODUCT_NAME payload matches expected GRF signature
    static bool matches_product_name(const uint8_t *buf, uint16_t len);

    // Turn the chosen return-type param into the 4-byte value to write for
    // the sensor's distance-output config.
    uint32_t build_distance_output_bitmask() const;

    // Pull a distance (in metres) out of a raw 8-byte sensor reading.
    // Rejects zero / out-of-range distances and echoes below the strength
    // threshold. Returns false if the reading should be discarded.
    bool parse_distance_cm_payload(const uint8_t *payload, uint16_t len,
                                   float &distance_m) const;
};

#endif  // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED || AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED
