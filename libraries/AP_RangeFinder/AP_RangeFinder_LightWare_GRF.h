#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include <AP_LightWareSerial/AP_LightWareSerial.h>

class AP_RangeFinder_LightWareGRF : public AP_RangeFinder_Backend_Serial, AP_LightWareSerial {
public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_LightWareGRF(_state, _params);
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Returns the MAVLink distance sensor type
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

    // Called periodically to fetch a new range reading
    bool get_reading(float &reading_m) override;

    // Returns read timeout in milliseconds
    uint16_t read_timeout_ms() const override { return 500; }

private:
    // Constructor
    AP_RangeFinder_LightWareGRF(RangeFinder::RangeFinder_State &_state,
                          AP_RangeFinder_Params &_params);

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

    // Send configuration messages to the rangefinder
    void configure_rangefinder();

    // Parses config responses and advances setup step
    void check_config(const MessageID &resp_cmd_id, const uint8_t* response_buf, const uint16_t& response_len);

    // Checks if PRODUCT_NAME payload matches expected GRF signature
    bool matches_product_name(const uint8_t *buf, const uint16_t len);

    // Processes the latest message held in the _msg structure
    void process_message(float &sum_m, uint8_t &count);

    uint32_t last_config_message_ms; // last time we sent an config message
    ConfigStep config_step; // current configuration step

    AP_Int8 return_selection; // first or last return, filtered or unfiltered
    AP_Int8 minimum_return_strength; // minimum acceptable signal strength in db
    AP_Int8 update_rate; // update rate in Hz
};

#endif // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED
