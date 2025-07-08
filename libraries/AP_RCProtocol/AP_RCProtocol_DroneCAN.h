#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_DRONECAN_ENABLED

#include "AP_RCProtocol_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

#include <AP_Common/missing/endian.h>

class AP_RCProtocol_DroneCAN : public AP_RCProtocol_Backend {
public:

    AP_RCProtocol_DroneCAN(AP_RCProtocol &_frontend) :
        AP_RCProtocol_Backend(_frontend) {
        _singleton = this;
    }

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    void update() override;

private:

    static class AP_RCProtocol_DroneCAN *_singleton;

    static void handle_rcinput(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_rc_RCInput &msg);

    static AP_RCProtocol_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    struct {
        uint8_t quality;
        union {
            uint16_t status;
            struct {
                uint8_t QUALITY_VALID : 1;
                uint8_t FAILSAFE : 1;
            } bits;
        };
        uint8_t num_channels;
        uint16_t channels[MAX_RCIN_CHANNELS];

        uint32_t last_sample_time_ms;
        HAL_Semaphore sem;
    } rcin;

    // Module Detection Registry
    static struct Registry {
        struct DetectedDevice {
            AP_DroneCAN* ap_dronecan;
            uint8_t node_id;
            AP_RCProtocol_DroneCAN *driver;
        } detected_devices[1];
        HAL_Semaphore sem;
    } registry;

    uint32_t last_receive_ms;
};


#endif  // AP_RCPROTOCOL_DRONECAN_ENABLED
