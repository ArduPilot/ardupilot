#pragma once

#include "AP_Compass.h"

#if AP_COMPASS_DRONECAN_ENABLED

#include "AP_Compass_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_Compass_DroneCAN : public AP_Compass_Backend {
public:
    AP_Compass_DroneCAN(AP_DroneCAN* ap_dronecan, uint32_t devid);

    void        read(void) override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_Compass_Backend* probe(uint8_t index);
    static uint32_t get_detected_devid(uint8_t index) { return _detected_modules[index].devid; }
    static void handle_magnetic_field(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength& msg);
    static void handle_magnetic_field_2(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength2 &msg);
#if AP_COMPASS_DRONECAN_HIRES_ENABLED
    static void handle_magnetic_field_hires(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes &msg);
#endif

private:
    bool init();

    // callback for DroneCAN messages
    void handle_mag_msg(const Vector3f &mag);

    static AP_Compass_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t sensor_id);

    uint8_t  _instance;

    uint32_t _devid;

    // Module Detection Registry
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;
        uint8_t node_id;
        uint8_t sensor_id;
        AP_Compass_DroneCAN *driver;
        uint32_t devid;
    } _detected_modules[COMPASS_MAX_BACKEND];

    static HAL_Semaphore _sem_registry;
};

#endif  // AP_COMPASS_DRONECAN_ENABLED
