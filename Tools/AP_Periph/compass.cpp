#include "AP_Periph.h"

#if AP_PERIPH_MAG_ENABLED

/*
  magnetometer support
 */

#include <dronecan_msgs.h>

#ifndef AP_PERIPH_MAG_MAX_RATE
#define AP_PERIPH_MAG_MAX_RATE 25U
#endif

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

#ifndef AP_PERIPH_MAG_HIRES
#define AP_PERIPH_MAG_HIRES 0
#endif

extern const AP_HAL::HAL &hal;

/*
  update CAN magnetometer
 */
void AP_Periph_FW::can_mag_update(void)
{
    if (!compass.available()) {
        return;
    }

#if AP_PERIPH_MAG_MAX_RATE > 0
    // don't flood the bus with very high rate magnetometers
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_mag_update_ms < (1000U / AP_PERIPH_MAG_MAX_RATE)) {
        return;
    }
#endif

    compass.read();

#if AP_PERIPH_PROBE_CONTINUOUS
    if (option_is_set(PeriphOptions::PROBE_CONTINUOUS) && !hal.util->get_soft_armed() && (compass.get_count() == 0)) {
        static uint32_t last_probe_ms;
        uint32_t now = AP_HAL::millis();
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            compass.init();
        }
    }
#endif

    if (last_mag_update_ms == compass.last_update_ms()) {
        return;
    }
    if (!compass.healthy()) {
        return;
    }

    last_mag_update_ms = compass.last_update_ms();
    Vector3f field_Ga = compass.get_field() * 0.001;

#if !AP_PERIPH_MAG_HIRES
    // normal message, float16 values
    uavcan_equipment_ahrs_MagneticFieldStrength pkt {};

    for (uint8_t i=0; i<3; i++) {
        pkt.magnetic_field_ga[i] = field_Ga[i];
    }
    
    uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_ahrs_MagneticFieldStrength_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE,
                    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#else
    // High resolution magnetometer, for magnetic surveys
    dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes pkt {};

    for (uint8_t i=0; i<3; i++) {
        pkt.magnetic_field_ga[i] = field_Ga[i];
    }
    
    uint8_t buffer[DRONECAN_SENSORS_MAGNETOMETER_MAGNETICFIELDSTRENGTHHIRES_MAX_SIZE];
    uint16_t total_size = dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(DRONECAN_SENSORS_MAGNETOMETER_MAGNETICFIELDSTRENGTHHIRES_SIGNATURE,
                     DRONECAN_SENSORS_MAGNETOMETER_MAGNETICFIELDSTRENGTHHIRES_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
#endif // AP_PERIPH_MAG_HIRES
}

#endif // AP_PERIPH_MAG_ENABLED
