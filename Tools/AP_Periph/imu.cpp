#include "AP_Periph.h"

#if AP_PERIPH_IMU_ENABLED
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  update CAN magnetometer
 */
void AP_Periph_FW::can_imu_update(void)
{
    while (true) {
        // we need to delay by a ms value as hal->schedule->delay_microseconds_boost
        // used in wait_for_sample() takes uint16_t
        const uint32_t delay_ms = 1000U / g.imu_sample_rate;
        hal.scheduler->delay(delay_ms);

        if (delay_ms == 0) {
            // sleep for a bit to avoid flooding the CPU
            hal.scheduler->delay_microseconds(100);
        }

        imu.update();

        if (!imu.healthy()) {
            continue;
        }

        uavcan_equipment_ahrs_RawIMU pkt {};

        Vector3f tmp;
        imu.get_delta_velocity(tmp, pkt.integration_interval);
        pkt.accelerometer_integral[0] = tmp.x;
        pkt.accelerometer_integral[1] = tmp.y;
        pkt.accelerometer_integral[2] = tmp.z;

        imu.get_delta_angle(tmp, pkt.integration_interval);
        pkt.rate_gyro_integral[0] = tmp.x;
        pkt.rate_gyro_integral[1] = tmp.y;
        pkt.rate_gyro_integral[2] = tmp.z;

        tmp = imu.get_accel();
        pkt.accelerometer_latest[0] = tmp.x;
        pkt.accelerometer_latest[1] = tmp.y;
        pkt.accelerometer_latest[2] = tmp.z;

        tmp = imu.get_gyro();
        pkt.rate_gyro_latest[0] = tmp.x;
        pkt.rate_gyro_latest[1] = tmp.y;
        pkt.rate_gyro_latest[2] = tmp.z;
        
        uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_ahrs_RawIMU_encode(&pkt, buffer, !canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE,
                         UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID,
                         CANARD_TRANSFER_PRIORITY_HIGH,
                         &buffer[0],
                         total_size);
    }
}
#endif // AP_PERIPH_IMU_ENABLED
