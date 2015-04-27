#include "MsgHandler_IMU_Base.h"

void MsgHandler_IMU_Base::update_from_msg_imu(uint8_t imu_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    uint8_t this_imu_mask = 1 << imu_offset;

    if (gyro_mask & this_imu_mask) {
        Vector3f gyro;
        require_field(msg, "Gyr", gyro);
        ins.set_gyro(imu_offset, gyro);
    }
    if (accel_mask & this_imu_mask) {
        Vector3f accel2;
        require_field(msg, "Acc", accel2);
        ins.set_accel(imu_offset, accel2);
    }

    dataflash.Log_Write_IMU(ins);
}
