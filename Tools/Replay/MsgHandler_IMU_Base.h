#ifndef AP_MSGHANDLER_IMU_BASE_H
#define AP_MSGHANDLER_IMU_BASE_H

#include <MsgHandler.h>

class MsgHandler_IMU_Base : public MsgHandler
{
public:
    MsgHandler_IMU_Base(log_Format &_f, DataFlash_Class &_dataflash,
                        uint64_t &_last_timestamp_usec,
                        uint8_t &_accel_mask, uint8_t &_gyro_mask,
                        AP_InertialSensor &_ins) :
        MsgHandler(_f, _dataflash, _last_timestamp_usec),
        accel_mask(_accel_mask),
        gyro_mask(_gyro_mask),
        ins(_ins) { };
    void update_from_msg_imu(uint8_t gps_offset, uint8_t *msg);

private:
    uint8_t &accel_mask;
    uint8_t &gyro_mask;
    AP_InertialSensor &ins;
};

#endif

