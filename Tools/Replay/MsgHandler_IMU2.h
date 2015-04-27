#include "MsgHandler_IMU_Base.h"

class MsgHandler_IMU2 : public MsgHandler_IMU_Base
{
public:
    MsgHandler_IMU2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec,
                    uint8_t &_accel_mask, uint8_t &_gyro_mask,
                    AP_InertialSensor &_ins)
        : MsgHandler_IMU_Base(_f, _dataflash, _last_timestamp_usec,
                              _accel_mask, _gyro_mask, _ins) {};

    virtual void process_message(uint8_t *msg);
};
