#include "MsgHandler_MAG_Base.h"

void MsgHandler_MAG_Base::update_from_msg_compass(uint8_t compass_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    Vector3f mag;
    require_field(msg, "Mag", mag);
    Vector3f mag_offset;
    require_field(msg, "Ofs", mag_offset);

    compass.setHIL(mag - mag_offset);
    // compass_offset is which compass we are setting info for;
    // mag_offset is a vector indicating the compass' calibration...
    compass.set_offsets(compass_offset, mag_offset);

    dataflash.Log_Write_Compass(compass);
}

