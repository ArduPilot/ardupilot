#pragma once

#include "AC_ForceTorque.h"
#include "AC_ForceTorque_Backend_Serial.h"

class AC_ForceTorque_DR304_Serial : public AC_ForceTorque_Backend_Serial
{

public:

    using AC_ForceTorque_Backend_Serial::AC_ForceTorque_Backend_Serial;
    //void init_serial(uint8_t serial_instance) override;
private:

    // get a reading
    bool get_reading(Vector3f &reading_force_N, Vector3f &reading_torque_Nm,Vector3f &reading_force_N2, Vector3f &reading_torque_Nm2) override;

    uint8_t  linebuf[60];
    uint8_t  linebuf_len;
};