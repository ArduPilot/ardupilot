#pragma once

#include "AC_ForceTorque_Backend.h"

class AC_ForceTorque_Backend_Serial : public AC_ForceTorque_Backend
{
public:
    // constructor
    AC_ForceTorque_Backend_Serial(ForceTorque::ForceTorque_State &_state,
                                  AC_ForceTorque_Params &_params);

    void init_serial(uint8_t serial_instance) override;
    // static detection function
    static bool detect(uint8_t serial_instance);

protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *uart = nullptr;

    // update state; not all backends call this!
    virtual void update(void) override;

    // it is essential that anyone relying on the base-class update to
    // implement this:
    virtual bool get_reading(Vector3f &reading_force_N, Vector3f &reading_torque_Nm,Vector3f &reading_force_N2, Vector3f &reading_torque_Nm2) = 0;

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const { return 200; }
};