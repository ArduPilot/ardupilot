#include "SIM_Airspeed_DLVR.h"

#include "SITL.h"

int SITL::Airspeed_DLVR::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    struct I2C::i2c_msg &msg = data->msgs[0];
    if (msg.flags == I2C_M_RD) {
        // driver is attempting to receive reading...
        if (msg.len != 4) {
            AP_HAL::panic("Unexpected message length (%u)", msg.len);
        }

        uint8_t status = 0;
        if (last_sent_ms == last_update_ms) {
            status |= 0b10;
        }
        last_sent_ms = last_update_ms;
        // if (electrical_fault_or_bad_config) {
        //     status |= 0b11;
        // }

        // calculation of packed-pressure value:

        // this is TYPE_I2C_DLVR_5IN
        const uint8_t range_inH2O = 5;

#define DLVR_OFFSET 8192.0f
#define DLVR_SCALE 16384.0f

//        const float pressure_pascals = 1555.2f;  // maximum transportable
        const float press_h2o = pressure * (1.0f/INCH_OF_H2O_TO_PASCAL);
        const uint32_t pressure_raw = ((press_h2o / (1.25f * 2.0f * range_inH2O)) * DLVR_SCALE) + DLVR_OFFSET;

        // calculation of packed-temperature value
        const uint32_t temp_raw = (temperature + 50.0f)  * (2047.0f/200.0f);

        const uint32_t packed =
            (status << 30) |
            ((pressure_raw & 0x3fff) << 16) |
            ((temp_raw & 0x7ff) << 5);

        msg.buf[0] = (packed >> 24) & 0xff;
        msg.buf[1] = (packed >> 16) & 0xff;
        msg.buf[2] = (packed >>  8) & 0xff;
        msg.buf[3] = (packed >>  0) & 0xff;

        return 0;
    }

    AP_HAL::panic("Should never be written to");
}


void SITL::Airspeed_DLVR::update(const class Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_update_ms < 50) { // 20Hz
        return;
    }
    last_update_ms = now_ms;

    pressure = AP::sitl()->state.airspeed_raw_pressure[0];

    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    // To Do: Add a sensor board temperature offset parameter
    temperature = AP_Baro::get_temperatureC_for_alt_amsl(sim_alt);
}
