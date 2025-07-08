/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  the BMI055 is unusual as it has separate chip-select for accel and
  gyro, which means it needs two SPIDevice pointers
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_SCHA63T : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

    enum reg_scha63t {
        RATE_XZ     = 0x01,
        RATE_Y      = 0x03,
        ACC_X       = 0x04,
        ACC_Y       = 0x05,
        ACC_Z       = 0x06,
        TEMP        = 0x07,
        S_SUM       = 0x0E,
        R_S1        = 0x10,
        A_S1        = 0x12,
        C_S1        = 0x14,
        C_S2        = 0x15,
        G_FILT_DYN  = 0x16,
        RESCTRL     = 0x18,
        MODE        = 0x19,
        A_FILT_DYN  = 0x1A,
        T_ID2       = 0x1C,
        T_ID0       = 0x1D,
        T_ID1       = 0x1E,
        SEL_BANK    = 0x1F,
    };

private:
    AP_InertialSensor_SCHA63T(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                              enum Rotation rotation);

    /*
      initialise driver
     */
    bool init();

    /*
      read data from the FIFOs
     */
    void read_accel();
    void read_gyro();

    bool read_register(uint8_t tp, reg_scha63t reg, uint8_t* val);
    bool write_register(uint8_t tp, reg_scha63t reg, uint16_t val);
    void set_temperature(uint8_t instance, int16_t temper);
    bool check_startup();

    AP_HAL::OwnPtr<AP_HAL::Device> dev_uno;
    AP_HAL::OwnPtr<AP_HAL::Device> dev_due;

    enum Rotation rotation;
};
