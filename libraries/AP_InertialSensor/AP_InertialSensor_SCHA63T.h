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

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (msb << 8u) | lsb;
}

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
        S_SUM       = 0x0E,
        R_S1        = 0x10,
        A_S1        = 0x12,
        C_S1        = 0x14,
        C_S2        = 0x15,
        G_FILT_DYN  = 0x16,
        RESCTRL     = 0x18,
        MODE        = 0x19,
        A_FILT_DYN  = 0x1A,
        X_1C        = 0x1C,
        X_1D        = 0x1D,
        X_1E        = 0x1E,
        SEL_BANK    = 0x1F,
        SET_EOI     = 0x20,
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
    void read_fifo_accel();
    void read_fifo_gyro();

    int16_t gyro_x;
    bool RegisterRead(int tp, reg_scha63t reg, uint8_t* val);
    bool RegisterWrite(int tp, reg_scha63t reg, uint8_t val);
    void set_temperature(uint8_t instance, uint8_t temper1, uint8_t temper2);
    uint8_t CalcTblCrc(uint8_t* ptr, short nLen);

    AP_HAL::OwnPtr<AP_HAL::Device> dev_accel;
    AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
};
