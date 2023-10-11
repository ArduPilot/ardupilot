/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  FCM320 barometer driver
 */

#include "AP_Baro_FBM320.h"

#if AP_BARO_FBM320_ENABLED

#include <utility>
#include <stdio.h>
#include <AP_Math/definitions.h>

extern const AP_HAL::HAL &hal;

#define FBM320_REG_ID   0x6B
#define FBM320_REG_DATA 0xF6
#define FBM320_REG_CMD  0xF4

#define FBM320_CMD_READ_T 0x2E
#define FBM320_CMD_READ_P 0xF4

#define FBM320_WHOAMI 0x42

AP_Baro_FBM320::AP_Baro_FBM320(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Baro_Backend(baro)
    , dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_FBM320::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev)
{
    if (!_dev) {
        return nullptr;
    }

    AP_Baro_FBM320 *sensor = new AP_Baro_FBM320(baro, std::move(_dev));
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/*
  read calibration data
 */
bool AP_Baro_FBM320::read_calibration(void)
{
    uint8_t tmp[2];
    uint16_t R[10];

    for (uint8_t i=0; i<9; i++) {
        if (!dev->read_registers(0xAA+(i*2),&tmp[0],1)) {
            return false;
        }
        if (!dev->read_registers(0xAB+(i*2),&tmp[1],1)) {
            return false;
        }
        R[i] = ((uint8_t)tmp[0] << 8 | tmp[1]);
    }

    if (!dev->read_registers(0xA4,&tmp[0],1)) {
        return false;
    }
    if (!dev->read_registers(0xF1,&tmp[0],1)) {
        return false;
    }
    R[9] = ((uint8_t)tmp[0] << 8) | tmp[1];


    /*    Use R0~R9 calculate C0~C12 of FBM320-02    */
    calibration.C0 = R[0] >> 4;
    calibration.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
    calibration.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
    calibration.C3 = R[2] >> 3;
    calibration.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
    calibration.C5 = R[4] >> 1;
    calibration.C6 = R[5] >> 3;
    calibration.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
    calibration.C8 = R[7] >> 3;
    calibration.C9 = R[8] >> 2;
    calibration.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
    calibration.C11 = R[9] & 0xFF;
    calibration.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);

    return true;
}

bool AP_Baro_FBM320::init()
{
    if (!dev) {
        return false;
    }
    dev->get_semaphore()->take_blocking();

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t whoami;
    if (!dev->read_registers(FBM320_REG_ID, &whoami, 1) ||
        whoami != FBM320_WHOAMI) {
        // not a FBM320
        dev->get_semaphore()->give();
        return false;
    }
    printf("FBM320 ID 0x%x\n", whoami);

    if (!read_calibration()) {
        dev->get_semaphore()->give();
        return false;
    }

    dev->write_register(FBM320_REG_CMD, FBM320_CMD_READ_T);

    instance = _frontend.register_sensor();

    dev->set_device_type(DEVTYPE_BARO_FBM320);
    set_bus_id(instance, dev->get_bus_id());
    
    dev->get_semaphore()->give();

    // request 50Hz update
    dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_FBM320::timer, void));

    return true;
}

/*
  calculate corrected pressure and temperature
 */
void AP_Baro_FBM320::calculate_PT(int32_t UT, int32_t UP, int32_t &pressure, int32_t &temperature)
{
    const struct fbm320_calibration &cal = calibration;
    int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

    DT  = ((UT - 8388608) >> 4) + (cal.C0 << 4);
    X01 = (cal.C1 + 4459) * DT >> 1;
    X02 = ((((cal.C2 - 256) * DT) >> 14) * DT) >> 4;
    X03 = (((((cal.C3 * DT) >> 18) * DT) >> 18) * DT);

    temperature = ((2500 << 15) - X01 - X02 - X03) >> 15;

    DT2 = (X01 + X02 + X03) >> 12;
    X11 = ((cal.C5 - 4443) * DT2);
    X12 = (((cal.C6 * DT2) >> 16) * DT2) >> 2;
    X13 = ((X11 + X12) >> 10) + ((cal.C4 + 120586) << 4);

    X21 = ((cal.C8 + 7180) * DT2) >> 10;
    X22 = (((cal.C9 * DT2) >> 17) * DT2) >> 12;
    X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);

    X24 = (X23 >> 11) * (cal.C7 + 166426);
    X25 = ((X23 & 0x7FF) * (cal.C7 + 166426)) >> 11;
    X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + cal.C7 + 166426) : (((X24 + X25) >> 11) + cal.C7 + 166426);

    PP1 = ((UP - 8388608) - X13) >> 3;
    PP2 = (X26 >> 11) * PP1;
    PP3 = ((X26 & 0x7FF) * PP1) >> 11;
    PP4 = (PP2 + PP3) >> 10;

    CF  = (2097152 + cal.C12 * DT2) >> 3;
    X31 = (((CF * cal.C10) >> 17) * PP4) >> 2;
    X32 = (((((CF * cal.C11) >> 15) * PP4) >> 18) * PP4);

    pressure = ((X31 + X32) >> 15) + PP4 + 99880;
}

//  accumulate a new sensor reading
void AP_Baro_FBM320::timer(void)
{
    uint8_t buf[3];

    if (!dev->read_registers(0xF6, buf, sizeof(buf))) {
        return;
    }
    int32_t value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

    if (step == 0) {
        value_T = value;
    } else {
        int32_t pressure, temperature;
        calculate_PT(value_T, value, pressure, temperature);
        if (pressure_ok(pressure)) {
            WITH_SEMAPHORE(_sem);
            pressure_sum += pressure;
            // sum and convert to degrees
            temperature_sum += temperature*0.01;
            count++;
        }
    }

    if (step++ >= 5) {
        dev->write_register(FBM320_REG_CMD, FBM320_CMD_READ_T);
        step = 0;
    } else {
        dev->write_register(FBM320_REG_CMD, FBM320_CMD_READ_P);
    }
}

// transfer data to the frontend
void AP_Baro_FBM320::update(void)
{
    if (count == 0) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    _copy_to_frontend(instance, pressure_sum/count, temperature_sum/count);
    pressure_sum = 0;
    temperature_sum = 0;
    count=0;
}

#endif  // AP_BARO_FBM320_ENABLED
