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
#ifndef __CAMERASENSOR_MT9V117_H__
#define __CAMERASENSOR_MT9V117_H__

#include "AP_HAL_Linux.h"
#include "CameraSensor.h"

enum mt9v117_res {
    MT9V117_QVGA,
};

class Linux::CameraSensor_Mt9v117 : public Linux::CameraSensor {
public:
    CameraSensor_Mt9v117(const char *device_path, AP_HAL::I2CDriver *i2c,
                         uint8_t addr, enum mt9v117_res res,
                         uint16_t nrst_gpio, uint32_t clock_freq);
private:
    uint8_t _read_reg8(uint16_t reg);
    void _write_reg8(uint16_t reg, uint8_t val);
    uint16_t _read_reg16(uint16_t reg);
    void _write_reg16(uint16_t reg, uint16_t val);
    void _write_reg32(uint16_t reg, uint32_t val);
    inline uint16_t _var2reg(uint16_t var, uint16_t offset);
    uint16_t _read_var16(uint16_t var, uint16_t offset);
    void _write_var16(uint16_t var, uint16_t offset, uint16_t value);
    uint8_t _read_var8(uint16_t var, uint16_t offset);
    void _write_var8(uint16_t var, uint16_t offset, uint8_t value);
    void _write_var32(uint16_t var, uint16_t offset, uint32_t value);
    void _config_change();
    void _itu656_enable();
    void _soft_reset();
    void _apply_patch();
    void _set_basic_settings();
    void _configure_sensor_qvga();
    void _init_sensor();

    AP_HAL::I2CDriver *_i2c;
    uint8_t _addr;
    uint16_t _nrst_gpio = 0xFFFF;
    uint32_t _clock_freq = 0;
};

#endif
