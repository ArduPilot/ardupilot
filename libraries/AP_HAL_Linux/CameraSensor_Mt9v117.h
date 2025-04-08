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
#pragma once

#include <AP_HAL/I2CDevice.h>

#include "AP_HAL_Linux.h"
#include "CameraSensor.h"

namespace Linux {

enum mt9v117_res {
    MT9V117_QVGA,
};

struct mt9v117_patch {
    uint8_t *data;
    uint8_t size;
};

#define MT9V117_PATCH_LINE_NUM 13

class CameraSensor_Mt9v117 : public CameraSensor {
public:
    CameraSensor_Mt9v117(const char *device_path,
                         AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                         enum mt9v117_res res,
                         uint16_t nrst_gpio, uint32_t clock_freq);

private:
    static const struct mt9v117_patch _patch_lines[MT9V117_PATCH_LINE_NUM];

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

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint32_t _clock_freq;
    uint16_t _nrst_gpio = 0xFFFF;
    uint8_t _addr;
};

}
