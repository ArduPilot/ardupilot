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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#include "CameraSensor_Mt9v117.h"

#include <utility>

#include "GPIO.h"

/* Cam sensor register definitions */
#define CHIP_ID                 0x0
#define MT9V117_CHIP_ID             0x2282
#define COMMAND_REGISTER        0x0040
#define     HOST_COMMAND_OK (1 << 15)
#define     HOST_COMMAND_2  (1 << 2)
#define     HOST_COMMAND_1  (1 << 1)
#define     HOST_COMMAND_0  (1 << 0)
#define PAD_SLEW                0x0030
#define RESET_AND_MISC_CONTROL  0x001a
#define     RESET_SOC_I2C   (1 << 0)
#define ACCESS_CTL_STAT         0x0982
#define PHYSICAL_ADDRESS_ACCESS 0x098a
#define LOGICAL_ADDRESS_ACCESS  0x098e
#define AE_TRACK_JUMP_DIVISOR   0xa812
#define CAM_AET_SKIP_FRAMES     0xc868

#define AE_RULE_VAR     9
#define     AE_RULE_ALGO_OFFSET                         4
#define         AE_RULE_ALGO_AVERAGE    0
#define         AE_RULE_ALGO_WEIGHTED   1
#define AE_TRACK_VAR    10
#define AWB_VAR         11
#define     AWB_PIXEL_THRESHOLD_COUNT_OFFSET            64
#define LOW_LIGHT_VAR   15
#define CAM_CTRL_VAR    18
#define     CAM_SENSOR_CFG_Y_ADDR_START_OFFSET          0
#define     CAM_SENSOR_CFG_X_ADDR_START_OFFSET          2
#define     CAM_SENSOR_CFG_Y_ADDR_END_OFFSET            4
#define     CAM_SENSOR_CFG_X_ADDR_END_OFFSET            6
#define     CAM_SENSOR_CFG_FRAME_LENGTH_LINES_OFFSET    14
#define     CAM_SENSOR_CFG_CPIPE_LAST_ROW_OFFSET        20
#define     CAM_SENSOR_CFG_FDPERIOD_60HZ                22
#define     CAM_SENSOR_CFG_FDPERIOD_50HZ                24
#define     CAM_SENSOR_CFG_MAX_FDZONE_60_OFFSET         26
#define     CAM_SENSOR_CFG_MAX_FDZONE_50_OFFSET         28
#define     CAM_SENSOR_CFG_TARGET_FDZONE_60_OFFSET      30
#define     CAM_SENSOR_CFG_TARGET_FDZONE_50_OFFSET      32
#define     CAM_SENSOR_CONTROL_READ_MODE_OFFSET         40
#define         CAM_SENSOR_CONTROL_Y_SKIP_EN                (1 << 2)
#define         CAM_SENSOR_CONTROL_VERT_FLIP_EN             (1 << 1)
#define         CAM_SENSOR_CONTROL_HORZ_MIRROR_EN           (1 << 0)
#define     CAM_FLICKER_PERIOD_OFFSET                   62
#define         CAM_FLICKER_PERIOD_60HZ 0
#define         CAM_FLICKER_PERIOD_50HZ 1
#define     CAM_CROP_WINDOW_XOFFSET_OFFSET              72
#define     CAM_CROP_WINDOW_YOFFSET_OFFSET              74
#define     CAM_CROP_WINDOW_WIDTH_OFFSET                76
#define     CAM_CROP_WINDOW_HEIGHT_OFFSET               78
#define     CAM_CROP_MODE_OFFSET                        80
#define     CAM_OUTPUT_WIDTH_OFFSET                     84
#define     CAM_OUTPUT_HEIGHT_OFFSET                    86
#define     CAM_OUTPUT_FORMAT_OFFSET                    88
#define         CAM_OUTPUT_FORMAT_RGB_565                   (0 << 12)
#define         CAM_OUTPUT_FORMAT_RGB_555                   (1 << 12)
#define         CAM_OUTPUT_FORMAT_RGB_444X                  (2 << 12)
#define         CAM_OUTPUT_FORMAT_RGB_X444                  (3 << 12)
#define         CAM_OUTPUT_FORMAT_BAYER_10                  (0 << 10)
#define         CAM_OUTPUT_FORMAT_YUV                       (0 << 8)
#define         CAM_OUTPUT_FORMAT_RGB                       (1 << 8)
#define         CAM_OUTPUT_FORMAT_BAYER                     (2 << 8)
#define         CAM_OUTPUT_FORMAT_BT656_ENABLE              (1 << 3)
#define         CAM_OUTPUT_FORMAT_MONO_ENABLE               (1 << 2)
#define         CAM_OUTPUT_FORMAT_SWAP_BYTES                (1 << 1)
#define         CAM_OUTPUT_FORMAT_SWAP_RED_BLUE             (1 << 0)
#define     CAM_STAT_AWB_HG_WINDOW_XSTART_OFFSET        236
#define     CAM_STAT_AWB_HG_WINDOW_YSTART_OFFSET        238
#define     CAM_STAT_AWB_HG_WINDOW_XEND_OFFSET          240
#define     CAM_STAT_AWB_HG_WINDOW_YEND_OFFSET          242
#define     CAM_STAT_AE_INITIAL_WINDOW_XSTART_OFFSET    244
#define     CAM_STAT_AE_INITIAL_WINDOW_YSTART_OFFSET    246
#define     CAM_STAT_AE_INITIAL_WINDOW_XEND_OFFSET      248
#define     CAM_STAT_AE_INITIAL_WINDOW_YEND_OFFSET      250
#define     CAM_LL_START_GAIN_METRIC_OFFSET             278
#define     CAM_LL_STOP_GAIN_METRIC_OFFSET              280
#define SYSMGR_VAR      23
#define     SYSMGR_NEXT_STATE_OFFSET                    0
#define PATCHLDR_VAR    24
#define     PATCHLDR_LOADER_ADDRESS_OFFSET              0
#define     PATCHLDR_PATCH_ID_OFFSET                    2
#define     PATCHLDR_FIRMWARE_ID_OFFSET                   4

extern const AP_HAL::HAL& hal;

using namespace Linux;

CameraSensor_Mt9v117::CameraSensor_Mt9v117(const char *device_path,
                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                           enum mt9v117_res res,
                                           uint16_t nrst_gpio, uint32_t clock_freq)
    : CameraSensor(device_path)
    , _dev(std::move(dev))
    , _nrst_gpio(nrst_gpio)
    , _clock_freq(clock_freq)
{
    if (!_dev) {
        AP_HAL::panic("Could not find I2C bus for CameraSensor_Mt9v117");
    }

    switch (res) {
    case MT9V117_QVGA:
        _init_sensor();
        _configure_sensor_qvga();
        break;
    default:
        AP_HAL::panic("mt9v117: unsupported resolution\n");
        break;
    }

    _itu656_enable();
    _config_change();
}

uint8_t CameraSensor_Mt9v117::_read_reg8(uint16_t reg)
{
    uint8_t buf[2];
    buf[0] = (uint8_t) (reg >> 8);
    buf[1] = (uint8_t) (reg & 0xFF);

    if (!_dev->transfer(buf, 2, buf, 1)) {
        hal.console->printf("mt9v117: error reading 0x%2x\n", reg);
        return 0;
    }

    return buf[0];
}

void CameraSensor_Mt9v117::_write_reg8(uint16_t reg, uint8_t val)
{
    uint8_t buf[3];
    buf[0] = (uint8_t) (reg >> 8);
    buf[1] = (uint8_t) (reg & 0xFF);
    buf[2] = val;

    if (!_dev->transfer(buf, 3, nullptr, 0)) {
        hal.console->printf("mt9v117: error writing 0x%2x\n", reg);
    }
}

uint16_t CameraSensor_Mt9v117::_read_reg16(uint16_t reg)
{
    uint8_t buf[2];
    buf[0] = (uint8_t) (reg >> 8);
    buf[1] = (uint8_t) (reg & 0xFF);

    if (!_dev->transfer(buf, 2, buf, 2)) {
        hal.console->printf("mt9v117: error reading 0x%4x\n", reg);
        return 0;
    }

    return (buf[0] << 8 | buf[1]);
}

void CameraSensor_Mt9v117::_write_reg16(uint16_t reg, uint16_t val)
{
    uint8_t buf[4];
    buf[0] = (uint8_t) (reg >> 8);
    buf[1] = (uint8_t) (reg & 0xFF);
    buf[2] = (uint8_t) (val >> 8);
    buf[3] = (uint8_t) (val & 0xFF);

    if (!_dev->transfer(buf, 4, nullptr, 0)) {
        hal.console->printf("mt9v117: error writing 0x%4x\n", reg);
    }
}

void CameraSensor_Mt9v117::_write_reg32(uint16_t reg, uint32_t val)
{
    uint8_t buf[6];
    buf[0] = (uint8_t) (reg >> 8);
    buf[1] = (uint8_t) (reg & 0xFF);
    buf[2] = (uint8_t) (val >> 24);
    buf[3] = (uint8_t) ((val >> 16) & 0xFF);
    buf[4] = (uint8_t) ((val >> 8) & 0xFF);
    buf[5] = (uint8_t) (val & 0xFF);

    if (!_dev->transfer(buf, 6, nullptr, 0)) {
        hal.console->printf("mt9v117: error writing 0x%8x\n", reg);
    }
}

inline uint16_t CameraSensor_Mt9v117::_var2reg(uint16_t var,
                                               uint16_t offset)
{
    return (0x8000 | (var << 10) | offset);
}

uint16_t CameraSensor_Mt9v117::_read_var16(uint16_t var, uint16_t offset)
{
    uint16_t reg = _var2reg(var, offset);
    return _read_reg16(reg);
}

void CameraSensor_Mt9v117::_write_var16(uint16_t var,
                                        uint16_t offset,
                                        uint16_t value)
{
    uint16_t reg = _var2reg(var, offset);
    _write_reg16(reg, value);
}

uint8_t CameraSensor_Mt9v117::_read_var8(uint16_t var, uint16_t offset)
{
    uint16_t reg = _var2reg(var, offset);
    return _read_reg8(reg);
}

void CameraSensor_Mt9v117::_write_var8(uint16_t var,
                                       uint16_t offset,
                                       uint8_t value)
{
    uint16_t reg = _var2reg(var, offset);
    return _write_reg8(reg, value);
}

void CameraSensor_Mt9v117::_write_var32(uint16_t var,
                                        uint16_t offset,
                                        uint32_t value)
{
    uint16_t reg = _var2reg(var, offset);
    return _write_reg32(reg, value);
}

void CameraSensor_Mt9v117::_config_change()
{
    uint16_t cmd_status;
    /* timeout 100ms delay 10ms */
    int timeout = 10;

    _write_var8(SYSMGR_VAR, SYSMGR_NEXT_STATE_OFFSET, 0x28);

    _write_reg16(COMMAND_REGISTER, HOST_COMMAND_OK | HOST_COMMAND_1);

    do {
        hal.scheduler->delay(10);
        cmd_status = _read_reg16(COMMAND_REGISTER);
        timeout--;
    } while (((cmd_status & HOST_COMMAND_1) != 0) &&
              (timeout > 0));

    if (timeout == 0) {
        hal.console->printf("mt9v117:"
                            "timeout waiting or command to complete\n");
    }

    if ((cmd_status & HOST_COMMAND_OK) == 0) {
        hal.console->printf("mt9v117:config change failed\n");
    }
}

void CameraSensor_Mt9v117::_itu656_enable()
{
    _write_var16(CAM_CTRL_VAR, CAM_OUTPUT_FORMAT_OFFSET,
                 _read_var16(CAM_CTRL_VAR, CAM_OUTPUT_FORMAT_OFFSET) |
                 CAM_OUTPUT_FORMAT_BT656_ENABLE);
}

void CameraSensor_Mt9v117::_soft_reset()
{
    _write_reg16(RESET_AND_MISC_CONTROL, RESET_SOC_I2C);
    _write_reg16(RESET_AND_MISC_CONTROL, 0);
    /* sleep 50ms after soft reset */
    hal.scheduler->delay(50);
}

void CameraSensor_Mt9v117::_apply_patch()
{
    uint16_t cmd_status;
    /* timeout 100ms delay 10ms */
    int timeout = 10;

    /* Errata item 2 */
    _write_reg16(0x301a, 0x10d0);
    _write_reg16(0x31c0, 0x1404);
    _write_reg16(0x3ed8, 0x879c);
    _write_reg16(0x3042, 0x20e1);
    _write_reg16(0x30d4, 0x8020);
    _write_reg16(0x30c0, 0x0026);
    _write_reg16(0x301a, 0x10d4);

    /* Errata item 6 */
    _write_var16(AE_TRACK_VAR, 0x0002, 0x00d3);
    _write_var16(CAM_CTRL_VAR, 0x0078, 0x00a0);
    _write_var16(CAM_CTRL_VAR, 0x0076, 0x0140);

    /* Errata item 8 */
    _write_var16(LOW_LIGHT_VAR, 0x0004, 0x00fc);
    _write_var16(LOW_LIGHT_VAR, 0x0038, 0x007f);
    _write_var16(LOW_LIGHT_VAR, 0x003a, 0x007f);
    _write_var16(LOW_LIGHT_VAR, 0x003c, 0x007f);
    _write_var16(LOW_LIGHT_VAR, 0x0004, 0x00f4);

    /* Patch 0403; Critical; Sensor optimization */
    _write_reg16(ACCESS_CTL_STAT, 0x0001);
    _write_reg16(PHYSICAL_ADDRESS_ACCESS, 0x7000);

    /* write patch */
    for (unsigned int i = 0; i < MT9V117_PATCH_LINE_NUM; i++) {
        _dev->transfer(_patch_lines[i].data, _patch_lines[i].size, nullptr, 0);
    }

    _write_reg16(LOGICAL_ADDRESS_ACCESS, 0x0000);

    _write_var16(PATCHLDR_VAR, PATCHLDR_LOADER_ADDRESS_OFFSET, 0x05d8);
    _write_var16(PATCHLDR_VAR, PATCHLDR_PATCH_ID_OFFSET, 0x0403);
    _write_var32(PATCHLDR_VAR, PATCHLDR_FIRMWARE_ID_OFFSET, 0x00430104);

    _write_reg16(COMMAND_REGISTER, HOST_COMMAND_OK | HOST_COMMAND_0);

    do {
        hal.scheduler->delay(10);
        cmd_status = _read_reg16(COMMAND_REGISTER);
        timeout--;
    } while (((cmd_status & HOST_COMMAND_0) != 0) &&
             (timeout > 0));

    if ((cmd_status & HOST_COMMAND_OK) == 0) {
        hal.console->printf("mt9v117:patch apply failed\n");
    }
}

void CameraSensor_Mt9v117::_set_basic_settings()
{
    _write_var32(AWB_VAR, AWB_PIXEL_THRESHOLD_COUNT_OFFSET, 50000);
    _write_var16(AE_RULE_VAR, AE_RULE_ALGO_OFFSET, AE_RULE_ALGO_AVERAGE);

    /* Set pixclk pad slew to 6 and data out pad slew to 1 */
    _write_reg16(PAD_SLEW, _read_reg16(PAD_SLEW) | 0x0600 | 0x0001);
}

void CameraSensor_Mt9v117::_configure_sensor_qvga()
{
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_X_ADDR_START_OFFSET, 16);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_X_ADDR_END_OFFSET, 663);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_Y_ADDR_START_OFFSET, 8);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_Y_ADDR_END_OFFSET,  501);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_CPIPE_LAST_ROW_OFFSET, 243);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_FRAME_LENGTH_LINES_OFFSET, 283);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CONTROL_READ_MODE_OFFSET,
                 CAM_SENSOR_CONTROL_Y_SKIP_EN);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_MAX_FDZONE_60_OFFSET, 1);
    _write_var16(CAM_CTRL_VAR, CAM_SENSOR_CFG_TARGET_FDZONE_60_OFFSET, 1);

    _write_reg8(AE_TRACK_JUMP_DIVISOR, 0x03);
    _write_reg8(CAM_AET_SKIP_FRAMES, 0x02);

    _write_var16(CAM_CTRL_VAR, CAM_OUTPUT_WIDTH_OFFSET, 320);
    _write_var16(CAM_CTRL_VAR, CAM_OUTPUT_HEIGHT_OFFSET, 240);

    /* Set gain metric for 111.2 fps
     * The final fps depends on the input clock
     * (89.2fps on bebop) so a modification may be needed here */
    _write_var16(CAM_CTRL_VAR, CAM_LL_START_GAIN_METRIC_OFFSET, 0x03e8);
    _write_var16(CAM_CTRL_VAR, CAM_LL_STOP_GAIN_METRIC_OFFSET, 0x1770);

    /* set crop window */
    _write_var16(CAM_CTRL_VAR, CAM_CROP_WINDOW_XOFFSET_OFFSET, 0);
    _write_var16(CAM_CTRL_VAR, CAM_CROP_WINDOW_YOFFSET_OFFSET, 0);
    _write_var16(CAM_CTRL_VAR, CAM_CROP_WINDOW_WIDTH_OFFSET, 640);
    _write_var16(CAM_CTRL_VAR, CAM_CROP_WINDOW_HEIGHT_OFFSET, 240);

    /* Enable auto-stats mode */
    _write_var8(CAM_CTRL_VAR, CAM_CROP_MODE_OFFSET, 3);
    _write_var16(CAM_CTRL_VAR, CAM_STAT_AWB_HG_WINDOW_XEND_OFFSET, 319);
    _write_var16(CAM_CTRL_VAR, CAM_STAT_AWB_HG_WINDOW_YEND_OFFSET, 239);
    _write_var16(CAM_CTRL_VAR, CAM_STAT_AE_INITIAL_WINDOW_XSTART_OFFSET, 2);
    _write_var16(CAM_CTRL_VAR, CAM_STAT_AE_INITIAL_WINDOW_YSTART_OFFSET, 2);
    _write_var16(CAM_CTRL_VAR, CAM_STAT_AE_INITIAL_WINDOW_XEND_OFFSET, 65);
    _write_var16(CAM_CTRL_VAR, CAM_STAT_AE_INITIAL_WINDOW_YEND_OFFSET, 49);
}

void CameraSensor_Mt9v117::_init_sensor()
{
    AP_HAL::DigitalSource *gpio_source;
    uint16_t id;

    if (_nrst_gpio != 0xFFFF) {
        gpio_source = hal.gpio->channel(_nrst_gpio);
        gpio_source->mode(HAL_GPIO_OUTPUT);
        gpio_source->write(1);
        uint32_t delay = 3.5f + (35.0f - 3.5f) *
                        (54000000.0f - (float)_clock_freq) /
                        (54000000.0f - 6000000.0f);
        hal.scheduler->delay(delay);
    }

    id = _read_reg16(CHIP_ID);
    if (id != MT9V117_CHIP_ID) {
        AP_HAL::panic("Mt9v117: bad chip id 0x%04x\n", id);
    }
    _soft_reset();
    _apply_patch();
    _set_basic_settings();
}

#endif
