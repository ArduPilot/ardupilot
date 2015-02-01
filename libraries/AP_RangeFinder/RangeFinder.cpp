// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_PX4.h"

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] PROGMEM = {
    // @Param: _TYPE
    // @DisplayName: 测距仪类型
    // @Description: 所连接的测距仪的类型
    // @Values: 0:None,1:Analog,2:APM2-MaxbotixI2C,3:APM2-PulsedLightI2C,4:PX4-I2C
    AP_GROUPINFO("_TYPE",    0, RangeFinder, _type[0], 0),

    // @Param: _PIN
    // @DisplayName: 测距仪针脚
    // @Description: 测距仪连接到的模拟针脚。对APM2设为0~9。对APM1设为64作为独立空速接口在板子的末端。（Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board.）在PX4上设为11作为模拟空速接口。在Pixhawk上设为15作为模拟空速接口。
    AP_GROUPINFO("_PIN",     1, RangeFinder, _pin[0], -1),

    // @Param: _SCALING
    // @DisplayName: 测距仪比例
    // @Description: 测距仪读数和距离之间的比例。对于线性和反向函数这是米/伏特。对于双曲线函数他的单位是米伏特。
    // @Units: meters/Volt
    // @Increment: 0.001
    AP_GROUPINFO("_SCALING", 2, RangeFinder, _scaling[0], 3.0),

    // @Param: _OFFSET
    // @DisplayName: 测距仪偏移
    // @Description: 距离为0时的电压
    // @Units: Volts
    // @Increment: 0.001
    AP_GROUPINFO("_OFFSET",  3, RangeFinder, _offset[0], 0.0),

    // @Param: _FUNCTION
    // @DisplayName: 测距仪函数
    // @Description: 控制用于计算距离的函数。罪域线性函数，距离=（电压-偏移）*比例。对于反向函数距离=（偏移-电压）*比例。对于双曲函数距离=比例/（电压-偏移）。函数返回以米为单位的距离。
    // @Values: 0:线性,1:反向,2:双曲
    AP_GROUPINFO("_FUNCTION", 4, RangeFinder, _function[0], 0),

    // @Param: _MIN_CM
    // @DisplayName: 测距仪最小距离
    // @Description: 测距仪所能可靠测到的最小厘米数。
	// @Units: 厘米
    // @Increment: 1
    AP_GROUPINFO("_MIN_CM",  5, RangeFinder, _min_distance_cm[0], 20),

    // @Param: _MAX_CM
    // @DisplayName: 测距仪最大距离
    // @Description: 测距仪能测到的最大厘米数。
	// @Units: 厘米
    // @Increment: 1
    AP_GROUPINFO("_MAX_CM",  6, RangeFinder, _max_distance_cm[0], 700),

    // @Param: _STOP_PIN
    // @DisplayName: 测距仪停止针脚
    // @Description: 用于启用/禁用测距仪测量的数字针脚。-1代表没有这个针脚。如果设置了，那么这个针设为1代表启用测距仪，0代表禁用。这可以确保多个声呐测距仪不会互相干扰。
    AP_GROUPINFO("_STOP_PIN", 7, RangeFinder, _stop_pin[0], -1),

    // @Param: _SETTLE
    // @DisplayName: 测距仪稳定读数时间
    // @Description: 测距仪稳定读数需要的毫秒单位时间。仅当STOP_PIN制定的时候需要这个数值。它决定了当我们设置STOP_PIN为高电平时测距仪给出读数需要多久。对于一个7m范围的声呐传感器，这需要大约50ms以允许声呐脉冲到达目标并返回。
    // @Units: 毫秒
    // @Increment: 1
    AP_GROUPINFO("_SETTLE", 8, RangeFinder, _settle_time_ms[0], 0),

    // @Param: _RMETRIC
    // @DisplayName: 比率计（Ratiometric）
    // @Description: 这个参数设定一个模拟测距仪是否符合比率计。大多数模拟测距仪都符合比率计，意味着他们的输出电压会受供电电压的影响。有些模拟测距仪（例如SF/02）有自己的内部稳压器，所以他们就不符合比率计。
    // @Values: 0:否,1:是
    AP_GROUPINFO("_RMETRIC", 9, RangeFinder, _ratiometric[0], 1),

    // 10..12 left for future expansion

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: 第二测距仪类型
    // @Description: 第二测距仪所连接的设备类型
    // @Values: 0:无,1:模拟,2:APM2-MaxbotixI2C,3:APM2-PulsedLightI2C,4:PX4-I2C
    AP_GROUPINFO("2_TYPE",    12, RangeFinder, _type[1], 0),

    // @Param: 2_PIN
    // @DisplayName: 测距仪针脚
    // @Description: 测距仪连接到的模拟针脚。对APM2设为0~9。对APM1设为64作为独立空速接口在板子的末端。（Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board.）在PX4上设为11作为模拟空速接口。在Pixhawk上设为15作为模拟空速接口。
    AP_GROUPINFO("2_PIN",     13, RangeFinder, _pin[1], -1),

    // @Param: 2_SCALING
    // @DisplayName: 测距仪比例
    // @Description: 测距仪读数和距离之间的比例。对于线性和反向函数这是米/伏特。对于双曲线函数他的单位是米伏特。
    // @Units: meters/Volt
    // @Increment: 0.001
    AP_GROUPINFO("2_SCALING", 14, RangeFinder, _scaling[1], 3.0),

    // @Param: 2_OFFSET
    // @DisplayName: 测距仪偏移
    // @Description: 距离为0时的电压
    // @Units: Volts
    // @Increment: 0.001
    AP_GROUPINFO("2_OFFSET",  15, RangeFinder, _offset[1], 0.0),

    // @Param: 2_FUNCTION
    // @DisplayName: 测距仪函数
    // @Description: 控制用于计算距离的函数。罪域线性函数，距离=（电压-偏移）*比例。对于反向函数距离=（偏移-电压）*比例。对于双曲函数距离=比例/（电压-偏移）。函数返回以米为单位的距离。
    // @Values: 0:线性,1:反向,2:双曲
    AP_GROUPINFO("2_FUNCTION",  16, RangeFinder, _function[1], 0),

    // @Param: 2_MIN_CM
    // @DisplayName: 测距仪最小距离
    // @Description: 测距仪所能可靠测到的最小厘米数。
	// @Units: 厘米
    // @Increment: 1
    AP_GROUPINFO("2_MIN_CM",  17, RangeFinder, _min_distance_cm[1], 20),

    // @Param: 2_MAX_CM
    // @DisplayName: 测距仪最大距离
    // @Description: 测距仪所能可靠测到的最大厘米数。
	// @Units: 厘米
    // @Increment: 1
    AP_GROUPINFO("2_MAX_CM",  18, RangeFinder, _max_distance_cm[1], 700),

    // @Param: 2_STOP_PIN
    // @DisplayName: 测距仪停止针脚
    // @Description: 用于启用/禁用测距仪测量的数字针脚。-1代表没有这个针脚。如果设置了，那么这个针设为1代表启用测距仪，0代表禁用。这可以确保多个声呐测距仪不会互相干扰。
    AP_GROUPINFO("2_STOP_PIN", 19, RangeFinder, _stop_pin[1], -1),

    // @Param: 2_SETTLE
    // @DisplayName: 测距仪稳定读数时间
    // @Description: 测距仪稳定读数需要的毫秒单位时间。仅当STOP_PIN制定的时候需要这个数值。它决定了当我们设置STOP_PIN为高电平时测距仪给出读数需要多久。对于一个7m范围的声呐传感器，这需要大约50ms以允许声呐脉冲到达目标并返回。
    // @Units: 毫秒
    // @Increment: 1
    AP_GROUPINFO("2_SETTLE", 20, RangeFinder, _settle_time_ms[1], 0),

    // @Param: 2_RMETRIC
    // @DisplayName: 比率计（Ratiometric）
    // @Description: 这个参数设定一个模拟测距仪是否符合比率计。大多数模拟测距仪都符合比率计，意味着他们的输出电压会受供电电压的影响。有些模拟测距仪（例如SF/02）有自己的内部稳压器，所以他们就不符合比率计。
    // @Values: 0:否,1:是
    AP_GROUPINFO("2_RMETRIC", 21, RangeFinder, _ratiometric[1], 1),
#endif

    AP_GROUPEND
};

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].healthy = false;
                continue;
            }
            drivers[i]->update();
        }
    }

    // work out primary instance - first healthy sensor
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && state[i].healthy) {
            primary_instance = i;
        }
    }
}
    
/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (type == RangeFinder_TYPE_PLI2C || 
        type == RangeFinder_TYPE_MBI2C) {
        // I2C sensor types are handled by the PX4Firmware code
        type = RangeFinder_TYPE_PX4;
    }
#endif
    if (type == RangeFinder_TYPE_PLI2C) {
        if (AP_RangeFinder_PulsedLightLRF::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_PulsedLightLRF(*this, instance, state[instance]);
            return;
        }
    } 
    if (type == RangeFinder_TYPE_MBI2C) {
        if (AP_RangeFinder_MaxsonarI2CXL::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_MaxsonarI2CXL(*this, instance, state[instance]);
            return;
        }
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (type == RangeFinder_TYPE_PX4) {
        if (AP_RangeFinder_PX4::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_PX4(*this, instance, state[instance]);
            return;
        }
    }
#endif
    if (type == RangeFinder_TYPE_ANALOG) {
        // note that analog must be the last to be checked, as it will
        // always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_analog(*this, instance, state[instance]);
            return;
        }
    }
}

