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
 * I2C driver for Sensiron SHT3x digital temperature sensor

 https://sensirion.com/media/documents/213E6A3B/63A5A569/Datasheet_SHT3x_DIS.pdf


 Testing:

pbarker@fx:~$ cat /tmp/extra.hwdef
define AP_TEMPERATURE_SENSOR_ENABLED 1
define AP_TEMPERATURE_SENSOR_BACKEND_DEFAULT_ENABLED 1
pbarker@fx:~$ ./waf configure --board=CubeOrange --extra-hwdef=/tmp/extra.hwdef
pbarker@fx:~/rc/ardupilot(pr/SHT3x)$ ./waf rover --upload

# when plugging into I2C2 on a CubeOrange:
param set TEMP1_TYPE 8
param set TEMP1_BUS 0
param set TEMP1_ADDR 0x44

this is bad:
AP: SHT3x reset failed
AP: SHT3x: SN1841CC41FC5

param set TEMP_LOG 1
param set LOG_DISARMED 1
...
param set LOG_DISARMED 0

Download log and check for temperatures

*/

#pragma once

#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_SHT3X_ENABLED

#include "AP_TemperatureSensor_Backend.h"

class AP_TemperatureSensor_SHT3x : public AP_TemperatureSensor_Backend {

    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

public:
    __INITFUNC__ void init(void) override;

    void update() override {};

private:
    // reset device
    bool reset(void) const;

    // prod device to start preparing a measurement:
    void start_next_sample();
    // read measurements from device:
    bool read_measurements(uint16_t &temp, uint16_t &humidity) const;

    // update the temperature, called at 20Hz
    void _timer(void);

};
#endif // AP_TEMPERATURE_SENSOR_SHT3X_ENABLED
