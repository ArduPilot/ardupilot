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
    Dump logged AIS data to the serial port
    ./Tools/autotest/sim_vehicle.py -v Rover -A --serial5=sim:AIS --custom-location 51.58689798356386,-3.9044570193067965,0,0

    param set SERIAL5_PROTOCOL 40
    param set AIS_TYPE 1
*/

#include "SIM_AIS.h"

#if HAL_SIM_AIS_ENABLED

#include <SITL/SITL.h>

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"

extern const AP_HAL::HAL& hal;

using namespace SITL;


AIS::AIS() : SerialDevice::SerialDevice()
{
    char* file_path;
    IGNORE_RETURN(asprintf(&file_path, AP_BUILD_ROOT "/libraries/SITL/SIM_AIS_data.txt"));

    file = fopen(file_path,"r");

    if (file == nullptr) {
        AP_HAL::panic("AIS could not open data file");
    }

    // seek past the header line
    char line[100];
    IGNORE_RETURN(fgets(line, sizeof(line), file));
}

void AIS::update()
{
    if (file == nullptr) {
        AP_HAL::panic("AIS lost data file");
    }

    // just send a line of data at 1Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 1000) {
        return;
    }
    last_sent_ms = now;

    char line[100];

    if (!fgets(line, sizeof(line), file)) {
        // got to the end of the file, circle back
        fseek(file,0,SEEK_SET);
        if (!fgets(line, sizeof(line), file)) {
            AP_HAL::panic("AIS lost data file");
        }
        return;
    }

    //hal.console->printf("%s",line);
    write_to_autopilot(line, strlen(line));

}

#endif  // HAL_SIM_AIS_ENABLED
