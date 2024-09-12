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
   This driver supports communicating with Torqeedo motors that implement the "TQ Bus" protocol
   which includes the Ultralight, Cruise 2.0 R, Cruise 4.0 R, Travel 503, Travel 1003 and Cruise 10kW

   The autopilot should be connected either to the battery's tiller connector or directly to the motor
   as described on the ArduPilot wiki. https://ardupilot.org/rover/docs/common-torqeedo.html
   TQ Bus is a serial protocol over RS-485 meaning that a serial to RS-485 converter is required.

       Tiller connection: Autopilot <-> Battery (master) <-> Motor
       Motor connection:  Autopilot (master) <-> Motor

    Communication between the components is always initiated by the master with replies sent within 25ms

    Example "Remote (0x01)" reply message to allow tiller to control motor speed
    Byte        Field Definition    Example Value   Comments
    ---------------------------------------------------------------------------------
    byte 0      Header              0xAC
    byte 1      TargetAddress       0x00            see MsgAddress enum
    byte 2      Message ID          0x00            only master populates this. replies have this set to zero
    byte 3      Flags               0x05            bit0=pin present, bit2=motor speed valid
    byte 4      Status              0x00            0x20 if byte3=4, 0x0 is byte3=5
    byte 5      Motor Speed MSB     ----            Motor Speed MSB (-1000 to +1000)
    byte 6      Motor Speed LSB     ----            Motor Speed LSB (-1000 to +1000)
    byte 7      CRC-Maxim           ----            CRC-Maxim value
    byte 8      Footer              0xAD

   More details of the TQ Bus protocol are available from Torqeedo after signing an NDA.
 */

#pragma once

#include "AP_Torqeedo_config.h"

#if HAL_TORQEEDO_ENABLED

#include "AP_Torqeedo.h"
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_Torqeedo_Backend : public AP_ESC_Telem_Backend {
public:
    AP_Torqeedo_Backend(AP_Torqeedo_Params &params, uint8_t instance);

    // do not allow copies
    CLASS_NO_COPY(AP_Torqeedo_Backend);

    // initialise driver
    virtual void init() = 0;

    // returns true if communicating with the motor
    virtual bool healthy() = 0;

    // clear motor errors
    virtual void clear_motor_error() = 0;

    // get latest battery status info.  returns true on success and populates arguments
    virtual bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED = 0;
    virtual bool get_batt_capacity_Ah(uint16_t &amp_hours) const = 0;

   protected:

    // parameter helper functions
    AP_Torqeedo::ConnectionType get_type() const { return (AP_Torqeedo::ConnectionType)_params.type.get(); }
    bool option_enabled(AP_Torqeedo::options opt) const { return ((uint16_t)_params.options.get() & (uint16_t)opt) != 0; }

    AP_Torqeedo_Params &_params;    // parameters for this backend
    uint8_t _instance;              // this instance's number
};

#endif // HAL_TORQEEDO_ENABLED
