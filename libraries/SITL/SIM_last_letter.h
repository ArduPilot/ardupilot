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
  simulator connection for the last_letter_lib flight dynamics library

  last_letter_lib speaks the standard JSON backend protocol, so all the wire
  handling is inherited from SITL::JSON. The only thing added here is launching
  the simulator process, which lives outside the ArduPilot tree:

      sim_vehicle.py -v ArduPlane --model last_letter:<aircraft>

  The aircraft name after the colon is passed to the child; it defaults to
  "ardupilot_plane". The binary is looked up on PATH as "last_letter_ardupilot",
  overridable with the LAST_LETTER_SITL_BIN environment variable.

  Optional features are selected with the SITL --config string, which
  sim_vehicle.py forwards through -A:

      sim_vehicle.py -v ArduPlane --model last_letter:<aircraft> -A "--config log"
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_LAST_LETTER_ENABLED

#include "SIM_JSON.h"

namespace SITL {

/*
  a last_letter simulator
 */
class last_letter : public JSON {
public:
    last_letter(const char *frame_str);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW last_letter(frame_str);
    }

    /*
      the UDP port is only known once this is called, so it is recorded here
      rather than in the constructor
     */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

    /*
      parse the SITL --config string
     */
    void set_config(const char *config) override;

    /*
      fork the last_letter_ardupilot binary
     */
    void launch_external_sim(void) override;

private:
    char model_name[64];

    // The port the child must bind, from set_interface_ports().
    uint16_t sim_port;

    // --config log
    bool want_log;
};

} // namespace SITL

#endif  // AP_SIM_LAST_LETTER_ENABLED
