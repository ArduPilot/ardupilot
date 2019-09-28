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
  base class for serially-attached simulated devices
*/

#pragma once

#include <unistd.h>

namespace SITL {

class SerialDevice {
public:

    SerialDevice();

    // return fd on which data from the device can be read
    // to the device can be written
    int fd() { return fd_their_end; }
    // return fd on which data to the device can be written
    int write_fd() { return read_fd_their_end; }

    ssize_t read_from_autopilot(char *buffer, size_t size);
    ssize_t write_to_autopilot(const char *buffer, size_t size);

protected:

    class SITL *_sitl;

    int fd_their_end;
    int fd_my_end;

    int read_fd_their_end;
    int read_fd_my_end;

    bool init_sitl_pointer();
};

}
