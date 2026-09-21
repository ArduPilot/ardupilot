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
  posix filesystem backend for SITL, adding a simulated disk size cap
  (SIM_DISK_MAX) so out-of-space handling can be tested without
  filling the host disk
 */
#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_SITL_ENABLED

#include "AP_Filesystem_posix.h"

class AP_Filesystem_SITL : public AP_Filesystem_Posix
{
public:
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int unlink(const char *pathname) override;
    int64_t disk_free(const char *path) override;
    int64_t disk_space(const char *path) override;

private:
    // SIM_DISK_MAX in bytes, or -1 if the cap is disabled
    int64_t disk_max_bytes();
    void rescan_used(const char *path);

    // approximate bytes used under the cap: charged by write(),
    // credited by unlink()
    int64_t used_bytes;
    int32_t last_max_mb = -1;
    // guards the accounting; the backend is called from several threads
    HAL_Semaphore sem;
};

#endif  // AP_FILESYSTEM_SITL_ENABLED
