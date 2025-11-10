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
#include "AP_Filesystem.h"

extern const AP_HAL::HAL& hal;

/*
  Load a file's contents into memory. Returned object must be `delete`d to free
  the data. The data is guaranteed to be null-terminated such that it can be
  treated as a string.
*/
FileData *AP_Filesystem_Backend::load_file(const char *filename)
{
    struct stat st;
    if (stat(filename, &st) != 0) {
        return nullptr;
    }
    FileData *fd = NEW_NOTHROW FileData(this);
    if (fd == nullptr) {
        return nullptr;
    }
    // add one byte for null termination; ArduPilot's malloc will zero it.
    void *data = malloc(st.st_size+1);
    if (data == nullptr) {
        delete fd;
        return nullptr;
    }
    int d = open(filename, O_RDONLY);
    if (d == -1) {
        free(data);
        delete fd;
        return nullptr;
    }
    if (read(d, data, st.st_size) != st.st_size) {
        close(d);
        free(data);
        delete fd;
        return nullptr;
    }
    close(d);
    fd->length = st.st_size; // length does not include our added termination
    fd->data = (const uint8_t *)data;
    return fd;
}

/*
  unload a FileData object
*/
void AP_Filesystem_Backend::unload_file(FileData *fd)
{
    if (fd->data != nullptr) {
        free(const_cast<uint8_t *>(fd->data));
        fd->data = nullptr;
    }
}

// return true if file operations are allowed
bool AP_Filesystem_Backend::file_op_allowed(void) const
{
    if (!hal.util->get_soft_armed() || !hal.scheduler->in_main_thread()) {
        return true;
    }
    return false;
}

/*
  destructor for FileData
 */
FileData::~FileData()
{
    if (backend != nullptr) {
        ((AP_Filesystem_Backend *)backend)->unload_file(this);
    }
}

