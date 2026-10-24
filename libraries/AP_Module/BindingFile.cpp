// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingFile.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Filesystem/AP_Filesystem.h>

namespace AP_Module
{

#if AP_FILESYSTEM_FILE_READING_ENABLED
class ReadFile
{
public:
    explicit ReadFile(int fd) : _fd(fd) {}
    CLASS_NO_COPY(ReadFile);
    ~ReadFile()
    {
        if (_fd >= 0) {
            AP::FS().close(_fd);
        }
    }
    bool close()
    {
        const int fd = _fd;
        _fd = -1;
        return AP::FS().close(fd) == 0;
    }
private:
    int _fd;
};

static bool read_at(int fd, uint32_t offset, uint16_t count, uint8_t *output, uint16_t &length)
{
    const int32_t position = AP::FS().lseek(fd, int32_t(offset), SEEK_SET);
    if (position < 0 || uint32_t(position) > offset) {
        return false;
    }
    length = 0;
    while (length < count) {
        const int32_t n = AP::FS().read(fd, output + length, count - length);
        if (n < 0 || uint32_t(n) > uint32_t(count - length)) {
            return false;
        }
        if (n == 0) {
            return true;
        }
        // Read-only FatFS clamps a seek beyond EOF. Accept that only when the
        // following read confirms EOF, never data from an unexpected position.
        if (uint32_t(position) != offset) {
            return false;
        }
        length += uint16_t(n);
    }
    return true;
}
#endif

int32_t binding_file_read_at(const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr || input_size != AP_MODULE_FILE_PATH_SIZE + 6U ||
        output_size != AP_MODULE_FILE_READ_MAX + 3U) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_FILESYSTEM_FILE_READING_ENABLED
    static_assert(AP_MODULE_FILE_PATH_SIZE == 256, "cstring wire size");
    const char *path = BindingWire::read_cstring(input);
    if (path == nullptr || path[0] == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    const char *component = path;
    while (*component == '/') {
        component++;
    }
    if (*component == '@') {
        // Virtual backends may regenerate a whole snapshot on every open.
        return AP_MODULE_STATUS_INVALID;
    }
    const uint32_t offset = BindingWire::read(input + AP_MODULE_FILE_PATH_SIZE, 4);
    const uint16_t count = BindingWire::read(input + AP_MODULE_FILE_PATH_SIZE + 4, 2);
    if (count == 0 || count > AP_MODULE_FILE_READ_MAX || uint64_t(offset) + count > INT32_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    const int fd = AP::FS().open(path, O_RDONLY);
    if (fd < 0) {
        // Like Lua io.open's nil result, not an ENOENT claim. errno is not
        // guaranteed thread-local on all HALs; do not classify it here.
        return AP_MODULE_STATUS_OK;
    }
    ReadFile file(fd);
    uint16_t length = 0;
    if (!read_at(fd, offset, count, output + 3, length)) {
        memset(output, 0, output_size);
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    if (!file.close()) {
        memset(output, 0, output_size);
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = 1;
    BindingWire::write(output + 1, length, 2);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif // AP_MODULE_ENABLED
