// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingFilesystem.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <fcntl.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint16_t PATH_SIZE = 256;
static constexpr uint16_t READ_MAX = 1024;
static constexpr uint16_t WRITE_MAX = 1024;
static constexpr uint16_t NAME_MAXIMUM = 255;

static const char *path_from(const uint8_t *input)
{
    const char *path = BindingWire::read_cstring(input);
    return path != nullptr && path[0] != 0 ? path : nullptr;
}

int32_t binding_filesystem_stat(const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != PATH_SIZE || output == nullptr || output_size != 10) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const char *path = path_from(input);
    if (path == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_FILESYSTEM_FILE_READING_ENABLED
    AP_Filesystem::stat_t stat;
    if (!AP::FS().stat(path, stat)) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = 1;
    BindingWire::write(output + 1, stat.size, 4);
    BindingWire::write(output + 5, stat.mtime, 4);
    output[9] = stat.is_directory();
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_filesystem_read_at(const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != PATH_SIZE + 6U || output == nullptr ||
        output_size != READ_MAX + 3U) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const char *path = path_from(input);
    const uint32_t offset = BindingWire::read(input + PATH_SIZE, 4);
    const uint16_t count = uint16_t(BindingWire::read(input + PATH_SIZE + 4, 2));
    if (path == nullptr || count == 0 || count > READ_MAX || uint64_t(offset) + count > INT32_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_FILESYSTEM_FILE_READING_ENABLED
    const int fd = AP::FS().open(path, O_RDONLY);
    if (fd < 0) {
        return AP_MODULE_STATUS_OK;
    }
    int32_t status = AP_MODULE_STATUS_OK;
    const int32_t position = AP::FS().lseek(fd, int32_t(offset), SEEK_SET);
    uint16_t length = 0;
    if (position < 0 || uint32_t(position) > offset) {
        status = AP_MODULE_STATUS_HOST_ERROR;
    } else {
        while (length < count) {
            const int32_t got = AP::FS().read(fd, output + 3 + length, count - length);
            if (got < 0 || uint32_t(got) > uint32_t(count - length)) {
                status = AP_MODULE_STATUS_HOST_ERROR;
                break;
            }
            if (got == 0) {
                break;
            }
            if (uint32_t(position) != offset) {
                status = AP_MODULE_STATUS_HOST_ERROR;
                break;
            }
            length += uint16_t(got);
        }
    }
    if (AP::FS().close(fd) != 0 && status == AP_MODULE_STATUS_OK) {
        status = AP_MODULE_STATUS_HOST_ERROR;
    }
    if (status != AP_MODULE_STATUS_OK) {
        memset(output, 0, output_size);
        return status;
    }
    output[0] = 1;
    BindingWire::write(output + 1, length, 2);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_filesystem_dir_entry(const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != PATH_SIZE + 2U || output == nullptr ||
        output_size != NAME_MAXIMUM + 3U) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const char *path = path_from(input);
    const uint16_t wanted = uint16_t(BindingWire::read(input + PATH_SIZE, 2));
    if (path == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_FILESYSTEM_FILE_READING_ENABLED
    auto *directory = AP::FS().opendir(path);
    if (directory == nullptr) {
        return AP_MODULE_STATUS_OK;
    }
    struct dirent *entry = nullptr;
    for (uint32_t index = 0; index <= wanted; index++) {
        entry = AP::FS().readdir(directory);
        if (entry == nullptr) {
            break;
        }
    }
    int32_t status = AP_MODULE_STATUS_OK;
    if (entry != nullptr) {
        const size_t length = strnlen(entry->d_name, NAME_MAXIMUM + 1U);
        if (length > NAME_MAXIMUM) {
            status = AP_MODULE_STATUS_HOST_ERROR;
        } else {
            output[0] = 1;
            BindingWire::write(output + 1, uint16_t(length), 2);
            memcpy(output + 3, entry->d_name, length);
        }
    }
    if (AP::FS().closedir(directory) != 0 && status == AP_MODULE_STATUS_OK) {
        status = AP_MODULE_STATUS_HOST_ERROR;
    }
    if (status != AP_MODULE_STATUS_OK) {
        memset(output, 0, output_size);
    }
    return status;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_filesystem_truncate(const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != PATH_SIZE || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *path = path_from(input);
    if (path == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    const int fd = AP::FS().open(path, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd < 0) {
        return AP_MODULE_STATUS_OK;
    }
    if (AP::FS().close(fd) != 0) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = 1;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_filesystem_append(const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != PATH_SIZE + 2U + WRITE_MAX ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *path = path_from(input);
    const uint16_t count = uint16_t(BindingWire::read(input + PATH_SIZE, 2));
    if (path == nullptr || count == 0 || count > WRITE_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = count; i < WRITE_MAX; i++) {
        if (input[PATH_SIZE + 2U + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    const int fd = AP::FS().open(path, O_WRONLY | O_CREAT | O_APPEND);
    if (fd < 0) {
        return AP_MODULE_STATUS_OK;
    }
    int32_t status = AP_MODULE_STATUS_OK;
    uint16_t written = 0;
    while (written < count) {
        const int32_t amount = AP::FS().write(fd, input + PATH_SIZE + 2U + written, count - written);
        if (amount <= 0 || uint32_t(amount) > uint32_t(count - written)) {
            status = AP_MODULE_STATUS_HOST_ERROR;
            break;
        }
        written += uint16_t(amount);
    }
    if (AP::FS().close(fd) != 0 && status == AP_MODULE_STATUS_OK) {
        status = AP_MODULE_STATUS_HOST_ERROR;
    }
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = 1;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_filesystem_unlink(const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != PATH_SIZE || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *path = path_from(input);
    if (path == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    output[0] = AP::FS().unlink(path) == 0;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_filesystem_format(const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (input != nullptr || input_size != 0 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_FILESYSTEM_FORMAT_ENABLED
    output[0] = AP::FS().format();
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
