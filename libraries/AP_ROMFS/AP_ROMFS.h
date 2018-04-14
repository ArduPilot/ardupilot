/*
  implement a file store for embedded firmware images
 */

#include <AP_HAL/AP_HAL.h>

class AP_ROMFS {
public:
    // find an embedded file
    static const uint8_t *find_file(const char *name, uint32_t &size);

private:
    struct embedded_file {
        const char *filename;
        uint32_t size;
        const uint8_t *contents;
    };
    static const struct embedded_file files[];
};
