/*
  implement a file store for embedded firmware images
 */
#pragma once

#include <stdint.h>

class AP_ROMFS {
public:
    // find a file and de-compress, assumning gzip format. The
    // decompressed data will be allocated with malloc(). You must
    // call AP_ROMFS::free() on the return value after use. The next byte after
    // the file data is guaranteed to be null.
    static const uint8_t *find_decompress(const char *name, uint32_t &size);

    // free returned data
    static void free(const uint8_t *data);

    // get the size of a file without decompressing
    static bool find_size(const char *name, uint32_t &size);

    /*
      directory listing interface. Start with ofs=0. Returns pathnames
      that match dirname prefix. Ends with nullptr return when no more
      files found
    */
    static const char *dir_list(const char *dirname, uint16_t &ofs);

private:
    struct embedded_file {
        const char *filename;
        uint32_t compressed_size;
        uint32_t decompressed_size;
        uint32_t crc;
        const uint8_t *contents;
    };

    // find an embedded file
    static const AP_ROMFS::embedded_file *find_file(const char *name);

    static const struct embedded_file files[];
};
