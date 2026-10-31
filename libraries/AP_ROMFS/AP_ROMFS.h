/*
  implement a file store for embedded firmware images
 */
#pragma once

#include <stdint.h>

class AP_ROMFS {
public:
    //  Find the named file and return its decompressed data and size. Caller
    //  must call AP_ROMFS::free() on the return value after use to free it.
    //  The data is guaranteed to be null-terminated such that it can be
    //  treated as a string. Firmware-link transformations are reversed so
    //  the returned bytes match the source file.
    static const uint8_t *find_decompress(const char *name, uint32_t &size);

    // Return a borrowed physical view of an uncompressed file. The pointer
    // remains valid for the firmware lifetime and must not be passed to
    // free(). A prelinked file contains its firmware-link transformations.
    static const uint8_t *find_uncompressed(const char *name, uint32_t &size);

    // free decompressed file data
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
    static constexpr uint32_t STORED_SIZE_MASK = 0x3FFFFFFFU;
    static constexpr uint32_t FLAG_UNCOMPRESSED = 1U << 31;
    static constexpr uint32_t FLAG_PRELINKED = 1U << 30;

    struct embedded_file {
        const char *filename;
        // The top two bits hold flags so the table retains its original size.
        uint32_t compressed_size;
        uint32_t decompressed_size;
        uint32_t crc;
        const uint8_t *contents;

        uint32_t stored_size() const { return compressed_size & STORED_SIZE_MASK; }
        bool uncompressed() const { return (compressed_size & FLAG_UNCOMPRESSED) != 0; }
        bool prelinked() const { return (compressed_size & FLAG_PRELINKED) != 0; }
    };

    // find an embedded file
    static const AP_ROMFS::embedded_file *find_file(const char *name);

    static const struct embedded_file files[];
};
