/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_FIRMWARE_VERSION_CHECKER_HPP_INCLUDED
#define UAVCAN_POSIX_FIRMWARE_VERSION_CHECKER_HPP_INCLUDED

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <cerrno>
#include <dirent.h>

#include <uavcan/protocol/firmware_update_trigger.hpp>

// TODO Get rid of the macro
#if !defined(DIRENT_ISFILE) && defined(DT_REG)
# define DIRENT_ISFILE(dtype)  ((dtype) == DT_REG)
#endif

#define ALT_APD_SIGNATURE 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06

namespace uavcan_posix
{
/**
 * Firmware version checking logic.
 * Refer to @ref FirmwareUpdateTrigger for details.
 */
class FirmwareVersionChecker : public uavcan::IFirmwareVersionChecker
{
    enum { FilePermissions = 438 }; ///< 0o666

    enum { MaxBasePathLength = 128 };

    /**
     * This type is used for the base path
     */
    typedef uavcan::MakeString<MaxBasePathLength>::Type BasePathString;

    /**
     * Maximum length of full path including / the file name
     */
    enum { MaxPathLength = uavcan::protocol::file::Path::FieldTypes::path::MaxSize + MaxBasePathLength };

    /**
     * This type is used internally for the full path to file
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

    BasePathString base_path_;
    BasePathString alt_base_path_;

    static void addSlash(BasePathString& path)
    {
        if (path.back() != getPathSeparator())
        {
            path.push_back(getPathSeparator());
        }
    }

    static void removeSlash(BasePathString& path)
    {
        if (path.back() == getPathSeparator())
        {
            path.pop_back();
        }
    }

    void setFirmwareBasePath(const char* path)
    {
        base_path_ = path;
    }

    void setFirmwareAltBasePath(const char* path)
    {
        alt_base_path_ = path;
    }

protected:
    /**
     * This method will be invoked when the class obtains a response to GetNodeInfo request.
     *
     * @param node_id                   Node ID that this GetNodeInfo response was received from.
     *
     * @param node_info                 Actual node info structure; refer to uavcan.protocol.GetNodeInfo for details.
     *
     * @param out_firmware_file_path    The implementation should return the firmware image path via this argument.
     *                                  Note that this path must be reachable via uavcan.protocol.file.Read service.
     *                                  Refer to @ref FileServer and @ref BasicFileServer for details.
     *
     * @return                          True - the class will begin sending update requests.
     *                                  False - the node will be ignored, no request will be sent.
     */
    virtual bool shouldRequestFirmwareUpdate(uavcan::NodeID,
                                             const uavcan::protocol::GetNodeInfo::Response& node_info,
                                             FirmwareFilePath& out_firmware_file_path)
    {
        using namespace std;

        /* This is a work  around for two issues.
         *  1) FirmwareFilePath is 40
         *  2) OK using is using 32 for max file names.
         *
         *  So for the file:
         *    org.pixhawk.px4cannode-v1-0.1.59efc137.uavcan.bin
         *    +---ufw
         *        +- board_id.bin
         */
        bool rv = false;
        uint16_t board_id = (node_info.hardware_version.major << 8) + node_info.hardware_version.minor;
        char bin_file_name[MaxBasePathLength + 1];
        int n = snprintf(bin_file_name, sizeof(bin_file_name), "%u.bin", board_id);

        if (n > 0 && n < (int)sizeof(bin_file_name) - 2)
        {
            char bin_file_path[MaxBasePathLength + 1];

            // Look on Primary location

            snprintf(bin_file_path, sizeof(bin_file_path), "%s%s",
                      getFirmwareBasePath().c_str(), bin_file_name);

           // We have a file path, is it a valid image

            AppDescriptor descriptor{0};

            bool found = getFileInfo(bin_file_path, descriptor) == 0;

            if (!found && !getFirmwareAltBasePath().empty())
            {
                snprintf(bin_file_name, sizeof(bin_file_name), "%u.bin", board_id);
                snprintf(bin_file_path, sizeof(bin_file_path), "%s/%s",
                         getFirmwareAltBasePath().c_str(), bin_file_name);

                found = getFileInfo(bin_file_path, descriptor) == 0;
            }

            if (found && (node_info.software_version.image_crc == 0 ||
                (node_info.software_version.major == 0 && node_info.software_version.minor == 0) ||
                descriptor.image_crc != node_info.software_version.image_crc))
            {
                rv = true;
                out_firmware_file_path = bin_file_name;
            }
        }
        return rv;
    }

    /**
     * This method will be invoked when a node responds to the update request with an error. If the request simply
     * times out, this method will not be invoked.
     * Note that if by the time of arrival of the response the node is already removed, this method will not be called.
     *
     * SPECIAL CASE: If the node responds with ERROR_IN_PROGRESS, the class will assume that further requesting
     *               is not needed anymore. This method will not be invoked.
     *
     * @param node_id                   Node ID that returned this error.
     *
     * @param error_response            Contents of the error response. It contains error code and text.
     *
     * @param out_firmware_file_path    New firmware path if a retry is needed. Note that this argument will be
     *                                  initialized with old path, so if the same path needs to be reused, this
     *                                  argument should be left unchanged.
     *
     * @return                          True - the class will continue sending update requests with new firmware path.
     *                                  False - the node will be forgotten, new requests will not be sent.
     */
    virtual bool shouldRetryFirmwareUpdate(uavcan::NodeID,
                                           const uavcan::protocol::file::BeginFirmwareUpdate::Response&,
                                           FirmwareFilePath&)
    {
        // TODO: Limit the number of attempts per node
        return true;
    }

public:
    struct AppDescriptor
    {
        uavcan::uint8_t signature[sizeof(uavcan::uint64_t)];
        union{
            uavcan::uint64_t image_crc;
            uavcan::uint32_t crc32_block1;
            uavcan::uint32_t crc32_block2;
        };
        uavcan::uint32_t image_size;
        uavcan::uint32_t vcs_commit;
        uavcan::uint8_t  major_version;
        uavcan::uint8_t  minor_version;
        uavcan::uint16_t board_id;
        uavcan::uint8_t  reserved[ 3 + 3 + 2];
    };

    static int getFileInfo(const char* path, AppDescriptor& descriptor, int limit = 0)
    {
        using namespace std;

        const unsigned MaxChunk = 512 / sizeof(uint64_t);

        // Make sure this does not present as a valid descriptor
        struct {
          union {
            uavcan::uint64_t l;
            uavcan::uint8_t  b[sizeof(uint64_t)]{ALT_APD_SIGNATURE};
          } signature;
          uavcan::uint8_t  zeropad[sizeof(AppDescriptor) - sizeof(uint64_t)]{0};
        } s;

        int rv = -ENOENT;
        uint64_t chunk[MaxChunk];
        int fd = open(path, O_RDONLY);

        if (fd >= 0)
        {
            AppDescriptor* pdescriptor = UAVCAN_NULLPTR;

            while (pdescriptor == UAVCAN_NULLPTR && limit >= 0)
            {
                int len = read(fd, chunk, sizeof(chunk));

                if (len == 0)
                {
                    break;
                }

                if (len < 0)
                {
                    rv = -errno;
                    goto out_close;
                }

                uint64_t* p = &chunk[0];

                if (limit > 0)
                {
                    limit -= sizeof(chunk);
                }

                do
                {
                    if (*p == s.signature.l)
                    {
                        pdescriptor = reinterpret_cast<AppDescriptor*>(p); // FIXME TODO This breaks strict aliasing
                        descriptor = *pdescriptor;
                        rv = 0;
                        break;
                    }
                }
                while (p++ <= &chunk[MaxChunk - (sizeof(AppDescriptor) / sizeof(chunk[0]))]);
            }

        out_close:
            (void)close(fd);
        }
        return rv;
    }

    const BasePathString& getFirmwareBasePath() const { return base_path_; }

    const BasePathString& getFirmwareAltBasePath() const { return alt_base_path_; }

    static char getPathSeparator()
    {
        return static_cast<char>(uavcan::protocol::file::Path::SEPARATOR);
    }

    /**
     * Creates the Directories were the files will be stored
     *
     * This is directory structure is in support of a workaround
     * for the issues that FirmwareFilePath is 40
     *
     *  It creates a path structure:
     *    +---(base_path)  <----------- Files are here.
     */

    int createFwPaths(const char* base_path, const char* alt_base_path = nullptr)
    {
        using namespace std;
        int rv = -uavcan::ErrInvalidParam;

        if (alt_base_path)
        {
            const int len = strlen(alt_base_path);
            if (len > 0 && len < base_path_.MaxSize)
            {
                setFirmwareAltBasePath(alt_base_path);
            }
            else
              {
                return rv;
              }
        }

        if (base_path)
            {
            const int len = strlen(base_path);

            if (len > 0 && len < base_path_.MaxSize)
            {
                setFirmwareBasePath(base_path);
                removeSlash(base_path_);
                const char* path = getFirmwareBasePath().c_str();

                rv = 0;
                struct stat sb;
                if (stat(path, &sb) != 0 || !S_ISDIR(sb.st_mode))
                {
                    rv = mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
                }
                addSlash(base_path_);
            }
        }
        return rv;
    }

    const char* getFirmwarePath() const
    {
        return getFirmwareBasePath().c_str();
    }

    const char* getAltFirmwarePath() const
    {
        return getFirmwareAltBasePath().c_str();
    }
};
}

#endif // Include guard
