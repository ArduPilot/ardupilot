#include "AP_Networking_Config.h"

#if AP_NETWORKING_CAPTURE_ENABLED

#include "AP_Networking_Capture.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

void AP_Networking_Capture::start(const char *filename)
{
    WITH_SEMAPHORE(sem);
    if (fd != -1) {
        // Already capturing - flush
        AP::FS().fsync(fd);
        return;
    }
    const struct pcap_hdr {
        uint32_t magic_number;
        uint16_t version_major;
        uint16_t version_minor;
        int32_t  thiszone;
        uint32_t sigfigs;
        uint32_t snaplen;
        uint32_t network;       // 1 = Ethernet
    } hdr = {
        0xa1b2c3d4, 2, 4, 0, 0, 1522, 1
    };
    auto &fs = AP::FS();
    fd = fs.open(filename, O_WRONLY|O_CREAT|O_TRUNC);
    if (fd != -1) {
        fs.write(fd, (const void *)&hdr, sizeof(hdr));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: capturing to %s", filename);
    }
}

void AP_Networking_Capture::stop()
{
    WITH_SEMAPHORE(sem);
    if (fd != -1) {
        const int old_fd = fd;
        fd = -1;
        AP::FS().close(old_fd);
    }
}

void AP_Networking_Capture::capture_frame(const uint8_t *frame, size_t len)
{
    WITH_SEMAPHORE(sem);
    if (fd == -1) {
        return;
    }
    uint64_t utc_usec = 0;
#if AP_RTC_ENABLED
    AP::rtc().get_utc_usec(utc_usec);
#endif
    if (utc_usec == 0) {
        utc_usec = AP_HAL::micros64();
    }
    const struct pcaprec_hdr {
        uint32_t ts_sec;
        uint32_t ts_usec;
        uint32_t incl_len;
        uint32_t orig_len;
    } rec {
        .ts_sec = uint32_t(utc_usec / 1000000ULL),
        .ts_usec = uint32_t(utc_usec % 1000000ULL),
        .incl_len = uint32_t(len),
        .orig_len = uint32_t(len)
    };
    auto &fs = AP::FS();
    fs.write(fd, (const void *)&rec, sizeof(rec));
    fs.write(fd, frame, len);
}

#endif // AP_NETWORKING_CAPTURE_ENABLED
