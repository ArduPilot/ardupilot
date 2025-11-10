#include "SIM_config.h"

#if AP_SIM_GPS_FILE_ENABLED

#include "SIM_GPS_FILE.h"

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

/*
  read file data logged from AP_GPS_DEBUG_LOGGING_ENABLED
 */
void GPS_FILE::publish(const GPS_Data *d)
{
    static int fd[2] = {-1,-1};
    static uint32_t base_time[2];
    const uint16_t lognum = uint16_t(_sitl->gps_log_num.get());
    if (instance > 1) {
        return;
    }
    if (fd[instance] == -1) {
        char fname[] = "gpsN_NNN.log";
        hal.util->snprintf(fname, 13, "gps%u_%03u.log", instance+1, lognum);
        fd[instance] = open(fname, O_RDONLY|O_CLOEXEC);
        if (fd[instance] == -1) {
            return;
        }
    }
    const uint32_t magic = 0x7fe53b04;
    struct {
        uint32_t magic;
        uint32_t time_ms;
        uint32_t n;
    } header;
    uint8_t *buf = nullptr;
    while (true) {
        if (::read(fd[instance], (void *)&header, sizeof(header)) != sizeof(header) ||
            header.magic != magic) {
            goto rewind_file;
        }
        if (header.time_ms+base_time[instance] > AP_HAL::millis()) {
            // not ready for this data yet
            ::lseek(fd[instance], -sizeof(header), SEEK_CUR);
            return;
        }
        buf = NEW_NOTHROW uint8_t[header.n];
        if (buf != nullptr && ::read(fd[instance], buf, header.n) == ssize_t(header.n)) {
            write_to_autopilot((const char *)buf, header.n);
            delete[] buf;
            buf = nullptr;
            continue;
        }
        goto rewind_file;
    }

rewind_file:
    ::printf("GPS[%u] rewind\n", unsigned(instance));
    base_time[instance] = AP_HAL::millis();
    ::lseek(fd[instance], 0, SEEK_SET);
    delete[] buf;
}

#endif  // AP_SIM_GPS_FILE_ENABLED
