#include "Util.h"

uint64_t HALSITL::Util::get_hw_rtc() const
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t seconds = ts.tv_sec;
    const uint64_t nanoseconds = ts.tv_nsec;
    return (seconds * 1000000ULL + nanoseconds/1000ULL);
}

/*
  get a (hopefully unique) machine ID
 */
bool HALSITL::Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    char *cbuf = (char *)buf;

    // try first to use machine-id file. Most systems will have this
    const char *paths[] = { "/etc/machine-id", "/var/lib/dbus/machine-id" };
    for (uint8_t i=0; i<ARRAY_SIZE(paths); i++) {
        int fd = open(paths[i], O_RDONLY);
        if (fd == -1) {
            continue;
        }
        ssize_t ret = read(fd, buf, len);
        close(fd);
        if (ret <= 0) {
            continue;
        }
        len = ret;
        char *p = strchr(cbuf, '\n');
        if (p) {
            *p = 0;
        }
        len = strnlen(cbuf, len);
        return true;
    }

    // fallback to hostname
    if (gethostname(cbuf, len) != 0) {
        // use a default name so this always succeeds. Without it we can't
        // implement some features (such as UAVCAN)
        strncpy(cbuf, "sitl-unknown", len);
    }
    len = strnlen(cbuf, len);
    return true;
}

/*
  as get_system_id_unformatted will already be ascii, we use the same
  ID here
 */
bool HALSITL::Util::get_system_id(char buf[40])
{
    uint8_t len = 40;
    return get_system_id_unformatted((uint8_t *)buf, len);
}
