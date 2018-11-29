#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#include "Heat_Pwm.h"
#include "ToneAlarm_Disco.h"
#include "Util.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
ToneAlarm_Disco Util::_toneAlarm;
#else
ToneAlarm Util::_toneAlarm;
#endif

void Util::init(int argc, char * const *argv) {
    saved_argc = argc;
    saved_argv = argv;

#ifdef HAL_UTILS_HEAT
#if HAL_UTILS_HEAT == HAL_LINUX_HEAT_PWM
    _heat = new Linux::HeatPwm(HAL_LINUX_HEAT_PWM_NUM,
                               HAL_LINUX_HEAT_KP,
                               HAL_LINUX_HEAT_KI,
                               HAL_LINUX_HEAT_PERIOD_NS);
#else
    #error Unrecognized Heat
#endif // #if
#else
    _heat = new Linux::Heat();
#endif // #ifdef
}

// set current IMU temperatue in degrees C
void Util::set_imu_temp(float current)
{
    _heat->set_imu_temp(current);
}

// set target IMU temperatue in degrees C
void Util::set_imu_target_temp(int8_t *target)
{
    _heat->set_imu_target_temp(target);
}

/**
   return commandline arguments, if available
*/
void Util::commandline_arguments(uint8_t &argc, char * const *&argv)
{
    argc = saved_argc;
    argv = saved_argv;
}

void Util::set_hw_rtc(uint64_t time_utc_usec)
{
#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
    // call superclass method to set time.  We've guarded this so we
    // don't reset the HW clock time on people's laptops.
    AP_HAL::Util::set_hw_rtc(time_utc_usec);
#endif
}

bool Util::is_chardev_node(const char *path)
{
    struct stat st;

    if (!path || lstat(path, &st) < 0) {
        return false;
    }

    return S_ISCHR(st.st_mode);
}

/*
  always report 256k of free memory. Using mallinfo() isn't useful as
  it only reported the current heap, which auto-expands. What we're
  trying to do here is ensure that code which checks for free memory
  before allocating objects does allow the allocation
 */
uint32_t Util::available_memory(void)
{
    return 256*1024;
}

#ifndef HAL_LINUX_DEFAULT_SYSTEM_ID
#define HAL_LINUX_DEFAULT_SYSTEM_ID "linux-unknown"
#endif

/*
  get a (hopefully unique) machine ID
 */
bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
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
        strncpy(cbuf, HAL_LINUX_DEFAULT_SYSTEM_ID, len);
    }
    len = strnlen(cbuf, len);
    return true;
}

/*
  as get_system_id_unformatted will already be ascii, we use the same
  ID here
 */
bool Util::get_system_id(char buf[40])
{
    uint8_t len = 40;
    return get_system_id_unformatted((uint8_t *)buf, len);
}


int Util::write_file(const char *path, const char *fmt, ...)
{
    errno = 0;

    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd == -1) {
        return -errno;
    }

    va_list args;
    va_start(args, fmt);

    int ret = vdprintf(fd, fmt, args);
    int errno_bkp = errno;
    close(fd);

    va_end(args);

    if (ret < 1) {
        return -errno_bkp;
    }

    return ret;
}

int Util::read_file(const char *path, const char *fmt, ...)
{
    errno = 0;

    FILE *file = fopen(path, "re");
    if (!file) {
        return -errno;
    }

    va_list args;
    va_start(args, fmt);

    int ret = vfscanf(file, fmt, args);
    int errno_bkp = errno;
    fclose(file);

    va_end(args);

    if (ret < 1) {
        return -errno_bkp;
    }

    return ret;
}

const char *Linux::Util::_hw_names[UTIL_NUM_HARDWARES] = {
    [UTIL_HARDWARE_RPI1]   = "BCM2708",
    [UTIL_HARDWARE_RPI2]   = "BCM2709",
    [UTIL_HARDWARE_BEBOP]  = "Mykonos3 board",
    [UTIL_HARDWARE_BEBOP2] = "Milos board",
    [UTIL_HARDWARE_DISCO]  = "Evinrude board",
};

#define MAX_SIZE_LINE 50
int Util::get_hw_arm32()
{
    char buffer[MAX_SIZE_LINE] = { 0 };
    FILE *f = fopen("/proc/cpuinfo", "r");
    if (f == nullptr) {
        return -errno;
    }

    while (fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
        if (strstr(buffer, "Hardware") == nullptr) {
            continue;
        }
        for (uint8_t i = 0; i < UTIL_NUM_HARDWARES; i++) {
            if (strstr(buffer, _hw_names[i]) == nullptr) {
                continue;
            }
            fclose(f);
            return i;
        }
    }

    fclose(f);
    return -ENOENT;
}
