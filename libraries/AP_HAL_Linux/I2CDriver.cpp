#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "I2CDriver.h"
#include "Util.h"

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#ifndef I2C_SMBUS_BLOCK_MAX
#include <linux/i2c.h>
#endif

extern const AP_HAL::HAL& hal;

using namespace Linux;

/*
  constructor
 */
I2CDriver::I2CDriver(AP_HAL::Semaphore* semaphore, const char *device) :
    _semaphore(semaphore)
{
    _device = strdup(device);

#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
    if (!((Util*)hal.util)->is_chardev_node(_device))
        hal.scheduler->panic("I2C device is not a chardev node");
#endif
}

/* Match a given device by the prefix its devpath, i.e. the path returned by
 * udevadm info -q path /dev/<i2c-device>'. This constructor can be used when
 * the number of the I2C bus is not stable across reboots. It matches the
 * first device with a prefix in @devpaths */
I2CDriver::I2CDriver(AP_HAL::Semaphore* semaphore,
                               const char * const devpaths[]) :
    _semaphore(semaphore)
{
    const char *dirname = "/sys/class/i2c-dev";
    struct dirent *de;
    DIR *d;

    d = opendir(dirname);
    if (!d)
        hal.scheduler->panic("Could not get list of I2C buses");

    for (de = readdir(d); de; de = readdir(d)) {
        const char *p, * const *t;
        char buf[PATH_MAX], buf2[PATH_MAX];

        if (strcmp(de->d_name, ".") == 0 || strcmp(de->d_name, "..") == 0)
            continue;

        if (snprintf(buf, sizeof(buf), "%s/%s", dirname, de->d_name) >= PATH_MAX)
            continue;

        p = realpath(buf, buf2);
        if (!p || strncmp(p, "/sys", sizeof("/sys") - 1))
            continue;

        p += sizeof("/sys") - 1;

        for (t = devpaths; t && *t; t++) {
            if (strncmp(p, *t, strlen(*t)) == 0)
                break;
        }

        if (!*t)
            continue;

        /* Found the device name, use the new path */
        asprintf(&_device, "/dev/%s", de->d_name);
        break;
    }

    closedir(d);

    if (!((Util*)hal.util)->is_chardev_node(_device))
        hal.scheduler->panic("I2C device is not a chardev node");
}

I2CDriver::~I2CDriver()
{
    free(_device);
}

/*
  called from HAL class init()
 */
void I2CDriver::begin() 
{
    if (_fd != -1) {
        close(_fd);
    }
    _fd = open(_device, O_RDWR);
}

void I2CDriver::end() 
{
    if (_fd != -1) {
        ::close(_fd);
        _fd = -1;
    }
}

/*
  tell the I2C library what device we want to talk to
 */
bool I2CDriver::set_address(uint8_t addr)
{
    if (_fd == -1) {
        return false;
    }
    if (_addr != addr) {
        ioctl(_fd, I2C_SLAVE, addr);
        _addr = addr;
    }
    return true;
}

void I2CDriver::setTimeout(uint16_t ms) 
{
    // unimplemented
}

void I2CDriver::setHighSpeed(bool active) 
{
    // unimplemented    
}

uint8_t I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    if (::write(_fd, data, len) != len) {
        return 1;
    }
    return 0; // success
}


uint8_t I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                       uint8_t len, uint8_t* data)
{
    uint8_t buf[len+1];
    buf[0] = reg;
    if (len != 0) {
        memcpy(&buf[1], data, len);
    }
    return write(addr, len+1, buf);
}

/*
  this is a copy of i2c_smbus_access() from i2c-dev.h. We need it for
  platforms with older headers
 */
static inline __s32 _i2c_smbus_access(int file, char read_write, __u8 command, 
                                      int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;
	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

uint8_t I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    if (!set_address(addr)) {
        return 1;
    }
    union i2c_smbus_data data;
    data.byte = val;
    if (_i2c_smbus_access(_fd,I2C_SMBUS_WRITE, reg,
                         I2C_SMBUS_BYTE_DATA, &data) == -1) {
        return 1;
    }
    return 0;
}

uint8_t I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    if (::read(_fd, data, len) != len) {
        return 1;
    }
    return 0;
}

uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    if (_fd == -1) {
        return 1;
    }
    struct i2c_msg msgs[] = {
        {
        addr  : addr,
        flags : 0,
        len   : 1,
        buf   : (typeof(msgs->buf))&reg
        },
        {
        addr  : addr,
        flags : I2C_M_RD,
        len   : len,
        buf   : (typeof(msgs->buf))data,
        }
    };
    struct i2c_rdwr_ioctl_data i2c_data = {
    msgs : msgs,
    nmsgs : 2
    };

    // prevent valgrind error
    memset(data, 0, len);

    if (ioctl(_fd, I2C_RDWR, &i2c_data) == -1) {
        return 1;
    }

    return 0;
}


uint8_t I2CDriver::readRegistersMultiple(uint8_t addr, uint8_t reg,
                                              uint8_t len, 
                                              uint8_t count, uint8_t* data)
{
#ifdef I2C_RDRW_IOCTL_MAX_MSGS
    const uint8_t max_count = I2C_RDRW_IOCTL_MAX_MSGS / 2;
#else
    const uint8_t max_count = 8;
#endif

    if (_fd == -1) {
        return 1;
    }
    while (count > 0) {
        uint8_t n = count > max_count ? max_count : count;
        struct i2c_msg msgs[2*n];
        struct i2c_rdwr_ioctl_data i2c_data = {
        msgs : msgs,
        nmsgs : (typeof(i2c_data.nmsgs))(2*n)
        };
        for (uint8_t i=0; i<n; i++) {
            msgs[i*2].addr = addr;
            msgs[i*2].flags = 0;
            msgs[i*2].len = 1;
            msgs[i*2].buf = (typeof(msgs->buf))&reg;
            msgs[i*2+1].addr = addr;
            msgs[i*2+1].flags = I2C_M_RD;
            msgs[i*2+1].len = len;
            msgs[i*2+1].buf = (typeof(msgs->buf))data;
            data += len;
        };
        if (ioctl(_fd, I2C_RDWR, &i2c_data) == -1) {
            return 1;
        }
        count -= n;
    }
    return 0;
}


uint8_t I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    union i2c_smbus_data v;
    memset(&v, 0, sizeof(v));
    if (_i2c_smbus_access(_fd,I2C_SMBUS_READ, reg,
                          I2C_SMBUS_BYTE_DATA, &v)) {
        return 1;
    }
    *data = v.byte;
    return 0;
}

uint8_t I2CDriver::lockup_count() 
{
    return 0;
}
#endif // CONFIG_HAL_BOARD
