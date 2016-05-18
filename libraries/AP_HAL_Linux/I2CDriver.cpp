#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/i2c-dev.h>
#ifndef I2C_SMBUS_BLOCK_MAX
#include <linux/i2c.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#include "I2CDriver.h"
#include "Util.h"

extern const AP_HAL::HAL &hal;

using namespace Linux;

I2CDriver::I2CDriver(uint8_t bus)
    : _fake_dev(I2CDeviceManager::from(hal.i2c_mgr)->get_device(bus, 0))
{
}

I2CDriver::I2CDriver(std::vector<const char *> devpaths)
    : _fake_dev(I2CDeviceManager::from(hal.i2c_mgr)->get_device(devpaths, 0))
{
}

/*
  tell the I2C library what device we want to talk to
 */
bool I2CDriver::set_address(uint8_t addr)
{
    if (!_fake_dev) {
        return false;
    }

    if (_addr == addr) {
        /* nothing to do */
        return true;
    }

    int fd = _fake_dev->get_fd();
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        if (errno != EBUSY) {
            return false;
        }
        /* Only print this message once per i2c bus */
        if (_print_ioctl_error) {
            hal.console->printf("couldn't set i2c slave address: %s\n",
                                strerror(errno));
            hal.console->printf("trying I2C_SLAVE_FORCE\n");
            _print_ioctl_error = false;
        }
        if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0) {
            return false;
        }
    }

    _addr = addr;

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
    if (::write(_fake_dev->get_fd(), data, len) != len) {
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
    if (_i2c_smbus_access(_fake_dev->get_fd(),I2C_SMBUS_WRITE, reg,
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
    if (::read(_fake_dev->get_fd(), data, len) != len) {
        return 1;
    }
    return 0;
}

uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    if (!_fake_dev) {
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

    if (ioctl(_fake_dev->get_fd(), I2C_RDWR, &i2c_data) == -1) {
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

    if (!_fake_dev) {
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
        if (ioctl(_fake_dev->get_fd(), I2C_RDWR, &i2c_data) == -1) {
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
    if (_i2c_smbus_access(_fake_dev->get_fd(),I2C_SMBUS_READ, reg,
                          I2C_SMBUS_BYTE_DATA, &v)) {
        return 1;
    }
    *data = v.byte;
    return 0;
}

/*
  main transfer function
 */
bool I2CDriver::do_transfer(uint8_t addr, const uint8_t *send,
                            uint32_t send_len, uint8_t *recv,
                            uint32_t recv_len)
{
    struct i2c_msg i2cmsg[2] = {
        {
        addr  : addr,
        flags : 0,
        len   : (typeof(i2cmsg->len))send_len,
        buf   : (typeof(i2cmsg->buf))send
        },
        {
        addr  : addr,
        flags : I2C_M_RD,
        len   : (typeof(i2cmsg->len))recv_len,
        buf   : (typeof(i2cmsg->buf))recv,
        }
    };
    struct i2c_rdwr_ioctl_data msg_rdwr;

    if (send_len == 0 && recv_len) {
        msg_rdwr.msgs = &i2cmsg[1];
        msg_rdwr.nmsgs = 1;
    } else if (send_len && recv_len == 0) {
        msg_rdwr.msgs = &i2cmsg[0];
        msg_rdwr.nmsgs = 1;
    }else if (send_len && recv_len) {
        msg_rdwr.msgs = &i2cmsg[0];
        msg_rdwr.nmsgs = 2;
    } else {
        return false;
    }
    return ioctl(_fake_dev->get_fd(), I2C_RDWR, &msg_rdwr) == (int)msg_rdwr.nmsgs;
}

uint8_t I2CDriver::lockup_count()
{
    return 0;
}

AP_HAL::Semaphore *I2CDriver::get_semaphore()
{
    return _fake_dev->get_semaphore();
}
