#include "GPIO_Sysfs.h"

#include <AP_HAL/AP_HAL.h>

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include "Util.h"

#define LOW             0
#define HIGH            1
#define assert_vpin(v_, max_, ...) do {                                       \
       if (v_ >= max_) {                                                      \
           hal.console->printf("warning (%s): vpin %u out of range [0, %u)\n",\
                               __PRETTY_FUNCTION__, v_, max_);                \
           return __VA_ARGS__;                                                \
       }                                                                      \
    } while (0)

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define UINT32_MAX_STR "4294967295"

/* Trick to use minimum stack space for each of the params */
union gpio_params {
    char export_gpio[sizeof("export")];
    char direction[sizeof("gpio" UINT32_MAX_STR "/direction")];
    char value[sizeof("gpio" UINT32_MAX_STR "/value")];
};

#define GPIO_BASE_PATH "/sys/class/gpio/"
#define GPIO_PATH_MAX (sizeof(GPIO_BASE_PATH) + sizeof(gpio_params) - 1)

DigitalSource_Sysfs::DigitalSource_Sysfs(unsigned pin, int value_fd)
    : _pin(pin)
    , _value_fd(value_fd)
{
}

DigitalSource_Sysfs::~DigitalSource_Sysfs()
{
    if (_value_fd >= 0) {
        ::close(_value_fd);
    }
}

uint8_t DigitalSource_Sysfs::read()
{
    char char_value;
    if (::pread(_value_fd, &char_value, 1, 0) < 0) {
        hal.console->printf("DigitalSource_Sysfs: Unable to read pin %u value.\n",
                            _pin);
        return LOW;
    }
    return char_value - '0';
}

void DigitalSource_Sysfs::write(uint8_t value)
{
    if (::pwrite(_value_fd, value == HIGH ? "1" : "0", 1, 0) < 0) {
        hal.console->printf("DigitalSource_Sysfs: Unable to write pin %u value.\n",
                            _pin);
    }
}

void DigitalSource_Sysfs::mode(uint8_t output)
{
    auto gpio_sysfs = GPIO_Sysfs::from(hal.gpio);
    gpio_sysfs->_pinMode(_pin, output);
}

void DigitalSource_Sysfs::toggle()
{
    write(!read());
}

void GPIO_Sysfs::init()
{
}

void GPIO_Sysfs::pinMode(uint8_t vpin, uint8_t output)
{
    assert_vpin(vpin, n_pins);

    _export_pin(vpin);
    _pinMode(pin_table[vpin], output);
}

void GPIO_Sysfs::_pinMode(unsigned int pin, uint8_t output)
{
    const char *dir = output ? "out" : "in";
    char path[GPIO_PATH_MAX];

    int r = snprintf(path, GPIO_PATH_MAX, GPIO_BASE_PATH "gpio%u/direction",
                     pin);
    if (r < 0 || r >= (int)GPIO_PATH_MAX
        || Util::from(hal.util)->write_file(path, "%s", dir) < 0) {
        hal.console->printf("GPIO_Sysfs: Unable to set pin %u mode.\n", pin);
    }
}

int GPIO_Sysfs::_open_pin_value(unsigned int pin, int flags)
{
    char path[GPIO_PATH_MAX];
    int fd;

    int r = snprintf(path, GPIO_PATH_MAX, GPIO_BASE_PATH "gpio%u/value", pin);
    if (r < 0 || r >= (int)GPIO_PATH_MAX
        || (fd = open(path, flags | O_CLOEXEC)) < 0) {
        hal.console->printf("GPIO_Sysfs: Unable to get value file descriptor for pin %u.\n",
                            pin);
        return -1;
    }

    return fd;
}

uint8_t GPIO_Sysfs::read(uint8_t vpin)
{
    assert_vpin(vpin, n_pins, LOW);

    const unsigned pin = pin_table[vpin];
    int fd = _open_pin_value(pin, O_RDONLY);

    if (fd < 0) {
        goto error;
    }

    char char_value;
    if (::pread(fd, &char_value, 1, 0) < 0) {
        goto error;
    }

    close(fd);

    return char_value - '0';

error:
    hal.console->printf("GPIO_Sysfs: Unable to read pin %u value.\n", vpin);
    return LOW;
}

void GPIO_Sysfs::write(uint8_t vpin, uint8_t value)
{
    assert_vpin(vpin, n_pins);

    const unsigned pin = pin_table[vpin];
    int fd = _open_pin_value(pin, O_WRONLY);

    if (fd < 0) {
        goto error;
    }

    if (::pwrite(fd, value == HIGH ? "1" : "0", 1, 0) < 0) {
        goto error;
    }

    close(fd);
    return;

error:
    hal.console->printf("GPIO_Sysfs: Unable to write pin %u value.\n", vpin);
}

void GPIO_Sysfs::toggle(uint8_t vpin)
{
    write(vpin, !read(vpin));
}

AP_HAL::DigitalSource* GPIO_Sysfs::channel(uint16_t vpin)
{
    assert_vpin(vpin, n_pins, nullptr);

    const unsigned pin = pin_table[vpin];
    int value_fd = -1;

    if (_export_pin(vpin)) {
        value_fd = _open_pin_value(pin, O_RDWR);
    }

    /* Even if we couldn't open the fd, return a new DigitalSource and let
     * reads and writes fail later due to invalid. Otherwise we
     * could crash in undesired places */
    return new DigitalSource_Sysfs(pin, value_fd);
}

bool GPIO_Sysfs::usb_connected(void)
{
    return false;
}

bool GPIO_Sysfs::_export_pin(uint8_t vpin)
{
    assert_vpin(vpin, n_pins, false);

    const unsigned int pin = pin_table[vpin];
    char gpio_path[GPIO_PATH_MAX];
    int fd;

    int r = snprintf(gpio_path, GPIO_PATH_MAX, GPIO_BASE_PATH "gpio%u", pin);
    if (r < 0 || r >= (int) GPIO_PATH_MAX) {
        goto fail_snprintf;
    }

    if (access(gpio_path, F_OK) == 0) {
        // Already exported
        return true;
    }

    fd = open(GPIO_BASE_PATH "export", O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        goto fail_open;
    }

    // Try to export
    if (dprintf(fd, "%u", pin) < 0) {
        goto fail_export;
    }

    close(fd);
    return true;

fail_export:
    close(fd);
fail_open:
fail_snprintf:
    hal.console->printf("GPIO_Sysfs: Unable to export pin %u.\n", pin);
    return false;
}
