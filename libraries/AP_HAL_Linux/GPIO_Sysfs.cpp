#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/limits.h>

#include "GPIO.h"
#include "GPIO_Sysfs.h"

#define LOW             0
#define HIGH            1
#define GPIO_PATH_FORMAT "/sys/class/gpio/gpio%u"
#define assert_vpin(v_, max_, ...) do {                                       \
       if (v_ >= max_) {                                                      \
           hal.console->printf("warning (%s): vpin %u out of range [0, %u)\n",\
                               __PRETTY_FUNCTION__, v_, max_);                \
           return __VA_ARGS__;                                                \
       }                                                                      \
    } while (0)

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

LinuxDigitalSource_Sysfs::LinuxDigitalSource_Sysfs(unsigned pin, int value_fd) :
    _pin(pin),
    _value_fd(value_fd)
{
}

LinuxDigitalSource_Sysfs::~LinuxDigitalSource_Sysfs()
{
    if (_value_fd >= 0) {
        ::close(_value_fd);
    }
}

uint8_t LinuxDigitalSource_Sysfs::read()
{
    char char_value;
    int r = ::pread(_value_fd, &char_value, 1, 0);
    if (r < 0) {
        hal.console->printf("warning: unable to read GPIO value for pin %u\n",
                            _pin);
        return LOW;
    }
    return char_value == '1' ? HIGH : LOW;
}

void LinuxDigitalSource_Sysfs::write(uint8_t value)
{
    int r = ::pwrite(_value_fd, value == HIGH ? "1" : "0", 1, 0);
    if (r < 0) {
        hal.console->printf("warning: unable to write GPIO value for pin %u\n",
                            _pin);
    }
}

void LinuxDigitalSource_Sysfs::mode(uint8_t output)
{
    auto gpio_sysfs = LinuxGPIO_Sysfs::from(hal.gpio);
    gpio_sysfs->_pinMode(_pin, output);
}

void LinuxDigitalSource_Sysfs::toggle()
{
    write(!read());
}

void LinuxGPIO_Sysfs::init()
{
}

void LinuxGPIO_Sysfs::pinMode(uint8_t vpin, uint8_t output)
{
    assert_vpin(vpin, n_pins);

    unsigned pin = pin_table[vpin];
    _pinMode(pin, output);
}

void LinuxGPIO_Sysfs::_pinMode(unsigned int pin, uint8_t output)
{
    char direction_path[PATH_MAX];

    snprintf(direction_path, PATH_MAX, GPIO_PATH_FORMAT "/direction", pin);

    int fd = open(direction_path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        hal.console->printf("unable to open %s\n", direction_path);
        return;
    }

    int r = dprintf(fd, output == HAL_GPIO_INPUT ? "in\n" : "out\n");
    if (r < 0) {
        hal.console->printf("unable to write to %s\n", direction_path);
    }

    close(fd);
}

int LinuxGPIO_Sysfs::_open_pin_value(unsigned int pin, int flags)
{
    char path[PATH_MAX];
    int fd;

    snprintf(path, PATH_MAX, GPIO_PATH_FORMAT "/value", pin);

    fd = open(path, flags | O_CLOEXEC);
    if (fd < 0) {
        hal.console->printf("warning: unable to open %s\n", path);
        return -1;
    }

    return fd;
}

uint8_t LinuxGPIO_Sysfs::read(uint8_t vpin)
{
    assert_vpin(vpin, n_pins, LOW);

    unsigned pin = pin_table[vpin];
    int fd = _open_pin_value(pin, O_RDONLY);

    if (fd < 0)
        return LOW;

    char char_value;
    uint8_t value;
    int r = ::pread(fd, &char_value, 1, 0);
    if (r < 0) {
        hal.console->printf("warning: unable to read pin %u\n", pin);
        value = LOW;
    } else {
        value = char_value == '1' ? HIGH : LOW;
    }

    close(fd);

    return value;
}

void LinuxGPIO_Sysfs::write(uint8_t vpin, uint8_t value)
{
    assert_vpin(vpin, n_pins);

    unsigned pin = pin_table[vpin];
    int fd = _open_pin_value(pin, O_WRONLY);

    if (fd < 0)
        return;

    int r = ::pwrite(fd, value == HIGH ? "1" : "0", 1, 0);
    if (r < 0) {
        hal.console->printf("warning: unable to write pin %u\n", pin);
    }

    close(fd);
}

void LinuxGPIO_Sysfs::toggle(uint8_t vpin)
{
    write(vpin, !read(vpin));
}

int8_t LinuxGPIO_Sysfs::analogPinToDigitalPin(uint8_t vpin)
{
    return -1;
}

AP_HAL::DigitalSource* LinuxGPIO_Sysfs::channel(uint16_t vpin)
{
    assert_vpin(vpin, n_pins, nullptr);

    unsigned pin = pin_table[vpin];
    int value_fd = -1;

    if (export_pin(vpin)) {
        char value_path[PATH_MAX];
        snprintf(value_path, PATH_MAX, GPIO_PATH_FORMAT "/value", pin);

        value_fd = open(value_path, O_RDWR | O_CLOEXEC);
        if (value_fd < 0) {
            hal.console->printf("unable to open %s\n", value_path);
        }
    }

    /* Even if we couldn't open the fd, return a new DigitalSource and let
     * reads and writes fail later due to invalid. Otherwise we
     * could crash in undesired places */
    return new LinuxDigitalSource_Sysfs(pin, value_fd);
}

bool LinuxGPIO_Sysfs::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                       uint8_t mode)
{
    return false;
}

bool LinuxGPIO_Sysfs::usb_connected(void)
{
    return false;
}

bool LinuxGPIO_Sysfs::export_pin(uint8_t vpin)
{
    uint8_t vpins[] = { vpin };
    return export_pins(vpins, 1);
}

bool LinuxGPIO_Sysfs::export_pins(uint8_t vpins[], size_t len)
{
    int fd = open("/sys/class/gpio/export", O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        hal.console->printf("unable to open /sys/class/gpio/export");
        return false;
    }

    bool success = true;

    for (unsigned int i = 0; i < len; i++) {
        if (vpins[i] >= n_pins) {
            hal.console->printf("LinuxGPIO_Sysfs: can't open pin %u\n",
                                vpins[i]);
            success = false;
            break;
        }
        unsigned int pin = pin_table[vpins[i]];
        char gpio_path[PATH_MAX];

        snprintf(gpio_path, PATH_MAX, GPIO_PATH_FORMAT, pin);

        // Verify if the pin is not already exported
        if (access(gpio_path, F_OK) == 0)
            continue;

        int r = dprintf(fd, "%u\n", pin);
        if (r < 0) {
            hal.console->printf("error on exporting gpio pin number %u\n", pin);
            success = false;
            break;
        }
    }

    close(fd);

    return success;
}

#endif
