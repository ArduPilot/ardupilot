#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_ERLE

#include "GPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

using namespace Linux;

LinuxGPIO::LinuxGPIO()
{}

void LinuxGPIO::init()
{
    int fd, len;
    char buf[64];

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
        perror("LinuxGPIO::init");
    }

    len = snprintf(buf, sizeof(buf), "%d", LED_AMBER);
    ::write(fd, buf, len);
    close(fd);
    
      fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
        perror("LinuxGPIO::init");
    }

    len = snprintf(buf, sizeof(buf), "%d", LED_BLUE);
    ::write(fd, buf, len);
    close(fd);

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
        perror("LinuxGPIO::init");
    }

    len = snprintf(buf, sizeof(buf), "%d", LED_SAFETY);
    ::write(fd, buf, len);
    close(fd);

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
        perror("LinuxGPIO::init");
    }

    len = snprintf(buf, sizeof(buf), "%d", SAFETY_SWITCH);
    ::write(fd, buf, len);
    close(fd);
}

void LinuxGPIO::pinMode(uint8_t pin, uint8_t output)
{
    int fd,len;
    char buf[64];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", pin);

    fd = ::open(buf, O_WRONLY);
    if (fd < 0) {
        fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);                              //try exporting GPIO pin
        if (fd < 0) {
           perror("LinuxGPIO::direction");
        }
 
        len = snprintf(buf, sizeof(buf), "%d", pin);
        ::write(fd, buf, len);
        close(fd);

        snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", pin);       //retry writing direction
        fd = ::open(buf, O_WRONLY);
        if (fd < 0) {
            perror("LinuxGPIO::direction");                                         //report faillure
        }
    }

    if (output == GPIO_OUTPUT)
        ::write(fd, "out", 4);
    else
        ::write(fd, "in", 3);

    close(fd);
}

int8_t LinuxGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t LinuxGPIO::read(uint8_t pin) {
    int fd;
    char buf[64];
    char ch;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", pin);

    fd = open(buf, O_RDONLY);
    if (fd < 0) {
        perror("LinuxGPIO::read");
    }

    ::read(fd, &ch, 1);
    
    close(fd);

    if (ch == '0') {
        return 0;
    } 
    else if (ch == '1'){
        return 1;
    }

    return -1;
}

void LinuxGPIO::write(uint8_t pin, uint8_t value)
{
    int fd;
    char buf[64];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", pin);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("LinuxGPIO::write");
    }

    if (value==LOW)
       ::write(fd, "0", 2);
    else
       ::write(fd, "1", 2);

    close(fd);
}

void LinuxGPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO::channel(uint16_t n) {
    return new LinuxDigitalSource(0);
}

/* Interrupt interface: */
bool LinuxGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool LinuxGPIO::usb_connected(void)
{
    return false;
}

LinuxDigitalSource::LinuxDigitalSource(uint8_t v) :
    _v(v)
{}

void LinuxDigitalSource::mode(uint8_t output)
{}

uint8_t LinuxDigitalSource::read() {
    return _v;
}

void LinuxDigitalSource::write(uint8_t value) {
    _v = value;
}

void LinuxDigitalSource::toggle() {
    _v = !_v;
}

#endif // CONFIG_HAL_BOARD
