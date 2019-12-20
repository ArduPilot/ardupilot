#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ROLLING_SPIDER

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "GPIO.h"

#define SIP6_VBUS_PIN 53

#define IOCTL_GPIO_MAGIC 'p'
#define IOCTL_GPIO_DIRECTION _IOW(IOCTL_GPIO_MAGIC, 0, struct gpio_direction)
#define IOCTL_GPIO_READ _IOWR(IOCTL_GPIO_MAGIC, 1, struct gpio_data)
#define IOCTL_GPIO_WRITE _IOW(IOCTL_GPIO_MAGIC, 2, struct gpio_data)

enum gpio_mode {
    GPIO_INPUT = 0,
    GPIO_OUTPUT_LOW,
    GPIO_OUTPUT_HIGH,
};

struct gpio_direction {
    int pin;
    enum gpio_mode mode;
};

struct gpio_data {
    int pin;
    int value;
};

using namespace Linux;

GPIO_SIP6::GPIO_SIP6()
{}

void GPIO_SIP6::init()
{
    gpio_fd = open("/dev/gpio", O_RDWR | O_CLOEXEC);
    if (gpio_fd == -1) {
        AP_HAL::panic("unable to open /dev/gpio");
    }
}

void GPIO_SIP6::pinMode(uint8_t pin, uint8_t output)
{
    gpio_direction direction = {
        .pin = pin,
        .mode = output == HAL_GPIO_INPUT ? GPIO_INPUT : GPIO_OUTPUT_HIGH
    };
    ioctl(gpio_fd, IOCTL_GPIO_DIRECTION, &direction);
}

uint8_t GPIO_SIP6::read(uint8_t pin)
{
    gpio_data data = {
        .pin = pin,
        .value = 0
    };
    ioctl(gpio_fd, IOCTL_GPIO_READ, &data);
    return data.value;
}

void GPIO_SIP6::write(uint8_t pin, uint8_t value)
{
    gpio_data data = {
        .pin = pin,
        .value = value
    };
    ioctl(gpio_fd, IOCTL_GPIO_WRITE, &data);
}

void GPIO_SIP6::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_SIP6::channel(uint16_t n)
{
    return new DigitalSource(n);
}

bool GPIO_SIP6::usb_connected(void)
{
    return read(SIP6_VBUS_PIN);
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ROLLING_SPIDER
