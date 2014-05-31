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
#include <sys/mman.h>
#include <sys/stat.h>

using namespace Linux;

struct GPIO{
    volatile unsigned *base;
    volatile unsigned *oe;
    volatile unsigned *in;
    volatile unsigned *out;
}gpio0,gpio1,gpio2,gpio3;



LinuxGPIO::LinuxGPIO()
{}

void LinuxGPIO::init()
{
    int mem_fd;
    // Enable all GPIO banks
    // Without this, access to deactivated banks (i.e. those with no clock source set up) will (logically) fail with SIGBUS
    // Idea taken from https://groups.google.com/forum/#!msg/beagleboard/OYFp4EXawiI/Mq6s3sg14HoJ
    system("echo 5 > /sys/class/gpio/export");
    system("echo 65 > /sys/class/gpio/export");
    system("echo 105 > /sys/class/gpio/export");

    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
            printf("can't open /dev/mem \n");
            exit (-1);
    }

    /* mmap GPIO */
    gpio0.base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO0_BASE);
    if ((char *)gpio0.base == MAP_FAILED) {
            printf("mmap error %d\n", (int)gpio0.base);
            exit (-1);
    }
    gpio1.base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO1_BASE);
    if ((char *)gpio1.base == MAP_FAILED) {
            printf("mmap error %d\n", (int)gpio1.base);
            exit (-1);
    }
    gpio2.base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO2_BASE);
    if ((char *)gpio2.base == MAP_FAILED) {
            printf("mmap error %d\n", (int)gpio2.base);
            exit (-1);
    }
    gpio3.base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO3_BASE);
    if ((char *)gpio3.base == MAP_FAILED) {
            printf("mmap error %d\n", (int)gpio3.base);
            exit (-1);
    }


    gpio0.oe = gpio0.base + GPIO_OE;
    gpio0.in = gpio0.base + GPIO_IN;
    gpio0.out = gpio0.base + GPIO_OUT;

    gpio1.oe = gpio1.base + GPIO_OE;
    gpio1.in = gpio1.base + GPIO_IN;
    gpio1.out = gpio1.base + GPIO_OUT;

    gpio2.oe = gpio2.base + GPIO_OE;
    gpio2.in = gpio2.base + GPIO_IN;
    gpio2.out = gpio2.base + GPIO_OUT;

    gpio3.oe = gpio3.base + GPIO_OE;
    gpio3.in = gpio3.base + GPIO_IN;
    gpio3.out = gpio3.base + GPIO_OUT;
    close(mem_fd);
}

void LinuxGPIO::pinMode(uint8_t pin, uint8_t output)
{
    if(output == GPIO_INPUT){
        if(pin/32 < 1){
            *gpio0.oe = *gpio0.oe | (1<<pin);
        }
        else if(pin/32 < 2){
            *gpio1.oe = *gpio1.oe | (1<<(pin-32));
        }
        else if(pin/32 < 3){
            *gpio2.oe = *gpio2.oe | (1<<(pin-64));
        }
        else if(pin/32 < 4){
            *gpio3.oe = *gpio3.oe | (1<<(pin-96));
        }
    }
    else{
        if(pin/32 < 1){
            *gpio0.oe = *gpio0.oe & (~(1<<pin));
        }
        else if(pin/32 < 2){
            *gpio1.oe = *gpio1.oe & (~(1<<(pin-32)));
        }
        else if(pin/32 < 3){
            *gpio2.oe = *gpio2.oe & (~(1<<(pin-64)));
        }
        else if(pin/32 < 4){
            *gpio3.oe = *gpio3.oe & (~(1<<(pin-96)));
        }
    }
}

int8_t LinuxGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t LinuxGPIO::read(uint8_t pin) {
    
    uint8_t value;

    if(pin/32 < 1){
        value = (bool)(*gpio0.in & (1<<pin));
    }
    else if(pin/32 < 2){
        value = (bool)(*gpio1.in & (1<<(pin-32)));
    }
    else if(pin/32 < 3){
        value = (bool)(*gpio2.in & (1<<(pin-64)));
    }
    else if(pin/32 < 4){
        value = (bool)(*gpio3.in & (1<<(pin-96)));
    }

    return value;
}

void LinuxGPIO::write(uint8_t pin, uint8_t value)
{
    if(value == HIGH){
        if(pin/32 < 1){
            *gpio0.out = *gpio0.out | (1<<pin);
        }
        else if(pin/32 < 2){
            *gpio1.out = *gpio1.out | (1<<(pin-32));
        }
        else if(pin/32 < 3){
            *gpio2.out = *gpio2.out | (1<<(pin-64));
        }
        else if(pin/32 < 4){
            *gpio3.out = *gpio3.out | (1<<(pin-96));
        }
    }
    else{
        if(pin/32 < 1){
            *gpio0.out = *gpio0.out & (~(1<<pin));
        }
        else if(pin/32 < 2){
            *gpio1.out = *gpio1.out & (~(1<<(pin-32)));
        }
        else if(pin/32 < 3){
            *gpio2.out = *gpio2.out & (~(1<<(pin-64)));
        }
        else if(pin/32 < 4){
            *gpio3.out = *gpio3.out & (~(1<<(pin-96)));
        }
    }
}

void LinuxGPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO::channel(uint16_t n) {
    return new LinuxDigitalSource(n);
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
{
   
}

void LinuxDigitalSource::mode(uint8_t output)
{
    uint8_t pin = _v;

    if(output == GPIO_INPUT){
        if(pin/32 < 1){
            *gpio0.oe = *gpio0.oe | (1<<pin);
        }
        else if(pin/32 < 2){
            *gpio1.oe = *gpio1.oe | (1<<(pin-32));
        }
        else if(pin/32 < 3){
            *gpio2.oe = *gpio2.oe | (1<<(pin-64));
        }
        else if(pin/32 < 4){
            *gpio3.oe = *gpio3.oe | (1<<(pin-96));
        }
    }
    else{
        if(pin/32 < 1){
            *gpio0.oe = *gpio0.oe & (~(1<<pin));
        }
        else if(pin/32 < 2){
            *gpio1.oe = *gpio1.oe & (~(1<<(pin-32)));
        }
        else if(pin/32 < 3){
            *gpio2.oe = *gpio2.oe & (~(1<<(pin-64)));
        }
        else if(pin/32 < 4){
            *gpio3.oe = *gpio3.oe & (~(1<<(pin-96)));
        }
    }

}

uint8_t LinuxDigitalSource::read() {
    uint8_t value, pin = _v;

    if(pin/32 < 1){
        value = (bool)(*gpio0.in & (1<<pin));
    }
    else if(pin/32 < 2){
        value = (bool)(*gpio1.in & (1<<(pin-32)));
    }
    else if(pin/32 < 3){
        value = (bool)(*gpio2.in & (1<<(pin-64)));
    }
    else if(pin/32 < 4){
        value = (bool)(*gpio3.in & (1<<(pin-96)));
    }

    return value;
}

void LinuxDigitalSource::write(uint8_t value) {
   
   uint8_t pin = _v;

   if(value == HIGH){
        if(pin/32 < 1){
            *gpio0.out = *gpio0.out | (1<<pin);
        }
        else if(pin/32 < 2){
            *gpio1.out = *gpio1.out | (1<<(pin-32));
        }
        else if(pin/32 < 3){
            *gpio2.out = *gpio2.out | (1<<(pin-64));
        }
        else if(pin/32 < 4){
            *gpio3.out = *gpio3.out | (1<<(pin-96));
        }
    }
    else{
        if(pin/32 < 1){
            *gpio0.out = *gpio0.out & (~(1<<pin));
        }
        else if(pin/32 < 2){
            *gpio1.out = *gpio1.out & (~(1<<(pin-32)));
        }
        else if(pin/32 < 3){
            *gpio2.out = *gpio2.out & (~(1<<(pin-64)));
        }
        else if(pin/32 < 4){
            *gpio3.out = *gpio3.out & (~(1<<(pin-96)));
        }
    }
}

void LinuxDigitalSource::toggle() {
    write(!read());
}

#endif // CONFIG_HAL_BOARD
