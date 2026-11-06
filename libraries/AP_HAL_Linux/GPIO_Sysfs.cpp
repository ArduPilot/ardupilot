#include "GPIO_Sysfs.h"

#include <AP_HAL/AP_HAL.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/inotify.h>
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

extern const AP_HAL::HAL& hal;

#define UINT32_MAX_STR "4294967295"

/* Trick to use minimum stack space for each of the params */
union gpio_params {
    char export_gpio[sizeof("export")];
    char direction[sizeof("gpio" UINT32_MAX_STR "/direction")];
    char value[sizeof("gpio" UINT32_MAX_STR "/value")];
    char edge[sizeof("edge" UINT32_MAX_STR "/edge")];
};

#define GPIO_BASE_PATH "/sys/class/gpio/"
#define GPIO_PATH_MAX (sizeof(GPIO_BASE_PATH) + sizeof(gpio_params) - 1)

DigitalSource_Sysfs::DigitalSource_Sysfs(unsigned pin, int value_fd)
    : _value_fd(value_fd)
    , _pin(pin)
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
#ifdef HAL_GPIO_SCRIPT
    if (!_script.thread_created) {
        _script.thread_created = true;
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&GPIO_Sysfs::_gpio_script_thread, void),
                                          "GPIO_Script", 4096, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
            AP_HAL::panic("Unable to create GPIO_Script thread");
        }
    }
#endif

    _interrupt_thread_created = false;
    _interrupt_inotify_fd = 0;
}

void GPIO_Sysfs::pinMode(uint8_t vpin, uint8_t output)
{
#ifdef HAL_GPIO_SCRIPT
    if (vpin >= n_pins && output) {
        return;
    }
#endif
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
#ifdef HAL_GPIO_SCRIPT
    if (vpin >= n_pins) {
        _gpio_script_write(vpin, value);
        return;
    }
#endif
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

void GPIO_Sysfs::_pinEdge(unsigned int pin, GPIO::INTERRUPT_TRIGGER_TYPE mode)
{
    const char *edge;
    char path[GPIO_PATH_MAX];

    switch(mode) {
        case INTERRUPT_FALLING:
            edge = "falling";
            break;

        case INTERRUPT_RISING:
            edge = "rising";
            break;

        case INTERRUPT_BOTH:
            edge = "both";
            break;

        default:
            hal.console->printf("GPIO_Sysfs: Failed to set pin %u interrupt edge to unknown mode.\n", pin);
            return;
    }

    int r = snprintf(path, GPIO_PATH_MAX, GPIO_BASE_PATH "gpio%u/edge", pin);
    if (r < 0 || r >= (int)GPIO_PATH_MAX
        || Util::from(hal.util)->write_file(path, "%s", edge) < 0) {
        hal.console->printf("GPIO_Sysfs: Unable to set pin %u interrupt edge to %s.\n", pin, edge);
    }
}

/***
 Interrupt interface:
                               ret , pin    , state,timestamp
 where:
   ret indicates the functor must return void
   pin is the pin which has triggered the interrupt
   state is the new state of the pin
   timestamp is the time in microseconds the interrupt occurred
**/
bool GPIO_Sysfs::attach_interrupt(uint8_t vpin, irq_handler_fn_t fn, GPIO::INTERRUPT_TRIGGER_TYPE mode)
{
    /* already attached interrupt for this pin? */
    if(_interrupt_watchers.count(vpin)) {
        hal.console->printf("GPIO_Sysfs: pin %u already has interrupt handler attached.\n", vpin);
        return false;
    }

    /* translate sysfs pin from ardupilot vpin */
    const unsigned pin = pin_table[vpin];

    /* attach or detach interrupt handler? */
    if(mode != INTERRUPT_NONE) {
        hal.console->printf("GPIO_Sysfs: attaching interrupt handler to pin %u\n", vpin);
        /* set interrupt mode for pin (rising/falling/both edges) */
        _pinEdge(pin, mode);

        /* create path to pin value */
        char path[GPIO_PATH_MAX];
        int r = snprintf(path, GPIO_PATH_MAX, GPIO_BASE_PATH "gpio%u", pin);
        if (r < 0 || r >= (int)GPIO_PATH_MAX) {
            hal.console->printf("GPIO_Sysfs: unable to build path\n");
            return false;
        }

        /* initialize inotify for our interrupt thread */
        if (!_interrupt_inotify_fd) {
            if ((_interrupt_inotify_fd = inotify_init()) < 0) {
                AP_HAL::panic( "GPIO_Sysfs: couldn't initialize inotify");
            }
        }
        /* add watch to /sys/class/gpio/gpioNNN directory */
        int wd;
        if((wd = inotify_add_watch(_interrupt_inotify_fd, path, IN_MODIFY)) < 0) {
            AP_HAL::panic( "GPIO_Sysfs: failed to add inotify watch for \"%s\": %s", path, strerror(errno));
        }

        /* remember interrupt handler & vpin for this inotify watcher */
        _interrupt_handlers[wd] = { .fn = fn, .vpin = vpin };
        /* remember inotify watcher used for this vpin */
        _interrupt_watchers[vpin] = wd;
    }

    /* detach interrupt handler */
    else {
        /* remove interrupt handler */
        hal.console->printf("GPIO_Sysfs: detaching interrupt handler from pin %u\n", vpin);
        int wd = _interrupt_watchers[vpin];
        inotify_rm_watch( _interrupt_inotify_fd, wd);
        /* forget interrupt handler */
        _interrupt_handlers.erase(wd);
        /* forget interrupt watcher */
        _interrupt_watchers.erase(vpin);
        return true;
    }

    /* interrupt thread already launched? */
    if(!_interrupt_thread_created) {
        _interrupt_thread_created = true;
        if (!hal.scheduler->thread_create(
                FUNCTOR_BIND_MEMBER(&GPIO_Sysfs::_gpio_interrupt_thread, void),
                "ap-gpio-interrupts",
                4096,
                AP_HAL::Scheduler::PRIORITY_BOOST,
                -1
        )) {
            AP_HAL::panic("GPIO_Sysfs: Unable to create GPIO_Interrupts thread");
        }
    }

    return true;
}

/* thread to handle inotify events and call interrupt handlers attached to pins accordingly */
void GPIO_Sysfs::_gpio_interrupt_thread(void)
{
    #define MAX_EVENTS 1024 /* max. number of events to process at one go */
    #define EVENT_SIZE  ( sizeof (struct inotify_event) ) /* size of one event */
    #define BUF_LEN     ( MAX_EVENTS * ( EVENT_SIZE + GPIO_PATH_MAX )) /* buffer to store the data of events */

    hal.console->printf("GPIO_Sysfs: interrupt thread started\n");

    /* keep reading events from inotify fd */
    char buffer[BUF_LEN];
    while (true) {
        int length = ::read( _interrupt_inotify_fd, buffer, BUF_LEN );

        if ( length < 0 ) {
            hal.console->printf("GPIO_Sysfs: read error: %s\n", strerror(errno) );
        }

        /* process all events that were read from buffer */
        struct inotify_event event;
        for (int i = 0; i < length; i += EVENT_SIZE + event.len) {
            /* copy event from buffer */
            memcpy(&event, &buffer[i], sizeof(event));
            /* event has no path? */
            if ( event.len == 0) {
                /* skip event */
                continue;
            }

            /* unknown event? */
            if (!(event.mask & IN_MODIFY)) {
                /* skip event */
                continue;
            }

            /* get current time */
            uint32_t timestamp = AP_HAL::millis();
            /* lookup interrupt handler from inotify watcher */
            _interrupt_and_vpin id = _interrupt_handlers[event.wd];
            /* our registered irq handler */
            irq_handler_fn_t fn = id.fn;
            /* the vpin that triggered this interrupt */
            uint8_t vpin = id.vpin;
            /* call interrupt handler */
            fn(vpin, read(vpin), timestamp);
        }

    }
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
    return NEW_NOTHROW DigitalSource_Sysfs(pin, value_fd);
}

bool GPIO_Sysfs::usb_connected(void)
{
    return false;
}

bool GPIO_Sysfs::_export_pin(uint8_t vpin)
{
#ifdef HAL_GPIO_SCRIPT
    if (vpin >= n_pins) {
        return false;
    }
#endif
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

#ifdef HAL_GPIO_SCRIPT
/*
  support using an external script triggered by a write to a GPIO
  value. This is called whenever a GPIO request is made that is for an
  unknown pin value. The script is called by a separate thread, and
  only one script can be run at a time. This prevents the scripts
  using too many resources
 */
void GPIO_Sysfs::_gpio_script_write(uint8_t vpin, uint8_t value)
{
    pin_value_t pv;
    pv.pin = vpin;
    pv.value = value;
    _script.pending.push(pv);

}

/*
  thread for running GPIO scripts
 */
void GPIO_Sysfs::_gpio_script_thread(void)
{
    while (true) {
        // don't run more than 20/sec
        hal.scheduler->delay(50);
        pin_value_t pv;
        if (_script.pending.pop(pv)) {
            char cmd[100];
            snprintf(cmd, sizeof(cmd)-1, "/bin/sh %s %u %u", HAL_GPIO_SCRIPT, pv.pin, pv.value);
            hal.console->printf("Running: %s\n", cmd);
            const int system_ret = system(cmd);
            if (system_ret != 0) {
                hal.console->printf("Unexpected return value (%d)\n", system_ret);
            }
        }
    }
}
#endif // HAL_GPIO_SCRIPT
