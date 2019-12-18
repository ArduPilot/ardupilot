#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ROLLING_SPIDER

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SIP6.h"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#define IOCTL_USND_MAGIC 'u'
#define IOCTL_BATTERY _IOR(IOCTL_USND_MAGIC, 5, unsigned int)
#define IOCTL_BATTERY_INIT _IO(IOCTL_USND_MAGIC, 13)

/// Constructor
AP_BattMonitor_SIP6::AP_BattMonitor_SIP6(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
}

// initialise
void AP_BattMonitor_SIP6::init() {
    ultra_snd_fd = open("/dev/ultra_snd", O_RDWR);
    if (ultra_snd_fd == -1) {
        AP_HAL::panic("unable to open /dev/ultra_snd: %s", strerror(errno));
    }

    unsigned int battery_init = 1;
    int ret = ioctl(ultra_snd_fd, IOCTL_BATTERY_INIT, &battery_init);
    if (ret != 0) {
        AP_HAL::panic("ioctl BATTERY_INIT failed: %s\n", strerror(errno));
    }
}

// read - read the voltage
void AP_BattMonitor_SIP6::read()
{
    unsigned int battery_raw = 0;
    int ret = ioctl(ultra_snd_fd, IOCTL_BATTERY, &battery_raw);
    if (ret != 0) {
      _state.healthy = false;
      return;
    }
    _state.healthy = true;
    _state.voltage = 4.678f * (battery_raw / 1000.0f);
    _state.last_time_micros = AP_HAL::micros();
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ROLLING_SPIDER