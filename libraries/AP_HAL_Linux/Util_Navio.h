
#ifndef __AP_HAL_LINUX_UTIL_NAVIO_H__
#define __AP_HAL_LINUX_UTIL_NAVIO_H__

#include "Util.h"


class Linux::LinuxUtilNavio : public Linux::LinuxUtil {
public:
    LinuxUtilNavio();
    /* return the Raspberry Pi version */
    int  getRaspberryPiVersion() const;
    
protected:
    // Called in the constructor once
    int  checkRaspberryPiVersion();
    
private:
    int _rpi_version = 0;
};

#endif // __AP_HAL_LINUX_UTIL_NAVIO_H__
