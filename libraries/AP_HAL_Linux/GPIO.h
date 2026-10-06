#pragma once

#include "AP_HAL_Linux.h"

namespace Linux {

class DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t v);
    void    mode(uint8_t output) override;
    uint8_t read() override;
    void    write(uint8_t value) override;
    void    toggle() override;
private:
    uint8_t _v;

};

}

#include <AP_HAL/AP_HAL_Boards.h>

#if HAL_LINUX_GPIO_BBB_ENABLED
#include "GPIO_BBB.h"
#elif HAL_LINUX_GPIO_NAVIGATOR_ENABLED
#include "GPIO_Navigator.h"
#elif HAL_LINUX_GPIO_RPI_ENABLED
#include "GPIO_RPI.h"
#elif HAL_LINUX_GPIO_NAVIO_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_Navio.h"
#elif HAL_LINUX_GPIO_NAVIO2_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_Navio2.h"
#elif HAL_LINUX_GPIO_EDGE_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_Edge.h"
#elif HAL_LINUX_GPIO_BEBOP_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_Bebop.h"
#elif HAL_LINUX_GPIO_DISCO_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_Disco.h"
#elif HAL_LINUX_GPIO_AERO_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_Aero.h"
#elif HAL_LINUX_GPIO_PILOTPI_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_PilotPi.h"
#elif HAL_LINUX_GPIO_PB2_ENABLED
#include "GPIO_PB2.h"
#elif HAL_LINUX_GPIO_T3_GEM_O1_ENABLED
#define HAL_LINUX_GPIO_SYSFS_ENABLED 1
#include "GPIO_T3_GEM_O1.h"
#endif
