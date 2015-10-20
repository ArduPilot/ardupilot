#ifndef __AP_HAL_LINUX_GPIO_H__
#define __AP_HAL_LINUX_GPIO_H__

#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#include "GPIO_BBB.h"
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#include "GPIO_RPI.h"
#endif

class Linux::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    uint8_t _v;

};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#endif // __AP_HAL_LINUX_GPIO_H__
