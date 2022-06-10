//Simple File I/O example

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_HAL_ChibiOS/hwdef/common/flash.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

static AP_BoardConfig board_config;

void setup()
{
    board_config.init();
    stm32_flash_start_dfu_boot();
    hal.scheduler->reboot(true);
}

void loop()
{}


AP_HAL_MAIN();
