#include "MsgHandler_Event.h"

extern const AP_HAL::HAL& hal;

#define DATA_ARMED                          10
#define DATA_DISARMED                       11

void MsgHandler_Event::process_message(uint8_t *msg)
{
    uint8_t id = require_field_uint8_t(msg, "Id");
    if (id == DATA_ARMED) {
        hal.util->set_soft_armed(true);
        printf("Armed at %lu\n", 
               (unsigned long)hal.scheduler->millis());
    } else if (id == DATA_DISARMED) {
        hal.util->set_soft_armed(false);
        printf("Disarmed at %lu\n", 
               (unsigned long)hal.scheduler->millis());
    }
    dataflash.WriteBlock(msg, f.length);
}
