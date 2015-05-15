#include "MsgHandler_ARM.h"

extern const AP_HAL::HAL& hal;

void MsgHandler_ARM::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    uint8_t ArmState = require_field_uint8_t(msg, "ArmState");
    hal.util->set_soft_armed(ArmState);
    printf("Armed state: %u at %lu\n", 
           (unsigned)ArmState,
           (unsigned long)hal.scheduler->millis());
    dataflash.WriteBlock(msg, f.length);
}
