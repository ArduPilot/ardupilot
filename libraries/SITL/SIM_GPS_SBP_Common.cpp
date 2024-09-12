#include "SIM_config.h"

#if HAL_SIM_GPS_ENABLED

#include "SIM_GPS_SBP_Common.h"

#include <AP_Math/crc.h>

using namespace SITL;

void GPS_SBP_Common::sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload)
{
    if (len != 0 && payload == 0) {
        return; //SBP_NULL_ERROR;
    }

    uint8_t preamble = 0x55;
    write_to_autopilot((char*)&preamble, 1);
    write_to_autopilot((char*)&msg_type, 2);
    write_to_autopilot((char*)&sender_id, 2);
    write_to_autopilot((char*)&len, 1);
    if (len > 0) {
        write_to_autopilot((char*)payload, len);
    }

    uint16_t crc;
    crc = crc16_ccitt((uint8_t*)&(msg_type), 2, 0);
    crc = crc16_ccitt((uint8_t*)&(sender_id), 2, crc);
    crc = crc16_ccitt(&(len), 1, crc);
    crc = crc16_ccitt(payload, len, crc);
    write_to_autopilot((char*)&crc, 2);
}

#endif
