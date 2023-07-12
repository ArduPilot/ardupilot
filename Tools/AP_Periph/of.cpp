#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_OPTICALFLOW

#include <dronecan_msgs.h>

void AP_Periph_FW::of_init()
{
    of.init(-1);
}

void AP_Periph_FW::of_send_can()
{
    const float dt = of.integral_dt();
    const Vector2f bodyRateIntegral = of.bodyRate() * dt;
    const Vector2f flowRateIntegral = of.flowRate() * dt;

    com_hex_equipment_flow_Measurement pkt {
        dt,
        {bodyRateIntegral.x, bodyRateIntegral.y},
        {flowRateIntegral.x, flowRateIntegral.y},
        of.quality()
    };

    uint8_t buffer[COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_MAX_SIZE];
    const uint16_t total_size = com_hex_equipment_flow_Measurement_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_SIGNATURE,
                    COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}

void AP_Periph_FW::of_update()
{
    of.update();

    // see if the library has new data:
    const uint32_t last_update_ms = of.last_update();
    if (last_update_ms != of_state.last_update_ms) {
        // library has new data ready
        of_state.last_update_ms = last_update_ms;
        of_send_can();
    }
}
#endif
