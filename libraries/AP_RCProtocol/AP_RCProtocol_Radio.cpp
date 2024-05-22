#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_RADIO_ENABLED

#include "AP_RCProtocol_Radio.h"
#include <AP_Radio/AP_Radio.h>

void AP_RCProtocol_Radio::update()
{
    auto *radio = AP_Radio::get_singleton();
    if (radio == nullptr) {
        return;
    }
    if (!init_done) {
        radio->init();
        init_done = true;
    }

    // allow the radio to handle mavlink on the main thread:
    radio->update();

    const uint32_t last_recv_us = radio->last_recv_us();
    if (last_recv_us == last_input_us) {
        // no new data
        return;
    }
    last_input_us = last_recv_us;

    const auto num_channels = radio->num_channels();
    uint16_t rcin_values[MAX_RCIN_CHANNELS];
    for (uint8_t i=0; i<num_channels; i++) {
        rcin_values[i] = radio->read(i);
    }

    add_input(
        num_channels,
        rcin_values,
        false,  // failsafe
        0, // check me
        0  // link quality
        );
}

void AP_RCProtocol_Radio::start_bind()
{
    auto *radio = AP_Radio::get_singleton();
    if (radio == nullptr) {
        return;
    }
    radio->start_recv_bind();
}

#endif // AP_RCPROTOCOL_RADIO_ENABLED
