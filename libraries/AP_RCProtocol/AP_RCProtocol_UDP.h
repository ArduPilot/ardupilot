#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_UDP_ENABLED

/*
 * Reads fixed-length packets containing either 8 or 16 2-byte values,
 * and interprets them as RC input.
 */

#include "AP_RCProtocol_Backend.h"

#include <AP_Common/missing/endian.h>
#include <AP_HAL/utility/Socket_native.h>

class AP_RCProtocol_UDP : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void update() override;

#if AP_RCPROTOCOL_FDM_ENABLED
    void set_fdm_backend(class AP_RCProtocol_FDM *_fdm_backend) {
        fdm_backend = _fdm_backend;
    }
#endif

private:

    bool init();
    bool init_done;

    uint32_t last_input_ms;

    void read_all_socket_input(void);
    SocketAPM_native rc_in{true};  // "true" means "datagram"

    // these are the values that will be fed into the autopilot.
    // Packets we receive usually only contain a subset of channel
    // values to insert into here:
    uint16_t pwm_input[16];
    uint8_t num_channels;

    void set_default_pwm_input_values();

#if AP_RCPROTOCOL_FDM_ENABLED
    AP_RCProtocol_FDM *fdm_backend;
#endif
};


#endif  // AP_RCPROTOCOL_UDP_ENABLED
