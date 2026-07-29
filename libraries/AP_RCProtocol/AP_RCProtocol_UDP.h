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

    explicit AP_RCProtocol_UDP(AP_RCProtocol &_frontend);

    void update() override;

    static AP_RCProtocol_UDP *get_singleton() { return _singleton; }

    // Copy the most recently received UDP channel values.
    // Returns false if no packet has been received yet.
    bool get_channels(uint16_t *channels, uint8_t &count) const;

    // IBus2Master calls this to take ownership of add_input.
    // When active, this backend stores received channels but does not call
    // add_input(); the IBus2 encode/decode chain calls add_input instead.
    void set_ibus2_active(bool active) { _ibus2_active = active; }

#if AP_RCPROTOCOL_FDM_ENABLED
    void set_fdm_backend(class AP_RCProtocol_FDM *_fdm_backend) {
        fdm_backend = _fdm_backend;
    }
#endif

private:

    static AP_RCProtocol_UDP *_singleton;
    bool _ibus2_active;

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
