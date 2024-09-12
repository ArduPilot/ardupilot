#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_UDP_ENABLED

#include "AP_RCProtocol_UDP.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <SITL/SITL.h>

#if AP_RCPROTOCOL_FDM_ENABLED
#include "AP_RCProtocol_FDM.h"
#endif

extern const AP_HAL::HAL& hal;

void AP_RCProtocol_UDP::set_default_pwm_input_values()
{
    pwm_input[0] = 1500;
    pwm_input[1] = 1500;
    pwm_input[2] = 1000;
    pwm_input[3] = 1500;
    pwm_input[4] = 1800;
    pwm_input[5] = 1000;
    pwm_input[6] = 1000;
    pwm_input[7] = 1800;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    // set correct default throttle for rover (allowing for reverse)
    pwm_input[2] = 1500;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub) || APM_BUILD_TYPE(APM_BUILD_Blimp)
    for(uint8_t i = 0; i < 8; i++) {
        pwm_input[i] = 1500;
    }
#endif

    num_channels = 8;
}

bool AP_RCProtocol_UDP::init()
{
    const auto sitl = AP::sitl();
    if (sitl == nullptr) {
        return false;
    }
    if (!rc_in.reuseaddress()) {
        return false;
    }
    if (!rc_in.bind("0.0.0.0", sitl->rcin_port)) {
        return false;
    }
    if (!rc_in.set_blocking(false)) {
        return false;
    }
    if (!rc_in.set_cloexec()) {
        return false;
    }

    set_default_pwm_input_values();

    return true;
}

void AP_RCProtocol_UDP::update()
{
#if AP_RCPROTOCOL_FDM_ENABLED
    // yield to the FDM backend if it is getting data
    if (fdm_backend->active()) {
        return;
    }
#endif

    if (!init_done) {
        if (!init()) {
            return;
        }
        init_done = true;
    }

    read_all_socket_input();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const auto sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    if (sitl->rc_fail == SITL::SIM::SITL_RCFail_NoPulses) {
        return;
    }
#endif

    // simulate RC input at 50Hz
    if (AP_HAL::millis() - last_input_ms < 20) {
        return;
    }
    last_input_ms = AP_HAL::millis();

    add_input(
        num_channels,
        pwm_input,
        false,  // failsafe
        0, // check me
        0  // link quality
        );
}

/*
  check for a SITL RC input packet
 */
void AP_RCProtocol_UDP::read_all_socket_input(void)
{
    struct pwm_packet {
        uint16_t pwm[16];
    } pwm_pkt;
    uint8_t pwm_pkt_num_channels = 0;

    ssize_t receive_size = 1;  // lies!
    uint16_t count = 0;
    while (receive_size > 0) {
        receive_size = rc_in.recv(&pwm_pkt, sizeof(pwm_pkt), 0);

        switch (receive_size) {
        case -1:
            break;
        case 8*2:
        case 16*2:
            pwm_pkt_num_channels = receive_size/2;
            break;
        default:
            fprintf(stderr, "Malformed SITL RC input (%ld)", (long)receive_size);
            return;
        }
        count++;
    }

    if (count > 100) {
        ::fprintf(stderr, "Read %u rc inputs\n", count);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const auto sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    // convert last packet received into pwm values
    switch (sitl->rc_fail) {
    case SITL::SIM::SITL_RCFail_Throttle950:
        // discard anything we just read from the "receiver" and set
        // values to bind values:
        for (uint8_t i=0; i<ARRAY_SIZE(pwm_input); i++) {
            pwm_input[i] = 1500;  // centre all inputs
        }
        pwm_input[2] = 950;  // reset throttle (assumed to be on channel 3...)
        return;
    case SITL::SIM::SITL_RCFail_NoPulses:
        // see also code in ::update
        return;
    case SITL::SIM::SITL_RCFail_None:
        break;
    }
#endif

    if (pwm_pkt_num_channels == 0) {
        return;
    }
    for (uint8_t i=0; i<pwm_pkt_num_channels; i++) {
        // setup the pwm input for the RC channel inputs
        const uint16_t pwm = pwm_pkt.pwm[i];
        if (pwm == 0) {
            // 0 means "ignore this value"
            continue;
        }
        pwm_input[i] = pwm;
    }
    num_channels = pwm_pkt_num_channels;  // or ARRAY_SIZE(pwm_input)?
}

#endif // AP_RCPROTOCOL_UDP_ENABLED
