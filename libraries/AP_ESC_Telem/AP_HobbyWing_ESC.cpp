#include "AP_HobbyWing_ESC.h"

#include "AP_ESC_Telem_config.h"

#if AP_HOBBYWING_ESC_ENABLED

#include <AP_HAL/UARTDriver.h>
#include <GCS_MAVLink/GCS.h>

#if 0
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#else
#define Debug(fmt, args ...)
#endif

void AP_HobbyWing_ESC::init()
{
    uart.begin(19200);
    uart.set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

/*
  update ESC telemetry
 */
bool AP_HobbyWing_ESC::read_uart_v345(uint8_t *packet, uint8_t packet_len, uint16_t frame_gap_us)
{
    bool ret = false;

    const uint32_t n = uart.available();

    if (n < packet_len) {
        // frame presumably still on its way in
        return false;
    }

    const uint64_t this_frame_us = uart.receive_time_constraint_us(packet_len);
    const uint64_t dt_us = this_frame_us - last_frame_us;
    if (dt_us < frame_gap_us) {
        // insufficient frame gap; discard this input
        Debug(MAV_SEVERITY_INFO, "Bad frame gap");
        goto OUT_DISCARD;
    }
    last_frame_us = this_frame_us;

    if (n > packet_len) {
        // too many bytes available from the uart.  That's corruption,
        // assuming scheduling is good.
        Debug(MAV_SEVERITY_INFO, "Bad size");
        goto OUT_DISCARD;
    }

    if (uart.read(packet, packet_len) != packet_len) {
        // strange - this shouldn't fail
        Debug(MAV_SEVERITY_INFO, "Bad read");
        goto OUT_DISCARD;
    }

    // insert more sanity checks here if they can be found - e.g. PWM
    // ranges, massive seqno badness etc etc

    // successfully read a packet; discard any further data in
    // the uart as we only want fresh data
    ret = true;

OUT_DISCARD:
    uart.discard_input();
    return ret;
}

bool AP_HobbyWing_ESC::get_telem(HWESC &hwesc)
{
    WITH_SEMAPHORE(decoded_sem);
    if (decoded_received_us == 0) {
        return false;
    }
    // only use once:
    decoded_received_us = 0;
    hwesc = decoded;
    return true;
}

#if HAL_WITH_ESC_TELEM
void AP_HobbyWing_ESC::update_telemetry()
{
    // FIXME: packet freshness?
    HWESC decoded_data {
    rpm: nanf(""),
    voltage: nanf(""),
    phase_current: nanf(""),
    current: nanf(""),
    temperature: nanf("")
    };
    if (!get_telem(decoded_data)) {
        return;
    }

    // ESC telem library wants an offset, not a channel number, thus -1:
    update_motor_data_from_HWESC(servo_channel-1, decoded_data);
}
#endif  // HAL_WITH_ESC_TELEM

void AP_HobbyWing_ESC::check_seq(uint32_t this_seq)
{
    const uint32_t packet_delta = this_seq - last_seq;
    if (packet_delta > 1) {
        if (last_seq != 0) {  // reboot might see high initial seqno
            lost_count += packet_delta - 1;
        }
    }
    last_seq = this_seq;
}

#endif
