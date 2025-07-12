/*
  ESC Telemetry for Hobbywing DataLink
 */
#include "AP_HobbyWing_DataLink.h"

#if AP_HOBBYWING_DATALINK_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>


#if 0
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#else
#define Debug(fmt, args ...)
#endif

const AP_Param::GroupInfo AP_HobbyWing_DataLink::var_info[] = {
    // @Param: _OFS
    // @DisplayName: HobbyWing DataLink ESC offset
    // @Description: Used to match incoming ESC telemetry with servo channel number.  If this is 8 then the ESC telemetry coming in from the device plugged into the DataLink device's "1" pins will be matched against ArduPilot's SERVO9 outputs.
    // @Range: 0 18
    // @User: Advanced
    AP_GROUPINFO("_OFS",  10, AP_HobbyWing_DataLink, offset, 0),

    // @Param: _PLS
    // @DisplayName: HobbyWing DataLink motor pole count
    // @Description: This allows calculation of true RPM from ESC's eRPM. The default is 14.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
   AP_GROUPINFO("_PLS",  11, AP_HobbyWing_DataLink, motor_poles, 14),

    AP_GROUPEND
};

void AP_HobbyWing_DataLink::init()
{
    uart.begin(115200);
    uart.set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);

    // initialise flag values in decoded data:
    for (uint8_t i=0; i<ARRAY_SIZE(decoded); i++) {
        decoded[i].rpm = nanf("");
        decoded[i].voltage = nanf("");
        decoded[i].phase_current = nanf("");
        decoded[i].current = nanf("");
        decoded[i].temperature = nanf("");
    }
}

/*
  update ESC telemetry - note that this is basically a copy of AP_HobbyWing_ESC::read_uart_v345.  Do we want a uart->read_packet(uint8_t *, uint8_t, frame_gap_us)?
 */
bool AP_HobbyWing_DataLink::read_uart(uint8_t *_packet, uint8_t packet_len, uint16_t frame_gap_us)
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

    if (uart.read(_packet, packet_len) != packet_len) {
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

void AP_HobbyWing_DataLink::update()
{
    // packet is allocated with the object to avoid large stack size

    // TODO: what's the true length of this frame gap?
    if (!read_uart((uint8_t*)&packet, sizeof(packet), 10000)) {
        return;
    }

    // check for valid frame header
    if (packet.header != 0x9b || packet.length != 160) {
        // bad header byte after a frame gap, or packet length byte wrong:
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Bad header");
        return;
    }

    if (packet.calc_checksum() != packet.crc) {
        // checksum failure
        return;
    }

    // extract packet sequence number and update count of dropped frames
    check_seq(be16toh(packet.counter));

    WITH_SEMAPHORE(decoded_sem);

    for (uint8_t i=0; i<ARRAY_SIZE(packet.esc_data); i++) {
        const auto &esc_data = packet.esc_data[i];
        auto &info = esc_info[i];
        if (esc_data.counter == 0 && !info.seen) {
            continue;
        }
        info.seen = true;

        // decoded.counter = be32toh(pkt.counter);
        // decoded.throttle_req = be16toh(pkt.throttle_req);
        // decoded.throttle = be16toh(pkt.throttle);
        decoded[i].rpm = be16toh(esc_data.rpm) / motor_poles * 10;
        decoded[i].voltage = be16toh(esc_data.voltage) * 0.1;
        decoded[i].phase_current = int16_t(be16toh(esc_data.phase_current)) * 0.01;
        decoded[i].current = int16_t(be16toh(esc_data.current)) * 0.01;
        decoded[i].temperature = MAX(AP_ESC_Telem_Backend::temperature_decode(esc_data.mos_temperature),
                                     AP_ESC_Telem_Backend::temperature_decode(esc_data.cap_temperature));
        const uint16_t status = be16toh(esc_data.status);
        if (status != 0) {
            decoded[i].error_count++;
        }
    }
    decoded_received_us = AP_HAL::micros();
}

void AP_HobbyWing_DataLink::update_telemetry()
{
    HWESC decodeds[8];
    AP_HobbyWing_DataLink::ESCInfo infos[8];
    if (!get_telem(decodeds, infos)) {
        return;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(infos); i++) {
        const auto &info = infos[i];
        if (!info.seen) {
            continue;
        }
        const auto &decoded_data = decodeds[i];

        update_motor_data_from_HWESC(offset + i, decoded_data);
    }
}

bool AP_HobbyWing_DataLink::get_telem(HWESC hwesc[8], AP_HobbyWing_DataLink::ESCInfo info[8])
{
    WITH_SEMAPHORE(decoded_sem);
    if (decoded_received_us == 0) {
        return false;
    }
    // only use once:
    decoded_received_us = 0;
    memcpy(hwesc, decoded, 8 * sizeof(HWESC));
    memcpy(info, esc_info, 8 * sizeof(AP_HobbyWing_DataLink::ESCInfo));
    return true;
}

void AP_HobbyWing_DataLink::check_seq(uint16_t this_seq)
{
    const uint32_t packet_delta = this_seq - last_seq;
    if (packet_delta > 1) {
        if (last_seq != 0) {  // reboot might see high initial seqno
            lost_count += packet_delta - 1;
        }
    }
    last_seq = this_seq;
}

#endif  // AP_HOBBYWING_DATALINK_ENABLED
