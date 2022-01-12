/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Simulator for the FETtecOneWireESC

  TODO:
 - verify the assertion that DMA is required
 - stop ignoring REQ_TYPE while in bootloader?
 - correct visibility of members in simulation
 - half-duplex will require the use of a thread as every time we call update() we expect to send out a configuration message
 - tidy break vs return in AP_FETtec::handle_message
 - determine if we should have a "REQ_OK" as well as an "OK"
 - should rename simulated ESC "pwm" field to "value" or "fettec_value" or something
 - periodically log _unknown_esc_message, _message_invalid_in_state_count, _period_too_short, _receive_buf_used to dataflash using a low prio thread.
 - log type, version, subversion and sn to dataflash once.



Protocol:
 - SET_FAST_COM_LENGTH could set a 32-bit bitmask that will be present rather than requring consecutive motors
 - Use two magic bytes in the header instead of just one
 - Use a 16bit CRC
 - the reply request needs to repeat the data that it replies to, to make sure the reply can be clearly assigned to a request
 - need to cope with reversals
 - in the case that we don't have ESC telemetry, consider probing ESCs periodically with an "OK"-request while disarmed
*/

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "SIM_FETtecOneWireESC.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include "SIM_Aircraft.h"

#include <stdio.h>
#include <errno.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo FETtecOneWireESC::var_info[] = {

    // @Param: ENA
    // @DisplayName: FETtec OneWire ESC simulator enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FETtecOneWireESC simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, FETtecOneWireESC, _enabled, 0),

    // @Param: PWOF
    // @DisplayName: Power off FETtec ESC mask
    // @Description: Allows you to turn power off to the simulated ESCs.  Bits correspond to the ESC ID, *NOT* their servo channel.
    // @User: Advanced
    AP_GROUPINFO("POW", 2, FETtecOneWireESC, _powered_mask, 0xfff),

    AP_GROUPEND
};

FETtecOneWireESC::FETtecOneWireESC() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise serial numbers and IDs
    for (uint8_t n=0; n<ARRAY_SIZE(escs); n++) {
        ESC &esc = escs[n];
        esc.ofs = n;  // so we can index for RPM, for example
        esc.id = n+1;  // really should parameterise this
        for (uint8_t i=0; i<ARRAY_SIZE(esc.sn); i++) {
            esc.sn[i] = n+1;
        }
    }
}

void FETtecOneWireESC::update_escs()
{
    // process the power-off mask
    for (auto  &esc : escs) {
        bool should_be_on = _powered_mask & (1U<<(esc.id-1));
        switch (esc.state) {
        case ESC::State::POWERED_OFF:
            if (should_be_on) {
                esc.state = ESC::State::IN_BOOTLOADER;
                esc.pwm = 0;
                esc.fast_com = {};
                esc.telem_request = false;
            }
            break;
        case ESC::State::IN_BOOTLOADER:
        case ESC::State::RUNNING:
        case ESC::State::RUNNING_START:
            if (!should_be_on) {
                esc.state = ESC::State::POWERED_OFF;
                break;
            }
        }
    }

    for (auto  &esc : escs) {
        switch (esc.state) {
        case ESC::State::POWERED_OFF:
        case ESC::State::IN_BOOTLOADER:
        case ESC::State::RUNNING:
            continue;
        case ESC::State::RUNNING_START:
            esc.set_state(ESC::State::RUNNING);
            send_response(PackedMessage<OK> {
                esc.id,
                OK{}
            });
        }
    }

    for (auto  &esc : escs) {
        if (esc.state != ESC::State::RUNNING) {
            continue;
        }
        // FIXME: this may not be an entirely accurate model of the
        // temperature profile of these ESCs.
        esc.temperature += esc.pwm/100000;
        esc.temperature *= 0.95;
    }
}

void FETtecOneWireESC::update(const class Aircraft &aircraft)
{
    if (!_enabled.get()) {
        return;
    }

    update_escs();

    update_input();
    update_send(aircraft);
}

void FETtecOneWireESC::handle_config_message()
{
    ESC &esc = escs[u.config_message_header.target_id-1];
    simfet_debug("Config message type=%u esc=%u", (unsigned)u.config_message_header.request_type, (unsigned)u.config_message_header.target_id);
    if ((ResponseFrameHeaderID)u.config_message_header.header != ResponseFrameHeaderID::MASTER) {
        AP_HAL::panic("Unexpected header ID");
    }
    switch (esc.state) {
    case ESC::State::POWERED_OFF:
        return;
    case ESC::State::IN_BOOTLOADER:
        return bootloader_handle_config_message(esc);
    case ESC::State::RUNNING_START:
        return;
    case ESC::State::RUNNING:
        return running_handle_config_message(esc);
    }
    AP_HAL::panic("Unknown state");
}

template <typename T>
void FETtecOneWireESC::send_response(const T &r)
{
    // simfet_debug("Sending response");
    if (write_to_autopilot((char*)&r, sizeof(r)) != sizeof(r)) {
        AP_HAL::panic("short write");
    }
}

void FETtecOneWireESC::bootloader_handle_config_message(FETtecOneWireESC::ESC &esc)
{
    switch ((ConfigMessageType)u.config_message_header.request_type) {
    case ConfigMessageType::OK: {
        PackedMessage<OK> msg {
            esc.id,
            OK{},
        };
        msg.header = (uint8_t)ResponseFrameHeaderID::BOOTLOADER;
        msg.update_checksum();
        send_response(msg);
        return;
    }
    case ConfigMessageType::BL_PAGE_CORRECT:   // BL only
    case ConfigMessageType::NOT_OK:
        break;
    case ConfigMessageType::BL_START_FW:       // BL only
        esc.set_state(ESC::State::RUNNING_START);
        // the main firmware sends an OK
        return;
    case ConfigMessageType::BL_PAGES_TO_FLASH: // BL only
        break;
    case ConfigMessageType::REQ_TYPE:
        // ignore this for now
        return;
    case ConfigMessageType::REQ_SN:
    case ConfigMessageType::REQ_SW_VER:
    case ConfigMessageType::BEEP:
    case ConfigMessageType::SET_FAST_COM_LENGTH:
    case ConfigMessageType::SET_TLM_TYPE: //1 for alternative telemetry. ESC sends full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
    case ConfigMessageType::SET_LED_TMP_COLOR:
        break;
    }
    return;
    AP_HAL::panic("Unhandled config message in bootloader (%u)",
                  (unsigned)u.config_message_header.request_type);
}

void FETtecOneWireESC::running_handle_config_message(FETtecOneWireESC::ESC &esc)
{
    switch ((ConfigMessageType)u.config_message_header.request_type) {

    case ConfigMessageType::OK:
        return send_response(PackedMessage<OK> {
            esc.id,
            OK{}
        });

    case ConfigMessageType::BL_PAGE_CORRECT:   // BL only
    case ConfigMessageType::NOT_OK:
        break;
    case ConfigMessageType::BL_START_FW:       // BL only
        hal.console->printf("received unexpected BL_START_FW message\n");
        AP_HAL::panic("received unexpected BL_START_FW message");
        return;
    case ConfigMessageType::BL_PAGES_TO_FLASH: // BL only
        break;

    case ConfigMessageType::REQ_TYPE:
        return send_response(PackedMessage<ESC_TYPE> {
            esc.id,
            ESC_TYPE{esc.type}
        });

    case ConfigMessageType::REQ_SN:
        return send_response(PackedMessage<SN> {
            esc.id,
            SN{esc.sn, ARRAY_SIZE(esc.sn)}
        });

    case ConfigMessageType::REQ_SW_VER:
        return send_response(PackedMessage<SW_VER> {
            esc.id,
            SW_VER{esc.sw_version, esc.sw_subversion}
        });

    case ConfigMessageType::BEEP:
        break;

    case ConfigMessageType::SET_FAST_COM_LENGTH:
        esc.fast_com.length = u.packed_set_fast_com_length.msg.length;
        esc.fast_com.byte_count = u.packed_set_fast_com_length.msg.byte_count;
        esc.fast_com.min_esc_id = u.packed_set_fast_com_length.msg.min_esc_id;
        esc.fast_com.id_count = u.packed_set_fast_com_length.msg.id_count;
        return send_response(PackedMessage<OK> {
            esc.id,
            OK{}
        });

    case ConfigMessageType::SET_TLM_TYPE: //1 for alternative telemetry. ESC sends full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
        return handle_config_message_set_tlm_type(esc);

    case ConfigMessageType::SET_LED_TMP_COLOR:
        break;
    }
    AP_HAL::panic("Unknown config message (%u)", (unsigned)u.config_message_header.request_type);
}


void FETtecOneWireESC::handle_config_message_set_tlm_type(ESC &esc)
{
    const TLMType type = (TLMType)u.packed_set_tlm_type.msg.type;
    switch (type) {
    case TLMType::NORMAL:
    case TLMType::ALTERNATIVE:
        esc.telem_type = type;
        send_response(PackedMessage<OK> {
            esc.id,
            OK{}
        });
        return;
    }
    AP_HAL::panic("unknown telem type=%u", (unsigned)type);
}

void FETtecOneWireESC::handle_fast_esc_data()
{
    // decode first byte - see driver for details
    const uint8_t telem_request = u.buffer[0] >> 4;

    // offset into escs array for first esc involved in fast-throttle
    // command:
    const uint8_t esc0_ofs = fast_com.min_esc_id - 1;

    // ::fprintf(stderr, "telem_request=%u\n", (unsigned)telem_request);
    uint16_t esc0_pwm;
    esc0_pwm = ((u.buffer[0] >> 3) & 0x1) << 10;
    if ((u.buffer[0] & 0b111) != 0x1) {
        AP_HAL::panic("expected fast-throttle command");
    }

    // decode second byte
    esc0_pwm |= (u.buffer[1] >> 5) << 7;
    if ((u.buffer[1] & 0b00011111) != 0x1f) {
        AP_HAL::panic("Unexpected 5-bit target id");
    }

    // decode enough of third byte to complete pwm[0]
    esc0_pwm |= u.buffer[2] >> 1;

    if (escs[esc0_ofs].state == ESC::State::RUNNING) {
        ESC &esc { escs[esc0_ofs] };
        esc.pwm = esc0_pwm;

        if (telem_request == esc.id) {
            esc.telem_request = true;
        }
        simfet_debug("esc=%u out: %u", esc.id, (unsigned)esc.pwm);
    }

    // decode remainder of ESC values

    // slides a window across the input buffer, extracting 11-bit ESC
    // values.  The top 11 bits in "window" are the ESC value.
    uint8_t byte_ofs = 2;
    uint32_t window = u.buffer[byte_ofs++]<<24;
    window <<= 7;
    uint8_t bits_free = 32-1;
    for (uint8_t i=esc0_ofs+1; i<esc0_ofs+fast_com.id_count; i++) {
        while (bits_free > 7) {
            window |= u.buffer[byte_ofs++] << (bits_free-8);
            bits_free -= 8;
        }
        ESC &esc { escs[i] };
        if (esc.state == ESC::State::RUNNING) {
            if (telem_request == esc.id) {
                esc.telem_request = true;
            }
            esc.pwm = window >> 21;
            simfet_debug("esc=%u out: %u", esc.id, (unsigned)esc.pwm);
        }
        window <<= 11;
        bits_free += 11;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(escs); i++) {
        const ESC &esc { escs[i] };
        if (esc.pwm == 0) {
            continue;
        }
        // this will need to adjust for reversals.  We should also set
        // one of the simulated ESCs up to have a pair of motor wires
        // crossed i.e. spin backwards.  Maybe a mask for it
        if (esc.pwm >= 1000 && esc.pwm <= 2000) {
            continue;
        }
        AP_HAL::panic("transmitted value out of range (%u)", esc.pwm);
    }
}

void FETtecOneWireESC::consume_bytes(uint8_t count)
{
    if (count > buflen) {
        AP_HAL::panic("Consuming more bytes than in buffer?");
    }
    if (buflen == count) {
        buflen = 0;
        return;
    }
    memmove(&u.buffer[0], &u.buffer[count], buflen - count);
    buflen -= count;
}

void FETtecOneWireESC::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&u.buffer[buflen], ARRAY_SIZE(u.buffer) - buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }

    // bool config_message_checksum_fail = false;

    if (buflen > offsetof(ConfigMessageHeader, header) &&
        u.config_message_header.header == 0x01 &&
        buflen > offsetof(ConfigMessageHeader, frame_len) &&
        buflen >= u.config_message_header.frame_len) {
        const uint8_t calculated_checksum = crc8_dvb_update(0, u.buffer, u.config_message_header.frame_len-1);
        const uint8_t received_checksum = u.buffer[u.config_message_header.frame_len-1];
        if (calculated_checksum == received_checksum) {
            handle_config_message();
            // consume the message:
            consume_bytes(u.config_message_header.frame_len);
            return;
        } else {
            simfet_debug("Checksum mismatch");
            abort();
            // config_message_checksum_fail = true;
        }
        return; // 1 message/loop....
    }

    // no config message, so let's see if there's fast PWM input.
    if (fast_com.id_count == 255) {
        // see if any ESC has been configured:
        for (uint8_t i=0; i<ARRAY_SIZE(escs); i++) {
            if (escs[i].fast_com.id_count == 255) {
                continue;
            }
            fast_com.id_count = escs[i].fast_com.id_count;
            fast_com.min_esc_id = escs[i].fast_com.min_esc_id;
            break;
        }
    }
    if (fast_com.id_count == 255) {
        // no ESC is configured.  Ummm.
        buflen = 0;
        return;
    }

    const uint16_t total_bits_required = 4 + 1 + 7  + (fast_com.id_count*11);
    const uint8_t bytes_required = (total_bits_required + 7) / 8 + 1;

    if (buflen < bytes_required) {
        return;
    }

    if (buflen == bytes_required) {
        const uint8_t calculated_checksum = crc8_dvb_update(0, u.buffer, buflen-1);
        if (u.buffer[buflen-1] != calculated_checksum) {
            AP_HAL::panic("checksum failure");
        }

        handle_fast_esc_data();
        consume_bytes(bytes_required);
        return;
    }

    // debug("Read (%d) bytes from autopilot (%u)", (signed)n, config_message_checksum_fail);
    if (n >= 0) {
        abort();
    }
    buflen = 0;
}

void FETtecOneWireESC::update_sitl_input_pwm(struct sitl_input &input)
{
    // overwrite the SITL input values passed through from
    // sitl_model->update_model with those we're receiving serially
    for (auto &esc : escs) {
        if (esc.id > ARRAY_SIZE(input.servos)) {
            // silently ignore; input.servos is 12-long, we are
            // usually 16-long
            continue;
        }
        input.servos[esc.id-1] = esc.pwm;
    }
}

void FETtecOneWireESC::send_esc_telemetry(const Aircraft &aircraft)
{
    for (auto &esc : escs) {
        if (!esc.telem_request) {
            continue;
        }
        esc.telem_request = false;
        if (esc.state != ESC::State::RUNNING) {
            continue;
        }
        if (esc.telem_type != TLMType::ALTERNATIVE) {
            // no idea what "normal" looks like
            abort();
        }

        const int8_t temp_cdeg = esc.temperature * 100;
        const uint16_t voltage = aircraft.get_battery_voltage() * 100;
        const uint16_t current = (6 + esc.id * 100);

        // FIXME: the vehicle models should be supplying this RPM!
        const uint16_t Kv = 1000;
        const float p = (esc.pwm-1000)/1000.0;
        int16_t rpm = aircraft.get_battery_voltage() * Kv * p;

        const uint16_t consumption_mah = 0;
        const uint16_t errcount = 17;
        send_response(PackedMessage<ESCTelem> {
            esc.id,
            ESCTelem{temp_cdeg, voltage, current, rpm, consumption_mah, errcount}
        });
    }
}

void FETtecOneWireESC::update_send(const Aircraft &aircraft)
{
    send_esc_telemetry(aircraft);
}
