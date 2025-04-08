#include <AP_HAL/AP_HAL.h>

#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

using namespace QURT;

// ESC specific definitions
#define ESC_PACKET_TYPE_PWM_CMD 1
#define ESC_PACKET_TYPE_FB_RESPONSE 128
#define ESC_PACKET_TYPE_FB_POWER_STATUS 132

// IO board specific definitions
#define IO_PACKET_TYPE_PWM_HIRES_CMD 6

// Generic definitions
#define PKT_HEADER 0xAF
#define PACKET_TYPE_VERSION_EXT_REQUEST 24
#define PACKET_TYPE_VERSION_EXT_RESPONSE 131

void RCOutput::init()
{
    baudrate = ESC_BAUDRATE;
    fd = sl_client_config_uart(QURT_UART_ESC_IO, baudrate);
    if (fd == -1) {
        HAP_PRINTF("Failed to open ESC / IO UART");
    }
    HAP_PRINTF("ESC / IO UART: %d", fd);
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    // no support for changing frequency
}

uint16_t RCOutput::get_freq(uint8_t ch)
{
    // return fixed fake value
    return 490;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(period)) {
        return;
    }
    enable_mask |= 1U<<ch;
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(period)) {
        return;
    }
    enable_mask &= ~1U<<ch;
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= ARRAY_SIZE(period)) {
        return;
    }
    period[ch] = period_us;
    if (!corked) {
        need_write = true;
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(period)) {
        return 0;
    }
    return period[ch];
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (auto i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

/*
  send a packet with CRC to the ESC or IO board
 */
void RCOutput::send_packet(uint8_t type, uint8_t *data, uint16_t size)
{
    uint16_t packet_size = size + 5;
    uint8_t out[packet_size];

    out[0] = PKT_HEADER;
    out[1] = packet_size;
    out[2] = type;

    memcpy(&out[3], data, size);

    uint16_t crc = calc_crc_modbus(&out[1], packet_size - 3);

    memcpy(&out[packet_size - 2], &crc, sizeof(uint16_t));

    sl_client_uart_write(fd, (const char *)out, packet_size);
}

/*
  convert 1000 to 2000 PWM to -800 to 800 for QURT ESCs
 */
static int16_t pwm_to_esc(uint16_t pwm)
{
    const float p = constrain_float((pwm-1000)*0.001, 0, 1);
    return int16_t(800*p);
}

void RCOutput::scan_for_hardware(void)
{
    // Alternate between sending a version request and looking for the
    // version response
    static bool request_version = true;
    if (request_version) {
        HAP_PRINTF("RCOUTPUT requesting version");

        uint8_t data = 0;
        send_packet(PACKET_TYPE_VERSION_EXT_REQUEST, &data, 1);
        request_version = false;
    } else {
        HAP_PRINTF("RCOUTPUT checking response");

        check_response();

        // If we still haven't discovered what HW is out there then
        // try a different baudrate
        if (hw_type == HWType::UNKNOWN) {
            if (baudrate == ESC_BAUDRATE) {
                baudrate = IO_BAUDRATE;
            } else {
                baudrate = ESC_BAUDRATE;
            }

            sl_client_config_uart(QURT_UART_ESC_IO, baudrate);

            request_version = true;
        }
    }
}

void RCOutput::send_esc_command(void)
{
    int16_t data[5] {};

    // We don't set any LEDs
    data[4] = 0;

    for (uint8_t i=0; i<esc_channel_count; i++) {
        data[i] = pwm_to_esc(pwm_output[i]);
        // Make sure feedback request bit is cleared for all ESCs
        data[i] &= 0xFFFE;
    }

    // TODO: Not used???
    need_write = false;

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_fb_req_ms > 5) {
        last_fb_req_ms = now_ms;
        // request feedback from one ESC
        last_fb_idx = (last_fb_idx+1) % 4;
        data[last_fb_idx] |= 1;
    }

    send_packet(ESC_PACKET_TYPE_PWM_CMD, (uint8_t *)data, sizeof(data));

    check_response();
}

void RCOutput::send_io_command(void)
{
    struct PACKED {
        uint8_t  command_type;
        uint16_t vals[8];
    } hires_pwm_cmd;

    hires_pwm_cmd.command_type = 0;

    // Resolution of commands in the packet is 0.05us = 50ns
    // Convert from standard 1us resolution to IO command resolution
    for (uint32_t idx=0; idx<io_channel_count; idx++) {
        hires_pwm_cmd.vals[idx] = pwm_output[idx] * 20;
    }

    send_packet(IO_PACKET_TYPE_PWM_HIRES_CMD, (uint8_t *) &hires_pwm_cmd, sizeof(hires_pwm_cmd));
}

void RCOutput::send_pwm_output(void)
{
    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
    uint32_t safety_mask = 0;

    if (boardconfig != nullptr) {
        // mask of channels to allow with safety on
        safety_mask = boardconfig->get_safety_mask();
    }

    for (uint8_t i=0; i<max_channel_count; i++) {
        uint16_t v = period[i];
        if (safety_on && (safety_mask & (1U<<i)) == 0) {
            // when safety is on we send 0, which allows us to still
            // get feedback telemetry data from ESC, including battery voltage
            v = 0;
        }
        pwm_output[i] = v;
    }

    switch (hw_type) {
    case HWType::ESC:
        send_esc_command();
        break;
    case HWType::IO:
        send_io_command();
        break;
    case HWType::UNKNOWN:
    default:
        break;
    }
}

/*
  send current commands to ESC or IO board
 */
void RCOutput::send_receive(void)
{
    // No point proceeding if we were not able to open the UART
    if (fd == -1) {
        return;
    }

    switch (hw_type) {
    case HWType::UNKNOWN:
        scan_for_hardware();
        break;
    case HWType::ESC:
    case HWType::IO:
        send_pwm_output();
        break;
    default:
        return;
    }
}

std::string RCOutput::board_id_to_name(uint16_t board_id)
{
    switch (board_id) {
    case 31: return std::string("ModalAi 4-in-1 ESC V2 RevB (M0049)");
    case 32: return std::string("Blheli32 4-in-1 ESC Type A (Tmotor F55A PRO F051)");
    case 33: return std::string("Blheli32 4-in-1 ESC Type B (Tmotor F55A PRO G071)");
    case 34: return std::string("ModalAi 4-in-1 ESC (M0117-1)");
    case 35: return std::string("ModalAi I/O Expander (M0065)");
    case 36: return std::string("ModalAi 4-in-1 ESC (M0117-3)");
    case 37: return std::string("ModalAi 4-in-1 ESC (M0134-1)");
    case 38: return std::string("ModalAi 4-in-1 ESC (M0134-3)");
    case 39: return std::string("ModalAi 4-in-1 ESC (M0129-1)");
    case 40: return std::string("ModalAi 4-in-1 ESC (M0129-3)");
    case 41: return std::string("ModalAi 4-in-1 ESC (M0134-6)");
    case 42: return std::string("ModalAi 4-in-1 ESC (M0138-1)");
    default: return std::string("Unknown Board");
    }
}

void RCOutput::handle_version_feedback(const struct extended_version_info &pkt)
{
    uint16_t hw_ver = pkt.hw_version;

    // Check to see if we have a recognized HW id
    if (hw_ver == 35) {
        // We found an IO board
        hw_type = HWType::IO;
    } else {
        // Just assume an ESC and don't try to compare against a list of
        // known ESCs since that list will likely expand over time.
        // If the hardware responded with a valid version packet at the ESC
        // baud rate then it is very likely an ESC.
        hw_type = HWType::ESC;
    }

    // Dump all the version information
    HAP_PRINTF("RCOUTPUT: Board ID: %i", pkt.id);
    HAP_PRINTF("RCOUTPUT: Board Type : %i: %s", hw_ver, board_id_to_name(hw_ver).c_str());

    HAP_PRINTF("RCOUTPUT: Unique ID  : 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
               pkt.unique_id[11], pkt.unique_id[10], pkt.unique_id[9], pkt.unique_id[8],
               pkt.unique_id[7], pkt.unique_id[6], pkt.unique_id[5], pkt.unique_id[4],
               pkt.unique_id[3], pkt.unique_id[2], pkt.unique_id[1], pkt.unique_id[0]);

    HAP_PRINTF("RCOUTPUT: Firmware   : version %4d, hash %.12s", pkt.sw_version, pkt.firmware_git_version);
    HAP_PRINTF("RCOUTPUT: Bootloader : version %4d, hash %.12s", pkt.bootloader_version, pkt.bootloader_git_version);
}

/*
  handle a telem feedback packet
 */
void RCOutput::handle_esc_feedback(const struct esc_response_v2 &pkt)
{
    const uint8_t idx = pkt.id_state>>4;
    if (idx >= esc_channel_count) {
        return;
    }
    update_rpm(idx, pkt.rpm);

    AP_ESC_Telem_Backend::TelemetryData tdata {};
    tdata.voltage = pkt.voltage*0.001;
    tdata.current = pkt.current*0.008;
    tdata.temperature_cdeg = pkt.temperature;

    update_telem_data(idx, tdata,
                      AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                      AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                      AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
}

/*
  handle a power status packet, making it available to AnalogIn
 */
void RCOutput::handle_power_status(const struct esc_power_status &pkt)
{
    esc_voltage = pkt.voltage * 0.001;
    esc_current = pkt.current * 0.008;
}

// check for responses
void RCOutput::check_response(void)
{
    uint8_t buf[256];
    struct PACKED data_packet {
        uint8_t header;
        uint8_t length;
        uint8_t type;
        union {
            struct extended_version_info ver_info;
            struct esc_response_v2 resp_v2;
            struct esc_power_status power_status;
        } u;
    };
    auto n = sl_client_uart_read(fd, (char *)buf, sizeof(buf));
    // TODO: Maintain a count of total received bytes over time
    // HAP_PRINTF("RCOUTPUT response bytes: %d", n);
    while (n >= 3) {
        const auto *pkt = (struct data_packet *)buf;
        if (pkt->header != PKT_HEADER || pkt->length > n) {
            return;
        }
        const uint16_t crc = calc_crc_modbus(&pkt->length, pkt->length-3);
        const uint16_t crc2 = buf[pkt->length-2] | buf[pkt->length-1]<<8;
        if (crc != crc2) {
            // TODO: Maintain a count of failed CRCs over time
            // HAP_PRINTF("RCOUTPUT CRC fail on input");
            return;
        }
        switch (pkt->type) {
        case PACKET_TYPE_VERSION_EXT_RESPONSE:
            handle_version_feedback(pkt->u.ver_info);
            break;
        case ESC_PACKET_TYPE_FB_RESPONSE:
            handle_esc_feedback(pkt->u.resp_v2);
            break;
        case ESC_PACKET_TYPE_FB_POWER_STATUS:
            handle_power_status(pkt->u.power_status);
            break;
        default:
            HAP_PRINTF("Unknown pkt %u", pkt->type);
            break;
        }
        if (n == pkt->length) {
            break;
        }
        memmove(&buf[0], &buf[pkt->length], n - pkt->length);
        n -= pkt->length;
    }
}

void RCOutput::cork(void)
{
    corked = true;
}

void RCOutput::push(void)
{
    if (corked) {
        corked = false;
        need_write = true;
        send_receive();
    }
}
