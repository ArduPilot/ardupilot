#include <AP_HAL/AP_HAL.h>

#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

using namespace QURT;

#define ESC_PACKET_TYPE_PWM_CMD 1
#define ESC_PACKET_TYPE_FB_RESPONSE 128
#define ESC_PACKET_TYPE_FB_POWER_STATUS 132

#define ESC_PKT_HEADER 0xAF

void RCOutput::init()
{
    fd = sl_client_config_uart(QURT_UART_ESC, baudrate);
    if (fd == -1) {
        HAP_PRINTF("Failed to open ESC UART");
    }
    HAP_PRINTF("ESC UART: %d", fd);
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
  send a packet with CRC to the ESC
 */
void RCOutput::send_esc_packet(uint8_t type, uint8_t *data, uint16_t size)
{
    uint16_t packet_size = size + 5;
    uint8_t out[packet_size];

    out[0] = ESC_PKT_HEADER;
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

/*
  send current commands to ESCs
 */
void RCOutput::send_receive(void)
{
    if (fd == -1) {
        return;
    }

    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
    uint32_t safety_mask = 0;

    if (boardconfig != nullptr) {
        // mask of channels to allow with safety on
        safety_mask = boardconfig->get_safety_mask();
    }
    
    int16_t data[5] {};

    for (uint8_t i=0; i<4; i++) {
        uint16_t v = period[i];
        if (safety_on && (safety_mask & (1U<<i)) == 0) {
            // when safety is on we send 0, which allows us to still
            // get feedback telemetry data, including battery voltage
            v = 0;
        }
        data[i] = pwm_to_esc(v);
    }

    need_write = false;

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_fb_req_ms > 5) {
        last_fb_req_ms = now_ms;
        // request feedback from one ESC
        last_fb_idx = (last_fb_idx+1) % 4;
        data[last_fb_idx] |= 1;
    }

    send_esc_packet(ESC_PACKET_TYPE_PWM_CMD, (uint8_t *)data, sizeof(data));

    check_response();
}

/*
  handle a telem feedback packet
 */
void RCOutput::handle_esc_feedback(const struct esc_response_v2 &pkt)
{
    const uint8_t idx = pkt.id_state>>4;
    if (idx >= ARRAY_SIZE(period)) {
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
    struct PACKED esc_packet {
        uint8_t header;
        uint8_t length;
        uint8_t type;
        union {
            struct esc_response_v2 resp_v2;
            struct esc_power_status power_status;
        } u;
    };
    auto n = sl_client_uart_read(fd, (char *)buf, sizeof(buf));
    while (n >= 3) {
        const auto *pkt = (struct esc_packet *)buf;
        if (pkt->header != ESC_PKT_HEADER || pkt->length > n) {
            return;
        }
        const uint16_t crc = calc_crc_modbus(&pkt->length, pkt->length-3);
        const uint16_t crc2 = buf[pkt->length-2] | buf[pkt->length-1]<<8;
        if (crc != crc2) {
            return;
        }
        switch (pkt->type) {
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
