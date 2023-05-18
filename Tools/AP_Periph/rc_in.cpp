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

#include <AP_RCProtocol/AP_RCProtocol_config.h>

#ifdef HAL_PERIPH_ENABLE_RCIN

#include <AP_RCProtocol/AP_RCProtocol.h>
#include "AP_Periph.h"
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

void AP_Periph_FW::rcin_init()
{
    if (g.rcin1_port == 0) {
        return;
    }

    // init uart for serial RC
    auto *uart = hal.serial(g.rcin1_port);
    if (uart == nullptr) {
        return;
    }

    serial_manager.set_protocol_and_baud(
        g.rcin1_port,
        AP_SerialManager::SerialProtocol_RCIN,
        115200  // baud doesn't matter; RC Protocol autobauds
        );

    auto &rc = AP::RC();
    rc.init();
    rc.set_rc_protocols(g.rcin_protocols);
    rc.add_uart(uart);

    rcin_initialised = true;
}

void AP_Periph_FW::rcin_update()
{
    if (!rcin_initialised) {
        return;
    }

    auto &rc = AP::RC();
    if (!rc.new_input()) {
        return;
    }

    // log discovered protocols:
    auto new_rc_protocol = rc.protocol_name();
    if (new_rc_protocol != rcin_rc_protocol) {
        can_printf("Decoding (%s)", new_rc_protocol);
        rcin_rc_protocol = new_rc_protocol;
    }

    // decimate the input to a parameterized rate
    const uint8_t rate_hz = g.rcin_rate_hz;
    if (rate_hz == 0) {
        return;
    }

    const auto now_ms = AP_HAL::millis();
    const auto interval_ms = 1000U / rate_hz;
    if (now_ms - rcin_last_sent_RCInput_ms < interval_ms) {
        return;
    }
    rcin_last_sent_RCInput_ms = now_ms;

    // extract data and send CAN packet:
    const uint8_t num_channels = rc.num_channels();
    uint16_t channels[MAX_RCIN_CHANNELS];
    rc.read(channels, num_channels);
    const int16_t rssi = rc.get_RSSI();

    can_send_RCInput((uint8_t)rssi, channels, num_channels, rc.failsafe_active(), rssi > 0 && rssi <256);
}

/*
  send an RCInput CAN message
 */
void AP_Periph_FW::can_send_RCInput(uint8_t quality, uint16_t *values, uint8_t nvalues, bool in_failsafe, bool quality_valid)
{
    uint16_t status = 0;
    if (quality_valid) {
        status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
    }
    if (in_failsafe) {
        status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;
    }

    // assemble packet
    dronecan_sensors_rc_RCInput pkt {};
    pkt.quality = quality;
    pkt.status = status;
    pkt.rcin.len = nvalues;
    for (uint8_t i=0; i<nvalues; i++) {
        pkt.rcin.data[i] = values[i];
    }

    // encode and send message:
    uint8_t buffer[DRONECAN_SENSORS_RC_RCINPUT_MAX_SIZE] {};

    uint16_t total_size = dronecan_sensors_rc_RCInput_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE,
                     DRONECAN_SENSORS_RC_RCINPUT_ID,
                     CANARD_TRANSFER_PRIORITY_HIGH,
                     buffer,
                     total_size);
}

#endif  // HAL_PERIPH_ENABLE_RCIN
