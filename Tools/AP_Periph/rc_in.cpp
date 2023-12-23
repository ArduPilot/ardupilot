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

#ifndef AP_PERIPH_RC1_PORT_DEFAULT
#define AP_PERIPH_RC1_PORT_DEFAULT -1
#endif

#ifndef AP_PERIPH_RC1_PORT_OPTIONS_DEFAULT
#define AP_PERIPH_RC1_PORT_OPTIONS_DEFAULT 0
#endif

#include <AP_RCProtocol/AP_RCProtocol.h>
#include "AP_Periph.h"
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Parameters_RCIN::var_info[] {
    // RC_PROTOCOLS copied from RC_Channel/RC_Channels_Varinfo.h
    // @Param: _PROTOCOLS
    // @DisplayName: RC protocols enabled
    // @Description: Bitmask of enabled RC protocols. Allows narrowing the protocol detection to only specific types of RC receivers which can avoid issues with incorrect detection. Set to 1 to enable all protocols.
    // @User: Advanced
    // @Bitmask: 0:All,1:PPM,2:IBUS,3:SBUS,4:SBUS_NI,5:DSM,6:SUMD,7:SRXL,8:SRXL2,9:CRSF,10:ST24,11:FPORT,12:FPORT2,13:FastSBUS
    AP_GROUPINFO("_PROTOCOLS", 1, Parameters_RCIN, rcin_protocols, 1),

    // RC_PROTOCOLS copied from RC_Channel/RC_Channels_Varinfo.h
    // @Param: _MSGRATE
    // @DisplayName: DroneCAN RC Message rate
    // @Description: Rate at which RC input is sent via DroneCAN
    // @User: Advanced
    // @Increment: 1
    // @Range: 0 255
    // @Units: Hz
    AP_GROUPINFO("_MSGRATE", 2, Parameters_RCIN, rcin_rate_hz, 50),

    // @Param: 1_PORT
    // @DisplayName: RC input port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to RC input.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_PORT", 3, Parameters_RCIN, rcin1_port, AP_PERIPH_RC1_PORT_DEFAULT),

    // @Param: 1_PORT_OPTIONS
    // @DisplayName: RC input port serial options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    AP_GROUPINFO("1_PORT_OPTIONS", 4, Parameters_RCIN, rcin1_port_options, AP_PERIPH_RC1_PORT_OPTIONS_DEFAULT),
    // @RebootRequired: True

    AP_GROUPEND
};

Parameters_RCIN::Parameters_RCIN(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Periph_FW::rcin_init()
{
    if (g_rcin.rcin1_port < 0) {
        return;
    }

    // init uart for serial RC
    auto *uart = hal.serial(g_rcin.rcin1_port);
    if (uart == nullptr) {
        return;
    }

    uart->set_options(g_rcin.rcin1_port_options);

    serial_manager.set_protocol_and_baud(
        g_rcin.rcin1_port,
        AP_SerialManager::SerialProtocol_RCIN,
        115200  // baud doesn't matter; RC Protocol autobauds
        );

    auto &rc = AP::RC();
    rc.init();
    rc.set_rc_protocols(g_rcin.rcin_protocols);
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
    const uint8_t rate_hz = g_rcin.rcin_rate_hz;
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
