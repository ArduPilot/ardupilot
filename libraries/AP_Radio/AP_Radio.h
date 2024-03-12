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
#pragma once

/*
 * base class for direct attached radios
 */

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Radio_backend;

class AP_Radio
{
public:
    friend class AP_Radio_backend;

    // constructor
    AP_Radio(void);

    // init - initialise radio
    bool init(void);

    // reset the radio
    bool reset(void);

    // send a packet
    bool send(const uint8_t *pkt, uint16_t len);

    // start bind process as a receiver
    void start_recv_bind(void);

    // return time in microseconds of last received R/C packet
    uint32_t last_recv_us(void);

    // return number of input channels
    uint8_t num_channels(void);

    // return current PWM of a channel
    uint16_t read(uint8_t chan);

    // update status, should be called from main thread
    void update(void);

    // get transmitter firmware version
    uint32_t get_tx_version(void);

    struct stats {
        uint32_t bad_packets;
        uint32_t recv_errors;
        uint32_t recv_packets;
        uint32_t lost_packets;
        uint32_t timeouts;
    };

    enum ap_radio_type {
        RADIO_TYPE_NONE=0,
        RADIO_TYPE_CYRF6936=1,
        RADIO_TYPE_CC2500=2,
        RADIO_TYPE_BK2425=3,
        RADIO_TYPE_AUTO=100,
    };

    enum ap_radio_protocol {
        PROTOCOL_AUTO=0,
        PROTOCOL_DSM2=1,
        PROTOCOL_DSMX=2,
        PROTOCOL_D16=3,
        PROTOCOL_CC2500_GFSK=4, // deviation 57kHz for update of cc2500 with GFSK
    };

    // get packet statistics
    const struct stats &get_stats(void);

    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AP_Radio *get_singleton(void)
    {
        return _singleton;
    }

    // handle a data96 mavlink packet for fw upload
    void handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m);

    // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    void set_wifi_channel(uint8_t channel);

    // play a tune on the TX
    void play_tune(const char *tune_str);

    // change TX mode
    void change_txmode(void);

private:
    AP_Radio_backend *driver;

    AP_Int8 radio_type;
    AP_Int8 protocol;
    AP_Int8 debug_level;
    AP_Int8 disable_crc;
    AP_Int8 rssi_chan;
    AP_Int8 pps_chan;
    AP_Int8 tx_rssi_chan;
    AP_Int8 tx_pps_chan;
    AP_Int8 telem_enable;
    AP_Int8 transmit_power;
    AP_Int8 tx_max_power;
    AP_Int8 fcc_test;
    AP_Int8 stick_mode;
    AP_Int8 factory_test;
    AP_Int8 tx_buzzer_adjust;
    AP_Int8 auto_bind_time;
    AP_Int8 auto_bind_rssi;

    static AP_Radio *_singleton;
};
