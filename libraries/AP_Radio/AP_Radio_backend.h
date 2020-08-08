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
 * backend class for direct attached radios
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Radio.h"

class AP_Radio_backend
{
public:
    AP_Radio_backend(AP_Radio &radio);
    virtual ~AP_Radio_backend();

    // init - initialise radio
    virtual bool init(void) = 0;

    // init - reset radio
    virtual bool reset(void) = 0;

    // send a packet
    virtual bool send(const uint8_t *pkt, uint16_t len) = 0;

    // start bind process as a receiver
    virtual void start_recv_bind(void) = 0;

    // return time in microseconds of last received R/C packet
    virtual uint32_t last_recv_us(void) = 0;

    // return number of input channels
    virtual uint8_t num_channels(void) = 0;

    // return current PWM of a channel
    virtual uint16_t read(uint8_t chan) = 0;

    // handle a data96 mavlink packet for fw upload
    virtual void handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m) {}

    // update status
    virtual void update(void) = 0;

    // get TX fw version
    virtual uint32_t get_tx_version(void) = 0;

    // get radio statistics structure
    virtual const AP_Radio::stats &get_stats(void) = 0;

    // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    virtual void set_wifi_channel(uint8_t channel) = 0;

protected:
    // These functions are for the radio code to access the parameters
    // that the user can set using the web interface.

    AP_Radio::ap_radio_protocol get_protocol(void) const
    {
        return (AP_Radio::ap_radio_protocol)radio.protocol.get();
    }

    uint8_t get_debug_level(void) const
    {
        return (uint8_t)radio.debug_level.get();
    }

    bool get_disable_crc(void) const
    {
        return (bool)radio.disable_crc.get();
    }

    uint8_t get_rssi_chan(void) const
    {
        return (uint8_t)radio.rssi_chan.get();
    }

    uint8_t get_pps_chan(void) const
    {
        return (uint8_t)radio.pps_chan.get();
    }

    uint8_t get_tx_rssi_chan(void) const
    {
        return (uint8_t)radio.tx_rssi_chan.get();
    }

    uint8_t get_tx_pps_chan(void) const
    {
        return (uint8_t)radio.tx_pps_chan.get();
    }

    bool get_telem_enable(void) const
    {
        return radio.telem_enable.get() > 0;
    }

    uint8_t get_transmit_power(void) const
    {
        return constrain_int16(radio.transmit_power.get(), 1, 8);
    }

    uint8_t get_tx_max_power(void) const
    {
        return constrain_int16(radio.tx_max_power.get(), 1, 8);
    }

    void set_tx_max_power_default(uint8_t v)
    {
        return radio.tx_max_power.set_default(v);
    }

    int8_t get_fcc_test(void) const
    {
        return radio.fcc_test.get();
    }

    uint8_t get_stick_mode(void) const
    {
        return radio.stick_mode.get();
    }

    uint8_t get_factory_test(void) const
    {
        return radio.factory_test.get();
    }

    uint8_t get_tx_buzzer_adjust(void) const
    {
        return radio.tx_buzzer_adjust.get();
    }

    uint8_t get_autobind_time(void) const
    {
        return radio.auto_bind_time.get();
    }

    uint8_t get_autobind_rssi(void) const
    {
        return radio.auto_bind_rssi.get();
    }

    AP_Radio &radio;
};
