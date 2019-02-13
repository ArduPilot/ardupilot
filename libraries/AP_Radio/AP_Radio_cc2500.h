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
  AP_Radio implementation for CC2500 2.4GHz radio. 

  With thanks to cleanflight and betaflight projects
 */

#include "AP_Radio_backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "hal.h"
#include "telem_structure.h"
#include "driver_cc2500.h"

#define CC2500_MAX_CHANNELS 16

class AP_Radio_cc2500 : public AP_Radio_backend
{
public:
    AP_Radio_cc2500(AP_Radio &radio);
    
    // init - initialise radio
    bool init(void) override;

    // rest radio
    bool reset(void) override;
    
    // send a packet
    bool send(const uint8_t *pkt, uint16_t len) override;

    // start bind process as a receiver
    void start_recv_bind(void) override;

    // return time in microseconds of last received R/C packet
    uint32_t last_recv_us(void) override;

    // return number of input channels
    uint8_t num_channels(void) override;

    // return current PWM of a channel
    uint16_t read(uint8_t chan) override;

    // handle a data96 mavlink packet for fw upload
    void handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m) override;

    // update status
    void update(void) override;

    // get TX fw version
    uint32_t get_tx_version(void) override {
        // pack date into 16 bits for vendor_id in AUTOPILOT_VERSION
        return (uint16_t(tx_date.firmware_year)<<12) + (uint16_t(tx_date.firmware_month)<<8) + tx_date.firmware_day;
    }
    
    // get radio statistics structure
    const AP_Radio::stats &get_stats(void) override;

    // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    void set_wifi_channel(uint8_t channel) {
        // t_status.wifi_chan = channel;
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    static AP_Radio_cc2500 *radio_instance;
    static thread_t *_irq_handler_ctx;
    static virtual_timer_t timeout_vt;

    static void irq_handler_thd(void* arg);
    static void trigger_irq_radio_event(void);
    static void trigger_timeout_event(void *arg);

    void radio_init(void);

    // semaphore between ISR and main thread
    HAL_Semaphore sem;

    AP_Radio::stats stats;
    AP_Radio::stats last_stats;

    uint16_t pwm_channels[CC2500_MAX_CHANNELS];

    Radio_CC2500 cc2500;

    uint8_t calData[255][3];
    uint8_t bindTxId[2];
    int8_t  bindOffset;
    uint8_t bindHopData[47];
    uint8_t rxNum;
    uint8_t listLength;
    uint8_t channr;
    uint8_t chanskip;
    int8_t fcc_chan;
    uint32_t packet_timer;
    static uint32_t irq_time_us;
    const uint32_t sync_time_us = 9000;
    uint8_t chan_count;
    uint32_t lost;
    uint32_t timeouts;
    bool have_bind_info;
    uint8_t packet3;
    bool telem_send_rssi;
    float rssi_filtered;
    uint64_t bind_mask;
    uint8_t best_lqi;
    int8_t best_bindOffset;

    uint32_t timeTunedMs;

    void initTuneRx(void);
    void initialiseData(uint8_t adr);
    void initGetBind(void);
    bool tuneRx(uint8_t ccLen, uint8_t *packet);
    bool getBindData(uint8_t ccLen, uint8_t *packet);
    bool check_best_LQI(void);
    void setChannel(uint8_t channel);
    void nextChannel(uint8_t skip);

    void parse_frSkyX(const uint8_t *packet);
    uint16_t calc_crc(const uint8_t *data, uint8_t len);
    bool check_crc(uint8_t ccLen, uint8_t *packet);

    void send_D16_telemetry(void);
    void send_SRT_telemetry(void);

    void irq_handler(void);
    void irq_timeout(void);

    // bind structure saved to storage
    static const uint16_t bind_magic = 0x120a;
    struct PACKED bind_info {
        uint16_t magic;
        uint8_t bindTxId[2];
        int8_t  bindOffset;
        uint8_t listLength;
        uint8_t bindHopData[47];
    };
    
    void save_bind_info(void);
    bool load_bind_info(void);
    
    enum {
        STATE_INIT = 0,
        STATE_BIND,
        STATE_BIND_TUNING,
        STATE_BIND_BINDING,
        STATE_BIND_COMPLETE,
        STATE_STARTING,
        STATE_DATA,
        STATE_TELEMETRY,
        STATE_RESUME,
        STATE_FCCTEST,
        STATE_SEARCH,
    } protocolState;

    struct config {
        uint8_t reg;
        uint8_t value;
    };
    static const config radio_config[];

    struct {
        mavlink_channel_t chan;
        bool need_ack;
        uint8_t counter;
        uint8_t sequence;
        uint32_t offset;
        uint32_t length;
        uint32_t acked;
        uint8_t len;
        enum telem_type fw_type;
        uint8_t pending_data[92];
    } fwupload;

    struct {
        uint8_t firmware_year;
        uint8_t firmware_month;
        uint8_t firmware_day;
    } tx_date;
        
    struct telem_status t_status;
    uint32_t last_pps_ms;
    uint8_t tx_rssi;
    uint8_t tx_pps;

    bool handle_D16_packet(const uint8_t *packet);
    bool handle_SRT_packet(const uint8_t *packet);

    // check sending of fw upload ack
    void check_fw_ack(void);
};


#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

