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
  AP_Radio implementation for Cypress 2.4GHz radio. 

  With thanks to the SuperBitRF project
  See http://wiki.paparazziuav.org/wiki/SuperbitRF

  This implementation uses the DSMX protocol on a CYRF6936
 */

#include "AP_Radio_backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "hal.h"
#endif
#include "telem_structure.h"

class AP_Radio_cypress : public AP_Radio_backend
{
public:
    AP_Radio_cypress(AP_Radio &radio);
    
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
        return (uint16_t(dsm.tx_firmware_year)<<12) + (uint16_t(dsm.tx_firmware_month)<<8) + dsm.tx_firmware_day;
    }
    
    // get radio statistics structure
    const AP_Radio::stats &get_stats(void) override;

    // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    void set_wifi_channel(uint8_t channel) {
        t_status.wifi_chan = channel;
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    static AP_Radio_cypress *radio_instance;

    void radio_init(void);
    
    void dump_registers(uint8_t n);

    void force_initial_state(void);
    void set_channel(uint8_t channel);
    uint8_t read_status_debounced(uint8_t adr);
    
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);
    void write_multiple(uint8_t reg, uint8_t n, const uint8_t *data);

    enum {
        STATE_RECV,
        STATE_BIND,
        STATE_AUTOBIND,
        STATE_SEND_TELEM,
        STATE_SEND_TELEM_WAIT,
        STATE_SEND_FCC
    } state;
    
    struct config {
        uint8_t reg;
        uint8_t value;
    };
    static const uint8_t pn_codes[5][9][8];
    static const uint8_t pn_bind[];
    static const config cyrf_config[];
    static const config cyrf_bind_config[];
    static const config cyrf_transfer_config[];

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    sem_t irq_sem;
    struct hrt_call wait_call;
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    virtual_timer_t timeout_vt;
    static thread_t *_irq_handler_ctx;
#endif
    void radio_set_config(const struct config *config, uint8_t size);

    void start_receive(void);
    
    // main IRQ handler
    void irq_handler(void);

    // IRQ handler for packet receive
    void irq_handler_recv(uint8_t rx_status);

    // handle timeout IRQ
    void irq_timeout(void);
    
    // trampoline functions to take us from static IRQ function to class functions

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    static int irq_radio_trampoline(int irq, void *context);
    static int irq_timeout_trampoline(int irq, void *context);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static void irq_handler_thd(void* arg);
    static void trigger_irq_radio_event(void);
    static void trigger_timeout_event(void *arg);
#endif    

    static const uint8_t max_channels = 16;

    uint32_t last_debug_print_ms;

    void print_debug_info(void);
    
    AP_Radio::stats stats;
    AP_Radio::stats last_stats;

    enum dsm_protocol {
        DSM_NONE   = 0,      // not bound yet
        DSM_DSM2_1 = 0x01,   // The original DSM2 protocol with 1 packet of data
        DSM_DSM2_2 = 0x02,   // The original DSM2 protocol with 2 packets of data
        DSM_DSMX_1 = 0xA2,   // The original DSMX protocol with 1 packet of data
        DSM_DSMX_2 = 0xB2,   // The original DSMX protocol with 2 packets of data
    };

    enum dsm2_sync {
        DSM2_SYNC_A,
        DSM2_SYNC_B,
        DSM2_OK
    };

    // semaphore between ISR and main thread
    AP_HAL::Semaphore *sem;    
    
    // dsm config data and status
    struct {
        uint8_t channels[23];
        enum dsm_protocol protocol;
        uint8_t mfg_id[4];
        uint8_t current_channel;
        uint8_t current_rf_channel;
        uint16_t crc_seed;
        uint8_t sop_col;
        uint8_t data_col;
        uint8_t last_sop_code[8];
        uint8_t last_data_code[16];

        uint32_t receive_start_us;
        uint32_t receive_timeout_msec;

        uint32_t last_recv_us;
        uint32_t last_parse_us;
        uint32_t last_recv_chan;
        uint32_t last_chan_change_us;
        uint16_t num_channels;
        uint16_t pwm_channels[max_channels];
        bool need_bind_save;
        enum dsm2_sync sync;
        uint32_t crc_errors;
        float rssi;
        bool last_discrc;
        uint8_t last_transmit_power;
        uint32_t send_irq_count;
        uint32_t send_count;
        uint16_t pkt_time1 = 3000;
        uint16_t pkt_time2 = 7000;
        uint8_t tx_firmware_year;
        uint8_t tx_firmware_month;
        uint8_t tx_firmware_day;
        int8_t forced_channel = -1;
        uint8_t tx_rssi;
        uint8_t tx_pps;
        uint32_t last_autobind_send;
        bool have_tx_pps;
        uint32_t telem_send_count;
        uint8_t tx_bl_version;
    } dsm;

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

    // bind structure saved to storage
    static const uint16_t bind_magic = 0x43F6;
    struct PACKED bind_info {
        uint16_t magic;
        uint8_t  mfg_id[4];
        enum dsm_protocol protocol;
    };

    struct telem_status t_status;
    
    // DSM specific functions
    void dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed);

    // generate DSMX channels
    void dsm_generate_channels_dsmx(uint8_t mfg_id[4], uint8_t channels[23]);

    // setup for DSMX transfers
    void dsm_setup_transfer_dsmx(void);

    // choose channel to receive on
    void dsm_choose_channel(void);

    // map for mode1/mode2
    void map_stick_mode(uint16_t *channels);
    
    // parse DSM channels from a packet
    bool parse_dsm_channels(const uint8_t *data);

    // process an incoming packet
    void process_packet(const uint8_t *pkt, uint8_t len);

    // process an incoming bind packet
    void process_bind(const uint8_t *pkt, uint8_t len);

    // load bind info from storage
    void load_bind_info(void);

    // save bind info to storage
    void save_bind_info(void);

    bool is_DSM2(void);

    // send a 16 byte packet
    void transmit16(const uint8_t data[16]);

    void send_telem_packet(void);
    void irq_handler_send(uint8_t tx_status);

    void send_FCC_test_packet(void);
    
    // check sending of fw upload ack
    void check_fw_ack(void);

    // re-sync DSM2
    void dsm2_start_sync(void);

    // check for double binding
    void check_double_bind(void);

    // setup a timeout handler
    void setup_timeout(uint32_t timeout_ms);
};

