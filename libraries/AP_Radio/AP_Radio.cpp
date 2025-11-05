#include "AP_Radio_config.h"

#if AP_RADIO_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Radio.h"
#include "AP_Radio_backend.h"
#include "AP_Radio_cypress.h"
#include "AP_Radio_cc2500.h"
#include "AP_Radio_bk2425.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Radio::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Set type of direct attached radio
    // @Description: This enables support for direct attached radio receivers
    // @Values: 0:None,1:CYRF6936,2:CC2500,3:BK2425

    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE",  1, AP_Radio, radio_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _PROT
    // @DisplayName: protocol
    // @Description: Select air protocol
    // @Values: 0:Auto,1:DSM2,2:DSMX
    // @User: Advanced
    AP_GROUPINFO("_PROT",  2, AP_Radio, protocol, PROTOCOL_AUTO),

    // @Param: _DEBUG
    // @DisplayName: debug level
    // @Description: radio debug level
    // @Range: 0 4
    // @User: Advanced
    AP_GROUPINFO("_DEBUG",  3, AP_Radio, debug_level, 0),

    // @Param: _DISCRC
    // @DisplayName: disable receive CRC
    // @Description: disable receive CRC (for debug)
    // @Values: 0:NotDisabled,1:Disabled
    // @User: Advanced
    AP_GROUPINFO("_DISCRC",  4, AP_Radio, disable_crc, 0),

    // @Param: _SIGCH
    // @DisplayName: RSSI signal strength
    // @Description: Channel to show receive RSSI signal strength, or zero for disabled
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_SIGCH",  5, AP_Radio, rssi_chan, 0),

    // @Param: _PPSCH
    // @DisplayName: Packet rate channel
    // @Description: Channel to show received packet-per-second rate, or zero for disabled
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_PPSCH",  6, AP_Radio, pps_chan, 0),

    // @Param: _TELEM
    // @DisplayName: Enable telemetry
    // @Description: If this is non-zero then telemetry packets will be sent over DSM
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("_TELEM",  7, AP_Radio, telem_enable, 0),

    // @Param: _TXPOW
    // @DisplayName: Telemetry Transmit power
    // @Description: Set telemetry transmit power. This is the power level (from 1 to 8) for telemetry packets sent from the RX to the TX
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("_TXPOW",  8, AP_Radio, transmit_power, 8),

    // @Param: _FCCTST
    // @DisplayName: Put radio into FCC test mode
    // @Description: If this is enabled then the radio will continuously transmit as required for FCC testing. The transmit channel is set by the value of the parameter. The radio will not work for RC input while this is enabled
    // @Values: 0:Disabled,1:MinChannel,2:MidChannel,3:MaxChannel,4:MinChannelCW,5:MidChannelCW,6:MaxChannelCW
    // @User: Advanced
    AP_GROUPINFO("_FCCTST", 9, AP_Radio, fcc_test, 0),

    // @Param: _STKMD
    // @DisplayName: Stick input mode
    // @Description: This selects between different stick input modes. The default is mode2, which has throttle on the left stick and pitch on the right stick. You can instead set mode1, which has throttle on the right stick and pitch on the left stick.
    // @Values: 1:Mode1,2:Mode2,3:Mode3,4:Mode4
    // @User: Advanced
    AP_GROUPINFO("_STKMD", 10, AP_Radio, stick_mode, 2),

    // @Param: _TESTCH
    // @DisplayName: Set radio to factory test channel
    // @Description: This sets the radio to a fixed test channel for factory testing. Using a fixed channel avoids the need for binding in factory testing.
    // @Values: 0:Disabled,1:TestChan1,2:TestChan2,3:TestChan3,4:TestChan4,5:TestChan5,6:TestChan6,7:TestChan7,8:TestChan8
    // @User: Advanced
    AP_GROUPINFO("_TESTCH", 11, AP_Radio, factory_test, 0),

    // @Param: _TSIGCH
    // @DisplayName: RSSI value channel for telemetry data on transmitter
    // @Description: Channel to show telemetry RSSI value as received by TX
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_TSIGCH", 12, AP_Radio, tx_rssi_chan, 0),

    // @Param: _TPPSCH
    // @DisplayName: Telemetry PPS channel
    // @Description: Channel to show telemetry packets-per-second value, as received at TX
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_TPPSCH", 13, AP_Radio, tx_pps_chan, 0),

    // @Param: _TXMAX
    // @DisplayName: Transmitter transmit power
    // @Description: Set transmitter maximum transmit power (from 1 to 8)
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("_TXMAX",  14, AP_Radio, tx_max_power, 8),

    // @Param: _BZOFS
    // @DisplayName: Transmitter buzzer adjustment
    // @Description: Set transmitter buzzer note adjustment (adjust frequency up)
    // @Range: 0 40
    // @User: Advanced
    AP_GROUPINFO("_BZOFS",  15, AP_Radio, tx_buzzer_adjust, 25),

    // @Param: _ABTIME
    // @DisplayName: Auto-bind time
    // @Description: When non-zero this sets the time with no transmitter packets before we start looking for auto-bind packets.
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("_ABTIME",  16, AP_Radio, auto_bind_time, 0),

    // @Param: _ABLVL
    // @DisplayName: Auto-bind level
    // @Description: This sets the minimum RSSI of an auto-bind packet for it to be accepted. This should be set so that auto-bind will only happen at short range to minimise the change of an auto-bind happening accidentially
    // @Range: 0 31
    // @User: Advanced
    AP_GROUPINFO("_ABLVL",  17, AP_Radio, auto_bind_rssi, 0),

    AP_GROUPEND
};

AP_Radio *AP_Radio::_singleton;

// constructor
AP_Radio::AP_Radio(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("Multiple AP_Radio declarations");
    }
    _singleton = this;
}

bool AP_Radio::init(void)
{
    switch (radio_type) {
#if AP_RADIO_CYRF6936_ENABLED
    case RADIO_TYPE_CYRF6936:
        driver = NEW_NOTHROW AP_Radio_cypress(*this);
        break;
#endif
#if AP_RADIO_CC2500_ENABLED
    case RADIO_TYPE_CC2500:
        driver = NEW_NOTHROW AP_Radio_cc2500(*this);
        break;
#endif
#if AP_RADIO_BK2425_ENABLED
    case RADIO_TYPE_BK2425:
        driver = NEW_NOTHROW AP_Radio_beken(*this);
        break;
#endif
#if AP_RADIO_AUTO_ENABLED
    case RADIO_TYPE_AUTO:
        // auto-detect between cc2500 and beken radios
#if AP_RADIO_CC2500_ENABLED
        if (AP_Radio_cc2500::probe()) {
            driver = NEW_NOTHROW AP_Radio_cc2500(*this);
        }
#endif
#if AP_RADIO_BK2425_ENABLED
        if (driver == nullptr) {
            driver = NEW_NOTHROW AP_Radio_beken(*this);
        }
#endif
        break;
#endif  // AP_RADIO_AUTO_ENABLED
    default:
        break;
    }
    if (!driver) {
        return false;
    }
    return driver->init();
}

bool AP_Radio::reset(void)
{
    if (!driver) {
        return false;
    }
    return driver->reset();
}

bool AP_Radio::send(const uint8_t *pkt, uint16_t len)
{
    if (!driver) {
        return false;
    }
    return driver->send(pkt, len);
}

void AP_Radio::start_recv_bind(void)
{
    if (!driver) {
        return;
    }
    return driver->start_recv_bind();
}

const AP_Radio::stats &AP_Radio::get_stats(void)
{
    return driver->get_stats();
}

uint8_t AP_Radio::num_channels(void)
{
    if (!driver) {
        return 0;
    }
    return driver->num_channels();
}

uint16_t AP_Radio::read(uint8_t chan)
{
    if (!driver) {
        return 0;
    }
    return driver->read(chan);
}

uint32_t AP_Radio::last_recv_us(void)
{
    if (!driver) {
        return 0;
    }
    return driver->last_recv_us();
}

// handle a data96 mavlink packet for fw upload
void AP_Radio::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
    if (driver) {
        driver->handle_data_packet(chan, m);
    }
}

// play a tune on the TX
void AP_Radio::play_tune(const char *tune_str)
{
    mavlink_data96_t pkt {};
    uint8_t len = MIN(strlen(tune_str), 92);
    pkt.len = len;
    pkt.type = 43;
    memcpy(&pkt.data[0], tune_str, len);
    handle_data_packet(MAVLINK_COMM_0, pkt);
}

// update status, should be called from main thread
void AP_Radio::update(void)
{
    if (driver) {
        driver->update();
    }
}

// get transmitter firmware version
uint32_t AP_Radio::get_tx_version(void)
{
    if (driver) {
        return driver->get_tx_version();
    }
    return 0;
}

// set the 2.4GHz wifi channel used by companion computer, so it can be avoided
void AP_Radio::set_wifi_channel(uint8_t channel)
{
    if (driver) {
        driver->set_wifi_channel(channel);
    }
}

// change TX mode, toggling between mode1 and mode2
void AP_Radio::change_txmode(void)
{
    if (stick_mode == 2) {
        stick_mode.set_and_save_ifchanged(1);
    } else {
        stick_mode.set_and_save_ifchanged(2);
    }
}

#endif  // AP_RADIO_ENABLED
