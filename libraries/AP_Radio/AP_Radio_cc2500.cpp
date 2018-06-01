/*
  driver for TI CC2500 radio

  Many thanks to the cleanflight and betaflight projects
 */
#include <AP_HAL/AP_HAL.h>

// #pragma GCC optimize("O0")

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if HAL_RCINPUT_WITH_AP_RADIO

#include <AP_Math/AP_Math.h>
#include "AP_Radio_cc2500.h"
#include <utility>
#include <stdio.h>
#include <StorageManager/StorageManager.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/crc.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
static THD_WORKING_AREA(_irq_handler_wa, 512);
#define TIMEOUT_PRIORITY 250	//Right above timer thread
#define EVT_TIMEOUT EVENT_MASK(0)
#define EVT_IRQ EVENT_MASK(1)
#define EVT_BIND EVENT_MASK(2)
#endif

extern const AP_HAL::HAL& hal;

#define Debug(level, fmt, args...)   do { if ((level) <= get_debug_level()) { hal.console->printf(fmt, ##args); }} while (0)

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

// object instance for trampoline
AP_Radio_cc2500 *AP_Radio_cc2500::radio_instance;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
thread_t *AP_Radio_cc2500::_irq_handler_ctx;
virtual_timer_t AP_Radio_cc2500::timeout_vt;
uint32_t AP_Radio_cc2500::irq_time_us;
#endif

/*
  constructor
 */
AP_Radio_cc2500::AP_Radio_cc2500(AP_Radio &_radio) :
    AP_Radio_backend(_radio),
    cc2500(hal.spi->get_device("cc2500"))
{
    // link to instance for irq_trampoline
    radio_instance = this;
}

/*
  initialise radio
 */
bool AP_Radio_cc2500::init(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if(_irq_handler_ctx != nullptr) {
        AP_HAL::panic("AP_Radio_cc2500: double instantiation of irq_handler\n");
    }
    chVTObjectInit(&timeout_vt);
    _irq_handler_ctx = chThdCreateStatic(_irq_handler_wa,
                     sizeof(_irq_handler_wa),
                     TIMEOUT_PRIORITY,        /* Initial priority.    */
                     irq_handler_thd,  /* Thread function.     */
                     NULL);                     /* Thread parameter.    */
#endif
    sem = hal.util->new_semaphore();    
    
    return reset();
}

/*
  reset radio
 */
bool AP_Radio_cc2500::reset(void)
{
    if (!cc2500.lock_bus()) {
        return false;
    }

    radio_init();
    cc2500.unlock_bus();

    return true;
}

/*
  return statistics structure from radio
 */
const AP_Radio::stats &AP_Radio_cc2500::get_stats(void)
{
    return stats;
}

/*
  read one pwm channel from radio
 */
uint16_t AP_Radio_cc2500::read(uint8_t chan)
{
    if (chan >= CC2500_MAX_CHANNELS) {
        return 0;
    }
    return pwm_channels[chan];
}

/*
  update status - called from main thread
 */
void AP_Radio_cc2500::update(void)
{
    check_fw_ack();
}
    

/*
  return number of active channels
 */
uint8_t AP_Radio_cc2500::num_channels(void)
{
    uint32_t now = AP_HAL::millis();
    uint8_t chan = get_rssi_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = t_status.rssi;
        chan_count = MAX(chan_count, chan);
    }

    chan = get_pps_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = t_status.pps;
        chan_count = MAX(chan_count, chan);
    }

    chan = get_tx_rssi_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = tx_rssi;
        chan_count = MAX(chan_count, chan);
    }

    chan = get_tx_pps_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = tx_pps;
        chan_count = MAX(chan_count, chan);
    }
    
    if (now - last_pps_ms > 1000) {
        last_pps_ms = now;
        t_status.pps = stats.recv_packets - last_stats.recv_packets;
        last_stats = stats;
        if (lost != 0 || timeouts != 0) {
            Debug(3,"lost=%u timeouts=%u TS=%u\n", unsigned(lost), unsigned(timeouts), sizeof(struct telem_packet_cc2500));
        }
        lost=0;
        timeouts=0;
    }
    return chan_count;
}

/*
  return time of last receive in microseconds
 */
uint32_t AP_Radio_cc2500::last_recv_us(void)
{
    return packet_timer;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_cc2500::send(const uint8_t *pkt, uint16_t len)
{
    // disabled for now
    return false;
}

const AP_Radio_cc2500::config AP_Radio_cc2500::radio_config[] = {
    {CC2500_00_IOCFG2,   0x01}, // GD2 high on RXFIFO filled or end of packet
    {CC2500_17_MCSM1,    0x0C}, // stay in RX on packet receive, CCA always, TX -> IDLE
    {CC2500_18_MCSM0,    0x18}, // XOSC expire 64, cal on IDLE -> TX or RX
    {CC2500_06_PKTLEN,   0x1E}, // packet length 30
    {CC2500_07_PKTCTRL1, 0x04}, // enable RSSI+LQI, no addr check, no autoflush, PQT=0
    {CC2500_08_PKTCTRL0, 0x01}, // var length mode, no CRC, FIFO enable, no whitening
    {CC2500_3E_PATABLE,  0xFF}, // ?? what are we doing to PA table here?
    {CC2500_0B_FSCTRL1,  0x0A}, // IF=253.90625kHz assuming 26MHz crystal
    {CC2500_0C_FSCTRL0,  0x00}, // freqoffs = 0
    {CC2500_0D_FREQ2,    0x5C}, // freq control high
    {CC2500_0E_FREQ1,    0x76}, // freq control middle
    {CC2500_0F_FREQ0,    0x27}, // freq control low
    {CC2500_10_MDMCFG4,  0x7B}, // data rate control
    {CC2500_11_MDMCFG3,  0x61}, // data rate control
    {CC2500_12_MDMCFG2,  0x13}, // 30/32 sync word bits, no manchester, GFSK, DC filter enabled
    {CC2500_13_MDMCFG1,  0x23}, // chan spacing exponent 3, preamble 4 bytes, FEC disabled
    {CC2500_14_MDMCFG0,  0x7A}, // chan spacing 299.926757kHz for 26MHz crystal
    {CC2500_15_DEVIATN,  0x51}, // modem deviation 25.128906kHz for 26MHz crystal
    {CC2500_19_FOCCFG,   0x16}, // frequency offset compensation
    {CC2500_1A_BSCFG,    0x6C}, // bit sync config
    {CC2500_1B_AGCCTRL2, 0x43}, // target amplitude 33dB
    {CC2500_1C_AGCCTRL1, 0x40}, // AGC control 2
    {CC2500_1D_AGCCTRL0, 0x91}, // AGC control 0
    {CC2500_21_FREND1,   0x56}, // frontend config1
    {CC2500_22_FREND0,   0x10}, // frontend config0
    {CC2500_23_FSCAL3,   0xA9}, // frequency synth cal3
    {CC2500_24_FSCAL2,   0x0A}, // frequency synth cal2
    {CC2500_25_FSCAL1,   0x00}, // frequency synth cal1
    {CC2500_26_FSCAL0,   0x11}, // frequency synth cal0
    {CC2500_29_FSTEST,   0x59}, // test bits
    {CC2500_2C_TEST2,    0x88}, // test settings
    {CC2500_2D_TEST1,    0x31}, // test settings
    {CC2500_2E_TEST0,    0x0B}, // test settings
    {CC2500_03_FIFOTHR,  0x07}, // TX fifo threashold 33, RX fifo threshold 32
    {CC2500_09_ADDR,     0x00}, // device address 0 (broadcast)
};

const uint16_t CRCTable[] = {
        0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
        0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
        0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
        0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
        0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
        0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
        0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
        0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
        0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
        0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
        0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
        0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
        0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
        0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
        0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
        0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
        0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
        0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
        0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
        0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
        0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
        0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
        0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
        0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
        0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
        0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
        0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
        0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
        0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
        0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
        0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
        0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
};

/*
  initialise the radio
 */
void AP_Radio_cc2500::radio_init(void)
{
    if (cc2500.ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST) != 0x80 ||
        cc2500.ReadReg(CC2500_31_VERSION | CC2500_READ_BURST) != 0x03) {
        Debug(1, "cc2500: radio not found\n");
        return;
    }

    Debug(1, "cc2500: radio_init starting\n");

    cc2500.Reset();
    hal.scheduler->delay_microseconds(100);
    for (uint8_t i=0; i<ARRAY_SIZE(radio_config); i++) {
        // write twice to cope with possible SPI errors
        cc2500.WriteRegCheck(radio_config[i].reg, radio_config[i].value);
    }
    cc2500.Strobe(CC2500_SIDLE);	// Go to idle...

    for (uint8_t c=0;c<0xFF;c++) {
        //calibrate all channels
        cc2500.Strobe(CC2500_SIDLE);
        cc2500.WriteRegCheck(CC2500_0A_CHANNR, c);
        cc2500.Strobe(CC2500_SCAL);
        hal.scheduler->delay_microseconds(900);
        calData[c][0] = cc2500.ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500.ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500.ReadReg(CC2500_25_FSCAL1);
    }

    hal.scheduler->delay_microseconds(10*1000);
    
    // setup handler for rising edge of IRQ pin
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    stm32_gpiosetevent(CYRF_IRQ_INPUT, true, false, false, irq_radio_trampoline);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    hal.gpio->attach_interrupt(HAL_GPIO_RADIO_IRQ, trigger_irq_radio_event, HAL_GPIO_INTERRUPT_RISING);
#endif

    initTuneRx();

    if (load_bind_info()) {
        Debug(3,"Loaded bind info\n");
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_SEARCH;
        chanskip = 1;
        nextChannel(1);
    } else {
        protocolState = STATE_BIND_TUNING;
    }

    chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
}

void AP_Radio_cc2500::trigger_irq_radio_event()
{
    //we are called from ISR context
    chSysLockFromISR();
    irq_time_us = AP_HAL::micros();
    chEvtSignalI(_irq_handler_ctx, EVT_IRQ);
    chSysUnlockFromISR();
}

void AP_Radio_cc2500::trigger_timeout_event(void *arg)
{
    (void)arg;
    //we are called from ISR context
    chSysLockFromISR();
    chVTSetI(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
    chEvtSignalI(_irq_handler_ctx, EVT_TIMEOUT);
    chSysUnlockFromISR();
}

void AP_Radio_cc2500::start_recv_bind(void)
{
    protocolState = STATE_BIND_TUNING;
    chan_count = 0;
    packet_timer = AP_HAL::micros();
    chEvtSignal(_irq_handler_ctx, EVT_BIND);
    Debug(1,"Starting bind\n");
}

// handle a data96 mavlink packet for fw upload
void AP_Radio_cc2500::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
    uint32_t ofs=0;
    memcpy(&ofs, &m.data[0], 4);
    Debug(4, "got data96 of len %u from chan %u at offset %u\n", m.len, chan, unsigned(ofs));
    if (sem->take_nonblocking()) {
        fwupload.chan = chan;
        fwupload.need_ack = false;
        fwupload.offset = ofs;
        fwupload.length = MIN(m.len-4, 92);
        fwupload.acked = 0;
        fwupload.sequence++;
        if (m.type == 43) {
            // sending a tune to play - for development testing
            fwupload.fw_type = TELEM_PLAY;
            fwupload.length = MIN(m.len, 90);
            fwupload.offset = 0;
            memcpy(&fwupload.pending_data[0], &m.data[0], fwupload.length);
        } else {
            // sending a chunk of firmware OTA upload
            fwupload.fw_type = TELEM_FW;
            memcpy(&fwupload.pending_data[0], &m.data[4], fwupload.length);
        }
        sem->give();
    } 
}

/*
  handle a FrSky D16 packet
 */
bool AP_Radio_cc2500::handle_D16_packet(const uint8_t *packet)
{
    if (packet[0] != 0x1D) {
        return false;
    }
    if (packet[1] != bindTxId[0] ||
        packet[2] != bindTxId[1]) {
        Debug(3, "p1=%02x p2=%02x p6=%02x\n", packet[1], packet[2], packet[6]);
        // not for us
        return false;
    }
    if (packet[7] == 0x00 ||
        packet[7] == 0x20 ||
        packet[7] == 0x10 ||
        packet[7] == 0x12 ||
        packet[7] == 0x14 ||
        packet[7] == 0x16 ||
        packet[7] == 0x18 ||
        packet[7] == 0x1A ||
        packet[7] == 0x1C ||
        packet[7] == 0x1E) {
        // channel packet or range check packet
        parse_frSkyX(packet);

        packet3 = packet[3];

        uint8_t hop_chan = packet[4] & 0x3F;
        uint8_t skip = (packet[4]>>6) | (packet[5]<<2);
        if (channr != hop_chan) {
            Debug(2, "channr=%u hop_chan=%u\n", channr, hop_chan);
        }
        channr = hop_chan;
        if (chanskip != skip) {
            Debug(2, "chanskip=%u skip=%u\n", chanskip, skip);
        }
        chanskip = skip;
        return true;
    }
    return false;
}

/*
  handle a SRT packet
 */
bool AP_Radio_cc2500::handle_SRT_packet(const uint8_t *packet)
{
    const struct srt_packet *pkt = (const struct srt_packet *)packet;
    if (pkt->length != sizeof(struct srt_packet)-1 ||
        pkt->txid[0] != bindTxId[0] ||
        pkt->txid[1] != bindTxId[1]) {
        Debug(3, "len=%u p1=%02x p2=%02x\n", pkt->length, pkt->txid[0], pkt->txid[1]);
        // not for us
        return false;
    }
    if (pkt->version != 1) {
        // only support version 1 so far
        return false;
    }
    pwm_channels[0] = pkt->chan1 + 1000 + ((pkt->chan_high&0xC0)<<2);
    pwm_channels[1] = pkt->chan2 + 1000 + ((pkt->chan_high&0x30)<<4);
    pwm_channels[2] = pkt->chan3 + 1000 + ((pkt->chan_high&0x0C)<<6);
    pwm_channels[3] = pkt->chan4 + 1000 + ((pkt->chan_high&0x03)<<8);
    // we map the buttons onto two PWM channels for ease of integration with ArduPilot
    pwm_channels[4] = 1000 + (pkt->buttons & 0x7) * 100;
    pwm_channels[5] = 1000 + (pkt->buttons >> 3) * 100;

    uint8_t data = pkt->data;
    /*
      decode special data field
     */
    switch (pkt->pkt_type) {
    case PKTYPE_VOLTAGE:
        // voltage from TX is in 0.025 volt units. Convert to 0.01 volt units for easier display
        pwm_channels[6] = data * 4;
        break;
    case PKTYPE_YEAR:
        tx_date.firmware_year = data;
        break;
    case PKTYPE_MONTH:
        tx_date.firmware_month = data;
        break;
    case PKTYPE_DAY:
        tx_date.firmware_day = data;
        break;
    case PKTYPE_TELEM_RSSI:
        tx_rssi = data;
        break;
    case PKTYPE_TELEM_PPS:
        tx_pps = data;
        break;
    case PKTYPE_BL_VERSION:
        // unused so far for cc2500
        break;
    case PKTYPE_FW_ACK: {
            // got an fw upload ack 
            Debug(4, "ack %u seq=%u acked=%u length=%u len=%u\n",
                  data, fwupload.sequence, unsigned(fwupload.acked), unsigned(fwupload.length), fwupload.len);
            if (fwupload.sequence == data && sem->take_nonblocking()) {
                fwupload.sequence++;
                fwupload.acked += fwupload.len;
                if (fwupload.acked == fwupload.length) {
                    // trigger send of DATA16 ack to client
                    fwupload.need_ack = true;
                }
                sem->give();
            }
        break;
    }
    }
    
    if (chan_count < 7) {
        chan_count = 7;
    }
    
    if (pkt->channr != channr) {
        Debug(2, "channr=%u hop_chan=%u\n", channr, pkt->channr);
        channr = pkt->channr;
    }
    if (pkt->chanskip != chanskip) {
        Debug(2, "chanskip=%u skip=%u\n", chanskip, pkt->chanskip);
        chanskip = pkt->chanskip;
    }
    return true;
}

// main IRQ handler
void AP_Radio_cc2500::irq_handler(void)
{
    uint8_t ccLen;
    bool matched = false;
    do {
        ccLen = cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
        hal.scheduler->delay_microseconds(20);
        uint8_t ccLen2 = cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
        matched = (ccLen == ccLen2);
    } while (!matched);

    if (ccLen & 0x80) {
        Debug(3,"Fifo overflow %02x\n", ccLen);
        // RX FIFO overflow
        cc2500.Strobe(CC2500_SFRX);
        cc2500.Strobe(CC2500_SRX);
        return;
    }

    uint8_t packet[ccLen];
    cc2500.ReadFifo(packet, ccLen);

    if (get_fcc_test() != 0) {
        // don't process interrupts in FCCTEST mode
        return;
    }
    
    if (ccLen != 32 && ccLen != sizeof(srt_packet)+2) {
        cc2500.Strobe(CC2500_SFRX);
        cc2500.Strobe(CC2500_SRX);
        Debug(3, "bad len %u\n", ccLen);
        return;
    }
    
    if (get_debug_level() > 6) {
        Debug(6, "CC2500 IRQ state=%u\n", unsigned(protocolState));
        Debug(6,"len=%u\n", ccLen);
        for (uint8_t i=0; i<ccLen; i++) {
            Debug(6, "%02x:%02x ", i, packet[i]);
            if ((i+1) % 16 == 0) {
                Debug(6, "\n");
            }
        }
        if (ccLen % 16 != 0) {
            Debug(6, "\n");
        }
    }

    if (!check_crc(ccLen, packet)) {
        Debug(3, "bad CRC ccLen=%u\n", ccLen);
        return;
    }
    
    switch (protocolState) {
    case STATE_BIND_TUNING:
        tuneRx(ccLen, packet);
        break;

    case STATE_BIND_BINDING:
        if (getBindData(ccLen, packet)) {
            Debug(2,"Bind complete\n");
            protocolState = STATE_BIND_COMPLETE;
        }
        break;

    case STATE_BIND_COMPLETE:
        protocolState = STATE_STARTING;
        save_bind_info();
        Debug(3,"listLength=%u\n", listLength);
        Debug(3,"Saved bind info\n");
        break;

    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_SEARCH;
        chanskip = 1;
        nextChannel(1);
        break;

    case STATE_SEARCH:
        protocolState = STATE_DATA;
        FALLTHROUGH;

    case STATE_DATA: {
        bool ok = false;
        if (ccLen == 32) {
            ok = handle_D16_packet(packet);
        } else if (ccLen == sizeof(srt_packet)+2) {
            ok = handle_SRT_packet(packet);
        }
        if (ok) {
            // get RSSI value from status byte
            uint8_t rssi_raw = packet[ccLen-2];
            float rssi_dbm;
            if (rssi_raw >= 128) {
                rssi_dbm = ((((uint16_t)rssi_raw) * 18) >> 5) - 82;
            } else {
                rssi_dbm = ((((uint16_t)rssi_raw) * 18) >> 5) + 65;
            }
            rssi_filtered = 0.95 * rssi_filtered + 0.05 * rssi_dbm;
            t_status.rssi = uint8_t(MAX(rssi_filtered, 1));
            
            stats.recv_packets++;

            packet_timer = irq_time_us;
            chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
        
            cc2500.Strobe(CC2500_SIDLE);
            cc2500.SetPower(get_transmit_power());
            if (ccLen == 32 || get_protocol() == AP_Radio::PROTOCOL_D16) {
                send_D16_telemetry();
            } else {
                send_SRT_telemetry();
            }
        
            // we can safely sleep here as we have a dedicated thread for radio processing. We need to sleep
            // for enough time for the packet to be fully transmitted
            cc2500.unlock_bus();
            hal.scheduler->delay_microseconds(3500);
            cc2500.lock_bus();
        
            nextChannel(chanskip);
        }
        break;
    }

    case STATE_FCCTEST:
        // nothing to do, all done in timeout code
        Debug(3,"IRQ in FCCTEST state\n");
        break;

    default:
        Debug(2,"state %u\n", (unsigned)protocolState);
        break;
    }
}

// handle timeout IRQ
void AP_Radio_cc2500::irq_timeout(void)
{
    if (get_fcc_test() != 0 && protocolState != STATE_FCCTEST) {
        protocolState = STATE_FCCTEST;
        Debug(1,"Starting FCCTEST %d\n", get_fcc_test());
        setChannel(labs(get_fcc_test()) * 10);
        send_D16_telemetry();
    }
    
    switch (protocolState) {
    case STATE_BIND_TUNING: {
        if (bindOffset >= 126) {
            if (check_best_LQI()) {
                return;
            }
            bindOffset = -126;
        }
        uint32_t now = AP_HAL::millis();    
        if (now - timeTunedMs > 20) {
            timeTunedMs = now;
            bindOffset += 5;
            Debug(6,"bindOffset=%d\n", int(bindOffset));
            cc2500.Strobe(CC2500_SIDLE);
            cc2500.WriteRegCheck(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
            cc2500.Strobe(CC2500_SFRX);
            cc2500.Strobe(CC2500_SRX);
        }
        break;
    }
        
    case STATE_DATA: {
        uint32_t now = AP_HAL::micros();
        
        if (now - packet_timer > 50*sync_time_us) {
            Debug(3,"searching %u\n", unsigned(now - packet_timer));
            cc2500.Strobe(CC2500_SIDLE);
            cc2500.Strobe(CC2500_SFRX);
            nextChannel(1);
            cc2500.Strobe(CC2500_SRX);
            timeouts++;
            protocolState = STATE_SEARCH;
        } else {
            nextChannel(chanskip);
            // to keep the timeouts 1ms behind the expected time we
            // need to set the timeout to 9ms
            chVTSet(&timeout_vt, MS2ST(9), trigger_timeout_event, nullptr);
            lost++;
        }
        break;
    }

    case STATE_SEARCH:
        // shift by one channel at a time when searching
        nextChannel(1);
        break;
            
    case STATE_FCCTEST: {
        if (get_fcc_test() == 0) {
            protocolState = STATE_DATA;
            Debug(1,"Ending FCCTEST\n");
        }
        setChannel(labs(get_fcc_test()) * 10);
        cc2500.SetPower(get_transmit_power());
        send_D16_telemetry();
        break;
    }

    default:
        break;
    }
}

void AP_Radio_cc2500::irq_handler_thd(void *arg)
{
    (void)arg;
    while(true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

        radio_instance->cc2500.lock_bus();
        
        switch(evt) {
        case EVT_IRQ:
            if (radio_instance->protocolState == STATE_FCCTEST) {
                hal.console->printf("IRQ FCC\n");
            }
            radio_instance->irq_handler();
            break;
        case EVT_TIMEOUT:
            if (radio_instance->cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x80) {
                irq_time_us = AP_HAL::micros();
                radio_instance->irq_handler();
            } else {
                radio_instance->irq_timeout();
            }
            break;
        case EVT_BIND:
            radio_instance->initTuneRx();
            break;
        default:
            break;
        }

        radio_instance->cc2500.unlock_bus();
    }
}

void AP_Radio_cc2500::initTuneRx(void)
{
    cc2500.WriteReg(CC2500_19_FOCCFG, 0x14);
    timeTunedMs = AP_HAL::millis();
    bindOffset = -126;
    best_lqi = 255;
    best_bindOffset = bindOffset;
    cc2500.WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    //cc2500.WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500.WriteReg(CC2500_18_MCSM0, 0x8);

    cc2500.Strobe(CC2500_SIDLE);
    cc2500.WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500.WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500.WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500.WriteReg(CC2500_0A_CHANNR, 0);
    cc2500.Strobe(CC2500_SFRX);
    cc2500.Strobe(CC2500_SRX);
}

void AP_Radio_cc2500::initialiseData(uint8_t adr)
{
    cc2500.WriteRegCheck(CC2500_0C_FSCTRL0, bindOffset);
    cc2500.WriteRegCheck(CC2500_18_MCSM0, 0x8);
    cc2500.WriteRegCheck(CC2500_09_ADDR, adr ? 0x03 : bindTxId[0]);
    //cc2500.WriteRegCheck(CC2500_07_PKTCTRL1, 0x0D); // address check, no broadcast, autoflush, status enable
    cc2500.WriteRegCheck(CC2500_19_FOCCFG, 0x16);
    hal.scheduler->delay_microseconds(10*1000);
}

void AP_Radio_cc2500::initGetBind(void)
{
    cc2500.Strobe(CC2500_SIDLE);
    cc2500.WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500.WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500.WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500.WriteReg(CC2500_0A_CHANNR, 0);
    cc2500.Strobe(CC2500_SFRX);
    hal.scheduler->delay_microseconds(20); // waiting flush FIFO

    cc2500.Strobe(CC2500_SRX);
    listLength = 0;
}

/*
  we've wrapped in the search for the best bind offset. Accept the
  best so far if its good enough
 */
bool AP_Radio_cc2500::check_best_LQI(void)
{
    if (best_lqi >= 50) {
        return false;
    }
    bindOffset = best_bindOffset;
    initGetBind();
    initialiseData(1);
    protocolState = STATE_BIND_BINDING;
    bind_mask = 0;
    listLength = 0;
    Debug(2,"Bind tuning %d with Lqi %u\n", best_bindOffset, best_lqi);
    return true;
}

/*
  check if we have received a packet with sufficiently good link
  quality to start binding
 */
bool AP_Radio_cc2500::tuneRx(uint8_t ccLen, uint8_t *packet)
{
    if (bindOffset >= 126) {
        // we've scanned the whole range, if any were below 50 then
        // accept it
        if (check_best_LQI()) {
            return true;
        }
        bindOffset = -126;
    }
    if ((packet[ccLen - 1] & 0x80) && packet[2] == 0x01) {
        uint8_t Lqi = packet[ccLen - 1] & 0x7F;
        if (Lqi < best_lqi) {
            best_lqi = Lqi;
            best_bindOffset = bindOffset;
        }
    }
    return false;
}

/*
  get a block of hopping data from a bind packet
 */
bool AP_Radio_cc2500::getBindData(uint8_t ccLen, uint8_t *packet)
{
    // parse a bind data packet */
    if ((packet[ccLen - 1] & 0x80) && packet[2] == 0x01) {
        if (bind_mask == 0) {
            bindTxId[0] = packet[3];
            bindTxId[1] = packet[4];
        } else if (bindTxId[0] != packet[3] ||
                   bindTxId[1] != packet[4]) {
            Debug(2,"Bind restart\n");
            bind_mask = 0;
            listLength = 0;
        }

        for (uint8_t n = 0; n < 5; n++) {
            uint8_t c = packet[5] + n;
            if (c < sizeof(bindHopData)) {
                bindHopData[c] = packet[6 + n];
                bind_mask |= (uint64_t(1)<<c);
                listLength = MAX(listLength, c+1);
            }
        }
        // bind has finished when we have hopping data for all channels
        return (listLength == 47 && bind_mask == ((uint64_t(1)<<47)-1));
    }
    return false;
}

void AP_Radio_cc2500::setChannel(uint8_t channel)
{
    cc2500.Strobe(CC2500_SIDLE);
    cc2500.WriteReg(CC2500_23_FSCAL3, calData[channel][0]);
    cc2500.WriteReg(CC2500_24_FSCAL2, calData[channel][1]);
    cc2500.WriteReg(CC2500_25_FSCAL1, calData[channel][2]);
    cc2500.WriteReg(CC2500_0A_CHANNR, channel);
    cc2500.Strobe(CC2500_SRX);
}

void AP_Radio_cc2500::nextChannel(uint8_t skip)
{
    channr = (channr + skip) % listLength;
    setChannel(bindHopData[channr]);
}

void AP_Radio_cc2500::parse_frSkyX(const uint8_t *packet)
{
    uint16_t c[8];

    c[0] = (uint16_t)((packet[10] <<8)& 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11]<<4)&0xFF0) | (packet[10]>>4);
    c[2] = (uint16_t)((packet[13] <<8)& 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14]<<4)&0xFF0) | (packet[13]>>4);
    c[4] = (uint16_t)((packet[16] <<8)& 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17]<<4)&0xFF0) | (packet[16]>>4);
    c[6] = (uint16_t)((packet[19] <<8)& 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20]<<4)&0xFF0) | (packet[19]>>4);

    uint8_t j;
    for (uint8_t i=0;i<8;i++) {
        if(c[i] > 2047)  {
            j = 8;
            c[i] = c[i] - 2048;
        } else {
            j = 0;
        }
        if (c[i] == 0) {
            continue;
        }
        uint16_t word_temp = (((c[i]-64)<<1)/3+860);
        if ((word_temp > 800) && (word_temp < 2200)) {
            uint8_t chan = i+j;
            if (chan < CC2500_MAX_CHANNELS) {
                pwm_channels[chan] = word_temp;
                if (chan >= chan_count) {
                    chan_count = chan+1;
                }
            }
        }
    }
}

uint16_t AP_Radio_cc2500::calc_crc(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for(uint8_t i=0; i < len; i++) {
        crc = (crc<<8) ^ (CRCTable[((uint8_t)(crc>>8) ^ *data++) & 0xFF]);
    }
    return crc;
}

bool AP_Radio_cc2500::check_crc(uint8_t ccLen, uint8_t *packet)
{
    if (ccLen == sizeof(srt_packet)+2) {
        struct srt_packet *pkt = (struct srt_packet *)packet;
        // SRT packet
        uint16_t lcrc = calc_crc(packet,sizeof(struct srt_packet)-2);
        return lcrc == ((pkt->crc[0]<<8) | pkt->crc[1]);
    } else if (ccLen == 32) {
        // D16 packet
        uint16_t lcrc = calc_crc(&packet[3],(ccLen-7));
        return ((lcrc >>8)==packet[ccLen-4] && (lcrc&0x00FF)==packet[ccLen-3]);
    }
    return false;
}

/*
  save bind info
 */
void AP_Radio_cc2500::save_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;
    
    info.magic = bind_magic;
    info.bindTxId[0] = bindTxId[0];
    info.bindTxId[1] = bindTxId[1];
    info.bindOffset = bindOffset;
    info.listLength = listLength;
    memcpy(info.bindHopData, bindHopData, sizeof(info.bindHopData));
    bind_storage.write_block(0, &info, sizeof(info));
}

/*
  load bind info
 */
bool AP_Radio_cc2500::load_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    if (!bind_storage.read_block(&info, 0, sizeof(info)) || info.magic != bind_magic) {
        return false;
    }
    
    bindTxId[0] = info.bindTxId[0];
    bindTxId[1] = info.bindTxId[1];
    bindOffset = info.bindOffset;
    listLength = info.listLength;
    memcpy(bindHopData, info.bindHopData, sizeof(bindHopData));

    return true;
}

/*
  send a D16 telemetry packet
 */
void AP_Radio_cc2500::send_D16_telemetry(void)
{
    uint8_t frame[15];

    memset(frame, 0, sizeof(frame));
    
    frame[0] = sizeof(frame)-1;
    frame[1] = bindTxId[0];
    frame[2] = bindTxId[1];
    frame[3] = packet3;
    if (telem_send_rssi) {
        frame[4] = MAX(MIN(t_status.rssi, 0x7f),1) | 0x80;
    } else {
        frame[4] = uint8_t(hal.analogin->board_voltage() * 10) & 0x7F;
    }
    telem_send_rssi = !telem_send_rssi;
    
    uint16_t lcrc = calc_crc(&frame[3], 10);
    frame[13] = lcrc>>8;
    frame[14] = lcrc;

    cc2500.Strobe(CC2500_SIDLE);
    cc2500.Strobe(CC2500_SFTX);
    if (get_fcc_test() >= 0) {
        // in negative FCC test modes we don't write to the FIFO, which gives
        // continuous transmission
        cc2500.WriteFifo(frame, sizeof(frame));
    }
    cc2500.Strobe(CC2500_STX);
}


/*
  send a SRT telemetry packet
 */
void AP_Radio_cc2500::send_SRT_telemetry(void)
{
    struct telem_packet_cc2500 pkt {};

    pkt.length = sizeof(pkt)-1;

    t_status.flags = 0;
    t_status.flags |= AP_Notify::flags.gps_status >= 3?TELEM_FLAG_GPS_OK:0;
    t_status.flags |= AP_Notify::flags.pre_arm_check?TELEM_FLAG_ARM_OK:0;
    t_status.flags |= AP_Notify::flags.failsafe_battery?0:TELEM_FLAG_BATT_OK;
    t_status.flags |= hal.util->get_soft_armed()?TELEM_FLAG_ARMED:0;
    t_status.flags |= AP_Notify::flags.have_pos_abs?TELEM_FLAG_POS_OK:0;
    t_status.flags |= AP_Notify::flags.video_recording?TELEM_FLAG_VIDEO:0;
    t_status.flight_mode = AP_Notify::flags.flight_mode;
    t_status.tx_max = get_tx_max_power();
    t_status.note_adjust = get_tx_buzzer_adjust();

    // send fw update packet for 7/8 of packets if any data pending
    if (fwupload.length != 0 &&
        fwupload.length > fwupload.acked &&
        ((fwupload.counter++ & 0x07) != 0) &&
        sem->take_nonblocking()) {
        pkt.type = fwupload.fw_type;
        pkt.payload.fw.seq = fwupload.sequence;
        uint32_t len = fwupload.length>fwupload.acked?fwupload.length - fwupload.acked:0;
        pkt.payload.fw.len = len<=8?len:8;
        pkt.payload.fw.offset = fwupload.offset+fwupload.acked;
        memcpy(&pkt.payload.fw.data[0], &fwupload.pending_data[fwupload.acked], pkt.payload.fw.len);
        fwupload.len = pkt.payload.fw.len;
        Debug(4, "sent fw seq=%u offset=%u len=%u type=%u\n",
               pkt.payload.fw.seq,
               pkt.payload.fw.offset,
               pkt.payload.fw.len,
               pkt.type);
        sem->give();
    } else {
        pkt.type = TELEM_STATUS;
        pkt.payload.status = t_status;
    }
    pkt.txid[0] = bindTxId[0];
    pkt.txid[1] = bindTxId[1];

    uint16_t lcrc = calc_crc((const uint8_t *)&pkt, sizeof(pkt)-2);
    pkt.crc[0] = lcrc>>8;
    pkt.crc[1] = lcrc&0xFF;

    cc2500.Strobe(CC2500_SIDLE);
    cc2500.Strobe(CC2500_SFTX);
    if (get_fcc_test() >= 0) {
        // in negative FCC test modes we don't write to the FIFO, which gives
        // continuous transmission
        cc2500.WriteFifo((const uint8_t *)&pkt, sizeof(pkt));
    }
    cc2500.Strobe(CC2500_STX);
}

/*
  send a fwupload ack if needed
 */
void AP_Radio_cc2500::check_fw_ack(void)
{
    if (fwupload.need_ack && sem->take_nonblocking()) {
        // ack the send of a DATA96 fw packet to TX
        fwupload.need_ack = false;
        uint8_t data16[16] {};
        uint32_t ack_to = fwupload.offset + fwupload.acked;
        memcpy(&data16[0], &ack_to, 4);
        mavlink_msg_data16_send(fwupload.chan, 42, 4, data16);
        Debug(4,"sent ack DATA16\n");
        sem->give();
    }
}

#endif // HAL_RCINPUT_WITH_AP_RADIO
#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
