#include "AP_Radio_config.h"

#if AP_RADIO_CYRF6936_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include "AP_Radio_cypress.h"
#include <utility>
#include <stdio.h>
#include <StorageManager/StorageManager.h>
#include <AP_HAL/utility/dsm.h>
#include <AP_Math/crc.h>
#include "telem_structure.h"
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>

/*
  driver for CYRF6936 radio

  Many thanks to the SuperBitRF project from Paparrazi for their DSM
  configuration code and register defines
   https://github.com/esden/superbitrf-firmware
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define TIMEOUT_PRIORITY 181
#define EVT_TIMEOUT EVENT_MASK(0)
#define EVT_IRQ EVENT_MASK(1)
#endif

#ifndef CYRF_SPI_DEVICE
# define CYRF_SPI_DEVICE "cypress"
#endif

#ifndef CYRF_IRQ_INPUT
# define CYRF_IRQ_INPUT (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)
#endif

#ifndef CYRF_RESET_PIN
# define CYRF_RESET_PIN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_EXTI|GPIO_PORTB|GPIO_PIN0)
#endif

extern const AP_HAL::HAL& hal;

#define Debug(level, fmt, args...)   do { if ((level) <= get_debug_level()) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ##args); }} while (0)

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

/* The SPI interface defines */
enum {
    CYRF_CHANNEL            = 0x00,
    CYRF_TX_LENGTH          = 0x01,
    CYRF_TX_CTRL            = 0x02,
    CYRF_TX_CFG             = 0x03,
    CYRF_TX_IRQ_STATUS      = 0x04,
    CYRF_RX_CTRL            = 0x05,
    CYRF_RX_CFG             = 0x06,
    CYRF_RX_IRQ_STATUS      = 0x07,
    CYRF_RX_STATUS          = 0x08,
    CYRF_RX_COUNT           = 0x09,
    CYRF_RX_LENGTH          = 0x0A,
    CYRF_PWR_CTRL           = 0x0B,
    CYRF_XTAL_CTRL          = 0x0C,
    CYRF_IO_CFG             = 0x0D,
    CYRF_GPIO_CTRL          = 0x0E,
    CYRF_XACT_CFG           = 0x0F,
    CYRF_FRAMING_CFG        = 0x10,
    CYRF_DATA32_THOLD       = 0x11,
    CYRF_DATA64_THOLD       = 0x12,
    CYRF_RSSI               = 0x13,
    CYRF_EOP_CTRL           = 0x14,
    CYRF_CRC_SEED_LSB       = 0x15,
    CYRF_CRC_SEED_MSB       = 0x16,
    CYRF_TX_CRC_LSB         = 0x17,
    CYRF_TX_CRC_MSB         = 0x18,
    CYRF_RX_CRC_LSB         = 0x19,
    CYRF_RX_CRC_MSB         = 0x1A,
    CYRF_TX_OFFSET_LSB      = 0x1B,
    CYRF_TX_OFFSET_MSB      = 0x1C,
    CYRF_MODE_OVERRIDE      = 0x1D,
    CYRF_RX_OVERRIDE        = 0x1E,
    CYRF_TX_OVERRIDE        = 0x1F,
    CYRF_TX_BUFFER          = 0x20,
    CYRF_RX_BUFFER          = 0x21,
    CYRF_SOP_CODE           = 0x22,
    CYRF_DATA_CODE          = 0x23,
    CYRF_PREAMBLE           = 0x24,
    CYRF_MFG_ID             = 0x25,
    CYRF_XTAL_CFG           = 0x26,
    CYRF_CLK_OFFSET         = 0x27,
    CYRF_CLK_EN             = 0x28,
    CYRF_RX_ABORT           = 0x29,
    CYRF_AUTO_CAL_TIME      = 0x32,
    CYRF_AUTO_CAL_OFFSET    = 0x35,
    CYRF_ANALOG_CTRL        = 0x39,
};
#define CYRF_DIR                (1<<7) /**< Bit for enabling writing */

// CYRF_MODE_OVERRIDE
#define CYRF_RST                (1<<0)

// CYRF_CLK_EN
#define CYRF_RXF                (1<<1)

// CYRF_XACT_CFG
enum {
    CYRF_MODE_SLEEP     = (0x0<<2),
    CYRF_MODE_IDLE      = (0x1<<2),
    CYRF_MODE_SYNTH_TX  = (0x2<<2),
    CYRF_MODE_SYNTH_RX  = (0x3<<2),
    CYRF_MODE_RX        = (0x4<<2),
};
#define CYRF_FRC_END      (1<<5)
#define CYRF_ACK_EN       (1<<7)

// CYRF_IO_CFG
#define CYRF_IRQ_GPIO     (1<<0)
#define CYRF_SPI_3PIN     (1<<1)
#define CYRF_PACTL_GPIO   (1<<2)
#define CYRF_PACTL_OD     (1<<3)
#define CYRF_XOUT_OD      (1<<4)
#define CYRF_MISO_OD      (1<<5)
#define CYRF_IRQ_POL      (1<<6)
#define CYRF_IRQ_OD       (1<<7)

// CYRF_FRAMING_CFG
#define CYRF_LEN_EN       (1<<5)
#define CYRF_SOP_LEN      (1<<6)
#define CYRF_SOP_EN       (1<<7)

// CYRF_RX_STATUS
enum {
    CYRF_RX_DATA_MODE_GFSK  = 0x00,
    CYRF_RX_DATA_MODE_8DR   = 0x01,
    CYRF_RX_DATA_MODE_DDR   = 0x10,
    CYRF_RX_DATA_MODE_NV    = 0x11,
};
#define CYRF_RX_CODE            (1<<2)
#define CYRF_BAD_CRC     (1<<3)
#define CYRF_CRC0        (1<<4)
#define CYRF_EOP_ERR     (1<<5)
#define CYRF_PKT_ERR     (1<<6)
#define CYRF_RX_ACK      (1<<7)

// CYRF_TX_IRQ_STATUS
#define CYRF_TXE_IRQ     (1<<0)
#define CYRF_TXC_IRQ     (1<<1)
#define CYRF_TXBERR_IRQ  (1<<2)
#define CYRF_TXB0_IRQ    (1<<3)
#define CYRF_TXB8_IRQ    (1<<4)
#define CYRF_TXB15_IRQ   (1<<5)
#define CYRF_LV_IRQ      (1<<6)
#define CYRF_OS_IRQ      (1<<7)

// CYRF_RX_IRQ_STATUS
#define CYRF_RXE_IRQ     (1<<0)
#define CYRF_RXC_IRQ     (1<<1)
#define CYRF_RXBERR_IRQ  (1<<2)
#define CYRF_RXB1_IRQ    (1<<3)
#define CYRF_RXB8_IRQ    (1<<4)
#define CYRF_RXB16_IRQ   (1<<5)
#define CYRF_SOPDET_IRQ  (1<<6)
#define CYRF_RXOW_IRQ    (1<<7)

// CYRF_TX_CTRL
#define CYRF_TXE_IRQEN    (1<<0)
#define CYRF_TXC_IRQEN    (1<<1)
#define CYRF_TXBERR_IRQEN (1<<2)
#define CYRF_TXB0_IRQEN   (1<<3)
#define CYRF_TXB8_IRQEN   (1<<4)
#define CYRF_TXB15_IRQEN  (1<<5)
#define CYRF_TX_CLR       (1<<6)
#define CYRF_TX_GO        (1<<7)

// CYRF_RX_CTRL
#define CYRF_RXE_IRQEN    (1<<0)
#define CYRF_RXC_IRQEN    (1<<1)
#define CYRF_RXBERR_IRQEN (1<<2)
#define CYRF_RXB1_IRQEN   (1<<3)
#define CYRF_RXB8_IRQEN   (1<<4)
#define CYRF_RXB16_IRQEN  (1<<5)
#define CYRF_RSVD         (1<<6)
#define CYRF_RX_GO        (1<<7)

// CYRF_RX_OVERRIDE
#define CYRF_ACE          (1<<1)
#define CYRF_DIS_RXCRC    (1<<2)
#define CYRF_DIS_CRC0     (1<<3)
#define CYRF_FRC_RXDR     (1<<4)
#define CYRF_MAN_RXACK    (1<<5)
#define CYRF_RXTX_DLY     (1<<6)
#define CYRF_ACK_RX       (1<<7)

// CYRF_TX_OVERRIDE
#define CYRF_TX_INV       (1<<0)
#define CYRF_DIS_TXCRC    (1<<2)
#define CYRF_OVRD_ACK     (1<<3)
#define CYRF_MAN_TXACK    (1<<4)
#define CYRF_FRC_PRE      (1<<6)
#define CYRF_ACK_TX       (1<<7)

// CYRF_RX_CFG
#define CYRF_VLD_EN       (1<<0)
#define CYRF_RXOW_EN      (1<<1)
#define CYRF_FAST_TURN_EN (1<<3)
#define CYRF_HILO         (1<<4)
#define CYRF_ATT          (1<<5)
#define CYRF_LNA          (1<<6)
#define CYRF_AGC_EN       (1<<7)

// CYRF_TX_CFG
enum {
    CYRF_PA_M35      = 0x0,
    CYRF_PA_M30      = 0x1,
    CYRF_PA_M24      = 0x2,
    CYRF_PA_M18      = 0x3,
    CYRF_PA_M13      = 0x4,
    CYRF_PA_M5       = 0x5,
    CYRF_PA_0        = 0x6,
    CYRF_PA_4        = 0x7,
};
enum {
    CYRF_DATA_MODE_GFSK   = (0x0 <<3),
    CYRF_DATA_MODE_8DR    = (0x1 <<3),
    CYRF_DATA_MODE_DDR    = (0x2 <<3),
    CYRF_DATA_MODE_SDR    = (0x3 <<3),
};
#define CYRF_DATA_CODE_LENGTH    (1<<5)


#define FLAG_WRITE      0x80
#define FLAG_AUTO_INC   0x40

#define DSM_MAX_CHANNEL 0x4F

#define DSM_SCAN_MIN_CH 8
#define DSM_SCAN_MID_CH 40
#define DSM_SCAN_MAX_CH 70

#define FCC_SUPPORT_CW_MODE 0

#define AUTOBIND_CHANNEL 12

// object instance for trampoline
AP_Radio_cypress *AP_Radio_cypress::radio_singleton;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
thread_t *AP_Radio_cypress::_irq_handler_ctx;
#endif
/*
  constructor
 */
AP_Radio_cypress::AP_Radio_cypress(AP_Radio &_radio) :
    AP_Radio_backend(_radio)
{
    // link to instance for irq_trampoline
    radio_singleton = this;
}

/*
  initialise radio
 */
bool AP_Radio_cypress::init(void)
{
    dev = hal.spi->get_device(CYRF_SPI_DEVICE);
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if (_irq_handler_ctx != nullptr) {
        AP_HAL::panic("AP_Radio_cypress: double instantiation of irq_handler\n");
    }
    chVTObjectInit(&timeout_vt);
    _irq_handler_ctx = chThdCreateFromHeap(NULL,
                                           THD_WORKING_AREA_SIZE(2048),
                                           "radio_cypress",
                                           TIMEOUT_PRIORITY,
                                           irq_handler_thd,
                                           NULL);
#endif
    load_bind_info();

    return reset();
}

/*
  reset radio
 */
bool AP_Radio_cypress::reset(void)
{
    dev->get_semaphore()->take_blocking();

    /*
      to reset radio hold reset high for 0.5s, then low for 0.5s
     */
#if defined(HAL_GPIO_RADIO_RESET)
    hal.scheduler->expect_delay_ms(2000); // avoid main-loop-delay internal error
    hal.gpio->write(HAL_GPIO_RADIO_RESET, 1);
    hal.scheduler->delay(500);
    hal.gpio->write(HAL_GPIO_RADIO_RESET, 0);
    hal.scheduler->delay(500);
#endif
    radio_init();
    dev->get_semaphore()->give();

    if (dsm.protocol == DSM_NONE &&
        get_autobind_time() == 0) {
        start_recv_bind();
    }

    return true;
}

/*
  return statistics structure from radio
 */
const AP_Radio::stats &AP_Radio_cypress::get_stats(void)
{
    return stats;
}

/*
  read one pwm channel from radio
 */
uint16_t AP_Radio_cypress::read(uint8_t chan)
{
    if (dsm.need_bind_save) {
        save_bind_info();
    }
    if (chan >= max_channels) {
        return 0;
    }
    return dsm.pwm_channels[chan];
}

/*
  update status - called from main thread
 */
void AP_Radio_cypress::update(void)
{
    check_fw_ack();
}


/*
  print one second debug info
 */
void AP_Radio_cypress::print_debug_info(void)
{
    Debug(2, "recv:%3u bad:%3u to:%3u re:%u N:%2u TXI:%u TX:%u 1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u 7:%4u 8:%4u 14:%u\n",
          unsigned(stats.recv_packets - last_stats.recv_packets),
          unsigned(stats.bad_packets - last_stats.bad_packets),
          unsigned(stats.timeouts - last_stats.timeouts),
          unsigned(stats.recv_errors - last_stats.recv_errors),
          num_channels(),
          unsigned(dsm.send_irq_count),
          unsigned(dsm.send_count),
          dsm.pwm_channels[0], dsm.pwm_channels[1], dsm.pwm_channels[2], dsm.pwm_channels[3],
          dsm.pwm_channels[4], dsm.pwm_channels[5], dsm.pwm_channels[6], dsm.pwm_channels[7],
          dsm.pwm_channels[13]);
}

/*
  return number of active channels
 */
uint8_t AP_Radio_cypress::num_channels(void)
{
    uint32_t now = AP_HAL::millis();
    uint8_t chan = get_rssi_chan();
    if (chan > 0) {
        dsm.pwm_channels[chan-1] = dsm.rssi;
        dsm.num_channels = MAX(dsm.num_channels, chan);
    }

    chan = get_pps_chan();
    if (chan > 0) {
        dsm.pwm_channels[chan-1] = t_status.pps;
        dsm.num_channels = MAX(dsm.num_channels, chan);
    }

    chan = get_tx_rssi_chan();
    if (chan > 0) {
        dsm.pwm_channels[chan-1] = dsm.tx_rssi;
        dsm.num_channels = MAX(dsm.num_channels, chan);
    }

    chan = get_tx_pps_chan();
    if (chan > 0) {
        dsm.pwm_channels[chan-1] = dsm.tx_pps;
        dsm.num_channels = MAX(dsm.num_channels, chan);
    }

    if (now - last_debug_print_ms > 1000) {
        last_debug_print_ms = now;
        if (get_debug_level() > 1) {
            print_debug_info();
        }

        t_status.pps = stats.recv_packets - last_stats.recv_packets;
        t_status.rssi = (uint8_t)dsm.rssi;
        last_stats = stats;
    }

    return dsm.num_channels;
}

/*
  send a fwupload ack if needed
 */
void AP_Radio_cypress::check_fw_ack(void)
{
    Debug(4,"check need_ack\n");
    if (fwupload.need_ack && sem.take_nonblocking()) {
        // ack the send of a DATA96 fw packet to TX
        fwupload.need_ack = false;
        uint8_t data16[16] {};
        uint32_t ack_to = fwupload.offset + fwupload.acked;
        memcpy(&data16[0], &ack_to, 4);
        mavlink_msg_data16_send(fwupload.chan, 42, 4, data16);
        Debug(4,"sent ack DATA16\n");
        sem.give();
    }
}

/*
  return time of last receive in microseconds
 */
uint32_t AP_Radio_cypress::last_recv_us(void)
{
    // we use the parse time, so it matches when channel values are filled in
    return dsm.last_parse_us;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_cypress::send(const uint8_t *pkt, uint16_t len)
{
    // disabled for now
    return false;
}

/* The PN codes */
const uint8_t AP_Radio_cypress::pn_codes[5][9][8] = {
    { /* Row 0 */
        /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
        /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
        /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
        /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
        /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
        /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
        /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
        /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
        /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
    },
    { /* Row 1 */
        /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
        /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
        /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
        /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
        /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
        /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
        /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
        /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
        /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
    },
    { /* Row 2 */
        /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
        /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
        /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
        /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
        /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
        /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
        /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
        /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
        /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
    },
    { /* Row 3 */
        /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
        /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
        /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
        /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
        /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
        /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
        /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
        /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
        /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
    },
    { /* Row 4 */
        /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
        /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
        /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
        /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
        /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
        /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
        /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
        /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
        /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
    },
};
const uint8_t AP_Radio_cypress::pn_bind[] = { 0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84 };

/*The CYRF initial config, binding config and transfer config */
const AP_Radio_cypress::config AP_Radio_cypress::cyrf_config[] = {
    {CYRF_MODE_OVERRIDE, CYRF_RST},                                         // Reset the device
    {CYRF_CLK_EN, CYRF_RXF},                                                // Enable the clock
    {CYRF_AUTO_CAL_TIME, 0x3C},                                             // From manual, needed for initialization
    {CYRF_AUTO_CAL_OFFSET, 0x14},                                           // From manual, needed for initialization
    {CYRF_RX_CFG, CYRF_LNA | CYRF_FAST_TURN_EN},                            // Enable low noise amplifier and fast turning
    {CYRF_TX_OFFSET_LSB, 0x55},                                             // From manual, typical configuration
    {CYRF_TX_OFFSET_MSB, 0x05},                                             // From manual, typical configuration
    {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},                     // Force in Synth RX mode
    {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
    {CYRF_DATA64_THOLD, 0x0E},                                              // From manual, typical configuration
    {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX},                                    // Set in Synth RX mode (again, really needed?)
    {CYRF_IO_CFG, CYRF_IRQ_POL},                                            // IRQ active high
};

const AP_Radio_cypress::config AP_Radio_cypress::cyrf_bind_config[] = {
    {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},   // Enable 64 chip codes, SDR mode and amplifier +4dBm
    {CYRF_FRAMING_CFG, CYRF_SOP_LEN | 0xE},                                  // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
    {CYRF_RX_OVERRIDE, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},                      // Force receive data rate and disable receive CRC checker
    {CYRF_EOP_CTRL, 0x02},                                                   // Only enable EOP symbol count of 2
    {CYRF_TX_OVERRIDE, CYRF_DIS_TXCRC},                                      // Disable transmit CRC generate
};
const AP_Radio_cypress::config AP_Radio_cypress::cyrf_transfer_config[] = {
    {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_4},   // Enable 64 chip codes, 8DR mode and amplifier +4dBm
    {CYRF_FRAMING_CFG, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},      // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
    {CYRF_TX_OVERRIDE, 0x00},                                                // Reset TX overrides
    {CYRF_RX_OVERRIDE, 0x00},                                                // Reset RX overrides
};

/*
  read radio status, handling the race condition between completion and error
 */
uint8_t AP_Radio_cypress::read_status_debounced(uint8_t adr)
{
    uint8_t ret;

    dev->set_chip_select(true);
    ret = read_register(adr);

    // If COMPLETE and ERROR bits mismatch, then re-read register
    if ((ret & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) != 0
        && (ret & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) != (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) {
        uint8_t v2;
        dev->read(&v2, 1);
        ret |= v2;   // re-read and make bits sticky
    }
    dev->set_chip_select(false);
    return ret;
}

/*
  force the initial state of the radio
 */
void AP_Radio_cypress::force_initial_state(void)
{
    while (true) {
        write_register(CYRF_XACT_CFG, CYRF_FRC_END);
        uint32_t start_ms = AP_HAL::millis();
        do {
            if ((read_register(CYRF_XACT_CFG) & CYRF_FRC_END) == 0) {
                return;                     // FORCE_END done (osc running)
            }
        } while (AP_HAL::millis() - start_ms < 5);

        // FORCE_END failed to complete, implying going SLEEP to IDLE and
        // oscillator failed to start.  Recover by returning to SLEEP and
        //  trying to start oscillator again.
        write_register(CYRF_XACT_CFG, CYRF_MODE_SLEEP);
    }
}

/*
  set desired channel
 */
void AP_Radio_cypress::set_channel(uint8_t channel)
{
    if (dsm.forced_channel != -1) {
        channel = dsm.forced_channel;
    }
    write_register(CYRF_CHANNEL, channel);
}

void AP_Radio_cypress::radio_set_config(const struct config *conf, uint8_t size)
{
    // setup required radio config
    for (uint8_t i=0; i<size; i++) {
        write_register(conf[i].reg, conf[i].value);
    }
}

/*
  initialise the radio
 */
void AP_Radio_cypress::radio_init(void)
{
    Debug(1, "Cypress: radio_init starting\n");

    // wait for radio to settle
    uint16_t i;
    for (i=0; i<1000; i++) {
        uint8_t chan = read_register(CYRF_CHANNEL);
        if (chan == 1) {
            break;
        }
        write_register(CYRF_CHANNEL, 1);
        hal.scheduler->delay(10);
    }
    if (i == 1000) {
        Debug(1, "Cypress: radio_init failed\n");
        return;
    }

    // base config
    radio_set_config(cyrf_config, ARRAY_SIZE(cyrf_config));

    // start with receive config
    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

    if (get_disable_crc()) {
        write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
    }

    dsm_setup_transfer_dsmx();

    write_register(CYRF_XTAL_CTRL,0x80);  // XOUT=BitSerial
    force_initial_state();
    write_register(CYRF_PWR_CTRL,0x20);   // Disable PMU

    // start in RECV state
    state = STATE_RECV;

    Debug(1, "Cypress: radio_init done\n");

    start_receive();

    // setup handler for rising edge of IRQ pin
    hal.gpio->attach_interrupt(HAL_GPIO_RADIO_IRQ, trigger_irq_radio_event, AP_HAL::GPIO::INTERRUPT_RISING);
}

void AP_Radio_cypress::dump_registers(uint8_t n)
{
    for (uint8_t i=0; i<n; i++) {
        uint8_t v = read_register(i);
        printf("%02x:%02x ", i, v);
        if ((i+1) % 16 == 0) {
            printf("\n");
        }
    }
    if (n % 16 != 0) {
        printf("\n");
    }
}

/*
  read one register value
 */
uint8_t AP_Radio_cypress::read_register(uint8_t reg)
{
    uint8_t v = 0;
    (void)dev->read_registers(reg, &v, 1);
    return v;
}


/*
  write multiple bytes
 */
void AP_Radio_cypress::write_multiple(uint8_t reg, uint8_t n, const uint8_t *data)
{
    uint8_t pkt[n+1];
    pkt[0] = reg | FLAG_WRITE;
    memcpy(&pkt[1], data, n);
    dev->transfer(pkt, n+1, nullptr, 0);
}

/*
  write one register value
 */
void AP_Radio_cypress::write_register(uint8_t reg, uint8_t value)
{
    dev->write_register(reg | FLAG_WRITE, value);
}


/*
  support all 4 rc input modes by swapping channels.
 */
void AP_Radio_cypress::map_stick_mode(uint16_t *channels)
{
    switch (get_stick_mode()) {
    case 1: {
        // mode1
        uint16_t tmp = channels[1];
        channels[1] = 3000 - channels[2];
        channels[2] = 3000 - tmp;
        break;
    }

    case 3: {
        // mode3
        uint16_t tmp = channels[1];
        channels[1] = 3000 - channels[2];
        channels[2] = 3000 - tmp;
        tmp = channels[0];
        channels[0] = channels[3];
        channels[3] = tmp;
        break;
    }

    case 4: {
        // mode4
        uint16_t tmp = channels[0];
        channels[0] = channels[3];
        channels[3] = tmp;
        break;
    }

    case 2:
    default:
        // nothing to do, transmitter is natively mode2
        break;
    }
}

/*
  check if we are the 2nd RX bound to this TX
 */
void AP_Radio_cypress::check_double_bind(void)
{
    if (dsm.tx_pps <= dsm.telem_send_count ||
        get_autobind_time() == 0) {
        return;
    }
    // the TX has received more telemetry packets in the last second
    // than we have ever sent. There must be another RX sending
    // telemetry packets. We will reset our mfg_id and go back waiting
    // for a new bind packet, hopefully with the right TX
    Debug(1,"Double-bind detected\n");
    memset(dsm.mfg_id, 1, sizeof(dsm.mfg_id));
    dsm.last_recv_us = 0;
    dsm_setup_transfer_dsmx();
}

/*
  parse channels from a packet
 */
bool AP_Radio_cypress::parse_dsm_channels(const uint8_t *data)
{
    uint16_t num_values = 0;
    uint16_t pwm_channels[max_channels] {};

    // default value for channels above 4 is previous value
    memcpy(&pwm_channels[4], &dsm.pwm_channels[4], (max_channels-4)*sizeof(uint16_t));

    if (!dsm_decode(AP_HAL::micros64(),
                    data,
                    pwm_channels,
                    &num_values,
                    ARRAY_SIZE(pwm_channels))) {
        // invalid packet
        Debug(2, "DSM: bad decode\n");
        return false;
    }
    if (num_values < 5) {
        Debug(2, "DSM: num_values=%u\n", num_values);
        return false;
    }

    // cope with mode1/mode2
    map_stick_mode(pwm_channels);

    memcpy(dsm.pwm_channels, pwm_channels, num_values*sizeof(uint16_t));

    dsm.last_parse_us = AP_HAL::micros();

    // suppress channel 8 ack values
    dsm.num_channels = num_values==8?7:num_values;

    if (num_values == 8) {
        // decode telemetry ack value and version
        uint16_t d=0;
        if (is_DSM2()) {
            d = data[14] << 8 | data[15];
        } else {
            // see chan_order[] for DSMX
            d = data[10] << 8 | data[11];
        }
        // extra data is sent on channel 8, with 3 bit key and 8 bit data
        uint8_t chan = d>>11;
        uint8_t key = (d >> 8) & 0x7;
        uint8_t v = d & 0xFF;
        if (chan == 7 && key == 0) {
            // got an ack from key 0
            Debug(4, "ack %u seq=%u acked=%u length=%u len=%u\n",
                  v, fwupload.sequence, unsigned(fwupload.acked), unsigned(fwupload.length), fwupload.len);
            if (fwupload.sequence == v && sem.take_nonblocking()) {
                fwupload.sequence++;
                fwupload.acked += fwupload.len;
                if (fwupload.acked == fwupload.length) {
                    // trigger send of DATA16 ack to client
                    fwupload.need_ack = true;
                }
                sem.give();
            }
        }
        if (chan == 7) {
            // extract telemetry extra data
            switch (key) {
            case 1:
                dsm.tx_firmware_year = v;
                break;
            case 2:
                dsm.tx_firmware_month = v;
                break;
            case 3:
                dsm.tx_firmware_day = v;
                break;
            case 4:
                dsm.tx_rssi = v;
                break;
            case 5:
                dsm.tx_pps = v;
                dsm.have_tx_pps = true;
                check_double_bind();
                break;
            case 6:
                if (v != dsm.tx_bl_version) {
                    if (v == 2) {
                        // TX with new filter gets a default power of 6
                        set_tx_max_power_default(6);
                    }
                }
                dsm.tx_bl_version = v;
                break;
            }
        }
    }
    return true;
}

/*
  process an incoming bind packet
 */
void AP_Radio_cypress::process_bind(const uint8_t *pkt, uint8_t len)
{
    if (len != 16) {
        return;
    }
    bool ok = (len==16 && pkt[0] == pkt[4] && pkt[1] == pkt[5] && pkt[2] == pkt[6] && pkt[3] == pkt[7]);

    // Calculate the first sum
    uint16_t bind_sum = 384 - 0x10;
    for (uint8_t i = 0; i < 8; i++) {
        bind_sum += pkt[i];
    }

    // Check the first sum
    if (pkt[8] != bind_sum >> 8 || pkt[9] != (bind_sum & 0xFF)) {
        ok = false;
    }

    // Calculate second sum
    for (uint8_t i = 8; i < 14; i++) {
        bind_sum += pkt[i];
    }

    // Check the second sum
    if (pkt[14] != bind_sum >> 8 || pkt[15] != (bind_sum & 0xFF)) {
        ok = false;
    }

    if (state == STATE_AUTOBIND) {
        uint8_t rssi = read_register(CYRF_RSSI) & 0x1F;
        Debug(3,"bind RSSI %u\n", rssi);
        if (rssi < get_autobind_rssi()) {
            ok = false;
        }
    }

    if (ok) {
        uint8_t mfg_id[4] = {uint8_t(~pkt[0]), uint8_t(~pkt[1]), uint8_t(~pkt[2]), uint8_t(~pkt[3])};
        uint8_t num_chan = pkt[11];
        uint8_t protocol = pkt[12];
        (void)num_chan;
        // change to normal receive
        memcpy(dsm.mfg_id, mfg_id, 4);
        state = STATE_RECV;

        radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

        if (get_disable_crc()) {
            write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
        }

        dsm.protocol = (enum dsm_protocol)protocol;
        dsm_setup_transfer_dsmx();

        Debug(1, "BIND OK: mfg_id={0x%02x, 0x%02x, 0x%02x, 0x%02x} N=%u P=0x%02x DSM2=%u\n",
              mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3],
              num_chan,
              protocol,
              is_DSM2());

        dsm.last_recv_us = AP_HAL::micros();

        if (is_DSM2()) {
            dsm2_start_sync();
        }

        dsm.need_bind_save = true;
    }
}

/*
  start DSM2 sync
 */
void AP_Radio_cypress::dsm2_start_sync(void)
{
    uint8_t factory_test = get_factory_test();
    if (factory_test != 0) {
        dsm.channels[0] = (factory_test*7) % DSM_MAX_CHANNEL;
        dsm.channels[1] = (dsm.channels[0] + 5) % DSM_MAX_CHANNEL;
        dsm.sync = DSM2_OK;
    } else {
        Debug(2, "DSM2 start sync\n");
        dsm.sync = DSM2_SYNC_A;
    }
}

/*
  setup a timeout in timeout_ms milliseconds
 */
void AP_Radio_cypress::setup_timeout(uint32_t timeout_ms)
{
    chVTSet(&timeout_vt, chTimeMS2I(timeout_ms), trigger_timeout_event, nullptr);
}

/*
  process an incoming packet
 */
void AP_Radio_cypress::process_packet(const uint8_t *pkt, uint8_t len)
{
    if (len == 16) {
        bool ok;
        const uint8_t *id = dsm.mfg_id;
        uint32_t now = AP_HAL::micros();

        if (is_DSM2()) {
            ok = (pkt[0] == ((~id[2])&0xFF) && pkt[1] == (~id[3]&0xFF));
        } else {
            ok = (pkt[0] == id[2] && pkt[1] == id[3]);
        }
        if (ok && is_DSM2() && dsm.sync < DSM2_OK) {
            if (dsm.sync == DSM2_SYNC_A) {
                dsm.channels[0] = dsm.current_rf_channel;
                dsm.sync = DSM2_SYNC_B;
                Debug(2, "DSM2 SYNCA chan=%u\n", dsm.channels[0]);
                dsm.last_recv_us = now;
            } else {
                if (dsm.current_rf_channel != dsm.channels[0]) {
                    dsm.channels[1] = dsm.current_rf_channel;
                    dsm.sync = DSM2_OK;
                    Debug(2, "DSM2 SYNCB chan=%u\n", dsm.channels[1]);
                    dsm.last_recv_us = now;
                }
            }
            return;
        }
        if (ok && (!is_DSM2() || dsm.sync >= DSM2_SYNC_B)) {
            ok = parse_dsm_channels(pkt);
        }
        if (ok) {
            uint32_t packet_dt_us = now - dsm.last_recv_us;

            dsm.last_recv_chan = dsm.current_channel;
            dsm.last_recv_us = now;
            if (dsm.crc_errors > 2) {
                dsm.crc_errors -= 2;
            }

            stats.recv_packets++;

            // sample the RSSI
            uint8_t rssi = read_register(CYRF_RSSI) & 0x1F;
            dsm.rssi = 0.95 * dsm.rssi + 0.05 * rssi;

            if (packet_dt_us < 5000) {
                dsm.pkt_time1 = packet_dt_us;
            } else if (packet_dt_us < 8000) {
                dsm.pkt_time2 = packet_dt_us;
            }

            if (get_telem_enable()) {
                if (packet_dt_us < 5000 &&
                    (get_autobind_time() == 0 || dsm.have_tx_pps)) {
                    /*
                      we have just received two packets rapidly, which
                      means we have about 7ms before the next
                      one. That gives time for a telemetry packet. We
                      send it 1ms after we receive the incoming packet

                      If auto-bind is enabled we don't send telemetry
                      till we've received a tx_pps value from the
                      TX. This allows us to detect double binding (two
                      RX bound to the same TX)
                    */
                    state = STATE_SEND_TELEM;
                    setup_timeout(1);
                }
            }
        } else {
            stats.bad_packets++;
        }
    } else {
        stats.bad_packets++;
    }
}


/*
  start packet receive
 */
void AP_Radio_cypress::start_receive(void)
{
    dsm_choose_channel();

    write_register(CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ);
    write_register(CYRF_RX_CTRL, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN);

    dsm.receive_start_us = AP_HAL::micros();
    if (state == STATE_AUTOBIND) {
        dsm.receive_timeout_msec = 90;
    } else if (state == STATE_BIND) {
        dsm.receive_timeout_msec = 15;
    } else {
        dsm.receive_timeout_msec = 12;
    }
    setup_timeout(dsm.receive_timeout_msec);
}

/*
  handle a receive IRQ
 */
void AP_Radio_cypress::irq_handler_recv(uint8_t rx_status)
{
    if ((rx_status & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) == 0) {
        // nothing interesting yet
        return;
    }

    uint8_t pkt[16];
    uint8_t rlen = read_register(CYRF_RX_COUNT);
    if (rlen > 16) {
        rlen = 16;
    }
    if (rlen > 0) {
        dev->read_registers(CYRF_RX_BUFFER, pkt, rlen);
    }

    if (rx_status & CYRF_RXE_IRQ) {
        uint8_t reason = read_register(CYRF_RX_STATUS);
        if (reason & CYRF_BAD_CRC) {
            dsm.crc_errors++;
            if (dsm.crc_errors > 20) {
                Debug(2, "Flip CRC\n");
                // flip crc seed, this allows us to resync with transmitter
                dsm.crc_seed = ~dsm.crc_seed;
                dsm.crc_errors = 0;
            }
        }
        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
        stats.recv_errors++;
    } else if (rx_status & CYRF_RXC_IRQ) {
        if (state == STATE_RECV) {
            process_packet(pkt, rlen);
        } else {
            process_bind(pkt, rlen);
        }
    }

    if (state == STATE_AUTOBIND) {
        state = STATE_RECV;
    }

    if (state != STATE_SEND_TELEM) {
        start_receive();
    }
}


/*
  handle a send IRQ
 */
void AP_Radio_cypress::irq_handler_send(uint8_t tx_status)
{
    if ((tx_status & (CYRF_TXC_IRQ | CYRF_TXE_IRQ)) == 0) {
        // nothing interesting yet
        return;
    }
    state = STATE_RECV;
    start_receive();
}


/*
  IRQ handler
 */
void AP_Radio_cypress::irq_handler(void)
{
    //hal.console->printf("IRQ\n");
    if (!dev->get_semaphore()->take_nonblocking()) {
        // we have to wait for timeout instead
        return;
    }
    // always read both rx and tx status. This ensure IRQ is cleared
    uint8_t rx_status = read_status_debounced(CYRF_RX_IRQ_STATUS);
    uint8_t tx_status = read_status_debounced(CYRF_TX_IRQ_STATUS);

    switch (state) {
    case STATE_AUTOBIND:
    // fallthrough
    case STATE_RECV:
    case STATE_BIND:
        irq_handler_recv(rx_status);
        break;

    case STATE_SEND_TELEM:
    case STATE_SEND_TELEM_WAIT:
        irq_handler_send(tx_status);
        break;

    case STATE_SEND_FCC:
        // stop transmit oscillator
        write_register(CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ);
        write_register(CYRF_RX_CTRL, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN);
        break;

    default:
        break;
    }
    dev->get_semaphore()->give();
}

/*
  called on radio timeout
 */
void AP_Radio_cypress::irq_timeout(void)
{
    stats.timeouts++;
    if (!dev->get_semaphore()->take_nonblocking()) {
        // schedule a new timeout
        setup_timeout(dsm.receive_timeout_msec);
        return;
    }

    if (get_fcc_test() != 0 && state != STATE_SEND_FCC) {
        Debug(3,"Starting FCC test\n");
        state = STATE_SEND_FCC;
    } else if (get_fcc_test() == 0 && state == STATE_SEND_FCC) {
        Debug(3,"Ending FCC test\n");
        state = STATE_RECV;
    }

    switch (state) {
    case STATE_SEND_TELEM:
        send_telem_packet();
        break;
    case STATE_SEND_FCC:
        send_FCC_test_packet();
        break;
    case STATE_AUTOBIND:
    case STATE_SEND_TELEM_WAIT:
        state = STATE_RECV;
    // fall through
    default:
        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
        start_receive();
        break;
    }

    dev->get_semaphore()->give();
}


/*
  called on HRT timeout
 */
void AP_Radio_cypress::irq_handler_thd(void *arg)
{
    _irq_handler_ctx = chThdGetSelfX();
    (void)arg;
    while (true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
        if (evt & EVT_IRQ) {
            radio_singleton->irq_handler();
        }
        if (evt & EVT_TIMEOUT) {
            radio_singleton->irq_timeout();
        }
    }
}

void AP_Radio_cypress::trigger_timeout_event(virtual_timer_t* vt, void *arg)
{
    (void)arg;
    //we are called from ISR context
    chSysLockFromISR();
    if (_irq_handler_ctx) {
        chEvtSignalI(_irq_handler_ctx, EVT_TIMEOUT);
    }
    chSysUnlockFromISR();
}

void AP_Radio_cypress::trigger_irq_radio_event()
{
    //we are called from ISR context
    chSysLockFromISR();
    if (_irq_handler_ctx) {
        chEvtSignalI(_irq_handler_ctx, EVT_IRQ);
    }
    chSysUnlockFromISR();
}

/*
 Set the current DSM channel with SOP, CRC and data code
 */
void AP_Radio_cypress::dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed)
{
    //printf("dsm_set_channel: %u\n", channel);

    uint8_t pn_row;
    pn_row = is_dsm2? channel % 5 : (channel-2) % 5;

    // set CRC seed
    write_register(CYRF_CRC_SEED_LSB, crc_seed & 0xff);
    write_register(CYRF_CRC_SEED_MSB, crc_seed >> 8);

    // set start of packet code
    if (memcmp(dsm.last_sop_code, pn_codes[pn_row][sop_col], 8) != 0) {
        write_multiple(CYRF_SOP_CODE, 8, pn_codes[pn_row][sop_col]);
        memcpy(dsm.last_sop_code, pn_codes[pn_row][sop_col], 8);
    }

    // set data code
    if (memcmp(dsm.last_data_code, pn_codes[pn_row][data_col], 16) != 0) {
        write_multiple(CYRF_DATA_CODE, 16, pn_codes[pn_row][data_col]);
        memcpy(dsm.last_data_code, pn_codes[pn_row][data_col], 16);
    }

    if (get_disable_crc() != dsm.last_discrc) {
        dsm.last_discrc = get_disable_crc();
        Debug(3,"Cypress: DISCRC=%u\n", dsm.last_discrc);
        write_register(CYRF_RX_OVERRIDE, dsm.last_discrc?CYRF_DIS_RXCRC:0);
    }

    if (get_transmit_power() != dsm.last_transmit_power+1) {
        dsm.last_transmit_power = get_transmit_power()-1;
        Debug(3,"Cypress: TXPOWER=%u\n", dsm.last_transmit_power);
        write_register(CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | dsm.last_transmit_power);
    }

    // Change channel
    set_channel(channel);
}

/*
  Generate the DSMX channels from the manufacturer ID
 */
void AP_Radio_cypress::dsm_generate_channels_dsmx(uint8_t mfg_id[4], uint8_t channels[23])
{
    // Calculate the DSMX channels
    int idx = 0;
    uint32_t id = ~((mfg_id[0] << 24) | (mfg_id[1] << 16) |
                    (mfg_id[2] << 8) | (mfg_id[3] << 0));
    uint32_t id_tmp = id;

    // While not all channels are set
    while (idx < 23) {
        int i;
        int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;

        id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F; // Randomization
        uint8_t next_ch = ((id_tmp >> 8) % 0x49) + 3;       // Use least-significant byte and must be larger than 3
        if (((next_ch ^ id) & 0x01 ) == 0) {
            continue;
        }

        // Go trough all already set channels
        for (i = 0; i < idx; i++) {
            // Channel is already used
            if (channels[i] == next_ch) {
                break;
            }

            // Count the channel groups
            if (channels[i] <= 27) {
                count_3_27++;
            } else if (channels[i] <= 51) {
                count_28_51++;
            } else {
                count_52_76++;
            }
        }

        // When channel is already used continue
        if (i != idx) {
            continue;
        }

        // Set the channel when channel groups aren't full
        if ((next_ch < 28 && count_3_27 < 8)                        // Channels 3-27: max 8
            || (next_ch >= 28 && next_ch < 52 && count_28_51 < 7)    // Channels 28-52: max 7
            || (next_ch >= 52 && count_52_76 < 8)) {                // Channels 52-76: max 8
            channels[idx++] = next_ch;
        }
    }

    Debug(2, "Generated DSMX channels\n");
}

/*
  setup for DSMX transfers
 */
void AP_Radio_cypress::dsm_setup_transfer_dsmx(void)
{
    dsm.current_channel = 0;

    dsm.crc_seed = ~((dsm.mfg_id[0] << 8) + dsm.mfg_id[1]);
    dsm.sop_col = (dsm.mfg_id[0] + dsm.mfg_id[1] + dsm.mfg_id[2] + 2) & 0x07;
    dsm.data_col = 7 - dsm.sop_col;

    dsm_generate_channels_dsmx(dsm.mfg_id, dsm.channels);
}

/*
  choose channel to receive on
 */
void AP_Radio_cypress::dsm_choose_channel(void)
{
    uint32_t now = AP_HAL::micros();
    uint32_t dt = now - dsm.last_recv_us;
    const uint32_t cycle_time = dsm.pkt_time1 + dsm.pkt_time2;
    uint8_t next_channel;


    if (state == STATE_BIND) {
        if (now - dsm.last_chan_change_us > 15000) {
            // always use odd channel numbers for bind
            dsm.current_rf_channel |= 1;
            dsm.current_rf_channel = (dsm.current_rf_channel+2) % DSM_MAX_CHANNEL;
            dsm.last_chan_change_us = now;
        }
        set_channel(dsm.current_rf_channel);
        return;
    }

    if (get_autobind_time() != 0 &&
        dsm.last_recv_us == 0 &&
        now - dsm.last_autobind_send > 300*1000UL &&
        now > get_autobind_time() * 1000*1000UL &&
        get_factory_test() == 0 &&
        state == STATE_RECV) {
        // try to receive an auto-bind packet
        dsm_set_channel(AUTOBIND_CHANNEL, true, 0, 0, 0);

        state = STATE_AUTOBIND;

        Debug(3,"recv autobind %u\n", unsigned(now - dsm.last_autobind_send));
        dsm.last_autobind_send = now;
        return;
    }

    if (is_DSM2() && dsm.sync == DSM2_SYNC_A) {
        if (now - dsm.last_chan_change_us > 15000) {
            // only even channels for DSM2 scan
            dsm.current_rf_channel &= ~1;
            dsm.current_rf_channel = (dsm.current_rf_channel+2) % DSM_MAX_CHANNEL;
            dsm.last_chan_change_us = now;
        }
        //hal.console->printf("%u chan=%u\n", AP_HAL::micros(), dsm.current_rf_channel);
        dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                        dsm.sop_col, dsm.data_col,
                        dsm.sync==DSM2_SYNC_B?~dsm.crc_seed:dsm.crc_seed);
        return;
    }

    if (dt < 1000) {
        // normal channel advance
        next_channel = dsm.last_recv_chan + 1;
    } else if (dt > 20*cycle_time) {
        // change channel slowly
        next_channel = dsm.last_recv_chan + (dt / (cycle_time*2));
    } else {
        // predict next channel
        next_channel = dsm.last_recv_chan + 1;
        next_channel += (dt / cycle_time) * 2;
        if (dt % cycle_time > (unsigned)(dsm.pkt_time1 + 1000U)) {
            next_channel++;
        }
    }

    uint8_t chan_count = is_DSM2()?2:23;
    dsm.current_channel = next_channel;
    if (dsm.current_channel >= chan_count) {
        dsm.current_channel %= chan_count;
        if (!is_DSM2()) {
            dsm.crc_seed = ~dsm.crc_seed;
        }
    }

    if (is_DSM2() && dsm.sync == DSM2_SYNC_B && dsm.current_channel == 1) {
        // scan to next channelb
        do {
            dsm.channels[1] &= ~1;
            dsm.channels[1] = (dsm.channels[1]+2) % DSM_MAX_CHANNEL;
        } while (dsm.channels[1] == dsm.channels[0]);
    }

    dsm.current_rf_channel = dsm.channels[dsm.current_channel];

    uint16_t seed = dsm.crc_seed;
    if (dsm.current_channel & 1) {
        seed = ~seed;
    }

    if (is_DSM2()) {
        if (now - dsm.last_recv_us > 5000000) {
            dsm2_start_sync();
        }
    }

    dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                    dsm.sop_col, dsm.data_col, seed);
}

/*
  setup radio for bind
 */
void AP_Radio_cypress::start_recv_bind(void)
{
    dev->get_semaphore()->take_blocking();

    Debug(1, "Cypress: start_recv_bind\n");

    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);

    state = STATE_BIND;

    radio_set_config(cyrf_bind_config, ARRAY_SIZE(cyrf_bind_config));

    write_register(CYRF_CRC_SEED_LSB, 0);
    write_register(CYRF_CRC_SEED_MSB, 0);

    write_multiple(CYRF_SOP_CODE, 8, pn_codes[0][0]);

    uint8_t data_code[16];
    memcpy(data_code, pn_codes[0][8], 8);
    memcpy(&data_code[8], pn_bind, 8);
    write_multiple(CYRF_DATA_CODE, 16, data_code);

    dsm.current_rf_channel = 1;

    start_receive();

    dev->get_semaphore()->give();
}

/*
  save bind info
 */
void AP_Radio_cypress::save_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    info.magic = bind_magic;
    memcpy(info.mfg_id, dsm.mfg_id, sizeof(info.mfg_id));
    info.protocol = dsm.protocol;

    if (bind_storage.write_block(0, &info, sizeof(info))) {
        dsm.need_bind_save = false;
    }
}

/*
  load bind info
 */
void AP_Radio_cypress::load_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    uint8_t factory_test = get_factory_test();

    if (factory_test != 0) {
        Debug(1, "In factory test %u\n", factory_test);
        memset(dsm.mfg_id, 0, sizeof(dsm.mfg_id));
        dsm.mfg_id[0] = factory_test;
        dsm.protocol = DSM_DSM2_2;
        dsm2_start_sync();
    } else if (bind_storage.read_block(&info, 0, sizeof(info)) && info.magic == bind_magic) {
        Debug(1, "Loaded mfg_id %02x:%02x:%02x:%02x\n",
              info.mfg_id[0], info.mfg_id[1], info.mfg_id[2], info.mfg_id[3]);
        memcpy(dsm.mfg_id, info.mfg_id, sizeof(info.mfg_id));
        dsm.protocol = info.protocol;
    }
}

bool AP_Radio_cypress::is_DSM2(void)
{
    if (get_protocol() != AP_Radio::PROTOCOL_AUTO) {
        return get_protocol() == AP_Radio::PROTOCOL_DSM2;
    }
    return dsm.protocol == DSM_DSM2_1 || dsm.protocol == DSM_DSM2_2;
}

/*
  transmit a 16 byte packet
  this is a blind send, not waiting for ack or completion
*/
void AP_Radio_cypress::transmit16(const uint8_t data[16])
{
    write_register(CYRF_TX_LENGTH, 16);
    write_register(CYRF_TX_CTRL, CYRF_TX_CLR);

    write_multiple(CYRF_TX_BUFFER, 16, data);
    write_register(CYRF_TX_CTRL, CYRF_TX_GO | CYRF_TXC_IRQEN);
    dsm.send_count++;
}


/*
  send a telemetry structure packet
 */
void AP_Radio_cypress::send_telem_packet(void)
{
    struct telem_packet_cypress pkt;

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
        sem.take_nonblocking()) {
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
        sem.give();
        pkt.crc = crc_crc8((const uint8_t *)&pkt.type, 15);
    } else {
        pkt.type = TELEM_STATUS;
        pkt.payload.status = t_status;
        pkt.crc = crc_crc8((const uint8_t *)&pkt.type, 15);
        dsm.telem_send_count++;
    }

    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    transmit16((uint8_t*)&pkt);

    state = STATE_SEND_TELEM_WAIT;
    setup_timeout(2);
}

/*
  send a FCC test packet
 */
void AP_Radio_cypress::send_FCC_test_packet(void)
{
    uint8_t pkt[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

    state = STATE_SEND_FCC;

    uint8_t channel=0;

    switch (get_fcc_test()) {
    case 0:
        // switch back to normal operation
        dsm.forced_channel = -1;
        send_telem_packet();
        return;
    case 1:
    case 4:
        channel = DSM_SCAN_MIN_CH;
        break;
    case 2:
    case 5:
        channel = DSM_SCAN_MID_CH;
        break;
    case 3:
    case 6:
    default:
        channel = DSM_SCAN_MAX_CH;
        break;
    }

    Debug(5,"FCC send %u\n", channel);

    if (channel != dsm.forced_channel) {
        Debug(1,"FCC channel %u\n", channel);
        dsm.forced_channel = channel;

        radio_set_config(cyrf_config, ARRAY_SIZE(cyrf_config));
        radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

        set_channel(channel);
    }

#if FCC_SUPPORT_CW_MODE
    if (get_fcc_test() > 3) {
        // continuous preamble transmit is closest approximation to CW
        // that is possible with this chip
        write_register(CYRF_PREAMBLE,0x01);
        write_register(CYRF_PREAMBLE,0x00);
        write_register(CYRF_PREAMBLE,0x00);

        write_register(CYRF_TX_OVERRIDE, CYRF_FRC_PRE);
        write_register(CYRF_TX_CTRL, CYRF_TX_GO);

        setup_timeout(500);
    } else {
        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
        transmit16(pkt);
        setup_timeout(10);
    }
#else
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    transmit16(pkt);
    setup_timeout(10);
#endif
}

// handle a data96 mavlink packet for fw upload
void AP_Radio_cypress::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
    uint32_t ofs=0;
    memcpy(&ofs, &m.data[0], 4);
    Debug(4, "got data96 of len %u from chan %u at offset %u\n", m.len, chan, unsigned(ofs));
    if (sem.take_nonblocking()) {
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
        sem.give();
    }
}

#endif  // AP_RADIO_CYRF6936_ENABLED
