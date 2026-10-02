/*
 * ChibiOS USB composite: single CDC-ACM + CDC-ECM (experimental spike).
 *
 * Endpoint map (FS, 64 B bulk MPS) — see local/cdc-ecm/EP_MAP_MATEK_H743.md:
 *   EP1 IN/OUT bulk  — ACM data
 *   EP2 IN interrupt — ACM notification
 *   EP3 IN interrupt — ECM notification
 *   EP4 IN/OUT bulk  — ECM data
 *
 * Gated by HAL_WITH_USB_CDC_ECM. Mutually exclusive with dual-CDC and
 * single-CDC-only usbcfg paths. Not used in the bootloader.
 *
 * Modified for use in AP_HAL by ArduPilot CDC-ECM spike.
 */
#include "hal.h"
#include "hwdef.h"

#include <stdlib.h>
#include <string.h>

#include "usbcfg.h"

#ifndef HAL_WITH_USB_CDC_ECM
#define HAL_WITH_USB_CDC_ECM 0
#endif

#if defined(HAL_USB_PRODUCT_ID) && HAL_WITH_USB_CDC_ECM

/* Platform floor: F4 out of scope for ECM */
#if defined(STM32F4XX) || defined(STM32F4)
#error "HAL_WITH_USB_CDC_ECM is out of scope on STM32F4 (use PPP on CDC-ACM)"
#endif

/*
 * Platform floor (locked): gadget OTG instance must have STM32_OTGn_ENDPOINTS >= 8
 * (H743 either OTG, or F76x when gadget is OTG2). Spike uses max EP# = 4.
 */
#if STM32_OTG2_IS_OTG1
#if !defined(STM32_OTG2_ENDPOINTS) || (STM32_OTG2_ENDPOINTS < 8)
#error "HAL_WITH_USB_CDC_ECM requires gadget OTG with STM32_OTGn_ENDPOINTS >= 8"
#endif
#elif (STM32_USB_USE_OTG1 == TRUE)
#if !defined(STM32_OTG1_ENDPOINTS) || (STM32_OTG1_ENDPOINTS < 8)
#error "HAL_WITH_USB_CDC_ECM requires gadget OTG1 with STM32_OTG1_ENDPOINTS >= 8"
#endif
#elif (STM32_USB_USE_OTG2 == TRUE)
#if !defined(STM32_OTG2_ENDPOINTS) || (STM32_OTG2_ENDPOINTS < 8)
#error "HAL_WITH_USB_CDC_ECM requires gadget OTG2 with STM32_OTG2_ENDPOINTS >= 8"
#endif
#else
#error "HAL_WITH_USB_CDC_ECM requires an enabled STM32 OTG gadget instance"
#endif

/*
 * Distinct USB PID when ECM composite is enabled (avoid host driver cache /
 * recovery confusion with stock ACM-only PID 0x5740 dual / 0x5741 single).
 * Lab board uses USB_PRODUCT 0x574E (MatekH743-ECM).
 */
#if (HAL_USB_PRODUCT_ID == 0x5740) || (HAL_USB_PRODUCT_ID == 0x5741)
#error "HAL_WITH_USB_CDC_ECM requires a distinct USB_PRODUCT (e.g. 0x574E), not stock ACM PID 0x5740/0x5741"
#endif

/*
 * Virtual serial port over USB (ACM only — single port for GCS / MAVLink).
 */
SerialUSBDriver SDU1;

static cdc_linecoding_t linecoding = {
    {0x00, 0x96, 0x00, 0x00}, /* 38400 */
    LC_STOP_1, LC_PARITY_NONE, 8
};

/*
 * Endpoints — single ACM + ECM (EP map signed in Phase 0).
 */
#define USB_ACM_DATA_REQUEST_EP         1
#define USB_ACM_DATA_AVAILABLE_EP       1
#define USB_ACM_INTERRUPT_REQUEST_EP    2
#define USB_ECM_INTERRUPT_REQUEST_EP    3
#define USB_ECM_DATA_AVAILABLE_EP       4   /* host OUT / device RX */
#define USB_ECM_DATA_REQUEST_EP         5   /* host IN  / device TX */

#define USB_INTERRUPT_REQUEST_SIZE      0x10
#define USB_DATA_SIZE                   0x40

/*
 * Interfaces: IAD ACM (2) + IAD ECM (2) = 4
 */
#define USB_NUM_INTERFACES              4
#define USB_ACM_CIF_NUM                 0
#define USB_ACM_DIF_NUM                 1
#define USB_ECM_CIF_NUM                 2
#define USB_ECM_DIF_NUM                 3

/* CDC-ECM subclass / descriptor / requests (USB-IF CDC PSTN/ECM) */
#ifndef CDC_ETHERNET_NETWORKING_CONTROL_MODEL
#define CDC_ETHERNET_NETWORKING_CONTROL_MODEL   0x06U
#endif
#ifndef CDC_ETHERNET_NETWORKING
#define CDC_ETHERNET_NETWORKING                 0x0FU
#endif
#ifndef CDC_SET_ETHERNET_PACKET_FILTER
#define CDC_SET_ETHERNET_PACKET_FILTER          0x43U
#endif
#ifndef CDC_NETWORK_CONNECTION
#define CDC_NETWORK_CONNECTION                  0x00U
#endif
#ifndef CDC_CONNECTION_SPEED_CHANGE
#define CDC_CONNECTION_SPEED_CHANGE             0x2AU
#endif

/* String indices: 1 mfr, 2 product, 3 serial, 4 ECM MAC */
#define USB_ECM_MAC_STRING_INDEX        4

/* Distinct bcdDevice when ECM composite is active */
#ifndef HAL_USB_BCD_DEVICE_ECM
#define HAL_USB_BCD_DEVICE_ECM          0x0301
#endif

/*
 * ECM runtime (Phase 1: altsetting / link notify; Phase 2: frame I/O).
 *
 * ecm_notify_pending:
 *   0 = idle
 *   1 = send NETWORK_CONNECTION (wValue from ecm_notify_connection[2])
 *   2 = send CONNECTION_SPEED_CHANGE after successful NC (connected only)
 */
static uint8_t ecm_data_altsetting;
static uint16_t ecm_packet_filter = 0x0001; /* PACKET_TYPE_DIRECTED default-ish */
static volatile uint8_t ecm_notify_pending;

/* NETWORK_CONNECTION notification (bmRequestType class/IF/IN, wValue=connected) */
static uint8_t ecm_notify_connection[8] = {
    0xA1, CDC_NETWORK_CONNECTION, 0x00, 0x00,
    USB_ECM_CIF_NUM, 0x00, 0x00, 0x00
};

/* CONNECTION_SPEED_CHANGE: 8-byte header + DL/UL bitrates (FS-ish 12 Mbps) */
#define USB_ECM_NOTIFY_BITRATE_BPS      12000000UL
static uint8_t ecm_notify_speed[16] = {
    0xA1, CDC_CONNECTION_SPEED_CHANGE, 0x00, 0x00,
    USB_ECM_CIF_NUM, 0x00, 0x08, 0x00,
    /* DLBitRate LE */
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS),
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS >> 8),
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS >> 16),
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS >> 24),
    /* ULBitRate LE */
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS),
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS >> 8),
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS >> 16),
    (uint8_t)(USB_ECM_NOTIFY_BITRATE_BPS >> 24)
};

/* Phase 2 frame I/O state */
#define USB_ECM_MAX_FRAME               1514
#define USB_ECM_RX_RING                 8

static uint8_t ecm_rx_pkt[USB_DATA_SIZE] __attribute__((aligned(4)));
static uint8_t ecm_rx_accum[USB_ECM_MAX_FRAME] __attribute__((aligned(4)));
static uint16_t ecm_rx_accum_len;
static volatile systime_t ecm_rx_last_st;
static volatile uint32_t ecm_stat_rx_pkts;
static volatile uint32_t ecm_stat_rx_frames;
static volatile uint32_t ecm_stat_tx_frames;
static volatile uint32_t ecm_stat_tx_fail;

static struct {
    uint8_t data[USB_ECM_MAX_FRAME];
    uint16_t len;
} ecm_rx_ring[USB_ECM_RX_RING];
static volatile uint8_t ecm_rx_w;
static volatile uint8_t ecm_rx_r;

static uint8_t ecm_tx_buf[USB_ECM_MAX_FRAME] __attribute__((aligned(4)));
static volatile uint16_t ecm_tx_len;
static volatile uint16_t ecm_tx_off;
static volatile uint8_t ecm_tx_busy;
static USBDriver *ecm_usbp;

/* Push assembled frame into RX ring (caller holds lock / ISR). */
static void ecm_rx_push_frame_I(void)
{
    if (ecm_rx_accum_len == 0) {
        return;
    }
    const uint8_t next = (uint8_t)((ecm_rx_w + 1U) % USB_ECM_RX_RING);
    if (next != ecm_rx_r) {
        memcpy(ecm_rx_ring[ecm_rx_w].data, ecm_rx_accum, ecm_rx_accum_len);
        ecm_rx_ring[ecm_rx_w].len = ecm_rx_accum_len;
        ecm_rx_w = next;
        ecm_stat_rx_frames++;
    }
    ecm_rx_accum_len = 0;
}

/*
 * Hosts (incl. macOS) often pad the last USB packet to MPS=64 and omit ZLP.
 * Infer end-of-frame from Ethernet/IP headers so we do not strand RX data.
 */
static bool ecm_frame_looks_complete_I(uint16_t len)
{
    uint16_t etype;
    uint16_t need;

    if (len < 14U) {
        return false;
    }
    etype = (uint16_t)(((uint16_t)ecm_rx_accum[12] << 8) | ecm_rx_accum[13]);
    if (etype == 0x0806U) {
        /* ARP is 42 bytes; hosts usually pad to 60–64 in one USB transaction */
        return len >= 60U || (len >= 42U && (len % USB_DATA_SIZE) != 0);
    }
    if (etype == 0x0800U && len >= 34U) {
        /* IPv4: total length at offset 16 */
        need = (uint16_t)(((uint16_t)ecm_rx_accum[16] << 8) | ecm_rx_accum[17]);
        if (need < 20U || need > 1500U) {
            return false;
        }
        return len >= (uint16_t)(14U + need);
    }
    if (etype == 0x86DDU && len >= 54U) {
        /* IPv6: payload length at offset 18, fixed 40-byte hdr */
        need = (uint16_t)(((uint16_t)ecm_rx_accum[18] << 8) | ecm_rx_accum[19]);
        return len >= (uint16_t)(14U + 40U + need);
    }
    /* Unknown ethertype: fall back to min frame size once we have a full MPS */
    return len >= 60U && (len % USB_DATA_SIZE) != 0;
}

static void ecm_start_rx_I(USBDriver *usbp);

/* Clear link + notify + frame state (reset / suspend / reconfigure). Call under lock if ISR. */
static void ecm_reset_link_state(void)
{
    ecm_data_altsetting = 0;
    ecm_notify_pending = 0;
    ecm_notify_connection[2] = 0;
    ecm_rx_accum_len = 0;
    ecm_rx_w = 0;
    ecm_rx_r = 0;
    ecm_tx_busy = 0;
    ecm_tx_len = 0;
    ecm_tx_off = 0;
}

/*
 * USB Device Descriptor — misc + IAD (composite).
 */
static const uint8_t vcom_device_descriptor_data[] = {
    USB_DESC_DEVICE(
        0x0200,                                 /* bcdUSB                   */
        0xEF,                                   /* bDeviceClass (misc)      */
        0x02,                                   /* bDeviceSubClass (common) */
        0x01,                                   /* bDeviceProtocol (IAD)    */
        USB_DATA_SIZE,                          /* bMaxPacketSize           */
        HAL_USB_VENDOR_ID,                      /* idVendor                 */
        HAL_USB_PRODUCT_ID,                     /* idProduct                */
        HAL_USB_BCD_DEVICE_ECM,                 /* bcdDevice (distinct)     */
        1,                                      /* iManufacturer            */
        2,                                      /* iProduct                 */
        3,                                      /* iSerialNumber            */
        1)                                      /* bNumConfigurations       */
};

static const USBDescriptor vcom_device_descriptor = {
    sizeof vcom_device_descriptor_data,
    vcom_device_descriptor_data
};

/* ACM IF set (same layout as usbcfg_dualcdc CDC_IF_DESC_SET) */
#define CDC_ACM_IF_DESC_SET_SIZE                                                \
    (USB_DESC_INTERFACE_SIZE + 5 + 5 + 4 + 5 + USB_DESC_ENDPOINT_SIZE +         \
     USB_DESC_INTERFACE_SIZE + (USB_DESC_ENDPOINT_SIZE * 2))

#define CDC_ACM_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)      \
    USB_DESC_INTERFACE(                                                         \
        comIfNum, 0x00, 0x01,                                                   \
        CDC_COMMUNICATION_INTERFACE_CLASS,                                      \
        CDC_ABSTRACT_CONTROL_MODEL,                                             \
        0x01, 0),                                                               \
    USB_DESC_BYTE(5),                                                           \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_HEADER),                                                  \
    USB_DESC_BCD(0x0110),                                                       \
    USB_DESC_BYTE(5),                                                           \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_CALL_MANAGEMENT),                                         \
    USB_DESC_BYTE(0x00),                                                        \
    USB_DESC_BYTE(datIfNum),                                                    \
    USB_DESC_BYTE(4),                                                           \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_ABSTRACT_CONTROL_MANAGEMENT),                             \
    USB_DESC_BYTE(0x02),                                                        \
    USB_DESC_BYTE(5),                                                           \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_UNION),                                                   \
    USB_DESC_BYTE(comIfNum),                                                    \
    USB_DESC_BYTE(datIfNum),                                                    \
    USB_DESC_ENDPOINT(                                                          \
        comInEp, USB_EP_MODE_TYPE_INTR,                                         \
        USB_INTERRUPT_REQUEST_SIZE, 0x01),                                      \
    USB_DESC_INTERFACE(                                                         \
        datIfNum, 0x00, 0x02,                                                   \
        CDC_DATA_INTERFACE_CLASS, 0x00, 0x00, 0x00),                            \
    USB_DESC_ENDPOINT(                                                          \
        datOutEp, USB_EP_MODE_TYPE_BULK, USB_DATA_SIZE, 0x00),                  \
    USB_DESC_ENDPOINT(                                                          \
        datInEp, USB_EP_MODE_TYPE_BULK, USB_DATA_SIZE, 0x00)

#define IAD_CDC_ACM_IF_DESC_SET_SIZE                                            \
    (USB_DESC_INTERFACE_ASSOCIATION_SIZE + CDC_ACM_IF_DESC_SET_SIZE)

#define IAD_CDC_ACM_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp) \
    USB_DESC_INTERFACE_ASSOCIATION(                                             \
        comIfNum, 2,                                                            \
        CDC_COMMUNICATION_INTERFACE_CLASS,                                      \
        CDC_ABSTRACT_CONTROL_MODEL,                                             \
        1, 0),                                                                  \
    CDC_ACM_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)

/*
 * ECM: comm IF + Header/Union/Ethernet + notify EP;
 * data IF alt0 (no EPs) + alt1 (bulk IN/OUT).
 */
/* Single data-IF alternate with bulk EPs so hosts that never SET_INTERFACE
 * still get a working data path (macOS has been observed to leave alt=0).
 * SET_INTERFACE alt>0 still accepted for link notify.
 */
#define CDC_ECM_IF_DESC_SET_SIZE                                                \
    (USB_DESC_INTERFACE_SIZE + 5 + 5 + 13 + USB_DESC_ENDPOINT_SIZE +            \
     USB_DESC_INTERFACE_SIZE + (USB_DESC_ENDPOINT_SIZE * 2))

#define CDC_ECM_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)     \
    /* ECM Communication Interface */                                           \
    USB_DESC_INTERFACE(                                                         \
        comIfNum, 0x00, 0x01,                                                   \
        CDC_COMMUNICATION_INTERFACE_CLASS,                                      \
        CDC_ETHERNET_NETWORKING_CONTROL_MODEL,                                  \
        0x00, 0),                                                               \
    /* Header Functional Descriptor */                                          \
    USB_DESC_BYTE(5),                                                           \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_HEADER),                                                  \
    USB_DESC_BCD(0x0110),                                                       \
    /* Union Functional Descriptor */                                           \
    USB_DESC_BYTE(5),                                                           \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_UNION),                                                   \
    USB_DESC_BYTE(comIfNum),                                                    \
    USB_DESC_BYTE(datIfNum),                                                    \
    /* Ethernet Networking Functional Descriptor */                             \
    USB_DESC_BYTE(13),                                                          \
    USB_DESC_BYTE(CDC_CS_INTERFACE),                                            \
    USB_DESC_BYTE(CDC_ETHERNET_NETWORKING),                                     \
    USB_DESC_BYTE(USB_ECM_MAC_STRING_INDEX), /* iMACAddress */                  \
    USB_DESC_BYTE(0x00), USB_DESC_BYTE(0x00), USB_DESC_BYTE(0x00),              \
    USB_DESC_BYTE(0x00),                     /* bmEthernetStatistics */         \
    USB_DESC_WORD(1514),                     /* wMaxSegmentSize */              \
    USB_DESC_WORD(0x0000),                   /* wNumberMCFilters */             \
    USB_DESC_BYTE(0x00),                     /* bNumberPowerFilters */          \
    /* Interrupt IN (network connection / speed) */                             \
    USB_DESC_ENDPOINT(                                                          \
        comInEp, USB_EP_MODE_TYPE_INTR,                                         \
        USB_INTERRUPT_REQUEST_SIZE, 0x08),                                      \
    /* Data IF alt 0 — bulk IN/OUT always present */                            \
    USB_DESC_INTERFACE(                                                         \
        datIfNum, 0x00, 0x02,                                                   \
        CDC_DATA_INTERFACE_CLASS, 0x00, 0x00, 0x00),                            \
    USB_DESC_ENDPOINT(                                                          \
        datOutEp, USB_EP_MODE_TYPE_BULK, USB_DATA_SIZE, 0x00),                  \
    USB_DESC_ENDPOINT(                                                          \
        datInEp, USB_EP_MODE_TYPE_BULK, USB_DATA_SIZE, 0x00)

#define IAD_CDC_ECM_IF_DESC_SET_SIZE                                            \
    (USB_DESC_INTERFACE_ASSOCIATION_SIZE + CDC_ECM_IF_DESC_SET_SIZE)

#define IAD_CDC_ECM_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp) \
    USB_DESC_INTERFACE_ASSOCIATION(                                             \
        comIfNum, 2,                                                            \
        CDC_COMMUNICATION_INTERFACE_CLASS,                                      \
        CDC_ETHERNET_NETWORKING_CONTROL_MODEL,                                  \
        0, 0),                                                                  \
    CDC_ECM_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)

#define USB_CONFIGURATION_TOTAL_SIZE                                            \
    (USB_DESC_CONFIGURATION_SIZE +                                              \
     IAD_CDC_ACM_IF_DESC_SET_SIZE +                                             \
     IAD_CDC_ECM_IF_DESC_SET_SIZE)

static const uint8_t vcom_configuration_descriptor_data[] = {
    USB_DESC_CONFIGURATION(
        USB_CONFIGURATION_TOTAL_SIZE,
        USB_NUM_INTERFACES,
        0x01,
        0,
        0xC0,
        50),
    IAD_CDC_ACM_IF_DESC_SET(
        USB_ACM_CIF_NUM,
        USB_ACM_DIF_NUM,
        USB_ENDPOINT_IN(USB_ACM_INTERRUPT_REQUEST_EP),
        USB_ENDPOINT_OUT(USB_ACM_DATA_AVAILABLE_EP),
        USB_ENDPOINT_IN(USB_ACM_DATA_REQUEST_EP)),
    IAD_CDC_ECM_IF_DESC_SET(
        USB_ECM_CIF_NUM,
        USB_ECM_DIF_NUM,
        USB_ENDPOINT_IN(USB_ECM_INTERRUPT_REQUEST_EP),
        USB_ENDPOINT_OUT(USB_ECM_DATA_AVAILABLE_EP),
        USB_ENDPOINT_IN(USB_ECM_DATA_REQUEST_EP)),
};

static const USBDescriptor vcom_configuration_descriptor = {
    sizeof vcom_configuration_descriptor_data,
    vcom_configuration_descriptor_data
};

static const uint8_t vcom_string0[] = {
    USB_DESC_BYTE(4),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    USB_DESC_WORD(0x0409)
};

/* Default MAC string for descriptor (12 hex chars). Phase 2 ties to netif MAC. */
#ifndef HAL_USB_ECM_MAC_STRING
#define HAL_USB_ECM_MAC_STRING "020000000114"
#endif

static USBDescriptor vcom_strings[] = {
    {sizeof vcom_string0, vcom_string0},
    {0, NULL}, /* manufacturer */
    {0, NULL}, /* product */
    {0, NULL}, /* serial */
    {0, NULL}, /* ECM MAC */
};

static uint8_t vcom_buffers[4][2 + 2 * USB_DESC_MAX_STRLEN];

static void setup_usb_string(USBDescriptor *desc, const char *str, uint8_t *b)
{
    char str2[USB_DESC_MAX_STRLEN];
    string_substitute(str, str2);
    uint8_t len = strlen(str2);
    desc->ud_size = 2 + 2 * len;
    desc->ud_string = (const uint8_t *)b;
    b[0] = USB_DESC_BYTE(desc->ud_size);
    b[1] = USB_DESC_BYTE(USB_DESCRIPTOR_STRING);
    uint8_t i;
    for (i = 0; i < len; i++) {
        b[2 + i * 2] = str2[i];
        b[2 + i * 2 + 1] = 0;
    }
}

void setup_usb_strings(void)
{
    setup_usb_string(&vcom_strings[1], HAL_USB_STRING_MANUFACTURER, vcom_buffers[0]);
    setup_usb_string(&vcom_strings[2], HAL_USB_STRING_PRODUCT, vcom_buffers[1]);
    setup_usb_string(&vcom_strings[3], HAL_USB_STRING_SERIAL, vcom_buffers[2]);
    setup_usb_string(&vcom_strings[4], HAL_USB_ECM_MAC_STRING, vcom_buffers[3]);
}

static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang)
{
    (void)usbp;
    (void)lang;
    switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
        return &vcom_device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
        return &vcom_configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
        if (dindex < 5) {
            return &vcom_strings[dindex];
        }
        break;
    }
    return NULL;
}

#if HAL_USE_SERIAL_USB
uint32_t get_usb_baud(uint16_t endpoint_id)
{
    if (endpoint_id == 0) {
        uint32_t rate;
        memcpy(&rate, &linecoding.dwDTERate[0], sizeof(rate));
        return rate;
    }
    return 0;
}

uint8_t get_usb_parity(uint16_t endpoint_id)
{
    if (endpoint_id == 0) {
        return linecoding.bParityType;
    }
    return 0;
}
#endif

/* --- Endpoint state / configs --- */

static USBInEndpointState ep1instate;
static USBOutEndpointState ep1outstate;

static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    sduDataTransmitted,
    sduDataReceived,
    USB_DATA_SIZE,
    USB_DATA_SIZE,
    &ep1instate,
    &ep1outstate,
    2,
    NULL
};

static USBInEndpointState ep2instate;

static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    USB_INTERRUPT_REQUEST_SIZE,
    0x0000,
    &ep2instate,
    NULL,
    1,
    NULL
};

static void ecmInterruptTransmitted(USBDriver *usbp, usbep_t ep)
{
    (void)usbp;
    (void)ep;
}

static USBInEndpointState ep3instate;

static const USBEndpointConfig ep3config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    ecmInterruptTransmitted,
    NULL,
    USB_INTERRUPT_REQUEST_SIZE,
    0x0000,
    &ep3instate,
    NULL,
    1,
    NULL
};

/* Start next bulk OUT receive when data alt=1 and EP idle. */
static void ecm_start_rx_I(USBDriver *usbp)
{
    if (usbp == NULL || ecm_data_altsetting != 1) {
        return;
    }
    if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
        return;
    }
    if (usbGetReceiveStatusI(usbp, USB_ECM_DATA_AVAILABLE_EP)) {
        return;
    }
    usbStartReceiveI(usbp, USB_ECM_DATA_AVAILABLE_EP, ecm_rx_pkt, USB_DATA_SIZE);
}

/* Kick next TX chunk or complete frame under lock. */
static void ecm_tx_continue_I(USBDriver *usbp)
{
    if (!ecm_tx_busy || usbp == NULL) {
        return;
    }
    if (usbGetDriverStateI(usbp) != USB_ACTIVE || ecm_data_altsetting != 1) {
        ecm_tx_busy = 0;
        ecm_tx_len = 0;
        ecm_tx_off = 0;
        return;
    }
    if (usbGetTransmitStatusI(usbp, USB_ECM_DATA_REQUEST_EP)) {
        return; /* still busy — wait for next IN complete */
    }
    if (ecm_tx_off < ecm_tx_len) {
        const uint16_t remain = (uint16_t)(ecm_tx_len - ecm_tx_off);
        const uint16_t chunk = remain > USB_DATA_SIZE ? USB_DATA_SIZE : remain;
        usbStartTransmitI(usbp, USB_ECM_DATA_REQUEST_EP,
                          &ecm_tx_buf[ecm_tx_off], chunk);
        ecm_tx_off = (uint16_t)(ecm_tx_off + chunk);
        return;
    }
    /* Full frame queued; if length was multiple of MPS, send ZLP */
    if (ecm_tx_len > 0 && (ecm_tx_len % USB_DATA_SIZE) == 0 && ecm_tx_off == ecm_tx_len) {
        ecm_tx_off = (uint16_t)(ecm_tx_len + 1); /* mark ZLP pending/done path */
        usbStartTransmitI(usbp, USB_ECM_DATA_REQUEST_EP, NULL, 0);
        return;
    }
    ecm_tx_busy = 0;
    ecm_tx_len = 0;
    ecm_tx_off = 0;
}

static void ecmDataTransmitted(USBDriver *usbp, usbep_t ep)
{
    (void)ep;
    ecm_tx_continue_I(usbp);
}

static void ecmDataReceived(USBDriver *usbp, usbep_t ep)
{
    const size_t n = usbGetReceiveTransactionSizeX(usbp, ep);
    ecm_stat_rx_pkts++;
    /* chVTGetSystemTimeX is ms-ish ticks on AP; use ST2MS if available */
    ecm_rx_last_st = chVTGetSystemTimeX();

    if (n > 0) {
        if ((uint16_t)(ecm_rx_accum_len + n) <= USB_ECM_MAX_FRAME) {
            memcpy(&ecm_rx_accum[ecm_rx_accum_len], ecm_rx_pkt, n);
            ecm_rx_accum_len = (uint16_t)(ecm_rx_accum_len + n);
        } else {
            ecm_rx_accum_len = 0;
        }
    }

    /* Short packet / ZLP, or header-length complete (no ZLP from host) */
    if (n < USB_DATA_SIZE || ecm_frame_looks_complete_I(ecm_rx_accum_len)) {
        ecm_rx_push_frame_I();
    }

    ecm_start_rx_I(usbp);
}

static USBOutEndpointState ep4outstate;
static USBInEndpointState ep5instate;

/* EP4: bulk OUT only (ECM RX) */
static const USBEndpointConfig ep4out_config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    NULL,
    ecmDataReceived,
    0x0000,
    USB_DATA_SIZE,
    NULL,
    &ep4outstate,
    1,
    NULL
};

/* EP5: bulk IN only (ECM TX) */
static const USBEndpointConfig ep5in_config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    ecmDataTransmitted,
    NULL,
    USB_DATA_SIZE,
    0x0000,
    &ep5instate,
    NULL,
    1,
    NULL
};

/* I-class: start NETWORK_CONNECTION on EP3 if idle and USB active. */
static bool ecm_start_network_connection_I(USBDriver *usbp)
{
    if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
        return false;
    }
    if (usbGetTransmitStatusI(usbp, USB_ECM_INTERRUPT_REQUEST_EP)) {
        return false; /* still busy — retry next SOF */
    }
    usbStartTransmitI(usbp, USB_ECM_INTERRUPT_REQUEST_EP,
                      ecm_notify_connection, sizeof ecm_notify_connection);
    return true;
}

/* I-class: start CONNECTION_SPEED_CHANGE on EP3 if idle. */
static bool ecm_start_speed_change_I(USBDriver *usbp)
{
    if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
        return false;
    }
    if (usbGetTransmitStatusI(usbp, USB_ECM_INTERRUPT_REQUEST_EP)) {
        return false;
    }
    usbStartTransmitI(usbp, USB_ECM_INTERRUPT_REQUEST_EP,
                      ecm_notify_speed, sizeof ecm_notify_speed);
    return true;
}

static void usb_event(USBDriver *usbp, usbevent_t event)
{
    extern SerialUSBDriver SDU1;

    switch (event) {
    case USB_EVENT_ADDRESS:
        return;
    case USB_EVENT_CONFIGURED:
        chSysLockFromISR();

        /* Always clear link notify state on configure (including SELECTED). */
        ecm_reset_link_state();

        if (usbp->state == USB_ACTIVE) {
            usbInitEndpointI(usbp, USB_ACM_DATA_REQUEST_EP, &ep1config);
            usbInitEndpointI(usbp, USB_ACM_INTERRUPT_REQUEST_EP, &ep2config);
            usbInitEndpointI(usbp, USB_ECM_INTERRUPT_REQUEST_EP, &ep3config);
            usbInitEndpointI(usbp, USB_ECM_DATA_AVAILABLE_EP, &ep4out_config);
            usbInitEndpointI(usbp, USB_ECM_DATA_REQUEST_EP, &ep5in_config);
            ecm_usbp = usbp;
            /* Single data alt has EPs from configure — treat as link up */
            ecm_data_altsetting = 1;
            ecm_notify_connection[2] = 0x01;
            ecm_notify_pending = 1;
            ecm_start_rx_I(usbp);

            sduConfigureHookI(&SDU1);
        } else if (usbp->state == USB_SELECTED) {
            usbDisableEndpointsI(usbp);
        }

        chSysUnlockFromISR();
        return;
    case USB_EVENT_RESET:
    case USB_EVENT_UNCONFIGURED:
    case USB_EVENT_SUSPEND:
        chSysLockFromISR();
        sduSuspendHookI(&SDU1);
        ecm_reset_link_state();
        chSysUnlockFromISR();
        return;
    case USB_EVENT_WAKEUP:
        chSysLockFromISR();
        sduWakeupHookI(&SDU1);
        chSysUnlockFromISR();
        return;
    case USB_EVENT_STALLED:
        return;
    }
    return;
}

static bool requests_hook(USBDriver *usbp)
{
    /* SET_INTERFACE — required for ECM data altsettings */
    if (((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE) &&
        (usbp->setup[1] == USB_REQ_SET_INTERFACE)) {
        const uint8_t iface = usbp->setup[4];
        const uint8_t alt = usbp->setup[2];
        if (iface == USB_ECM_DIF_NUM) {
            /* Single operational data alt (0). Host may also SET_INTERFACE(0). */
            if (alt > 0) {
                return false;
            }
            ecm_data_altsetting = 1;
            ecm_notify_connection[2] = 0x01;
            ecm_rx_accum_len = 0;
            ecm_usbp = usbp;
            ecm_start_rx_I(usbp);
            ecm_notify_pending = 1;
            usbSetupTransfer(usbp, NULL, 0, NULL);
            return true;
        }
        /* Other interfaces: only alt 0 */
        if (alt != 0) {
            return false;
        }
        usbSetupTransfer(usbp, NULL, 0, NULL);
        return true;
    }

    /* GET_INTERFACE */
    if (((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE) &&
        (usbp->setup[1] == USB_REQ_GET_INTERFACE)) {
        static uint8_t alt_reply;
        const uint8_t iface = usbp->setup[4];
        alt_reply = (iface == USB_ECM_DIF_NUM) ? ecm_data_altsetting : 0;
        usbSetupTransfer(usbp, &alt_reply, 1, NULL);
        return true;
    }

    /* ACM class requests on comm interface 0 */
    if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS &&
        usbp->setup[4] == USB_ACM_CIF_NUM && usbp->setup[5] == 0x00) {
        switch (usbp->setup[1]) {
        case CDC_GET_LINE_CODING:
            usbSetupTransfer(usbp, (uint8_t *)&linecoding, sizeof(linecoding), NULL);
            return true;
        case CDC_SET_LINE_CODING:
            usbSetupTransfer(usbp, (uint8_t *)&linecoding, sizeof(linecoding), NULL);
            return true;
        case CDC_SET_CONTROL_LINE_STATE:
            usbSetupTransfer(usbp, NULL, 0, NULL);
            return true;
        }
    }

    /* ECM class requests on comm interface 2 — whitelist only */
    if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS &&
        usbp->setup[4] == USB_ECM_CIF_NUM && usbp->setup[5] == 0x00) {
        switch (usbp->setup[1]) {
        case CDC_SET_ETHERNET_PACKET_FILTER:
            ecm_packet_filter = (uint16_t)usbp->setup[2] | ((uint16_t)usbp->setup[3] << 8);
            ecm_data_altsetting = 1;
            ecm_notify_connection[2] = 0x01;
            ecm_usbp = usbp;
            ecm_start_rx_I(usbp);
            if (ecm_notify_pending == 0) {
                ecm_notify_pending = 1;
            }
            usbSetupTransfer(usbp, NULL, 0, NULL);
            return true;
        default:
            /* STALL unknown ECM class requests (do not ACK GETs with empty ZLP) */
            return false;
        }
    }

    return sduRequestsHook(usbp);
}

static void sof_handler(USBDriver *usbp)
{
    osalSysLockFromISR();
    sduSOFHookI(&SDU1);
    /* Keep ECM bulk OUT armed while data alt is active */
    if (ecm_data_altsetting == 1) {
        ecm_start_rx_I(usbp);
        /* Complete partial frame if host omitted short-packet/ZLP (common on some stacks) */
        if (ecm_rx_accum_len >= 14) {
            if (chVTTimeElapsedSinceX(ecm_rx_last_st) >= TIME_MS2I(2)) {
                ecm_rx_push_frame_I();
            }
        }
    }
    if (ecm_notify_pending == 1) {
        if (ecm_start_network_connection_I(usbp)) {
            /* After NC connected=1, queue CONNECTION_SPEED_CHANGE */
            if (ecm_notify_connection[2] != 0) {
                ecm_notify_pending = 2;
            } else {
                ecm_notify_pending = 0;
            }
        }
        /* else EP3 busy or not active — retry next SOF */
    } else if (ecm_notify_pending == 2) {
        if (ecm_start_speed_change_I(usbp)) {
            ecm_notify_pending = 0;
        }
    }
    osalSysUnlockFromISR();
}

const USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    requests_hook,
    sof_handler
};

const SerialUSBConfig serusbcfg1 = {
#if STM32_OTG2_IS_OTG1
    &USBD2,
#else
    &USBD1,
#endif
    USB_ACM_DATA_REQUEST_EP,
    USB_ACM_DATA_AVAILABLE_EP,
    USB_ACM_INTERRUPT_REQUEST_EP
};

/* Phase 2 helpers (link state + frame I/O for netif) */
uint8_t usb_ecm_get_data_altsetting(void)
{
    return ecm_data_altsetting;
}

bool usb_ecm_link_up(void)
{
    return ecm_data_altsetting == 1;
}

bool usb_ecm_send_frame(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > USB_ECM_MAX_FRAME) {
        return false;
    }
    if (!usb_ecm_link_up() || ecm_usbp == NULL) {
        return false;
    }

    /* Wait for prior TX (network thread — sleep, do not starve ISRs). */
    uint32_t waits = 0;
    while (ecm_tx_busy) {
        if (++waits > 200U) {
            return false;
        }
        chThdSleepMicroseconds(100);
    }

    osalSysLock();
    if (ecm_tx_busy || ecm_data_altsetting != 1 || ecm_usbp == NULL) {
        osalSysUnlock();
        return false;
    }
    memcpy(ecm_tx_buf, data, len);
    ecm_tx_len = len;
    ecm_tx_off = 0;
    ecm_tx_busy = 1;
    ecm_tx_continue_I(ecm_usbp);
    osalSysUnlock();

    waits = 0;
    while (ecm_tx_busy) {
        if (++waits > 500U) {
            osalSysLock();
            ecm_tx_busy = 0;
            ecm_tx_len = 0;
            ecm_tx_off = 0;
            osalSysUnlock();
            ecm_stat_tx_fail++;
            return false;
        }
        chThdSleepMicroseconds(100);
    }
    ecm_stat_tx_frames++;
    return true;
}

uint16_t usb_ecm_recv_frame(uint8_t *data, uint16_t maxlen)
{
    uint16_t n = 0;
    if (data == NULL || maxlen == 0) {
        return 0;
    }
    osalSysLock();
    /* Timeout-complete if SOF path missed it */
    if (ecm_rx_accum_len >= 14) {
        if (chVTTimeElapsedSinceX(ecm_rx_last_st) >= TIME_MS2I(2)) {
            ecm_rx_push_frame_I();
        }
    }
    if (ecm_rx_r != ecm_rx_w) {
        n = ecm_rx_ring[ecm_rx_r].len;
        if (n > maxlen) {
            n = maxlen;
        }
        memcpy(data, ecm_rx_ring[ecm_rx_r].data, n);
        ecm_rx_r = (uint8_t)((ecm_rx_r + 1U) % USB_ECM_RX_RING);
    }
    osalSysUnlock();
    return n;
}

void usb_ecm_get_stats(uint32_t *rx_pkts, uint32_t *rx_frames,
                       uint32_t *tx_frames, uint32_t *tx_fail)
{
    if (rx_pkts) {
        *rx_pkts = ecm_stat_rx_pkts;
    }
    if (rx_frames) {
        *rx_frames = ecm_stat_rx_frames;
    }
    if (tx_frames) {
        *tx_frames = ecm_stat_tx_frames;
    }
    if (tx_fail) {
        *tx_fail = ecm_stat_tx_fail;
    }
}

#endif /* HAL_USB_PRODUCT_ID && HAL_WITH_USB_CDC_ECM */
