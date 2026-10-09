/*
 * TinyUSB CDC wrapper for ArduPilot's ESP32 HAL.
 */

#include "ardupilot_tusb.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

static const char *TAG = "ardu_tusb";

static ardupilot_tusb_rx_cb_t s_rx_cb;
static ardupilot_tusb_line_state_cb_t s_line_state_cb;
static void *s_callback_arg;
static bool s_initialized;
static bool s_mounted;
static bool s_open;

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

enum {
    EPNUM_CDC_NOTIF = 0x81,
    EPNUM_CDC_OUT = 0x02,
    EPNUM_CDC_IN = 0x82,
};

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC_INTERFACE,
};

enum {
    CDC_DESC_CONFIG_LEN = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN,
};

static tusb_desc_device_t s_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = TINYUSB_ESPRESSIF_VID,
    .idProduct = 0x4001,
    .bcdDevice = 0x0100,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

static const uint8_t s_fs_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attributes, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CDC_DESC_CONFIG_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC,
                       STRID_CDC_INTERFACE,
                       EPNUM_CDC_NOTIF,
                       8,
                       EPNUM_CDC_OUT,
                       EPNUM_CDC_IN,
                       64),
};

static char s_serial_string[17];
static const char *s_string_descriptors[] = {
    (const char[]){0x09, 0x04},
    "ArduPilot",
    "ArduPilot ESP32-S3 MAVLink",
    s_serial_string,
    "MAVLink CDC",
    NULL,
};

static void ardupilot_tusb_event_cb(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    if (event == NULL) {
        return;
    }

    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        s_mounted = true;
        ESP_LOGI(TAG, "CDC attached");
        break;
    case TINYUSB_EVENT_DETACHED:
        s_mounted = false;
        s_open = false;
        ESP_LOGW(TAG, "CDC detached");
        break;
    default:
        break;
    }
}

static void ardupilot_tusb_rx_callback(int itf, cdcacm_event_t *event)
{
    (void)event;
    if (itf == TINYUSB_CDC_ACM_0 && s_rx_cb != NULL) {
        s_rx_cb(s_callback_arg);
    }
}

static void ardupilot_tusb_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    if (itf != TINYUSB_CDC_ACM_0 || event == NULL) {
        return;
    }

    s_mounted = true;
    s_open = event->line_state_changed_data.dtr || event->line_state_changed_data.rts;

    if (s_line_state_cb != NULL) {
        s_line_state_cb(event->line_state_changed_data.dtr,
                        event->line_state_changed_data.rts,
                        s_callback_arg);
    }
}

bool ardupilot_tusb_init(ardupilot_tusb_rx_cb_t rx_cb,
                         ardupilot_tusb_line_state_cb_t line_state_cb,
                         void *arg)
{
    if (s_initialized) {
        s_rx_cb = rx_cb;
        s_line_state_cb = line_state_cb;
        s_callback_arg = arg;
        return true;
    }

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_serial_string, sizeof(s_serial_string), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &s_device_descriptor;
    tusb_cfg.descriptor.qualifier = NULL;
    tusb_cfg.descriptor.string = s_string_descriptors;
    tusb_cfg.descriptor.string_count = STRID_CDC_INTERFACE + 1;
    tusb_cfg.descriptor.full_speed_config = s_fs_configuration_descriptor;
    tusb_cfg.descriptor.high_speed_config = NULL;
    tusb_cfg.event_cb = ardupilot_tusb_event_cb;
    tusb_cfg.event_arg = NULL;
    if (tinyusb_driver_install(&tusb_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install failed");
        return false;
    }

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = ardupilot_tusb_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = ardupilot_tusb_line_state_changed_callback,
        .callback_line_coding_changed = NULL,
    };
    if (tinyusb_cdcacm_init(&acm_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_cdcacm_init failed");
        (void)tinyusb_driver_uninstall();
        return false;
    }

    s_rx_cb = rx_cb;
    s_line_state_cb = line_state_cb;
    s_callback_arg = arg;
    s_mounted = false;
    s_open = false;
    s_initialized = true;
    return true;
}

void ardupilot_tusb_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    (void)tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);
    (void)tinyusb_driver_uninstall();
    s_rx_cb = NULL;
    s_line_state_cb = NULL;
    s_callback_arg = NULL;
    s_mounted = false;
    s_open = false;
    s_initialized = false;
}

size_t ardupilot_tusb_read(uint8_t *buffer, size_t buffer_size)
{
    size_t rx_size = 0;

    if (!s_initialized || buffer == NULL || buffer_size == 0) {
        return 0;
    }

    if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buffer, buffer_size, &rx_size) != ESP_OK) {
        return 0;
    }
    return rx_size;
}

size_t ardupilot_tusb_write(const uint8_t *buffer, size_t size)
{
    if (!s_initialized || buffer == NULL || size == 0) {
        return 0;
    }
    return tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, buffer, size);
}

esp_err_t ardupilot_tusb_flush(uint32_t timeout_ticks)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    return tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, timeout_ticks);
}

bool ardupilot_tusb_is_connected(void)
{
    return s_initialized && s_mounted;
}

bool ardupilot_tusb_is_ready(void)
{
    return s_initialized && tud_ready();
}

bool ardupilot_tusb_is_cdc_connected(void)
{
    return s_initialized && tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0) &&
           tud_cdc_n_connected(TINYUSB_CDC_ACM_0);
}

bool ardupilot_tusb_is_open(void)
{
    return s_initialized && s_open;
}

bool ardupilot_tusb_is_initialized(void)
{
    return s_initialized;
}

bool ardupilot_tusb_task_alive(void)
{
    return xTaskGetHandle("TinyUSB") != NULL;
}

bool ardupilot_tusb_tx_pending(void)
{
    if (!s_initialized || !tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0)) {
        return false;
    }
    return tud_cdc_n_write_available(TINYUSB_CDC_ACM_0) < CFG_TUD_CDC_TX_BUFSIZE;
}

size_t ardupilot_tusb_write_available(void)
{
    if (!s_initialized || !tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0)) {
        return 0;
    }
    return tud_cdc_n_write_available(TINYUSB_CDC_ACM_0);
}
