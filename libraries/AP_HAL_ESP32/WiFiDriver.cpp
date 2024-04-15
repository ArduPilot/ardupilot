/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL_ESP32/WiFiDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ESP32/Scheduler.h>

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

WiFiDriver::WiFiDriver()
{
    _state = NOT_INITIALIZED;
    accept_socket = -1;

    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
        socket_list[i] = -1;
    }
}

void WiFiDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_state == NOT_INITIALIZED) {
        initialize_wifi();

    // keep main tasks that need speed on CPU 0
    // pin potentially slow stuff to CPU 1, as we have disabled the WDT on that core.
    #define FASTCPU 0
    #define SLOWCPU 1

	if (xTaskCreatePinnedToCore(_wifi_thread, "APM_WIFI1", Scheduler::WIFI_SS1, this, Scheduler::WIFI_PRIO1, &_wifi_task_handle,FASTCPU) != pdPASS) {
           hal.console->printf("FAILED to create task _wifi_thread on FASTCPU\n");
        } else {
           hal.console->printf("OK created task _wifi_thread for TCP with PORT 5760 on FASTCPU\n");
        }

        _readbuf.set_size(RX_BUF_SIZE);
        _writebuf.set_size(TX_BUF_SIZE);
        _state = INITIALIZED;
    }
}

void WiFiDriver::_end()
{
    //TODO
}

void WiFiDriver::_flush()
{
}

bool WiFiDriver::is_initialized()
{
    return _state != NOT_INITIALIZED;
}

bool WiFiDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t WiFiDriver::_available()
{
    if (_state != CONNECTED) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t WiFiDriver::txspace()
{
    if (_state != CONNECTED) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

ssize_t WiFiDriver::_read(uint8_t *buf, uint16_t count)
{
    if (_state != CONNECTED) {
        return 0;
    }
    return _readbuf.read(buf, count);
}

bool WiFiDriver::start_listen()
{
    accept_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (accept_socket < 0) {
        accept_socket = -1;
        return false;
    }
    int opt;
    setsockopt(accept_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(5760);
    int err = bind(accept_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        close(accept_socket);
        accept_socket = 0;
        return false;
    }
    err = listen(accept_socket, 5);
    if (err != 0) {
        close(accept_socket);
        accept_socket = -1;
        return false;
    }
    return true;

}

bool WiFiDriver::try_accept()
{
    struct sockaddr_in sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    short i = available_socket();
    if (i != WIFI_MAX_CONNECTION) {
        socket_list[i] = accept(accept_socket, (struct sockaddr *)&sourceAddr, &addrLen);
        if (socket_list[i] >= 0) {
            fcntl(socket_list[i], F_SETFL, O_NONBLOCK);
            return true;
        }
    }
    return false;
}

bool WiFiDriver::read_data()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        do {
            count = recv(socket_list[i], (void *)_buffer, sizeof(_buffer), 0);
            if (count > 0) {
                _readbuf.write(_buffer, count);
                if (count == sizeof(_buffer)) {
                    _more_data = true;
                }
            } else if (count < 0 && errno != EAGAIN) {
                shutdown(socket_list[i], 0);
                close(socket_list[i]);
                socket_list[i] = -1;
                _state = INITIALIZED;
                return false;
            }
        } while (count > 0);
    }
    return true;
}

bool WiFiDriver::write_data()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        _write_mutex.take_blocking();
        do {
            count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
            if (count > 0) {
                count = send(socket_list[i], (void*) _buffer, count, 0);
                if (count > 0) {
                    _writebuf.advance(count);
                    if (count == sizeof(_buffer)) {
                        _more_data = true;
                    }
                } else if (count < 0 && errno != EAGAIN) {
                    shutdown(socket_list[i], 0);
                    close(socket_list[i]);
                    socket_list[i] = -1;
                    _state = INITIALIZED;
                    _write_mutex.give();
                    return false;
                }
            }
        } while (count > 0);
    }
    _write_mutex.give();
    return true;
}

#if WIFI_STATION

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#ifndef ESP_STATION_MAXIMUM_RETRY
#define ESP_STATION_MAXIMUM_RETRY 10
#endif

static const char *TAG = "wifi station";
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

static void _sta_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_STATION_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
#endif

void WiFiDriver::initialize_wifi()
{
#ifndef WIFI_PWD
    #default WIFI_PWD "ardupilot1"
#endif
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config));

/*
	Acting as an Access Point (softAP)
*/
#if !WIFI_STATION
#ifndef WIFI_SSID
    #define WIFI_SSID "ardupilot"
#endif
#ifndef WIFI_CHANNEL
	#define WIFI_CHANNEL 1
#endif

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    strcpy((char *)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.ap.password, WIFI_PWD);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID),
    wifi_config.ap.max_connection = WIFI_MAX_CONNECTION,
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.channel = WIFI_CHANNEL;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    hal.console->printf("WiFi softAP init finished. SSID: %s password: %s channel: %d\n",
                        wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);

/*
	Acting as a Station (WiFi Client)
*/
#else
#ifndef WIFI_SSID_STATION
    #define WIFI_SSID_STATION "ardupilot"
#endif
#ifndef WIFI_HOSTNAME
    #define WIFI_HOSTNAME "ArduPilotESP32"
#endif
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, WIFI_HOSTNAME);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &_sta_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &_sta_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID_STATION);
    strcpy((char *)wifi_config.sta.password, WIFI_PWD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    hal.console->printf("WiFi Station init finished. Connecting:\n");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID: %s password: %s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s, password: %s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
#endif
}

size_t WiFiDriver::_write(const uint8_t *buffer, size_t size)
{
    if (_state != CONNECTED) {
        return 0;
    }
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

void WiFiDriver::_wifi_thread(void *arg)
{
    WiFiDriver *self = (WiFiDriver *) arg;
    if (!self->start_listen()) {
        vTaskDelete(nullptr);
    }
    while (true) {
        if (self->try_accept()) {
            self->_state = CONNECTED;
            while (true) {
                self->_more_data = false;
                if (!self->read_data()) {
                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->write_data()) {
                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->_more_data) {
                    hal.scheduler->delay_microseconds(1000);
                }
            }
        }
        hal.scheduler->delay_microseconds(10); // don't flog the thread if nothing to accept.

    }
}

bool WiFiDriver::_discard_input()
{
    return false;
}

unsigned short WiFiDriver::available_socket()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i)
        if (socket_list[i] == -1) {
            return i;
        }

    return WIFI_MAX_CONNECTION;
}
