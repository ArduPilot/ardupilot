#include <AP_HAL_ESP32/WiFiDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ESP32/Scheduler.h>

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
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
    read_socket = -1;
    accept_socket = -1;
}

void WiFiDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void WiFiDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_state == NOT_INITIALIZED) {
        initialize_wifi();
        xTaskCreate(_wifi_thread, "APM_WIFI", Scheduler::WIFI_SS, this, Scheduler::WIFI_PRIO, &_wifi_task_handle);
        _readbuf.set_size(RX_BUF_SIZE);
        _writebuf.set_size(TX_BUF_SIZE);
        _state = INITIALIZED;
    }
}

void WiFiDriver::end()
{
    //TODO
}

void WiFiDriver::flush()
{
}

bool WiFiDriver::is_initialized()
{
    return _state != NOT_INITIALIZED;
}

void WiFiDriver::set_blocking_writes(bool blocking)
{
    //blocking writes do not used anywhere
}

bool WiFiDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t WiFiDriver::available()
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

int16_t WiFiDriver::read()
{
    if (_state != CONNECTED) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    return byte;
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
    read_socket = accept(accept_socket, (struct sockaddr *)&sourceAddr, &addrLen);
    if (read_socket >= 0) {
        fcntl(read_socket, F_SETFL, O_NONBLOCK);
        return true;
    }
    return false;
}

bool WiFiDriver::read_data()
{
    int count = 0;
    do {
        count = recv(read_socket, (void *)_buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
            if (count == sizeof(_buffer)) {
                _more_data = true;
            }
        } else if (count < 0 &&  errno != EAGAIN) {
            shutdown(read_socket, 0);
            close(read_socket);
            _state = INITIALIZED;
            return false;

        }
    } while (count > 0);
    return true;
}

bool WiFiDriver::write_data()
{
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = send(read_socket, (void*) _buffer, count, 0);
            if (count > 0) {
                _writebuf.advance(count);
                if (count == sizeof(_buffer)) {
                    _more_data = true;
                }
            } else if (count < 0 && errno != EAGAIN) {
                shutdown(read_socket, 0);
                close(read_socket);
                _state = INITIALIZED;
                _write_mutex.give();
                return false;
            }
        }
    } while (count > 0);
    _write_mutex.give();
    return true;
}

void WiFiDriver::initialize_wifi()
{
    tcpip_adapter_init();
    nvs_flash_init();
    esp_event_loop_init(nullptr, nullptr);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char *)wifi_config.ap.ssid, "ardupilot");
    strcpy((char *)wifi_config.ap.password, "ardupilot1");
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 4;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    esp_wifi_start();
}

size_t WiFiDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t WiFiDriver::write(const uint8_t *buffer, size_t size)
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
    }
}
