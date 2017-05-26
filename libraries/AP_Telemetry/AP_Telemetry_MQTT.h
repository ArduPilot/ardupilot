/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANacdacoTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <string>
#include "AP_Telemetry_Backend.h"

#if defined(__cplusplus)
extern "C" {
#endif

#include "define_MQTT.h"
#include "MQTTAsync.h"
#include "LinkedList.h"

#if defined( __cplusplus)
}
#endif

#define MQTT_ENABLED 1

// mqtt send_log on / off
enum Mqtt_send_log {
    MQTT_SEND_LOG_OFF = 0,
    MQTT_SEND_LOG_ON  = 1,
};

// mqtt status for mqtt connection
enum Mqtt_connection_status {
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTED = 1,
};

class AP_Telemetry_MQTT : public AP_Telemetry_Backend {
public:
    // get_MQTTClient - provide the address of the mqtt_client instance
    static MQTTAsync* get_MQTTClient();

    //get_telemetry_mqtt - provide the address of the AP_Telemetry_MQTT's client instance
    static AP_Telemetry_MQTT* get_telemetry_mqtt();

    // init_telemetry_mqtt - initialize the AP_Telemetry_MQTT library and mqtt client
    static AP_Telemetry_MQTT* init_telemetry_mqtt(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart);

    // set_mqtt_xxx - set the parameter for the mqtt connection (read from command line options)
    static void set_mqtt_server(const char* server);
    static void set_mqtt_user(const char* user);
    static void set_mqtt_password(const char* password);

    // update - provide an opportunity to read/send telemetry
    void update() override;

    // send_log - Send gps location on the log topic
    void send_log(const char *str) override;

    // recv_mavlink_message - Read message if any from the wating queue
    int recv_mavlink_message(mavlink_message_t *msg) override;

    // subscribe_mqtt_topic - Mqtt action to directely subscribe to a topic
    void subscribe_mqtt_topic(const char* topic, int qos);

    // send_message - Mqtt action to send a payload on a mqtt topic
    void send_message(const char* str, const char* topic);

    // pop_mqtt_message - Retrieve mqtt message from the waiting queue
    void pop_mqtt_message(char* str_mqtt);

    // append_mqtt_message - Append a message to the waiting queue
    void append_mqtt_message(MQTTAsync_message* message);

    // MQTTHandle_error - Error handler when Mqtt function return a failure
    void MQTTHandle_error(int rc);

    // connection_status - Current status of the mqtt connection
    enum Mqtt_connection_status connection_status = MQTT_DISCONNECTED;

    // send_log_flag - Flag that enable to send mqtt message
    enum Mqtt_send_log send_log_flag = MQTT_SEND_LOG_ON;


private:
    // Private constructor to limit to a single instance
    AP_Telemetry_MQTT(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart);

    // recv_msg_list - Waiting list for mqtt message
    List* recv_msg_list;

    // telemetry_mqtt - Pointer to single instance of the AP_Telemetry_MQTT
    static AP_Telemetry_MQTT* telemetry_mqtt;

    // mqtt_client - Instance of the mqtt client
    static MQTTAsync mqtt_client;

    // conn_options - Structure that store the option for the mqtt connection
    static MQTTAsync_connectOptions conn_options;

    // mqtt_server - Endpoint for the mqtt message broker
    static char const* mqtt_server;

    // init_mqtt - Create and initialize the mqtt connection
    void init_mqtt();

    // mqtt_mutex - Mutex to protect the mqtt critical section
    pthread_mutex_t* mqtt_mutex;
    pthread_mutex_t mqtt_mutex_store;

    // _last_send_ms - Timer to schedule the sending of message
    uint32_t _last_send_ms;
};
