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
#include "Mqtt/MQTTAsync.h"
#include "Mqtt/LinkedList.h"

#if defined( __cplusplus)
}
#endif

#define MQTT_ENABLED 1
 
#define MQTT_SERVICE MQTT_ENABLED


// mqtt send_log on / off
enum Mqtt_send_log {
    MQTT_SEND_LOG_OFF = 0,
    MQTT_SEND_LOG_ON  = 1,
};

//mqtt status for connection
enum Mqtt_connection_status {
  MQTT_DISCONNECTED = 0,
  MQTT_CONNECTED = 1,
};

class AP_Telemetry_MQTT : public AP_Telemetry_Backend
{
public:
  static MQTTAsync* get_MQTTClient();
  static AP_Telemetry_MQTT* get_telemetry_mqtt();
  static AP_Telemetry_MQTT* init_telemetry_mqtt(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart);
  static void set_mqtt_server(const char* server);
  static void set_mqtt_user(const char* user);
  static void set_mqtt_password(const char* password);
  // update - provide an opportunity to read/send telemetry
  void update() override;
  void send_log(const char *str) override;
  int recv_mavlink_message(mavlink_message_t *msg) override;
  void subscribe_mqtt_topic(const char* topic, int qos);
  void send_message(const char* str, const char* topic);
  void pop_mqtt_message(char* str_mqtt);
  void append_mqtt_message(MQTTAsync_message* message);
  enum Mqtt_connection_status connection_status = MQTT_DISCONNECTED;
  enum Mqtt_send_log send_log_flag = MQTT_SEND_LOG_ON;


private:
  AP_Telemetry_MQTT(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart);
  List* recv_msg_list;
  static AP_Telemetry_MQTT* telemetry_mqtt;
  static MQTTAsync mqtt_client;
  static MQTTAsync_connectOptions conn_options;
  void init_mqtt();
  void MQTTHandle_error(int rc);
  pthread_mutex_t* mqtt_mutex;
  pthread_mutex_t mqtt_mutex_store;
  int mqtt_send_log_timer_val;
  int mqtt_send_log_timer;
  uint32_t _last_send_ms;
};
