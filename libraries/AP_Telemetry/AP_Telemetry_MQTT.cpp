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

#include "AP_Telemetry_MQTT.h"
#include "define_MQTT.h"
#include <stdio.h>

#include <stdlib.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <string>

#include <getopt.h>

extern const AP_HAL::HAL& hal;

extern int mqtt_to_mavlink_message(const char* cmd, mavlink_message_t *msg);

MQTTAsync AP_Telemetry_MQTT::mqtt_client;
AP_Telemetry_MQTT* AP_Telemetry_MQTT::telemetry_mqtt = nullptr;
MQTTAsync_connectOptions AP_Telemetry_MQTT::conn_options = MQTTAsync_connectOptions_initializer;
char const* AP_Telemetry_MQTT::mqtt_server;
int mqtt_msg_arrived(void *context, char *topicname, int topicLen, MQTTAsync_message* message);
void onConnect(void *context, MQTTAsync_successData* response);
void onConnectFailure(void* context, MQTTAsync_failureData* response);

// MQTT Client accessor
MQTTAsync* AP_Telemetry_MQTT::get_MQTTClient()
{
    return &mqtt_client;
}

AP_Telemetry_MQTT* AP_Telemetry_MQTT::get_telemetry_mqtt()
{
    return telemetry_mqtt;
}

AP_Telemetry_MQTT* AP_Telemetry_MQTT::init_telemetry_mqtt(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart)
{
    if (telemetry_mqtt == nullptr) {
        telemetry_mqtt = new AP_Telemetry_MQTT(frontend, uart);
        telemetry_mqtt->init_mqtt();
    }
    return telemetry_mqtt;
}

AP_Telemetry_MQTT::AP_Telemetry_MQTT(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart) :
    AP_Telemetry_Backend(frontend, uart)
{}

void AP_Telemetry_MQTT::init_mqtt()
{
    int rc;
    mqtt_mutex_store = PTHREAD_MUTEX_INITIALIZER;
    mqtt_mutex = &mqtt_mutex_store;
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);

    recv_msg_list = ListInitialize();
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
    if ((rc = pthread_mutex_init(mqtt_mutex, &attr)) != 0) {
        printf("init: error %d initializing mqtt_mutex\n", rc);
        MQTTHandle_error(rc);
    } else {

        conn_options.keepAliveInterval = 20;
        conn_options.cleansession = 1;

        conn_options.onSuccess = onConnect;
        conn_options.onFailure = onConnectFailure;
        conn_options.context = mqtt_client;

        char clientid[9] = "no_id";
        srand((unsigned int)time(0));
        sprintf(clientid, "client_%d", rand()%100);

        if ((rc = MQTTAsync_create(&mqtt_client, mqtt_server, clientid, MQTTCLIENT_PERSISTENCE_NONE, NULL)) != 0) {
            printf("Failed to create Client, return code %d\n", rc);
            MQTTHandle_error(rc);
        }

        MQTTAsync_setCallbacks(mqtt_client, NULL, NULL, mqtt_msg_arrived, NULL);
        if ((rc = MQTTAsync_connect(mqtt_client, &conn_options)) != MQTTASYNC_SUCCESS) {
            printf("Failed to start connect, return code %d\n", rc);
            MQTTHandle_error(rc);
        }
        connection_status = MQTT_CONNECTED;
    }
}

void AP_Telemetry_MQTT::set_mqtt_server(const char* server)
{
    mqtt_server = server;
}

void AP_Telemetry_MQTT::set_mqtt_user(const char* user)
{
    conn_options.username = user;
}

void AP_Telemetry_MQTT::set_mqtt_password(const char* password)
{
    conn_options.password = password;
}

void AP_Telemetry_MQTT::send_log(const char* str)
{
    char topic[MAX_PAYLOAD];
    if (send_log_flag == MQTT_SEND_LOG_ON) {
        if (connection_status == MQTT_CONNECTED) {
            sprintf(topic, "$ardupilot/copter/quad/log/%04d/location",
                    mavlink_system.sysid);
            send_message(str, topic);
        }
    }
}

void AP_Telemetry_MQTT::send_message(const char *str, const char *topic)
{
    int rc;
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    char tmp[MAX_PAYLOAD];

    strcpy(tmp, str);
    pubmsg.payload = tmp;
    pubmsg.payloadlen = strlen(str);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    if ((rc = MQTTAsync_sendMessage(mqtt_client, topic, &pubmsg, NULL)) != MQTTASYNC_SUCCESS) {
        printf("Failed to start sendMessage, return code %d\n", rc);
        MQTTHandle_error(rc);
    }
}

void AP_Telemetry_MQTT::subscribe_mqtt_topic(const char *topic, int qos)
{
    if (connection_status == MQTT_CONNECTED) {
        int rc;
        if ((rc = MQTTAsync_subscribe(mqtt_client, topic, qos, NULL)) != MQTTASYNC_SUCCESS) {
            printf("Failed to start subscribe, return code %d\n", rc);
            MQTTHandle_error(rc);
        }
    }
}

int AP_Telemetry_MQTT::recv_mavlink_message(mavlink_message_t *msg)
{
    int ret = 0;
    char str_mqtt[MAX_PAYLOAD];
    str_mqtt[0] = '\0';
    AP_Telemetry_MQTT::pop_mqtt_message(str_mqtt);
    if (str_mqtt[0] != '\0') {
        ret = mqtt_to_mavlink_message(str_mqtt, msg);
    }
    return ret;
}

void AP_Telemetry_MQTT::pop_mqtt_message(char* str_mqtt)
{
    MQTTAsync_message* message = nullptr;
    if (pthread_mutex_lock(AP_Telemetry_MQTT::mqtt_mutex) == 0) {
        message = (MQTTAsync_message*)ListDetachHead(recv_msg_list);
        pthread_mutex_unlock(AP_Telemetry_MQTT::mqtt_mutex);
        if (message != nullptr) {
            strncpy(str_mqtt, (char*)message->payload, message->payloadlen);
            str_mqtt[message->payloadlen] = 0;
        }
    }
}

void AP_Telemetry_MQTT::append_mqtt_message(MQTTAsync_message* message)
{
    if (pthread_mutex_lock(mqtt_mutex) == 0) {
        ListAppend(recv_msg_list, message, sizeof(MQTTAsync_message));
        pthread_mutex_unlock(mqtt_mutex);
    } else {
        MQTTAsync_freeMessage(&message);
    }
}

void AP_Telemetry_MQTT::MQTTHandle_error(int rc)
{
    switch (rc) {
    case MQTTASYNC_SUCCESS:
        break;
    default :
        connection_status = MQTT_DISCONNECTED;
        MQTTAsync_reconnect(mqtt_client);
        break;
    }

}

void onConnect(void *context, MQTTAsync_successData* response)
{
    char topic[MAX_TOPIC];
    AP_Telemetry_MQTT* tele_mqtt = AP_Telemetry_MQTT::get_telemetry_mqtt();
    tele_mqtt->connection_status = MQTT_CONNECTED;
    sprintf(topic, "$ardupilot/copter/quad/command/%04d/#", mavlink_system.sysid);
    tele_mqtt->subscribe_mqtt_topic(topic, QOS);
    char payload[MAX_PAYLOAD];
    sprintf(payload, "New client: %04d", mavlink_system.sysid);
    tele_mqtt->send_message(payload, "$ardupilot/identification");
    tele_mqtt->send_log_flag = MQTT_SEND_LOG_OFF;
}

void onConnectFailure(void* context, MQTTAsync_failureData* response)
{
    AP_Telemetry_MQTT::get_telemetry_mqtt()->MQTTHandle_error(MQTTASYNC_DISCONNECTED);
}

int mqtt_msg_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message* message)
{
    AP_Telemetry_MQTT* tele_mqtt = AP_Telemetry_MQTT::get_telemetry_mqtt();
    tele_mqtt->append_mqtt_message(message);
    MQTTAsync_free(topicName);
    return 1;
}

// update - provide an opportunity to read/send telemetry
void AP_Telemetry_MQTT::update()
{
    // exit immediately if no uart
    if (_uart != nullptr && _frontend._ahrs != nullptr) {
        // send telemetry data once per second
        uint32_t now = AP_HAL::millis();
        if (_last_send_ms == 0 || (now - _last_send_ms) > 1000) {
            _last_send_ms = now;
            Location loc;
            if (_frontend._ahrs->get_position(loc)) {
                char buf[100];
                ::sprintf(buf,"lat:%ld lon:%ld alt:%ld\n",
                          (long)loc.lat,
                          (long)loc.lng,
                          (long)loc.alt);
            }
        }
    }
}
