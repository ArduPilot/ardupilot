/*
 AnalogReadSerial
 Reads an analog input on pin 0, prints the result to the serial monitor

 This example code is in the public domain.
 */

#include <GCS_MAVLink.h>

int packetDrops = 0;

void handleMessage(mavlink_message_t * msg) {
    Serial.print(", received mavlink message: ");
    Serial.print(msg->msgid, DEC);
}

void setup() {
    Serial.begin(57600);
    Serial3.begin(57600);
    mavlink_comm_0_port = &Serial3;
    packetDrops = 0;
}

void loop() {
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlink_system.type,
                               MAV_AUTOPILOT_ARDUPILOTMEGA);
    Serial.print("heartbeat sent");

    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;

    Serial.print(", bytes available: ");
    Serial.print(comm_get_available(MAVLINK_COMM_0));
    while (comm_get_available( MAVLINK_COMM_0)) {
        uint8_t c = comm_receive_ch(MAVLINK_COMM_0);

        // Try to get a new message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
            handleMessage(&msg);
    }

    // Update packet drops counter
    packetDrops += status.packet_rx_drop_count;

    Serial.print(", dropped packets: ");
    Serial.println(packetDrops);
    delay(1000);
}
