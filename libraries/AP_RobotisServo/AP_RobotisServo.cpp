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
/*
  implementation of Robotis Dynamixel 2.0 protocol for controlling servos

  Portions of this code are based on the dynamixel_sdk code:
  https://github.com/ROBOTIS-GIT/DynamixelSDK
  which is under the following license:

* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "AP_RobotisServo.h"

#if AP_ROBOTISSERVO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define BROADCAST_ID 0xFE
#define MAX_ID 0xFC

// DXL protocol common commands
#define INST_PING          1
#define INST_READ          2
#define INST_WRITE         3
#define INST_REG_WRITE     4
#define INST_ACTION        5
#define INST_FACTORY_RESET 6
#define INST_CLEAR        16
#define INST_SYNC_WRITE  131
#define INST_BULK_READ   146

// 2.0 protocol commands
#define INST_REBOOT       8
#define INST_STATUS      85
#define INST_SYNC_READ  130
#define INST_BULK_WRITE 147

// 2.0 protocol packet offsets
#define PKT_HEADER0     0
#define PKT_HEADER1     1
#define PKT_HEADER2     2
#define PKT_RESERVED    3
#define PKT_ID          4
#define PKT_LENGTH_L    5
#define PKT_LENGTH_H    6
#define PKT_INSTRUCTION 7
#define PKT_ERROR       8
#define PKT_PARAMETER0  8

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// register offsets
#define REG_OPERATING_MODE 11
#define   OPMODE_CURR_CONTROL    0
#define   OPMODE_VEL_CONTROL     1
#define   OPMODE_POS_CONTROL     3
#define   OPMODE_EXT_POS_CONTROL 4

#define REG_TORQUE_ENABLE  64

#define REG_STATUS_RETURN  68
#define   STATUS_RETURN_NONE 0
#define   STATUS_RETURN_READ 1
#define   STATUS_RETURN_ALL  2

#define REG_GOAL_POSITION 116

// how many times to send servo configure msgs
#define CONFIGURE_SERVO_COUNT 4

// how many times to send servo detection
#define DETECT_SERVO_COUNT 4

const AP_Param::GroupInfo AP_RobotisServo::var_info[] = {

    // @Param: POSMIN
    // @DisplayName: Robotis servo position min
    // @Description: Position minimum at servo min value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMIN",  1, AP_RobotisServo, pos_min, 0),

    // @Param: POSMAX
    // @DisplayName: Robotis servo position max
    // @Description: Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMAX",  2, AP_RobotisServo, pos_max, 4095),
    
    AP_GROUPEND
};

// constructor
AP_RobotisServo::AP_RobotisServo(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_RobotisServo::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Robotis,0);
    if (port) {
        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Robotis, 0);
        us_per_byte = 10 * 1e6 / baudrate;
        us_gap = 4 * 1e6 / baudrate;
    }
}

/*
  addStuffing() from Robotis SDK. This pads the packet as required by the protocol
*/
void AP_RobotisServo::add_stuffing(uint8_t *packet)
{
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
  
    if (packet_length_in < 8) {
        // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
        return;
    }

    uint8_t *packet_ptr;
    uint16_t packet_length_before_crc = packet_length_in - 2;
    for (uint16_t i = 3; i < packet_length_before_crc; i++) {
        packet_ptr = &packet[i+PKT_INSTRUCTION-2];
        if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD) {
            packet_length_out++;
        }
    }
  
    if (packet_length_in == packet_length_out) {
        // no stuffing required
        return;
    }
  
    uint16_t out_index  = packet_length_out + 6 - 2;  // last index before crc
    uint16_t in_index   = packet_length_in + 6 - 2;   // last index before crc

    while (out_index != in_index) {
        if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF) {
            packet[out_index--] = 0xFD; // byte stuffing
            if (out_index != in_index) {
                packet[out_index--] = packet[in_index--]; // FD
                packet[out_index--] = packet[in_index--]; // FF
                packet[out_index--] = packet[in_index--]; // FF
            }
        } else {
            packet[out_index--] = packet[in_index--];
        }
    }

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

/*
  send a protocol 2.0 packet
 */
void AP_RobotisServo::send_packet(uint8_t *txpacket)
{
    add_stuffing(txpacket);

    // check max packet length
    uint16_t total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;

    // make packet header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]  = 0x00;

    // add CRC16
    uint16_t crc = crc_crc16_ibm(0, txpacket, total_packet_length - 2);    // 2: CRC16
    txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

    port->write(txpacket, total_packet_length);

    delay_time_us += total_packet_length * us_per_byte + us_gap;
}

/*
  use a broadcast ping to find attached servos
 */
void AP_RobotisServo::detect_servos(void)
{
    uint8_t txpacket[10] {};

    txpacket[PKT_ID] = BROADCAST_ID;
    txpacket[PKT_LENGTH_L] = 3;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_PING;

    send_packet(txpacket);

    // give plenty of time for replies from all servos
    last_send_us = AP_HAL::micros();
    delay_time_us += 1000 * us_per_byte;
}

/*
  broadcast configure all servos
 */
void AP_RobotisServo::configure_servos(void)
{
    // disable torque control
    send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);

        // disable replies unless we read
    send_command(BROADCAST_ID, REG_STATUS_RETURN, STATUS_RETURN_READ, 1);

    // use position control mode
    send_command(BROADCAST_ID, REG_OPERATING_MODE, OPMODE_POS_CONTROL, 1);

    // enable torque control
    send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 1);
}


/*
  send a command to a single servo, changing a register value
 */
void AP_RobotisServo::send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len)
{
    uint8_t txpacket[16] {};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH_L] = 5 + len;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_WRITE;
    txpacket[PKT_INSTRUCTION+1] = DXL_LOBYTE(reg);
    txpacket[PKT_INSTRUCTION+2] = DXL_HIBYTE(reg);

    // Register values are transmitted as little-endian.
    value = htole32(value);
    memcpy(&txpacket[PKT_INSTRUCTION+3], &value, MIN(len,4));

    send_packet(txpacket);
}

/*
  read response bytes
 */
void AP_RobotisServo::read_bytes(void)
{
    uint32_t n = port->available();
    if (n == 0 && pktbuf_ofs < PKT_INSTRUCTION) {
        return;
    }

    if (n > sizeof(pktbuf) - pktbuf_ofs) {
        n = sizeof(pktbuf) - pktbuf_ofs;
    }
    for (uint8_t i=0; i<n; i++) {
        pktbuf[pktbuf_ofs++] = port->read();
    }

    // discard bad leading data. This should be rare
    while (pktbuf_ofs >= 4 &&
           (pktbuf[0] != 0xFF || pktbuf[1] != 0xFF || pktbuf[2] != 0xFD || pktbuf[3] != 0x00)) {
        memmove(pktbuf, &pktbuf[1], pktbuf_ofs-1);
        pktbuf_ofs--;
    }

    if (pktbuf_ofs < 10) {
        // not enough data yet
        return;
    }

    const uint16_t total_packet_length = DXL_MAKEWORD(pktbuf[PKT_LENGTH_L], pktbuf[PKT_LENGTH_H]) + PKT_INSTRUCTION;
    if (total_packet_length > sizeof(pktbuf)) {
        pktbuf_ofs = 0;
        return;
    }
    if (pktbuf_ofs < total_packet_length) {
        // more data needed
        return;
    }

    // check CRC
    const uint16_t crc = DXL_MAKEWORD(pktbuf[total_packet_length-2], pktbuf[total_packet_length-1]);
    const uint16_t calc_crc = crc_crc16_ibm(0, pktbuf, total_packet_length - 2);
    if (calc_crc != crc) {
        memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
        pktbuf_ofs -= total_packet_length;
        return;
    }

    // process full packet
    process_packet(pktbuf, total_packet_length);

    memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
    pktbuf_ofs -= total_packet_length;
}

/*
  process a packet from a servo
 */
void AP_RobotisServo::process_packet(const uint8_t *pkt, uint8_t length)
{
    uint8_t id = pkt[PKT_ID];
    if (id > 16 || id < 1) {
        // discard packets from servos beyond max or min. Note that we
        // don't allow servo 0, to make mapping to SERVOn_* parameters
        // easier
        return;
    }
    uint32_t id_mask = (1U<<(id-1));
    if (!(id_mask & servo_mask)) {
        // mark the servo as present
        servo_mask |= id_mask;
        DEV_PRINTF("Robotis: new servo %u\n", id);
    }
}


void AP_RobotisServo::update()
{
    if (!initialised) {
        initialised = true;
        init();
        last_send_us = AP_HAL::micros();
        return;
    }
    
    if (port == nullptr) {
        return;
    }

    read_bytes();

    uint32_t now = AP_HAL::micros();
    if (last_send_us != 0 && now - last_send_us < delay_time_us) {
        // waiting for last send to complete
        return;
    }

    if (detection_count < DETECT_SERVO_COUNT) {
        detection_count++;
        detect_servos();
        return;
    }

    if (servo_mask == 0) {
        return;
    }

    if (configured_servos < CONFIGURE_SERVO_COUNT) {
        configured_servos++;
        last_send_us = now;
        configure_servos();
        return;
    }

    last_send_us = now;
    delay_time_us = 0;

    // loop for all 16 channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (((1U<<i) & servo_mask) == 0) {
            continue;
        }
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            continue;
        }
        const uint16_t pwm = c->get_output_pwm();
        const uint16_t min = c->get_output_min();
        const uint16_t max = c->get_output_max();
        float v = float(pwm - min) / (max - min);
        uint32_t value = pos_min + v * (pos_max - pos_min);
        send_command(i+1, REG_GOAL_POSITION, value, 4);
    }
}

#endif  // AP_ROBOTISSERVO_ENABLED
