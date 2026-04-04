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
  Send and receive JSON backend data to alow a second AP instance to ride along
*/

#include "SIM_JSON_Master.h"

#if AP_SIM_JSON_MASTER_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <errno.h>
#include <stdio.h>
#include <SITL/SITL.h>

using namespace SITL;

void JSON_Master::init(const int32_t num_slaves)
{
    socket_list *list = &_list;
    uint8_t i = 1;
    for (i = 1 ; i <= num_slaves; i++) {
        // init each socket and instance
        list->instance = i;
        uint16_t port = 9002 + 10 * i;

        if (!list->sock_in.reuseaddress()) {
            AP_HAL::panic("JSON master: socket reuseaddress failed on port: %d - %s", port, strerror(errno));
        }

        if (!list->sock_in.bind("127.0.0.1", port)) {
            AP_HAL::panic("JSON master: socket reuseaddress failed on port: %d - %s", port, strerror(errno));
        }

        if (!list->sock_in.set_blocking(false)) {
            AP_HAL::panic( "JSON master: socket set_blocking(false) failed on port: %d - %s", port, strerror(errno));
        }

        printf("Slave %u: listening on %u\n", list->instance, port);
        list->next = NEW_NOTHROW socket_list;
        list = list->next;

        initialized = true;
    }
}

// Receive PWM outs from ride along controlers
void JSON_Master::receive(struct sitl_input &input)
{
    if (!initialized) {
        return;
    }

    uint8_t master_instance = AP::sitl()->ride_along_master.get();

    for (socket_list *list = &_list; list->next; list=list->next) {
        // cycle through all ride along instances
        struct servo_packet {
            uint16_t magic;
            uint16_t frame_rate;
            uint32_t frame_count;
            uint16_t pwm[16];
        } buffer{};

        while (true) {
            ssize_t ret = list->sock_in.recv(&buffer, sizeof(buffer), 100);
            if (ret == 0) {
                // wait some more
                continue;
            }
            if (buffer.magic != 18458) {
                // magic value does not match
                continue;
            }
            if (!list->connected) {
                // connect back to the last address for send
                uint16_t port;
                const char *_ip = nullptr;
                list->sock_in.last_recv_address(_ip, port);
                list->connected = list->sock_out.connect(_ip, port) && (port != 0);
                if (list->connected) {
                    printf("Slave %u connected to %s:%u\n", list->instance, _ip, port);
                }
            }
            if (list->connected) {
                break;
            }
        }

#if HAL_LOGGING_ENABLED
        const bool use_servos = list->instance == master_instance;

// @LoggerMessage: SLV1
// @Description: Log data received from JSON simulator 1
// @Field: TimeUS: Time since system startup (us)
// @Field: Instance: Slave instance
// @Field: magic: magic JSON protocol key
// @Field: frame_rate: Slave instance's desired frame rate
// @Field: frame_count: Slave instance's current frame count
// @Field: active: 1 if the servo outputs are being used from this instance
        AP::logger().WriteStreaming("SLV1", "TimeUS,Instance,magic,frame_rate,frame_count,active",
                       "s#----",
                       "F?????",
                       "QBHHIB",
                       AP_HAL::micros64(),
                       list->instance,
                       buffer.magic,
                       buffer.frame_rate,
                       buffer.frame_count,
                       use_servos);

// @LoggerMessage: SLV2
// @Description: Log data received from JSON simulator 2
// @Field: TimeUS: Time since system startup
// @Field: Instance: Slave instance
// @Field: C1: channel 1 output
// @Field: C2: channel 2 output
// @Field: C3: channel 3 output
// @Field: C4: channel 4 output
// @Field: C5: channel 5 output
// @Field: C6: channel 6 output
// @Field: C7: channel 7 output
// @Field: C8: channel 8 output
// @Field: C9: channel 9 output
// @Field: C10: channel 10 output
// @Field: C11: channel 11 output
// @Field: C12: channel 12 output
// @Field: C13: channel 13 output
// @Field: C14: channel 14 output
        AP::logger().WriteStreaming("SLV2", "TimeUS,Instance,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
                       "s#YYYYYYYYYYYYYY",
                       "F?--------------",
                       "QBHHHHHHHHHHHHHH",
                       AP_HAL::micros64(),
                       list->instance,
                       buffer.pwm[0],
                       buffer.pwm[1],
                       buffer.pwm[2],
                       buffer.pwm[3],
                       buffer.pwm[4],
                       buffer.pwm[5],
                       buffer.pwm[6],
                       buffer.pwm[7],
                       buffer.pwm[8],
                       buffer.pwm[9],
                       buffer.pwm[10],
                       buffer.pwm[11],
                       buffer.pwm[12],
                       buffer.pwm[13]);
#endif

        if (list->instance == master_instance) {
            // Use the servo outs from this instance
            memcpy(input.servos,buffer.pwm,sizeof(buffer.pwm));
        }
    }
}

// send vehicle state to ride along controlers
void JSON_Master::send(const struct sitl_fdm &output, const Vector3d &position)
{
    if (!initialized) {
        return;
    }

    // message is the same to all slaves
    int length = asprintf(&json_out,"\n{\"timestamp\":%f,\"imu\":{\"gyro\":[%f,%f,%f],\"accel_body\":[%f,%f,%f]},\"position\":[%f,%f,%f],\"quaternion\":[%f,%f,%f,%f],\"velocity\":[%f,%f,%f],\"no_time_sync\":1}\n",
                            output.timestamp_us * 1e-6,
                            radians(output.rollRate), radians(output.pitchRate), radians(output.yawRate),
                            output.xAccel, output.yAccel, output.zAccel,
                            position.x, position.y, position.z,
                            output.quaternion.q1, output.quaternion.q2, output.quaternion.q3, output.quaternion.q4,
                            output.speedN, output.speedE, output.speedD);

    for (socket_list *list = &_list; list->next; list=list->next) {
        list->sock_out.send(json_out,length);
    }
    free(json_out);
}


#endif  // AP_SIM_JSON_MASTER_ENABLED
