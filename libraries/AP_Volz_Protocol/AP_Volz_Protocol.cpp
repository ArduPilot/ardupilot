/*
 * AP_VOLZ_PROTOCOL.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: guy
 */
#include "AP_Volz_Protocol.h"

#if AP_VOLZ_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define SET_EXTENDED_POSITION_CMD      0xDC

// Extended Position Data Format defines -100 as 0x0080 decimal 128, we map this to a PWM of 1000 (if range is default)
#define PWM_POSITION_MIN               1000
#define ANGLE_POSITION_MIN            -100.0
#define EXTENDED_POSITION_MIN          0x0080

// Extended Position Data Format defines +100 as 0x0F80 decimal 3968, we map this to a PWM of 2000 (if range is default)
#define PWM_POSITION_MAX               2000
#define ANGLE_POSITION_MAX             100.0
#define EXTENDED_POSITION_MAX          0x0F80

#define UART_BUFSIZE_RX                128
#define UART_BUFSIZE_TX                128

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Volz_Protocol::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of volz servo protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16,16:Channel17,17:Channel18,18:Channel19,19:Channel20,20:Channel21,21:Channel22,22:Channel23,23:Channel24,24:Channel25,25:Channel26,26:Channel27,28:Channel29,29:Channel30,30:Channel31,31:Channel32
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_Volz_Protocol, bitmask, 0),

    // @Param: RANGE
    // @DisplayName: Range of travel
    // @Description: Range to map between 1000 and 2000 PWM. Default value of 200 gives full +-100 deg range of extended position command. This results in 0.2 deg movement per US change in PWM. If the full range is not needed it can be reduced to increase resolution. 40 deg range gives 0.04 deg movement per US change in PWM, this is higher resolution than possible with the VOLZ protocol so further reduction in range will not improve resolution. Reduced range does allow PWMs outside the 1000 to 2000 range, with 40 deg range 750 PWM results in a angle of -30 deg, 2250 would be +30 deg. This is still limited by the 200 deg maximum range of the actuator.
    // @Units: deg
    AP_GROUPINFO("RANGE", 2, AP_Volz_Protocol, range, 200),

    AP_GROUPEND
};

// constructor
AP_Volz_Protocol::AP_Volz_Protocol(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Volz_Protocol::init(void)
{
    if (uint32_t(bitmask.get()) == 0) {
        // No servos enabled
        return;
    }

    const AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
    if (port == nullptr) {
        // No port configured
        return;
    }

    // Create thread to handle output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Volz_Protocol::loop, void),
                                          "Volz",
                                           1024, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        AP_BoardConfig::allocation_error("Volz thread");
    }
}

void AP_Volz_Protocol::loop()
{
    const uint32_t baudrate = 115200;
    port->begin(baudrate, UART_BUFSIZE_RX, UART_BUFSIZE_TX);
    port->set_unbuffered_writes(true);
    port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    // Calculate the amount of time it should take to send a command
    // Multiply by 10 to convert from bit rate to byte rate (8 data bits + start and stop bits)
    // B/s to s/B, 1000000 converts to microseconds, multiply by number of bytes
    // 6 bytes at 11520 bytes per second takes 520 us
    const uint16_t send_us =  (sizeof(CMD) * 1000000 * 10) / baudrate;

    // receive packet is same length as sent, double to allow some time for the servo respond
    const uint16_t receive_us = send_us * 2;

    // This gives a total time of 1560ms, message rate of 641 Hz.
    // One servo at 641Hz, two at 320.5 each, three at 213.7 each ect...

    while (port != nullptr) {

        // Wait the expected amount of time for the send and receive to complete so we don't step on the response
        hal.scheduler->delay_microseconds(send_us + receive_us);

        while (port->txspace() < sizeof(CMD)) {
            // Wait until there is enough space to transmit a full command
            hal.scheduler->delay_microseconds(100);
        }

        // loop for all channels
        for (uint8_t i=0; i<ARRAY_SIZE(servo_pwm); i++) {
            // Send each channels in turn
            const uint8_t index = (last_sent_index + 1 + i) % ARRAY_SIZE(servo_pwm);
            if ((uint32_t(bitmask.get()) & (1U<<index)) == 0) {
                // Not configured to send
                continue;
            }
            last_sent_index = index;

            // Get PWM from saved array
            const uint16_t pwm = servo_pwm[index];
            if (pwm == 0) {
                // Never use zero PWM, the check in update should ensure this never happens
                // If we were to use zero the range extrapolation would result in a -100 deg angle request
                continue;
            }

            // Map PWM to angle, this is a un-constrained interpolation
            // ratio = 0 at PWM_POSITION_MIN to 1 at PWM_POSITION_MAX
            const float ratio = (float(pwm) - PWM_POSITION_MIN) / (PWM_POSITION_MAX - PWM_POSITION_MIN);
            // Convert ratio to +-0.5 and multiply by stroke
            const float angle = (ratio - 0.5) * constrain_float(range, 0.0, 200.0);

            // Map angle to command out of full range, add 0.5 so that float to int truncation rounds correctly
            const uint16_t value = linear_interpolate(EXTENDED_POSITION_MIN, EXTENDED_POSITION_MAX, angle, ANGLE_POSITION_MIN, ANGLE_POSITION_MAX) + 0.5;

            // prepare Volz protocol data.
            CMD cmd;
            cmd.ID = SET_EXTENDED_POSITION_CMD;
            cmd.actuator_id = index + 1; // send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
            cmd.arg1 = HIGHBYTE(value);
            cmd.arg2 = LOWBYTE(value);

            send_command(cmd);
            break;
        }
    }
}

// Called each time the servo outputs are sent
void AP_Volz_Protocol::update()
{
    if (!initialised) {
        // One time setup
        initialised = true;
        init();
    }

    if (port == nullptr) {
        // no point if we don't have a valid port
        return;
    }

    // take semaphore and loop for all channels
    for (uint8_t i=0; i<ARRAY_SIZE(servo_pwm); i++) {
        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            continue;
        }
        // 0 PMW should stop outputting, for example in "safe"
        // There is no way to de-power, move to trim
        const uint16_t pwm = c->get_output_pwm();
        servo_pwm[i] = (pwm == 0) ? c->get_trim() : pwm;
    }
}

// calculate CRC for volz serial protocol and send the data.
void AP_Volz_Protocol::send_command(CMD &cmd)
{
    uint16_t crc = 0xFFFF;

    // calculate Volz CRC value according to protocol definition
    for(uint8_t i=0; i<4; i++) {
        // take input data into message that will be transmitted.
        crc = (cmd.data[i] << 8) ^ crc;

        for(uint8_t j=0; j<8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc = crc << 1;
            }
        }
    }

    // add CRC result to the message
    cmd.crc1 = HIGHBYTE(crc);
    cmd.crc2 = LOWBYTE(crc);
    port->write(cmd.data, sizeof(cmd));
}

#endif  // AP_VOLZ_ENABLED
