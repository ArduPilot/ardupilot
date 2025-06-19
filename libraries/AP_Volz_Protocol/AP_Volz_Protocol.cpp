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

#include <AP_Servo_Telem/AP_Servo_Telem.h>

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

    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
    if (port == nullptr) {
        // No port configured
        return;
    }

    // update baud param in case user looks at it
    serial_manager.set_and_default_baud(AP_SerialManager::SerialProtocol_Volz, 0, 115200);

    // Create thread to handle output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Volz_Protocol::loop, void),
                                          "Volz",
                                           1024, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        AP_BoardConfig::allocation_error("Volz thread");
    }
}

#if AP_SERVO_TELEM_ENABLED
// Request telem data, cycling through each servo and telem item
void AP_Volz_Protocol::request_telem()
{
    // Request the queued item, making sure the servo is enabled
    if ((uint32_t(bitmask.get()) & (1U<<telem.actuator_id)) != 0) {
        // Assemble command
        CMD cmd {};
        cmd.ID = telem.types[telem.request_type];
        cmd.actuator_id = telem.actuator_id + 1;
        send_command(cmd);

        // Increment the request type
        telem.request_type++;
        if (telem.request_type < ARRAY_SIZE(telem.types)) {
            // Request the next telem type from the same actuators on the next call
            return;
        }
    }

    // Requested all items from a id or invalid id.
    // start again with a new id
    telem.request_type = 0;

    // Same logic as `send_position_cmd`
    for (uint8_t i=0; i<ARRAY_SIZE(telem.data); i++) {
        const uint8_t index = (telem.actuator_id + 1 + i) % ARRAY_SIZE(telem.data);
        if ((uint32_t(bitmask.get()) & (1U<<index)) == 0) {
            continue;
        }
        telem.actuator_id = index;
        break;
    }
}
#endif

void AP_Volz_Protocol::loop()
{
    const uint32_t baudrate = 115200;
    port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_RTS_DE);
    port->begin(baudrate, UART_BUFSIZE_RX, UART_BUFSIZE_TX);
    port->set_unbuffered_writes(true);

    // Calculate the amount of time it should take to send a command
    // Multiply by 10 to convert from bit rate to byte rate (8 data bits + start and stop bits)
    // B/s to s/B, 1000000 converts to microseconds, multiply by number of bytes
    // 6 bytes at 11520 bytes per second takes 520 us
    const uint16_t send_us =  (sizeof(CMD) * 1000000 * 10) / baudrate;

    // receive packet is same length as sent, double to allow some time for the servo respond
    const uint16_t receive_us = send_us * 2;

    // This gives a total time of 1560us, message rate of 641 Hz.
    // One servo at 641Hz, two at 320.5 each, three at 213.7 each ect...
    // Note that we send a telem request every time servo sending is complete. This is like a extra servo.
    // So for a single servo position commands are at 320.5Hz and telem at 320.5Hz.

    while (port != nullptr) {

        // Wait the expected amount of time for the send and receive to complete so we don't step on the response
        hal.scheduler->delay_microseconds(send_us + receive_us);

        while (port->txspace() < sizeof(CMD)) {
            // Wait until there is enough space to transmit a full command
            hal.scheduler->delay_microseconds(100);
        }

#if AP_SERVO_TELEM_ENABLED
        // Send a command for each servo, then one telem request
        const uint8_t num_servos = __builtin_popcount(bitmask.get());
        if (sent_count < num_servos) {
            send_position_cmd();
            sent_count++;

        } else {
            request_telem();
            sent_count = 0;
        }

        {
            // Read in telem data if available
            WITH_SEMAPHORE(telem.sem);
            read_telem();
        }

#else // No telem, send only
        send_position_cmd();
#endif
    }
}

// Send postion commands from PWM, cycle through each servo
void AP_Volz_Protocol::send_position_cmd()
{

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
        cmd.ID = CMD_ID::SET_EXTENDED_POSITION;
        cmd.actuator_id = index + 1; // send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
        cmd.arg1 = HIGHBYTE(value);
        cmd.arg2 = LOWBYTE(value);

        send_command(cmd);

#if AP_SERVO_TELEM_ENABLED
        {
            // Update the commanded angle
            WITH_SEMAPHORE(telem.sem);
            static_assert(ARRAY_SIZE(servo_pwm) == ARRAY_SIZE(telem.data), "actuator index invalid for telem data array");
            telem.data[index].desired_angle = angle;
        }
#endif

        break;
    }
}

#if AP_SERVO_TELEM_ENABLED
void AP_Volz_Protocol::process_response(const CMD &cmd)
{
    // Convert to 0 indexed
    const uint8_t index = cmd.actuator_id - 1;
    if (index >= ARRAY_SIZE(telem.data)) {
        // Invalid ID
        return;
    }

    switch (cmd.ID) {
    case CMD_ID::EXTENDED_POSITION_RESPONSE:
        // Map back to angle
        telem.data[index].angle = linear_interpolate(ANGLE_POSITION_MIN, ANGLE_POSITION_MAX, UINT16_VALUE(cmd.arg1, cmd.arg2), EXTENDED_POSITION_MIN, EXTENDED_POSITION_MAX);
        break;

    case CMD_ID::CURRENT_RESPONSE:
        // Current is reported in 20mA increments (0.02A)
        telem.data[index].primary_current = cmd.arg1 * 0.02;
        telem.data[index].secondary_current = cmd.arg2 * 0.02;
        break;

    case CMD_ID::VOLTAGE_RESPONSE:
        // Voltage is reported in 200mv increments (0.2v)
        telem.data[index].primary_voltage = cmd.arg1 * 0.2;
        telem.data[index].secondary_voltage = cmd.arg2 * 0.2;
        break;

    case CMD_ID::TEMPERATURE_RESPONSE:
        // Temperature is reported relative to -50 deg C
        telem.data[index].motor_temp_deg = -50 + cmd.arg1;
        telem.data[index].pcb_temp_deg = -50 + cmd.arg2;
        break;

    default:
        // This should never happen
        return;
    }

    telem.data[index].last_response_ms = AP_HAL::millis();
}

// Return true if the given ID is a valid response
bool AP_Volz_Protocol::is_response(uint8_t ID) const
{
    switch(ID) {
    case (uint8_t)CMD_ID::EXTENDED_POSITION_RESPONSE:
    case (uint8_t)CMD_ID::CURRENT_RESPONSE:
    case (uint8_t)CMD_ID::VOLTAGE_RESPONSE:
    case (uint8_t)CMD_ID::TEMPERATURE_RESPONSE:
        return true;

    default:
        break;
    }

    return false;
}

void AP_Volz_Protocol::read_telem()
{
    // Try and read data a few times, this could be a while loop, using a for loop gives a upper bound to run time
    for (uint8_t attempts = 0; attempts < sizeof(telem.cmd_buffer) * 4; attempts++) {

        uint32_t n = port->available();
        if (n == 0) {
            // No data available
            return;
        }
        if (telem.buffer_offset < sizeof(telem.cmd_buffer)) {
            // Read enough bytes to fill buffer
            ssize_t nread = port->read(&telem.cmd_buffer.data[telem.buffer_offset], MIN(n, unsigned(sizeof(telem.cmd_buffer)-telem.buffer_offset)));
            if (nread <= 0) {
                // Read failed
                return;
            }
            telem.buffer_offset += nread;
        }

        // Check for valid response start byte
        if (!is_response(telem.cmd_buffer.data[0])) {

            // Search for a valid response start byte
            uint8_t cmd_start;
            for (cmd_start = 1; cmd_start < telem.buffer_offset; cmd_start++) {
                if (is_response(telem.cmd_buffer.data[cmd_start])) {
                    // Found one
                    break;
                }
            }

            // Shift buffer to put start on valid byte, or if no valid byte was found clear
            const uint8_t n_move = telem.buffer_offset - cmd_start;
            if (n_move > 0) {
                // No need to move 0 bytes
                memmove(&telem.cmd_buffer.data[0], &telem.cmd_buffer.data[cmd_start], n_move);
            }
            telem.buffer_offset = 0;

            // Since the buffer is the same length as a full command, we can never get a valid packet after shifting
            // Always need to read in some more data
            continue;
        }

        if (telem.buffer_offset < sizeof(telem.cmd_buffer)) {
            // Not enough data to make up packet
            continue;
        }

        // Have valid ID and enough data, check crc
        if (UINT16_VALUE(telem.cmd_buffer.crc1, telem.cmd_buffer.crc2) != calculate_crc(telem.cmd_buffer)) {
            // Probably lost sync shift by one and try again
            memmove(&telem.cmd_buffer.data[0], &telem.cmd_buffer.data[1], telem.buffer_offset - 1);
            telem.buffer_offset -= 1;
            continue;
        }

        // Valid packet passed crc check
        process_response(telem.cmd_buffer);

        // zero offset and continue
        telem.buffer_offset = 0;
    }

    // Used up all attempts without running out of data.
    // Really should not end up here
}
#endif // AP_SERVO_TELEM_ENABLED

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

#if AP_SERVO_TELEM_ENABLED
    // Report telem data
    AP_Servo_Telem *servo_telem = AP_Servo_Telem::get_singleton();
    if (servo_telem != nullptr) {
        const uint32_t now_ms = AP_HAL::millis();

        WITH_SEMAPHORE(telem.sem);
        for (uint8_t i=0; i<ARRAY_SIZE(telem.data); i++) {
            if ((telem.data[i].last_response_ms == 0) || ((now_ms - telem.data[i].last_response_ms) > 5000)) {
                // Never seen telem, or not had a response for more than 5 seconds
                continue;
            }

            const AP_Servo_Telem::TelemetryData telem_data {
                .command_position = telem.data[i].desired_angle,
                .measured_position = telem.data[i].angle,
                .voltage = telem.data[i].primary_voltage,
                .current = telem.data[i].primary_current,
                .motor_temperature_cdeg = int16_t(telem.data[i].motor_temp_deg * 100),
                .pcb_temperature_cdeg = int16_t(telem.data[i].pcb_temp_deg * 100),
                .present_types = AP_Servo_Telem::TelemetryData::Types::COMMANDED_POSITION |
                                 AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION |
                                 AP_Servo_Telem::TelemetryData::Types::VOLTAGE |
                                 AP_Servo_Telem::TelemetryData::Types::CURRENT |
                                 AP_Servo_Telem::TelemetryData::Types::MOTOR_TEMP |
                                 AP_Servo_Telem::TelemetryData::Types::PCB_TEMP
            };

            servo_telem->update_telem_data(i, telem_data);
        }
    }
#endif // AP_SERVO_TELEM_ENABLED
}

// Return the crc for a given command packet
uint16_t AP_Volz_Protocol::calculate_crc(const CMD &cmd) const
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

    return crc;
}

// calculate CRC for volz serial protocol and send the data.
void AP_Volz_Protocol::send_command(CMD &cmd)
{
    const uint16_t crc = calculate_crc(cmd);

    // add CRC result to the message
    cmd.crc1 = HIGHBYTE(crc);
    cmd.crc2 = LOWBYTE(crc);
    port->write(cmd.data, sizeof(cmd));
}

#endif  // AP_VOLZ_ENABLED
