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

#define SET_EXTENDED_POSITION_CMD      0xDC

// Extended Position Data Format defines -100 as 0x0080 decimal 128, we map this to a PWM of 1000 (if range is default)
#define PWM_POSITION_MIN               1000
#define ANGLE_POSITION_MIN            -100.0
#define EXTENDED_POSITION_MIN          0x0080

// Extended Position Data Format defines +100 as 0x0F80 decimal 3968, we map this to a PWM of 2000 (if range is default)
#define PWM_POSITION_MAX               2000
#define ANGLE_POSITION_MAX             100.0
#define EXTENDED_POSITION_MAX          0x0F80

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
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
    update_volz_bitmask(bitmask);
}

void AP_Volz_Protocol::update()
{
    if (!initialised) {
        initialised = true;
        init();
    }
    
    if (port == nullptr) {
        return;
    }

    if (last_used_bitmask != uint32_t(bitmask.get())) {
        update_volz_bitmask(bitmask);
    }

    uint32_t now = AP_HAL::micros();
    if (((now - last_volz_update_time) < volz_time_frame_micros) ||
        (port->txspace() < sizeof(CMD))) {
        return;
    }

    last_volz_update_time = now;


    // loop for all channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        // check if current channel is needed for Volz protocol
        if (last_used_bitmask & (1U<<i)) {

            const SRV_Channel *c = SRV_Channels::srv_channel(i);
            if (c == nullptr) {
                continue;
            }
            uint16_t pwm = c->get_output_pwm();
            if (pwm == 0) {
                // 0 PMW should stop outputting, for example in "safe"
                // There is no way to de-power, move to trim
                pwm = c->get_trim();
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
            cmd.actuator_id = i + 1; // send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
            cmd.arg1 = HIGHBYTE(value);
            cmd.arg2 = LOWBYTE(value);

            send_command(cmd);
        }
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

void AP_Volz_Protocol::update_volz_bitmask(uint32_t new_bitmask)
{
    const uint8_t count = __builtin_popcount(new_bitmask);
    last_used_bitmask = new_bitmask;

    // have a safety margin of 20% to allow for not having full uart
    // utilisation. We really don't want to start filling the uart
    // buffer or we'll end up with servo lag
    const float safety = 1.3;

    // each channel take about 425.347us to transmit so total time will be ~ number of channels * 450us
    // rounded to 450 to make sure we don't go over the baud rate.
    uint32_t channels_micros = count * 450 * safety;

    // limit the minimum to 2500 will result a max refresh frequency of 400hz.
    if (channels_micros < 2500) {
        channels_micros = 2500;
    }

    volz_time_frame_micros = channels_micros;
}

#endif  // AP_VOLZ_ENABLED
