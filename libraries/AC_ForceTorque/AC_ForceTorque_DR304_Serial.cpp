#include "AC_ForceTorque_DR304_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <ctype.h>

#define DR304_HDR 0x03   // Header Byte from DR304_Serial
#define DR304_DATA_LENGTH 0x0E // length of Data for Byte of DR304_Serial

extern const AP_HAL::HAL& hal;

#define DR304_FRAME_HEADER 0x03
#define DR304_FRAME_LENGTH 18

#define FORCETORQUE_FORCE_MAX_N 50
#define FORCETORQUE_TORQUE_MAX_NM 2 

// format of serial packets received from inclination sensor HDA436T
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x03
// byte 1               Frame header    0x03
// byte 2               DATA_LENGTH     roll pitch yaw temperature these 4 data totle length of byte, default is 0x0E
// byte 3               Fx_d3           force in x axis raw data 3 high 8 bits
// byte 4               Fx_d2           force in x axis raw data 2 high 8 bits
// byte 5               Fx_d1           force in x axis raw data 1 low 8 bits
// bute 6               Fx_d0           force in x axis raw data 0 low 8 bits
// byte 7               Fy_d3           force in y axis raw data 3 high 8 bits
// bute 8               Fy_d2           force in y axis raw data 2 high 8 bits
// byte 9               Fy_d1           force in y axis raw data 1 low 8 bits
// byte 10              Fy_d0           force in y axis raw data 0 low 8 bits
// byte 11              Fz_d3           force in z axis raw data 3 high 8 bits
// byte 12              Fz_d2           force in z axis raw data 2 high 8 bits
// byte 13              Fz_d1           force in z axis raw data 1 low 8 bits
// byte 14              Fz_d0           force in z axis raw data 0 low 8 bits
// byte 15              Tx_d3           torque in x axis raw data 3 high 8 bits
// byte 16              Tx_d2           torque in x axis raw data 2 high 8 bits
// byte 17              Tx_d1           torque in x axis raw data 1 low 8 bits
// bute 18              Tx_d0           torque in x axis raw data 0 low 8 bits
// byte 19              Ty_d3           torque in y axis raw data 3 high 8 bits
// bute 20              Ty_d2           torque in y axis raw data 2 high 8 bits
// byte 21              Ty_d1           torque in y axis raw data 1 low 8 bits
// byte 22              Ty_d0           torque in y axis raw data 0 low 8 bits
// byte 23              Tz_d3           torque in z axis raw data 3 high 8 bits
// byte 24              Tz_d2           torque in z axis raw data 2 high 8 bits
// byte 25              Tz_d1           torque in z axis raw data 1 low 8 bits
// byte 26              Tz_d0           torque in z axis raw data 0 low 8 bits
// byte 27              Checksum       high 8 bits of Checksum byte, sum of bytes 0 to bytes 16
// byte 28              Checksum       low  8 bits of Checksum byte, sum of bytes 0 to bytes 16


// read - return last value measured by sensor
bool AC_ForceTorque_DR304_Serial::get_reading(Vector3f &reading_force_N, Vector3f &reading_torque_Nm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_roll_deg = 0;
    float sum_yaw_deg = 0;
    uint16_t count = 0;
    uint16_t count_out_of_positive_range = 0;
    uint16_t count_out_of_negtive_range = 0;

    // read any available lines from the inclination
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }

        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x03, add to buffer
        if (linebuf_len == 0) {
            if (c == DR304_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == DR304_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
             // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 19 items try to decode it
            if (linebuf_len == DR304_FRAME_LENGTH) {
                // calculate checksum
                uint16_t crc = (linebuf[18]<<8) | linebuf[17];
                if (crc == calc_crc_modbus(linebuf, 17)) {
                    // calculate roll angle
                    int32_t roll_raw = ((uint32_t)linebuf[6] << 24) | ((uint32_t)linebuf[5] << 16) | ((uint16_t)linebuf[4] << 8) | linebuf[3];
                    // int32_t pitch_raw = ((uint32_t)linebuf[10] << 24) | ((uint32_t)linebuf[9] << 16) | ((uint16_t)linebuf[8] << 8) | linebuf[7];
                    int32_t yaw_raw = ((uint32_t)linebuf[14] << 24) | ((uint32_t)linebuf[13] << 16) | ((uint16_t)linebuf[12] << 8) | linebuf[11];
                    float roll = (float)((roll_raw - ROLL_YAW_OFFSET)*0.001); 
                    // float pitch = (float)((pitch_raw - PITCH_OFFSET)*0.001);  
                    float yaw = (float)((yaw_raw - ROLL_YAW_OFFSET)*0.001); 
                    if (roll > INCLINATION_ROLL_MAX_DEGREE || yaw > INCLINATION_YAW_MAX_DEGREE) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if((roll < - INCLINATION_ROLL_MAX_DEGREE) || (yaw < - INCLINATION_YAW_MAX_DEGREE)){
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        //hal.console->printf("555inclination tilt sensor uart: %f\t, %lu\t,  %lu\r\n", roll, roll_raw, (roll_raw - ROLL_YAW_OFFSET));
                        sum_roll_deg += roll;
                        sum_yaw_deg += yaw;
                        count++;
                    }
                }                

                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_roll_deg = sum_roll_deg / count;   
        reading_yaw_deg = sum_yaw_deg / count;  
        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if out of range readings return maximum range for the positive angle
        reading_roll_deg = INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg = INCLINATION_YAW_MAX_DEGREE;
        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if out of range readings return maximum range for the negtive angle
        reading_roll_deg = - INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg = - INCLINATION_YAW_MAX_DEGREE;
        return true;
    }

    // no readings so return false
    return false;
}