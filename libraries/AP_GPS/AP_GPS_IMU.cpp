// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
 *       GPS_MTK.cpp - Ublox GPS library for Arduino
 *       Code by Jordi Mu�oz and Jose Julio. DIYDrones.com
 *       This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)
 *
 *       This library is free software; you can redistribute it and/or
 *               modify it under the terms of the GNU Lesser General Public
 *               License as published by the Free Software Foundation; either
 *               version 2.1 of the License, or (at your option) any later version.
 *
 *       GPS configuration : Costum protocol
 *       Baud rate : 38400
 *
 *       Methods:
 *               init() : GPS initialization
 *               update() : Call this funcion as often as you want to ensure you read the incomming gps data
 *
 *       Properties:
 *               lattitude : lattitude * 10000000 (int32_t value)
 *               longitude : longitude * 10000000 (int32_t value)
 *               altitude :      altitude * 100 (meters) (int32_t value)
 *               ground_speed : Speed (m/s) * 100 (int32_t value)
 *               ground_course : Course (degrees) * 100 (int32_t value)
 *               new_data : 1 when a new data is received.
 *                                                       You need to write a 0 to new_data when you read the data
 *               fix : 1: GPS NO fix, 2: 2D fix, 3: 3D fix.
 *
 */

#include <AP_HAL.h>
#include "AP_GPS_IMU.h"

// Public Methods //////////////////////////////////////////////////////////////
void
AP_GPS_IMU::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
    // we expect the stream to already be open at the corret bitrate
	_port = s;
    idleTimeout = 1200;
}

// optimization : This code doesn't wait for data. It only proccess the data available.
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_IMU_gps() to parse and update the GPS info.
bool
AP_GPS_IMU::read(void)
{
    uint8_t data;
    int16_t numc = 0;

    numc = _port->available();

    if (numc > 0) {
        for (int16_t i=0; i<numc; i++) {    // Process bytes received

            data = _port->read();

            switch(step) {           //Normally we start from zero. This is a state machine
            case 0:
                if(data == 0x44)        // IMU sync char 1
                    step++;  //OH first data packet is correct, so jump to the next step
                break;

            case 1:
                if(data == 0x49)        // IMU sync char 2
                    step++;  //ooh! The second data packet is correct, jump to the step 2
                else
                    step=0;  //Nop, is not correct so restart to step zero and try again.
                break;

            case 2:
                if(data == 0x59)        // IMU sync char 3
                    step++;  //ooh! The second data packet is correct, jump to the step 2
                else
                    step=0;  //Nop, is not correct so restart to step zero and try again.
                break;

            case 3:
                if(data == 0x64)        // IMU sync char 4
                    step++;  //ooh! The second data packet is correct, jump to the step 2
                else
                    step=0;  //Nop, is not correct so restart to step zero and try again.
                break;

            case 4:
                payload_length = data;
                checksum(payload_length);
                step++;
                if (payload_length > 28) {
                    step = 0;        //Bad data, so restart to step zero and try again.
                    payload_counter = 0;
                    ck_a = 0;
                    ck_b = 0;
                    //payload_error_count++;
                }
                break;

            case 5:
                message_num = data;
                checksum(data);
                step++;
                break;

            case 6: // Payload data read...
                // We stay in this state until we reach the payload_length
                buffer[payload_counter] = data;
                checksum(data);
                payload_counter++;
                if (payload_counter >= payload_length) {
                    step++;
                }
                break;

            case 7:
                GPS_ck_a = data;         // First checksum byte
                step++;
                break;

            case 8:
                GPS_ck_b = data;         // Second checksum byte

                // We end the IMU/GPS read...
                // Verify the received checksum with the generated checksum..
                if((ck_a == GPS_ck_a) && (ck_b == GPS_ck_b)) {
                    if (message_num == 0x02) {
                        join_data();
                    } else if (message_num == 0x03) {
                        GPS_join_data();
                    } else if (message_num == 0x04) {
                        join_data_xplane();
                    } else if (message_num == 0x0a) {
                        //PERF_join_data();
                    } else {
//                                                      _error("Invalid message number = %d\n", (int)message_num);
                    }
                } else {
//                                              _error("XXX Checksum error\n"); //bad checksum
                    //imu_checksum_error_count++;
                }
                // Variable initialization
                step = 0;
                payload_counter = 0;
                ck_a = 0;
                ck_b = 0;
                break;
            }
        } // End for...
    }
    return true;
}

/****************************************************************
*
****************************************************************/

void AP_GPS_IMU::join_data(void)
{
    //Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other IMU classes..
    //In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.

    //Storing IMU roll
    memcpy(&roll_sensor, &buffer[0], sizeof(roll_sensor));

    //Storing IMU pitch
    memcpy(&pitch_sensor, &buffer[2], sizeof(pitch_sensor));

    //Storing IMU heading (yaw)
    memcpy(&ground_course, &buffer[4], sizeof(ground_course));
    imu_ok = true;
}

void AP_GPS_IMU::join_data_xplane()
{
    //Storing IMU roll
    memcpy(&roll_sensor, &buffer[0], sizeof(roll_sensor));


    //Storing IMU pitch
    memcpy(&pitch_sensor, &buffer[2], sizeof(pitch_sensor));

    //Storing IMU heading (yaw)
    memcpy(&ground_course, &buffer[4], sizeof(ground_course));

    //Storing airspeed
    memcpy(&airspeed, &buffer[6], sizeof(airspeed));

    imu_ok = true;

}

void AP_GPS_IMU::GPS_join_data(void)
{
    memcpy(&longitude, &buffer[0], sizeof(longitude));// degrees * 10e7
    latitude  = *(int32_t *)&buffer[4];

    //Storing GPS Height above the sea level
    int16_t tmp_altitude;
    memcpy(&tmp_altitude, &buffer[8], sizeof(tmp_altitude));
    altitude = (int32_t) tmp_altitude * 10;

    //Storing Speed
    int16_t tmp_speed;
    memcpy(&tmp_speed, &buffer[10], sizeof(tmp_speed));
    ground_speed = (float)tmp_speed;
    speed_3d = ground_speed;

    //We skip the gps ground course because we use yaw value from the IMU for ground course
    memcpy(&time, &buffer[14], sizeof(time));

    imu_health = buffer[15];

    new_data        = true;
    fix             = true;
}



/****************************************************************
*
****************************************************************/
// checksum algorithm
void AP_GPS_IMU::checksum(unsigned char data)
{
    ck_a += data;
    ck_b += ck_a;
}


/****************************************************************
* Unused
****************************************************************/
void AP_GPS_IMU::setHIL(uint32_t _time, float _latitude, float _longitude, float _altitude,
                        float _ground_speed, float _ground_course, float _speed_3d, uint8_t _num_sats) {
};
