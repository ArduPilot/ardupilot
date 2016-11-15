// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//
// Driver to read VectorNAV protocol
// Swee Warman sweewarman@gmail.com
// ***
#pragma once

#include <AP_HAL/Util.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

/// Vector NAV parse
///

#define GROUP1_LEN 198
#define GROUP4_LEN 112
#define GROUP6_LEN 130
#define CRC_LEN 2
#define GROUPS 3
#define HEADER (1+(2*GROUPS))

#define DATA_LEN HEADER+CRC_LEN+GROUP4_LEN+GROUP6_LEN+GROUP1_LEN

#define VECNAV_SET_BINARY "$VNWRG,06,0*XX\r\n$VNWRG,75,1,80,29,FFFF,0FFF,07FF*XX\r\n"

struct BinGroup1{
  uint64_t TimeStartup; //8
  uint64_t TimeGps;     //8
  uint64_t TimeSyncIn;  //8
  float YawPitchRoll[3];//12
  float Quaternion[4];  //16
  float AngularRate[3]; //12
  double LLA[3];        //24
  float Velocity[3];    //12
  float Accel[3];       //12
  float Imu[6];         //24
  float MagPres[5];     //20
  float DeltaThetaVel[7]; //28
  uint16_t InsStatus;  //2
  uint32_t SyncInCnt;  //4
  uint64_t TimeGpsPps; //8
};

struct BinGroup4{
  //UTC
  int8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint16_t ms[2];
  
  uint64_t Tow;
  uint16_t Week;
  uint8_t NumSats;
  uint8_t Fix;
  double PosLla[3];
  double PosEcef[3];
  float VelNed[3];
  float VelEcef[3];
  float PosU[3];
  float VelU;
  uint32_t TimeU;
};

struct BinGroup6{
  uint16_t InsStatus;
  double PosLla[3];
  double PosEcef[3];
  float VelBody[3];
  float VelNed[3];
  float VelEcef[3];
  float MagEcef[3];
  float AccelEcef[3];
  float LinearAccelEcef[3];
  float PosU;
  float VelU;
};

class AP_GPS_VecNAV : public AP_GPS_Backend{

    public: 
        AP_GPS_VecNAV(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);
        bool read(void);

    private:
        BinGroup1 msg1;
        BinGroup4 msg4;
        BinGroup6 msg6;

        int msgState = 0;
        int count = 0;
        uint16_t fieldSize1 = 0;
        uint16_t fieldSize2 = 0;
        uint16_t fieldSize3 = 0;
        uint16_t CRC = 0;  
        int packetsize = 0;
  
        uint8_t dataPl[600];
        uint16_t CRCV;

        uint16_t calculateCRC(unsigned char data[], unsigned int length);

        void ExtractDataGroup1(uint8_t *payload, struct BinGroup1 *msg);

        void ExtractDataGroup4(uint8_t *payload, struct BinGroup4 *msg);

        void ExtractDataGroup6(uint8_t *payload, struct BinGroup6 *msg);

        bool ProcessGPSMessage(uint8_t c, struct BinGroup1* msg1, struct BinGroup4* msg4,struct BinGroup6* msg6 );

        static const char _initialisation_blob[];  


};