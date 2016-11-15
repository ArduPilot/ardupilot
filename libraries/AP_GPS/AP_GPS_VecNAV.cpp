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

#include "AP_GPS.h"
#include "AP_GPS_VecNAV.h"
#include <AP_HAL/Util.h>

#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdlib.h>

#define VECNAV_INITMSG \
        "$VNWRG,06,0*XX\r\n"                      /* Disable ascii output from GPS */ \
        "$VNWRG,75,2,80,29,FFFF,0FFF,07FF*XX\r\n" /* Enable Binary output - Common,GPS and INS group */


extern const AP_HAL::HAL& hal;

const char AP_GPS_VecNAV::_initialisation_blob[] = VECNAV_INITMSG;

AP_GPS_VecNAV::AP_GPS_VecNAV(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{

    gps.send_blob_start(state.instance, _initialisation_blob, sizeof(_initialisation_blob));
}

void AP_GPS_VecNAV::ExtractDataGroup1(uint8_t *payload, struct BinGroup1 *msg){

  // extract group 1
  uint8_t *p = payload;
  memcpy(&(msg->TimeStartup),p,8);p = p+8;
  memcpy(&(msg->TimeGps),p,8);p = p+8;
  memcpy(&(msg->TimeSyncIn),p,8);p = p+8;
  memcpy(&(msg->YawPitchRoll),p,12);p = p+12;
  memcpy(&(msg->Quaternion),p,16);p = p+16;
  memcpy(&(msg->AngularRate),p,12);p = p+12;
  memcpy(&(msg->LLA),p,24);p = p+24;
  memcpy(&(msg->Velocity),p,12);p = p+12;
  memcpy(&(msg->Accel),p,12);p = p+12;
  memcpy(&(msg->Imu),p,24);p = p+24;
  memcpy(&(msg->MagPres),p,20);p = p+20;
  memcpy(&(msg->DeltaThetaVel),p,28);p = p+28;
  memcpy(&(msg->InsStatus),p,2);p = p+2;
  memcpy(&(msg->SyncInCnt),p,4);p = p+4;
  memcpy(&(msg->TimeGpsPps),p,8);
}

void AP_GPS_VecNAV::ExtractDataGroup4(uint8_t *payload, struct BinGroup4 *msg){

  uint8_t *p = payload;
  memcpy(&(msg->year),p,1); p = p+1;
  memcpy(&(msg->month),p,1); p = p+1;
  memcpy(&(msg->day),p,1); p = p+1;
  memcpy(&(msg->hour),p,1); p = p+1;
  memcpy(&(msg->min),p,1); p = p+1;
  memcpy(&(msg->sec),p,1); p = p+1;
  memcpy(&(msg->ms),p,2); p = p+2;
  memcpy(&(msg->Tow),p,8); p = p+8;
  memcpy(&(msg->Week),p,2); p = p+2;
  memcpy(&(msg->NumSats),p,1); p = p+1;
  memcpy(&(msg->Fix),p,1); p = p+1;
  memcpy(&(msg->PosLla),p,24); p = p+24;
  memcpy(&(msg->PosEcef),p,24); p = p+24;
  memcpy(&(msg->VelNed),p,12); p = p+12;
  memcpy(&(msg->VelEcef),p,12); p = p+12;
  memcpy(&(msg->PosU),p,12); p = p+12;
  memcpy(&(msg->VelU),p,4); p = p+4;
  memcpy(&(msg->TimeU),p,4); 
}

void AP_GPS_VecNAV::ExtractDataGroup6(uint8_t *payload, struct BinGroup6 *msg){

  uint8_t *p = payload;
  memcpy(&(msg->InsStatus),p,2); p = p+2;
  memcpy(&(msg->PosLla),p,24); p = p+24;
  memcpy(&(msg->PosEcef),p,24); p = p+24;
  memcpy(&(msg->VelBody),p,12); p = p+12;
  memcpy(&(msg->VelNed),p,12); p = p+12;
  memcpy(&(msg->VelEcef),p,12); p = p+12;
  memcpy(&(msg->MagEcef),p,12); p = p+12;
  memcpy(&(msg->AccelEcef),p,12); p = p+12;
  memcpy(&(msg->LinearAccelEcef),p,12); p = p+12;
  memcpy(&(msg->PosU),p,4); p = p+4;
  memcpy(&(msg->VelU),p,4);   
}

uint16_t AP_GPS_VecNAV::calculateCRC(unsigned char data[], unsigned int length)
{
 unsigned int i;
 uint16_t crc = 0;
 for(i=0; i<length; i++){
  crc = (unsigned char)(crc >> 8) | (crc << 8);
  crc ^= data[i];
  crc ^= (unsigned char)(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0x00ff) << 5;
 }

 return crc;
}

bool AP_GPS_VecNAV::ProcessGPSMessage(uint8_t c, struct BinGroup1* msgG1, struct BinGroup4* msgG4, struct BinGroup6* msgG6){

  
  uint8_t *p = dataPl;
  
  

  switch(msgState){
  case 0:       
    // Checking for header
    count     = 0;   
    memset(dataPl,0,600);
    if(c == 0xFA){
      packetsize = 0;
      packetsize++;
      msgState = 1;
      //hal.console->printf("Received header\n");
      //printf("Received header\n");
    }
    break;

  case 1:
    // Groups active    
    dataPl[packetsize-1] = c;
    packetsize++;
    msgState = 2;
    count = 0;
    //printf("Group %02x is active\n",c);
    //hal.console->printf("Group %02x is active\n",c);
    
    break;
    
  case 2:
    // Active fields in each group
    dataPl[packetsize-1] = c;
    packetsize++;
    if(count == 0){
      fieldSize1 = 0;
      fieldSize1 = fieldSize1 | c;
      count++;
    }
    else if(count == 1){
      fieldSize1 = (fieldSize1 << 8) | c;
      count++;
    }
    else if(count == 2){
      fieldSize2 = 0;
      fieldSize2 = fieldSize2 | c;
      count++;
    }
    else if(count == 3){
      fieldSize2 = (fieldSize2 << 8) | c;      
      count++;
    }
    else if(count == 4){
      fieldSize3 = 0;
      fieldSize3 = fieldSize3 | c;
      count++;
    }
    else if(count == 5){
      fieldSize3 = (fieldSize3 << 8) | c;      
      count = 0;
      msgState = 3;
    }

    break;

  case 3:
    //Payload
    //printf("datasize = %d\n",packetsize);
    dataPl[packetsize-1] = c;
    packetsize++;
    count++;    
    if(count == (DATA_LEN - HEADER - CRC_LEN)){      
      //printf("Receive payload, count=%d\n",count);
       //hal.console->printf("Receive payload, count=%d\n",count);
      
      msgState = 4;
      count = 0;      
    }

    break;
    
  case 4:
    //CRC
    //printf("datasize = %d\n",packetsize);
    dataPl[packetsize-1] = c;
    packetsize++;
    if(count == 0){
      CRC = 0;
      CRC = CRC | c;
      count++;
    }
    else if(count == 1){
      CRC = (CRC << 8) | c;
      msgState = 5;
      count = 0;
      
      //printf("Received checksum\n");
    }

    break;

  case 5:
    // Validate data    
    CRCV = calculateCRC(dataPl,DATA_LEN);
    msgState = 0;
    count = 0;
    if(CRCV == 0x0000){
      //printf("Valid data obtained\n");
      //hal.console->print("Valid data obtained");

      p = dataPl + HEADER;
      ExtractDataGroup1(p,msgG1);
      
      p = dataPl + HEADER + GROUP1_LEN;
      ExtractDataGroup4(p,msgG4);

      p = dataPl + HEADER + GROUP1_LEN + GROUP4_LEN;
      ExtractDataGroup6(p,msgG6);
      
      return true;
    }
    else{      
      return false;
    }
    
    break;
  }//end of switch

  return false;
}

bool AP_GPS_VecNAV::read(void){

    int16_t numc;
    bool parsed = false;
      
    numc = port->available();

    //hal.console->printf("Available data %d\n",numc);

    while (numc--) {
        
        char c = port->read();
        //hal.console->printf("%c",c);
        parsed = ProcessGPSMessage(c,&msg1,&msg4,&msg6);

        if(parsed){
            
            state.location.lng    = (int32_t) (msg6.PosLla[1]*1e7);
            state.location.lat    = (int32_t) (msg6.PosLla[0]*1e7);
            state.location.alt    = (int32_t) (msg6.PosLla[2] * 100); // altitude in centimeters

            if(msg4.Fix == 0 || msg4.Fix == 1){
                state.status = AP_GPS::NO_FIX;
            }
            else if(msg4.Fix == 2){
                state.status = AP_GPS:: GPS_OK_FIX_2D;
            }
            else if(msg4.Fix == 3){
                state.status = AP_GPS:: GPS_OK_FIX_3D;
            }

            //state.horizontal_accuracy = msg4.PosU[0]*100;
            //state.vertical_accuracy = msg4.PosU[2]*100;
            state.have_horizontal_accuracy = false;
            state.have_vertical_accuracy = false;
            
            state.have_vertical_velocity = true;
            state.velocity.x = msg6.VelNed[0];
            state.velocity.y = msg6.VelNed[1];
            state.velocity.z = msg6.VelNed[2];
            state.ground_course = wrap_360(degrees(atan2f(state.velocity.y, state.velocity.x)));
            state.ground_speed = norm(state.velocity.y, state.velocity.x);
            state.have_speed_accuracy = true;
            state.speed_accuracy = msg6.VelU;
            state.num_sats = msg4.NumSats;

            

            return true;
        }
    }

    return false;
}

