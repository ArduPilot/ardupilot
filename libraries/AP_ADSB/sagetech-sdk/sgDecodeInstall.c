/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file sgDecodeInstall.c
 * @author Jacob.Garrison
 *
 * @date Mar 9, 2021
 *      
 */

#include <string.h>
#include <stdbool.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_REG_LEN    7     // The number of bytes in the registration field

#define SG_STL_ANTENNA      0x03
#define SG_STL_ALT_RES      0x08
#define SG_STL_HDG_TYPE     0x10
#define SG_STL_SPD_TYPE     0x20
#define SG_STL_HEATER       0x40
#define SG_STL_WOW          0x80

typedef struct __attribute__((packed))
{
   uint8_t  start;
   uint8_t  type;
   uint8_t  id;
   uint8_t  payloadLen;
   uint8_t  icao[3];
   char     registration[SG_REG_LEN];
   uint8_t  rsvd1[2];
   uint8_t  com0;
   uint8_t  com1;
   uint8_t  ipAddress[4];
   uint8_t  subnetMask[4];
   uint8_t  port[2];
   uint8_t  gpsIntegrity;
   uint8_t  emitterSet;
   uint8_t  emitterType;
   uint8_t  size;
   uint8_t  maxSpeed;
   uint8_t  altOffset[2];
   uint8_t  rsvd2[2];
   uint8_t  config;
   uint8_t  rsvd3[2];
   uint8_t  checksum;
} stl_t;

/*
 * Documented in the header file.
 */
bool sgDecodeInstall(uint8_t *buffer, sg_install_t *stl)
{
   memset(&stl->reg[0], '\0', sizeof(stl->reg));  // Ensure registration is null-terminated

   stl_t sgStl;
   memcpy(&sgStl, buffer, sizeof(stl_t));

   stl->icao           = toIcao(sgStl.icao);
   strcpy(stl->reg,      sgStl.registration);
   memset(&stl->reg[SG_REG_LEN], 0, 1);  // Ensure registration is null-terminated
   stl->com0           = (sg_baud_t)(sgStl.com0);
   stl->com1           = (sg_baud_t)(sgStl.com1);
   stl->eth.ipAddress  = toUint32(sgStl.ipAddress);
   stl->eth.subnetMask = toUint32(sgStl.subnetMask);
   stl->eth.portNumber = toUint16(sgStl.port);
   stl->sil            = (sg_sil_t)(sgStl.gpsIntegrity >> 4);
   stl->sda            = (sg_sda_t)(sgStl.gpsIntegrity & 0x0F);
   stl->emitter        = (sg_emitter_t)(0x10 * sgStl.emitterSet + sgStl.emitterType);
   stl->size           = (sg_size_t)sgStl.size;
   stl->maxSpeed       = (sg_airspeed_t)sgStl.maxSpeed;
   stl->altOffset      = toUint16(sgStl.altOffset);
   stl->antenna        = (sg_antenna_t)sgStl.config & SG_STL_ANTENNA;
   stl->altRes100      = sgStl.config & SG_STL_ALT_RES;
   stl->hdgTrueNorth   = sgStl.config & SG_STL_HDG_TYPE;
   stl->airspeedTrue   = sgStl.config & SG_STL_SPD_TYPE;
   stl->heater         = sgStl.config & SG_STL_HEATER;
   stl->wowConnected   = sgStl.config & SG_STL_WOW;

   return true;
}
