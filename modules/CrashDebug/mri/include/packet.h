/* Copyright 2014 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* 'Class' to manage the sending and receiving of packets to/from gdb.  Takes care of crc and ack/nak handling too. */
#ifndef _PACKET_H_
#define _PACKET_H_

#include <stdio.h>
#include "buffer.h"

typedef struct
{
    Buffer*        pBuffer;
    char           lastChar;
    unsigned char  calculatedChecksum;
    unsigned char  expectedChecksum;
} Packet;

/* Real name of functions are in __mri namespace. */
void    __mriPacket_GetFromGDB(Packet* pPacket, Buffer* pBuffer);
void    __mriPacket_SendToGDB(Packet* pPacket, Buffer* pBuffer);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define Packet_Init         __mriPacket_Init
#define Packet_GetFromGDB   __mriPacket_GetFromGDB
#define Packet_SendToGDB    __mriPacket_SendToGDB


#endif /* _PACKET_H_ */
