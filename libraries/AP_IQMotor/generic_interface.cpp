/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  IQ C++ API is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  IQ C++ API is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Name: generic_interface.cpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#include "generic_interface.hpp"
#include "crc_helper.h"
#include "string.h" // for memcpy

GenericInterface::GenericInterface()
{
  InitBQ(&index_queue, pf_index_data, GENERIC_PF_INDEX_DATA_SIZE);
  InitPacketFinder(&pf, &index_queue);
  tx_bipbuf = BipBuffer(tx_buffer, GENERIC_TX_BUFFER_SIZE);
}

int8_t GenericInterface::GetBytes()
{
  // I can't do anything on my own since I don't have hardware
  // Use SetRxBytes(uint8_t* data_in, uint16_t length_in)
  return 0;
}

int8_t GenericInterface::SetRxBytes(uint8_t* data_in, uint16_t length_in)
{
  if(data_in == nullptr)
    return -1;
  
  if(length_in)
  {
    //copy it over
    PutBytes(&pf, data_in, length_in); 
    return 1;
  }
  else
    return 0;
}

int8_t GenericInterface::PeekPacket(uint8_t **packet, uint8_t *length)
{
  return(::PeekPacket(&pf, packet, length));
}

int8_t GenericInterface::DropPacket()
{
  return(::DropPacket(&pf));
}

int8_t GenericInterface::SendPacket(uint8_t msg_type, uint8_t *data, uint16_t length)
{
  // This function must not be interrupted by another call to SendBytes or 
  // SendPacket, or else the packet it builds will be spliced/corrupted.

  uint8_t header[3];
  header[0] = kStartByte;                   // const defined by packet_finder.c
  header[1] = length;
  header[2] = msg_type;
  SendBytes(header, 3);
  
  SendBytes(data, length);
  
  uint8_t footer[2];
  uint16_t crc;
  crc = MakeCrc(&(header[1]), 2);
  crc = ArrayUpdateCrc(crc, data, length);
  footer[0] = crc & 0x00FF;
  footer[1] = crc >> 8;
  SendBytes(footer, 2);
  
  return(1);
}

int8_t GenericInterface::SendBytes(uint8_t *bytes, uint16_t length)
{
  uint16_t length_temp = 0;
  uint8_t* location_temp;
  int8_t ret = 0;
    
  // Reserve space in the buffer
  location_temp = tx_bipbuf.Reserve(length, length_temp);
  
  // If there's room, do the copy
  if(length == length_temp)
  {
    memcpy(location_temp, bytes, length_temp);   // do copy
    tx_bipbuf.Commit(length_temp);
    ret = 1;
  }
  else
  {
    tx_bipbuf.Commit(0); // Call the restaurant, cancel the reservations
  }
    
  return ret;
}

int8_t GenericInterface::GetTxBytes(uint8_t* data_out, uint8_t& length_out)
{
  uint16_t length_temp;
  uint8_t* location_temp;
  
  location_temp = tx_bipbuf.GetContiguousBlock(length_temp);
  if(length_temp)
  {
    memcpy(data_out, location_temp, length_temp);
    length_out = length_temp;
    tx_bipbuf.DecommitBlock(length_temp);
    
    location_temp = tx_bipbuf.GetContiguousBlock(length_temp);
    memcpy(&data_out[length_out], location_temp, length_temp);
    length_out = length_out + length_temp;
    tx_bipbuf.DecommitBlock(length_temp);
    return 1;
  }
  return 0;
}

void GenericInterface::SendNow()
{
  // I'm useless.
}

void GenericInterface::ReadMsg(CommunicationInterface& com, uint8_t* data, uint8_t length)
{
  // I currently don't support being talked to
  (void)com;
  (void)data;
  (void)length;
}