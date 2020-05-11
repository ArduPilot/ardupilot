/*
 * \file
 * This module provides the connection from the automatically generated public
 * protocol code to the Orion packet routines.
 */

#include "OrionPublicProtocol.h"
#include "OrionPublicPacketShim.h"


//! \return the packet data pointer from the packet
uint8_t* getOrionPublicPacketData(void* pkt)
{
	return ((OrionPkt_t*)pkt)->Data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getOrionPublicPacketDataConst(const void* pkt)
{
	return ((const OrionPkt_t*)pkt)->Data;
}

//! Complete a packet after the data have been encoded
void finishOrionPublicPacket(void* pkt, int size, uint32_t packetID)
{
	MakeOrionPacket((OrionPkt_t*)pkt, (uint8_t)packetID, (uint16_t)size);
}

//! \return the size of a packet from the packet header
int getOrionPublicPacketSize(const void* pkt)
{
	return ((OrionPkt_t*)pkt)->Length;
}

//! \return the ID of a packet from the packet header
uint32_t getOrionPublicPacketID(const void* pkt)
{
	return ((OrionPkt_t*)pkt)->ID;
}
