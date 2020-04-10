#ifndef ORIONPACKETSHIM_H
#define ORIONPACKETSHIM_H

#include "TrilliumPacket.h"
#include "Constants.h"
#include "OrionPublicProtocol.h"
#include "OrionPublicPacket.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Orion-specific constants
#define ORION_SYNC         0xD00D
#define ORION_PKT_MAX_SIZE TRILLIUM_PKT_MAX_SIZE
#define ORION_PKT_OVERHEAD TRILLIUM_PKT_OVERHEAD

// Orion packets share a structure with a generic Trillium packet
typedef TrilliumPkt_t OrionPkt_t;
typedef TrilliumPktInfo_t OrionPktInfo_t;

// And they share the same basic parsing functions
#define LookForOrionPacketInByte(a, b)      LookForTrilliumPacketInByte((TrilliumPkt_t *)a, ORION_SYNC, b)
#define LookForOrionPacketInByteEx(a, b, c) LookForTrilliumPacketInByteEx((TrilliumPkt_t *)a, (TrilliumPktInfo_t *)b, ORION_SYNC, c)
#define MakeOrionPacket(a, b, c)            MakeTrilliumPacket(a, ORION_SYNC, b, c)

// Defines for backward compatibility. NOTE: THESE *WILL* BE DEPRECATED IN THE FUTURE
#define encodeOrionCmdPacketStructure encodeOrionCmdPacket
#define decodeOrionCmdPacketStructure decodeOrionCmdPacket

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ORIONPACKETSHIM_H
