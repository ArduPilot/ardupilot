#ifndef PACKETS_H
#define PACKETS_H

#include "Types.h"

#define TRILLIUM_PKT_MAX_SIZE       140
#define TRILLIUM_PKT_HEADER_SIZE    4
#define TRILLIUM_PKT_OVERHEAD       (TRILLIUM_PKT_HEADER_SIZE + 2)

typedef struct
{
    UInt16 State;
    UInt16 MaxState;
    UInt16 Check0;
    UInt16 Check1;
} TrilliumPktInfo_t;

typedef struct
{
    UInt8 Sync0;
    UInt8 Sync1;
    UInt8 ID;
    UInt8 Length;
    UInt8 Data[TRILLIUM_PKT_MAX_SIZE + 2];

    // For packet parsing use only
    TrilliumPktInfo_t Info;
} TrilliumPkt_t;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

BOOL LookForTrilliumPacketInByteEx(TrilliumPkt_t *pPkt, TrilliumPktInfo_t *pInfo, UInt16 Sync, UInt8 Byte);
#define LookForTrilliumPacketInByte(pPkt, Sync, Byte) LookForTrilliumPacketInByteEx(pPkt, &(pPkt)->Info, Sync, Byte)

BOOL MakeTrilliumPacket(TrilliumPkt_t *pPkt, UInt16 Sync, UInt8 Type, UInt16 Length);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // PACKETS_H
