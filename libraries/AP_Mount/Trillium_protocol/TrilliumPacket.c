#include "TrilliumPacket.h"

// Running checksum calculation functions
static void InitChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB);
static void UpdateChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB);

BOOL LookForTrilliumPacketInByteEx(TrilliumPkt_t *pPkt, TrilliumPktInfo_t *pInfo, UInt16 Sync, UInt8 Byte)
{
    // If there's no packet tracking info, we can't do this
    if (pInfo == NULL)
        return 0;
    // If the user wants to keep the packet data, put the byte into the packet structure
    else if ((pPkt != NULL) && (pInfo->State < TRILLIUM_PKT_MAX_SIZE + TRILLIUM_PKT_OVERHEAD))
        ((UInt8 *)pPkt)[pInfo->State] = Byte;

    // If the max state looks screwed up, reset everything
    if (pInfo->MaxState > TRILLIUM_PKT_MAX_SIZE + TRILLIUM_PKT_OVERHEAD)
        pInfo->State = pInfo->MaxState = 0;

    // Update the running checksum as necessary
    if (pInfo->State == 0)
        InitChecksum(Byte, &pInfo->Check0, &pInfo->Check1);
    else if ((pInfo->State < TRILLIUM_PKT_HEADER_SIZE) || (pInfo->State < pInfo->MaxState - 2))
        UpdateChecksum(Byte, &pInfo->Check0, &pInfo->Check1);

    // Decide what to do based on our handy dandy state tracking variable
    switch (++pInfo->State)
    {
    // Sync byte 0
    case 1:
        // If this byte doesn't match the first sync byte, restart the state machine
        if (Byte != (UInt8)(Sync >> 8))
            pInfo->State = 0;
        break;

    // Sync byte 1
    case 2:
        // If this byte doesn't match the second sync byte, restart the state machine
        if (Byte != (UInt8)(Sync & 0xFF))
            pInfo->State = 0;
        break;

    // Data length specifier - does not include header or checksum
    case 4:
        // Make sure the packet's not too big
        if (Byte <= TRILLIUM_PKT_MAX_SIZE)
            pInfo->MaxState = Byte + TRILLIUM_PKT_OVERHEAD;
        else
            pInfo->State = 0;
        break;

    // All other states
    default:
        // If we're into the data payload
        if (pInfo->State > TRILLIUM_PKT_HEADER_SIZE)
        {
            // If this is the first checksum byte and it doesn match up, restart the state machine
            if ((pInfo->State == pInfo->MaxState - 1) && ((pInfo->Check0 & 0xFF) != Byte))
                pInfo->State = 0;
            // If we got at least one packet's worth of bytes and it looks like we've got all this packet's data
            else if (pInfo->State >= pInfo->MaxState)
            {
                // Reset the packet tracking state
                pInfo->State = 0;

                // And finally, return true if the checksum... checks out
                return (pInfo->Check1 & 0xFF) == Byte;
            }
        }
        break;
    };

    // If we haven't already returned, we ain't done yet!
    return FALSE;

}// LookForOrionPacketInByte

BOOL MakeTrilliumPacket(TrilliumPkt_t *pPkt, UInt16 Sync, UInt8 ID, UInt16 Length)
{
    // Get a byte pointer to the start of the packet structure
    UInt8 *pData = (UInt8 *)pPkt, i;

    // If this is an invalid data length, return FALSE immediately
    if (Length > TRILLIUM_PKT_MAX_SIZE)
        return FALSE;

    // Dump in all the rote data and zero out the checksum tracker
    pPkt->Sync0 = (UInt8)(Sync >> 8);
    pPkt->Sync1 = (UInt8)(Sync & 0xFF);
    pPkt->ID = ID;
    pPkt->Length = (UInt8)Length;
    pPkt->Info.Check0 = 1;
    pPkt->Info.Check1 = 0;

    // Roll each byte into the running checksum
    for (i = 0; i < Length + TRILLIUM_PKT_HEADER_SIZE; i++)
        UpdateChecksum(pData[i], &pPkt->Info.Check0, &pPkt->Info.Check1);

    // Negate the checksum and paste its bytes onto the end of the data payload
    pPkt->Data[Length++] = (UInt8)(pPkt->Info.Check0 & 0xFF);
    pPkt->Data[Length++] = (UInt8)(pPkt->Info.Check1 & 0xFF);

    // If we made it here, this is a "good" packet
    return TRUE;

}// MakeTrilliumPacket aliased as MakeOrionPacket

static void InitChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB)
{
    // For the first iteration, both checksum bytes should be equal
    *pA = *pB = (((UInt16)Byte & 0xFF) + 1) % 251;

}// InitChecksum

static void UpdateChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB)
{
    // Surprise! It's just Fletcher's checksum mod 251 (prime number) instead of 255
    *pA = (*pA + ((UInt16)Byte & 0xFF)) % 251;
    *pB = (*pB + *pA) % 251;

}// UpdateChecksum
