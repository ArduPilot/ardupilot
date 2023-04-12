/*
  support for sending UDP packets on MAVLink packet boundaries.
 */

#include <GCS_MAVLink/GCS.h>

#if HAL_GCS_ENABLED

#include "packetise.h"

/*
  return the number of bytes to send for a packetised connection
 */
uint16_t mavlink_packetise(ByteBuffer &writebuf, uint16_t n)
{
    int16_t b = writebuf.peek(0);
    if (b != MAVLINK_STX_MAVLINK1 && b != MAVLINK_STX) {
        /*
          we have a non-mavlink packet at the start of the
          buffer. Look ahead for a MAVLink start byte, up to 256 bytes
          ahead
         */
        uint16_t limit = n>256?256:n;
        uint16_t i;
        for (i=0; i<limit; i++) {
            b = writebuf.peek(i);
            if (b == MAVLINK_STX_MAVLINK1 || b == MAVLINK_STX) {
                n = i;
                break;
            }
        }
        // if we didn't find a MAVLink marker then limit the send size to 256
        if (i == limit) {
            n = limit;
        }
        return n;
    }

    // cope with both MAVLink1 and MAVLink2 packets
    uint8_t min_length = (b == MAVLINK_STX_MAVLINK1)?8:12;

    // this looks like a MAVLink packet - try to write on
    // packet boundaries when possible
    if (n < min_length) {
        // we need to wait for more data to arrive
        return 0;
    }

    // the length of the packet is the 2nd byte
    int16_t len = writebuf.peek(1);
    if (b == MAVLINK_STX) {
        // This is Mavlink2. Check for signed packet with extra 13 bytes
        int16_t incompat_flags = writebuf.peek(2);
        if (incompat_flags & MAVLINK_IFLAG_SIGNED) {
            min_length += MAVLINK_SIGNATURE_BLOCK_LEN;
        }
    }

    if (n < len+min_length) {
        // we don't have a full packet yet
        return 0;
    }
    if (n > len+min_length) {
        // send just 1 packet at a time (so MAVLink packets
        // are aligned on UDP boundaries)
        n = len+min_length;
    }
    return n;
}

#endif // HAL_GCS_ENABLED
