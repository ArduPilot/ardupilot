#pragma once
#include "AP_Frsky_Backend.h"

/*

  Wire Protocol:

  Several SPort packets make up a MAVlite message.

  A maximum of six relevant data bytes are present in an SPort packet.

  Each SPort packet starts with a sequence number, starting with zero.

  If the sequence number is zero then the parser is reset.

  The first sport packet contains len at offset 1.  It is the
  *payload* length - does not include checksum, for example.  msgid is
  at offset 2, then payload bytes.

  Subsequent SPort packets contain a sequence number at offset zero,
  followed by more payload bytes.

  When sufficient payload bytes have been received (based on "len"), a
  single checksum byte arrives.  Sometimes this checksum byte goes
  into an SPort packet all on its own, sharing space only with the
  sequence number.

*/

#define MAVLITE_MAX_PAYLOAD_LEN                 31 // 7 float params + cmd_id + options
#define MAVLITE_MSG_SPORT_PACKETS_COUNT(LEN)    static_cast<uint8_t>(1 + ceilf((LEN-2)/5.0f)) // number of sport packets required to transport a message with LEN payload
#define SPORT_PACKET_QUEUE_LENGTH               static_cast<uint8_t>(30U*MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN))
