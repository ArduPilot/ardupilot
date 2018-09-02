#pragma once

#define MAX_OVERLOAD_COUNT 10
#define MAX_FROZEN_COUNT 10

extern union UU msgbuf;


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"



union UU {
    mavlink_message_t m;

    byte bytes[0x40]; // for font uploading 
} msgbuf;


#include "protocols/MAVLink.h"

#pragma GCC diagnostic pop





