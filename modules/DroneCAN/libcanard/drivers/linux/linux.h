/*
 * Copyright (c) 2023 DroneCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 */
/*
  this wraps the socketcan and multicast drivers, allowing either to be selected
 */

#pragma once

#include <canard.h>
#include "../mcast/mcast.h"
#include "../socketcan/socketcan.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    MCASTCANInstance *mcast;
    SocketCANInstance *socketcan;
} LinuxCANInstance;

/*
 Initializes the instance.
 Returns 0 on success, negative on error.
*/
#if CANARD_ENABLE_CANFD
int16_t LinuxCANInit(LinuxCANInstance* out_ins, const char* can_iface_name, bool canfd);
#else
int16_t LinuxCANInit(LinuxCANInstance* out_ins, const char* can_iface_name);
#endif

/*
 Deinitializes the mcast instance.
 Returns 0 on success, negative on error.
 */
int16_t LinuxCANClose(LinuxCANInstance* ins);

/*
 Transmits a CanardCANFrame to the CAN socket.
 Use negative timeout to block infinitely.
 Returns 1 on successful transmission, 0 on timeout, negative on error.
 */
int16_t LinuxCANTransmit(LinuxCANInstance* ins, const CanardCANFrame* frame, int32_t timeout_msec);

/*
 Receives a CanardCANFrame from the CAN socket.
 Use negative timeout to block infinitely.
 Returns 1 on successful reception, 0 on timeout, negative on error.
 */
int16_t LinuxCANReceive(LinuxCANInstance* ins, CanardCANFrame* out_frame, int32_t timeout_msec);

#ifdef __cplusplus
}
#endif
