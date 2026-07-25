/*
 * Copyright (c) 2023 DroneCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#pragma once

#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    int fd_in;
    int fd_out;
#ifdef CANARD_ENABLE_CANFD
    bool canfd;
#endif
} MCASTCANInstance;

/*
 Initializes the instance.
 Returns 0 on success, negative on error.
*/
#if CANARD_ENABLE_CANFD
int16_t mcastInit(MCASTCANInstance* out_ins, const char* can_iface_name, bool canfd);
#else
int16_t mcastInit(MCASTCANInstance* out_ins, const char* can_iface_name);
#endif

/*
 Deinitializes the mcast instance.
 Returns 0 on success, negative on error.
 */
int16_t mcastClose(MCASTCANInstance* ins);

/*
 Transmits a CanardCANFrame to the CAN socket.
 Use negative timeout to block infinitely.
 Returns 1 on successful transmission, 0 on timeout, negative on error.
 */
int16_t mcastTransmit(MCASTCANInstance* ins, const CanardCANFrame* frame, int32_t timeout_msec);

/*
 Receives a CanardCANFrame from the CAN socket.
 Use negative timeout to block infinitely.
 Returns 1 on successful reception, 0 on timeout, negative on error.
 */
int16_t mcastReceive(MCASTCANInstance* ins, CanardCANFrame* out_frame, int32_t timeout_msec);

#ifdef __cplusplus
}
#endif
