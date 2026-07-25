/*
 * Copyright (c) 2016-2018 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    int fd;
#ifdef CANARD_ENABLE_CANFD
    bool canfd;
#endif
} SocketCANInstance;

/**
 * Initializes the SocketCAN instance.
 * Returns 0 on success, negative on error.
 */
#if CANARD_ENABLE_CANFD
int16_t socketcanInit(SocketCANInstance* out_ins, const char* can_iface_name, bool canfd);
#else
int16_t socketcanInit(SocketCANInstance* out_ins, const char* can_iface_name);
#endif

/**
 * Deinitializes the SocketCAN instance.
 * Returns 0 on success, negative on error.
 */
int16_t socketcanClose(SocketCANInstance* ins);

/**
 * Transmits a CanardCANFrame to the CAN socket.
 * Use negative timeout to block infinitely.
 * Returns 1 on successful transmission, 0 on timeout, negative on error.
 */
int16_t socketcanTransmit(SocketCANInstance* ins, const CanardCANFrame* frame, int32_t timeout_msec);

/**
 * Receives a CanardCANFrame from the CAN socket.
 * Use negative timeout to block infinitely.
 * Returns 1 on successful reception, 0 on timeout, negative on error.
 */
int16_t socketcanReceive(SocketCANInstance* ins, CanardCANFrame* out_frame, int32_t timeout_msec);

/**
 * Returns the file descriptor of the CAN socket.
 * Can be used for external IO multiplexing.
 */
int socketcanGetSocketFileDescriptor(const SocketCANInstance* ins);

#ifdef __cplusplus
}
#endif

#endif
