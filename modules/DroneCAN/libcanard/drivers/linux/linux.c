/*
  implement LinuxCAN, wrapper around socketcan and multicast UDP
 */

#include "linux.h"
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/*
 Initializes the instance.
 Returns 0 on success, negative on error.
*/
#if CANARD_ENABLE_CANFD
int16_t LinuxCANInit(LinuxCANInstance* out_ins, const char* can_iface_name, bool canfd)
#else
int16_t LinuxCANInit(LinuxCANInstance* out_ins, const char* can_iface_name)
#endif
{
    out_ins->socketcan = NULL;
    out_ins->mcast = NULL;

    if (strncmp(can_iface_name, "mcast", 5) == 0) {
        out_ins->mcast = (MCASTCANInstance *)calloc(1, sizeof(MCASTCANInstance));
        if (out_ins->mcast == NULL) {
            return -ENOMEM;
        }
#if CANARD_ENABLE_CANFD
        return mcastInit(out_ins->mcast, can_iface_name, can_fd);
#else
        return mcastInit(out_ins->mcast, can_iface_name);
#endif
    }

    out_ins->socketcan = (SocketCANInstance *)calloc(1, sizeof(SocketCANInstance));
    if (out_ins->socketcan == NULL) {
        return -ENOMEM;
    }
#if CANARD_ENABLE_CANFD
    return socketcanInit(out_ins->socketcan, can_iface_name, can_fd);
#else
    return socketcanInit(out_ins->socketcan, can_iface_name);
#endif
}

/*
 Deinitializes the mcast instance.
 Returns 0 on success, negative on error.
 */
int16_t LinuxCANClose(LinuxCANInstance* ins)
{
    if (ins->socketcan != NULL) {
        return socketcanClose(ins->socketcan);
    }
    if (ins->mcast != NULL) {
        return mcastClose(ins->mcast);
    }
    return -EINVAL;
}

/*
 Transmits a CanardCANFrame to the CAN socket.
 Use negative timeout to block infinitely.
 Returns 1 on successful transmission, 0 on timeout, negative on error.
 */
int16_t LinuxCANTransmit(LinuxCANInstance* ins, const CanardCANFrame* frame, int32_t timeout_msec)
{
    if (ins->socketcan != NULL) {
        return socketcanTransmit(ins->socketcan, frame, timeout_msec);
    }
    if (ins->mcast != NULL) {
        return mcastTransmit(ins->mcast, frame, timeout_msec);
    }
    return -EINVAL;
}

/*
 Receives a CanardCANFrame from the CAN socket.
 Use negative timeout to block infinitely.
 Returns 1 on successful reception, 0 on timeout, negative on error.
 */
int16_t LinuxCANReceive(LinuxCANInstance* ins, CanardCANFrame* out_frame, int32_t timeout_msec)
{
    if (ins->socketcan != NULL) {
        return socketcanReceive(ins->socketcan, out_frame, timeout_msec);
    }
    if (ins->mcast != NULL) {
        return mcastReceive(ins->mcast, out_frame, timeout_msec);
    }
    return -EINVAL;
}
