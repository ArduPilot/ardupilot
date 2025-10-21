#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS_Dummy.h"

#include <stdio.h>

#define FORCE_VERSION_H_INCLUDE
#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
#undef FORCE_VERSION_H_INCLUDE

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] {};

/*
  send_text implementation for dummy GCS
 */
void GCS_Dummy::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t dest_bitmask)
{
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
    DEV_PRINTF("TOGCS: ");
    hal.console->vprintf(fmt, arg_list);
    DEV_PRINTF("\n");
#else
    ::printf("TOGCS: ");
    ::vprintf(fmt, arg_list);
    ::printf("\n");
#endif
}

#endif  // HAL_GCS_ENABLED
