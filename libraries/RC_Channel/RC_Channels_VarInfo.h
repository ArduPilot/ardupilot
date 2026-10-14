#pragma once

#include "RC_Channel_config.h"

#if AP_RC_CHANNEL_ENABLED

#include "RC_Channel.h"


/*
  this header file is expected to be #included by Vehicle subclasses
  of RC_Channels after defining RC_CHANNELS_SUBCLASS and
  RC_CHANNEL_SUBCLASS - for example, Rover defines
  RC_CHANNELS_SUBCLASS to be RC_Channels_Rover in Rover/RC_Channels.cpp, and then includes this header.

  This scheme reduces code duplicate between the Vehicles, and avoids the chance of things getting out of sync.
*/

const AP_Param::GroupInfo RC_Channels::var_info[] = {
    // @Group: 1_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 2_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 3_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 4_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 5_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 6_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 7_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 8_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 9_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 10_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_", 10, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 11_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_", 11, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 12_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_", 12, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 13_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_", 13, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 14_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_", 14, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 15_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_", 15, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 16_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_", 16, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

#if NUM_RC_CHANNELS > 16
    // @Group: 17_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[16], "17_", 17, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 17
    // @Group: 18_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[17], "18_", 18, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 18
    // @Group: 19_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[18], "19_", 19, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 19
    // @Group: 20_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[19], "20_", 20, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 20
    // @Group: 21_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[20], "21_", 21, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 21
    // @Group: 22_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[21], "22_", 22, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 22
    // @Group: 23_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[22], "23_", 23, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 23
    // @Group: 24_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[23], "24_", 24, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 24
    // @Group: 25_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[24], "25_", 25, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 25
    // @Group: 26_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[25], "26_", 26, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 26
    // @Group: 27_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[26], "27_", 27, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 27
    // @Group: 28_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[27], "28_", 28, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 28
    // @Group: 29_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[28], "29_", 29, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 29
    // @Group: 30_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[29], "30_", 30, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 30
    // @Group: 31_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[30], "31_", 31, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),
#endif
#if NUM_RC_CHANNELS > 31
    // @Group: 32_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[31], "32_", 36, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS), // ATTENTION: is 36, not 32, as 32 was already occupied
#endif

    // @Param: _OVERRIDE_TIME
    // @DisplayName: RC override timeout
    // @Description: Timeout after which RC overrides will no longer be used, and RC input will resume, 0 will disable RC overrides, -1 will never timeout, and continue using overrides until they are disabled
    // @User: Advanced
    // @Range: 0.0 120.0
    // @Units: s
    AP_GROUPINFO("_OVERRIDE_TIME", 32, RC_CHANNELS_SUBCLASS, _override_timeout, 3.0),

    // @Param: _OPTIONS
    // @DisplayName: RC options
    // @Description: RC input options
    // @User: Advanced
    // @Bitmask: 0:Ignore RC Receiver, 1:Ignore MAVLink Overrides, 2:Ignore Receiver Failsafe bit but allow other RC failsafes if setup, 3:FPort Pad, 4:Log RC input bytes, 5:Arming check throttle for 0 input, 6:Skip the arming check for neutral Roll/Pitch/Yaw sticks, 7:Allow Switch reverse, 8:Use passthrough for CRSF telemetry, 9:Suppress CRSF mode/rate message for ELRS systems,10:Enable multiple receiver support, 11:Use Link Quality for RSSI with CRSF, 12:Annotate CRSF flight mode with * on disarm, 13: Use 420kbaud for ELRS protocol, 14: Clear MAVLink overrides on any stick input
    AP_GROUPINFO("_OPTIONS", 33, RC_CHANNELS_SUBCLASS, _options, (uint32_t)RC_Channels::Option::ARMING_CHECK_THROTTLE),

    // _PROTOCOLS copied to AP_Periph/Parameters.cpp
    // @Param: _PROTOCOLS
    // @DisplayName: RC protocols enabled
    // @Description: Bitmask of enabled RC protocols. Allows narrowing the protocol detection to only specific types of RC receivers which can avoid issues with incorrect detection. Set to 1 to enable all protocols.
    // @User: Advanced
    // @Bitmask: 0:All,1:PPM,2:IBUS,3:SBUS,4:SBUS_NI,5:DSM,6:SUMD,7:SRXL,8:SRXL2,9:CRSF,10:ST24,11:FPORT,12:FPORT2,13:FastSBUS,14:DroneCAN,15:Ghost,16:MAVRadio,18:SITL UDP
    AP_GROUPINFO("_PROTOCOLS", 34, RC_CHANNELS_SUBCLASS, _protocols, 1),

    // @Param: _FS_TIMEOUT
    // @DisplayName: RC Failsafe timeout
    // @Description: RC failsafe will trigger this many seconds after loss of RC
    // @User: Standard
    // @Range: 0.1 10.0
    // @Units: s
    AP_GROUPINFO("_FS_TIMEOUT", 35, RC_CHANNELS_SUBCLASS, _fs_timeout, 1.0),

    AP_GROUPEND
};

#endif  // AP_RC_CHANNEL_ENABLED
