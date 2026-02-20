#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE==1

#pragma once
#include <GCS_MAVLink/GCS.h>
#include "AP_CustomStorage/AP_CustomStorage.h"

#define MAX_AB_PARAM_SIZE MAVLINK_MSG_AIRBOUND_PARAMETER_GETSET_FIELD_DATA_VALUE_LEN

class AP_CustomMavlinkHandler {
public:
    static void handle_custom_message(mavlink_channel_t chan, const mavlink_message_t &msg);
    static void init(void);
// Manually define our message structure
#pragma pack(push, 1)
    typedef struct {
        uint8_t target_sysid;
        uint8_t target_compid;
        uint8_t param;
        uint8_t action;
        char value[MAX_AB_PARAM_SIZE];
    } uuid_update_t;
#pragma pack(pop)
    static const uint16_t CUSTOM_MSG_ID = 15222;
};

#endif