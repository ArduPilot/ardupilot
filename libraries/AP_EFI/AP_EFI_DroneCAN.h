#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

#ifndef HAL_EFI_DRONECAN_ENABLED
#define HAL_EFI_DRONECAN_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024 && HAL_CANMANAGER_ENABLED
#endif

#if HAL_EFI_DRONECAN_ENABLED
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <uavcan/equipment/ice/reciprocating/Status.hpp>

class EFIStatusCb;

class AP_EFI_DroneCAN : public AP_EFI_Backend {
public:
    AP_EFI_DroneCAN(AP_EFI &_frontend);

    void update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static void trampoline_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const EFIStatusCb &cb);

private:
    void handle_status(const uavcan::equipment::ice::reciprocating::Status &pkt);

    // singleton for trampoline
    static AP_EFI_DroneCAN *driver;
};
#endif // HAL_EFI_DRONECAN_ENABLED

