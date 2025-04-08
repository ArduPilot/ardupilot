#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_DRONECAN_ENABLED
#include "AP_EFI.h"
#include "AP_EFI_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_EFI_DroneCAN : public AP_EFI_Backend {
public:
    AP_EFI_DroneCAN(AP_EFI &_frontend);

    void update() override;

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void trampoline_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ice_reciprocating_Status &msg);

private:
    void handle_status(const uavcan_equipment_ice_reciprocating_Status &pkt);

    // singleton for trampoline
    static AP_EFI_DroneCAN *driver;
};
#endif // AP_EFI_DRONECAN_ENABLED

