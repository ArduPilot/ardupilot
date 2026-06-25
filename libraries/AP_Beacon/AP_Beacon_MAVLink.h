#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_MAVLINK_ENABLED

class AP_Beacon_MAVLink : public AP_Beacon_Backend {
public:
  using AP_Beacon_Backend::AP_Beacon_Backend;

  bool healthy() override;
  void update() override {}
  void handle_msg(const struct __mavlink_message &msg) override;

private:
  bool handle_range_msg(const struct __mavlink_ranging_beacon_t& bcn_range);

  uint32_t last_update_ms;
};

#endif // AP_BEACON_MAVLINK_ENABLED
