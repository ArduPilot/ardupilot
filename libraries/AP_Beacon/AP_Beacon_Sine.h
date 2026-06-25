#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_SINE_ENABLED

class AP_Beacon_Sine : public AP_Beacon_Backend {
public:
  AP_Beacon_Sine(AP_Beacon &frontend);

  bool healthy() override;
  void update() override;
  void handle_msg(const struct __mavlink_message &msg) override;

private:
  void warmup();
  bool handle_range_msg(const struct __mavlink_ranging_beacon_t& bcn_range);

  uint32_t last_update_ms;

  uint32_t warmup_readings;
  bool warmup_complete;
};

#endif // AP_BEACON_SINE_ENABLED
