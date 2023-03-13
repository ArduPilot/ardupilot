#ifndef IFCI_TELEMETRY_HPP_
#define IFCI_TELEMETRY_HPP_

struct IFCITelemetryData {
  int16_t mcu_temp; // TODO add units
  int16_t coil_temp;
  int16_t voltage;
  int16_t current;
  int16_t consumption;
  int16_t speed;
  uint32_t uptime;
};
#endif // IFCI_TELEMETRY_HPP_