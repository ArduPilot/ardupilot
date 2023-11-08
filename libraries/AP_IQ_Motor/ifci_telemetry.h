#ifndef IFCI_TELEMETRY_HPP_
#define IFCI_TELEMETRY_HPP_

//Structure to hold all of the data gotten in an IFCI telemetry packet
struct IFCITelemetryData {
  int16_t mcu_temp; //centi degC
  int16_t coil_temp; //centi degC
  int16_t voltage; //cV
  int16_t current; //cA
  int16_t consumption; //mAh
  int16_t speed; //rad/s
  uint32_t uptime; //seconds
};
#endif // IFCI_TELEMETRY_HPP_