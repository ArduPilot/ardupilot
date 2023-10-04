#include "client_communication.hpp"
#include "ifci_telemetry.h"

class IFCI
{
public:
  IFCI();
  void BroadcastPackedControlMessage(CommunicationInterface &com, uint16_t* values, uint8_t length, uint8_t telem);
  void SendPackedControlMessage(CommunicationInterface &com, uint16_t* values, uint8_t length, uint8_t telem, uint8_t obj_id);
  int ReadTelemetry(uint8_t* rx_data, uint8_t rx_length);
  uint8_t get_last_telemetry_receeived_id();
  ClientEntry<IFCITelemetryData>  telemetry_;
private:
  static const uint8_t kPackedControlMessage =  0;
  static const uint8_t kTelemetry = 1;
  static const uint8_t kTypeIFCI = 88;
  static const uint8_t kBroadcastID = 63;
  uint8_t last_telemetry_received_id_ = 63;
  
};