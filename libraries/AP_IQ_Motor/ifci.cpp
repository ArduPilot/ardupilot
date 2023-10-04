#include "ifci.hpp"

IFCI::IFCI() :
telemetry_(kTypeIFCI, 63, kTelemetry),
last_telemetry_received_id_(63)
{
};

void IFCI::BroadcastPackedControlMessage(CommunicationInterface &com, uint16_t* values, uint8_t length, uint8_t telem)
{
  SendPackedControlMessage(com, values, length, telem, kBroadcastID);
}

void IFCI::SendPackedControlMessage(CommunicationInterface &com, uint16_t* values, uint8_t length, uint8_t telem, uint8_t obj_id)
{
  uint8_t cv_array_size = 2 * length;
  uint8_t payload_size = cv_array_size + 1;
  uint8_t total_size = 2 + payload_size;
  uint8_t tx_msg[total_size]; // must fit outgoing message
  tx_msg[0] = kPackedControlMessage;
  tx_msg[1] = (obj_id<<2) | Access::kSet; // high six | low two
  memcpy(&tx_msg[2], &values[0], cv_array_size);
  tx_msg[total_size - 1] = telem;
  com.SendPacket(kTypeIFCI, tx_msg, total_size);
}

int IFCI::ReadTelemetry(uint8_t* rx_data, uint8_t rx_length)
{
  uint8_t type_idn = rx_data[0];
  uint8_t sub_idn = rx_data[1];
  uint8_t obj_idn = rx_data[2] >> 2; // high 6 bits are obj_idn
  Access dir = static_cast<Access>(rx_data[2] & 0b00000011); // low two bits
  if(dir == kReply)
  {
    // if sub_idn is within array range (safe to access array at this location)
    if(sub_idn == kTelemetry && type_idn == kTypeIFCI)
    {
      // ... then we have a valid message
      telemetry_.Reply(&rx_data[3],rx_length-3);
      last_telemetry_received_id_ = obj_idn;
      return obj_idn; // I parsed something
    }
  }
  return kBroadcastID; // I didn't parse anything
}

uint8_t IFCI::get_last_telemetry_receeived_id()
{
  return last_telemetry_received_id_;
}