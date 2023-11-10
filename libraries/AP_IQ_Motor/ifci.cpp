#include "ifci.hpp"

IFCI::IFCI() :
    telemetry_(_kTypeIFCI, 63, _kTelemetry),
    _last_telemetry_received_id_(63)
{
};

void IFCI::BroadcastPackedControlMessage(Communication_Interface &com, uint16_t* values, uint8_t length, uint8_t telem)
{
    //Calls the targeted send message, but with the broadcast ID placed as the module ID. This allows all modules to receive the message
    SendPackedControlMessage(com, values, length, telem, _kBroadcastID);
}

void IFCI::SendPackedControlMessage(Communication_Interface &com, uint16_t* values, uint8_t length, uint8_t telem, uint8_t obj_id)
{
    //all control values are 2 bytes, so we need to make sure we have enough space
    uint8_t cv_array_size = 2 * length;

    //Make space for the telemetry ID tail byte
    uint8_t payload_size = cv_array_size + 1;

    //Make space for the CRC bytes
    uint8_t total_size = 2 + payload_size;

    uint8_t tx_msg[total_size]; // must fit outgoing message

    //Load in the type, object, and access. This is always a set/
    tx_msg[0] = _kPackedControlMessage;
    tx_msg[1] = (obj_id<<2) | Access::kSet; // high six | low two

    //Add the rest of our data to the array
    memcpy(&tx_msg[2], &values[0], cv_array_size);

    //Set in the ID of the module whose telemetry we want then send the data
    tx_msg[total_size - 1] = telem;
    com.SendPacket(_kTypeIFCI, tx_msg, total_size);
}

int IFCI::ReadTelemetry(uint8_t* rx_data, uint8_t rx_length)
{
    //Based on the Vertiq standard, grab the data out of the correct places
    uint8_t type_idn = rx_data[0];
    uint8_t sub_idn = rx_data[1];
    uint8_t obj_idn = rx_data[2] >> 2; // high 6 bits are obj_idn
    Access dir = static_cast<Access>(rx_data[2] & 0b00000011); // low two bits
    if (dir == kReply) {
        // if sub_idn is within array range (safe to access array at this location)
        if (sub_idn == _kTelemetry && type_idn == _kTypeIFCI) {
            // ... then we have a valid message
            telemetry_.Reply(&rx_data[3],rx_length-3);
            _last_telemetry_received_id_ = obj_idn;
            return obj_idn; // I parsed something
        }
    }
    return _kBroadcastID; // I didn't parse anything
}

uint8_t IFCI::get_last_telemetry_received_id()
{
    return _last_telemetry_received_id_;
}