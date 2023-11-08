
#ifndef IFCI_HPP
#define IFCI_HPP

#include "client_communication.hpp"
#include "ifci_telemetry.h"

//This class handles all IFCI communication to and from a Vertiq module
class IFCI
{
public:
    //Construct a new IFCI object
    IFCI();

    //Sends a packed control message to all vertiq modules connected to the serial bus
    void BroadcastPackedControlMessage(CommunicationInterface &com, uint16_t* values, uint8_t length, uint8_t telem);

    //Sends a packed control message to a specified module on the serial bus
    void SendPackedControlMessage(CommunicationInterface &com, uint16_t* values, uint8_t length, uint8_t telem, uint8_t obj_id);

    //Extracts the telemetry data out of a buffer of serial data
    int ReadTelemetry(uint8_t* rx_data, uint8_t rx_length);

    //Returns the module ID of the last received telemetry data
    uint8_t get_last_telemetry_received_id();

    //A client entry of type IFCITelemetryData
    ClientEntry<IFCITelemetryData>  telemetry_;

private:
    static const uint8_t kPackedControlMessage =  0;
    static const uint8_t kTelemetry = 1;
    static const uint8_t kTypeIFCI = 88;
    static const uint8_t kBroadcastID = 63;
    uint8_t last_telemetry_received_id_ = 63;

};

#endif //IFCI_HPP