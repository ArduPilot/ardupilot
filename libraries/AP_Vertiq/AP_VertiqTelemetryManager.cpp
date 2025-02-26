#include "AP_VertiqTelemetryManager.h"

extern const AP_HAL::HAL& hal;

AP_VertiqTelemetryManager::AP_VertiqTelemetryManager(IFCIPackedMessage * transmission_message, AP_VertiqClientManager * client_manager):
    _using_telemetry(false),
    _number_telem_ids(0),
    _current_telem_id_index(0),
    _last_telem_request_time(0),
    _transmission_message(transmission_message),
    _client_manager(client_manager)
{
    for (uint8_t ifci_ptr = 0; ifci_ptr < MAX_SUPPORTED_TELEM_IDS; ifci_ptr++) {
        _telemetry_ifci_clients[ifci_ptr] = nullptr;
    }
}

void AP_VertiqTelemetryManager::Init(int32_t telem_bitmask)
{
    //If there's even anything for us to check
    if (telem_bitmask) {
        //At least one motor wants telemetry, make sure to let anyone who wants to know that know
        _using_telemetry = true;

        //Go through the bitmask, and pull out the module IDs we need to care about
        for (uint8_t target_bit = 0; target_bit < MAX_SUPPORTED_TELEM_MODULE_ID; target_bit++) {
            //There's a 1 in the bottom bit which means you want telemetry from it
            if ((telem_bitmask & 0x00000001) && (_number_telem_ids < MAX_SUPPORTED_TELEM_IDS)) {
                _module_ids_in_use[_current_telem_id_index] = target_bit;

                //Make our IFCI client to go at this index, and make sure the client manager knows about it
                _telemetry_ifci_clients[_current_telem_id_index] = new IQUartFlightControllerInterfaceClient(target_bit);
                _client_manager->RegisterNewClient(_telemetry_ifci_clients[_current_telem_id_index]);

                _current_telem_id_index++;
                _number_telem_ids++;
            }

            //We're done with checking this bit, shift on down
            telem_bitmask = telem_bitmask >> 1;
        }

        //Set our active IFCI client to whoever is in the first index. We know there's someone there since the bitmask isn't 0
        _active_ifci_client = _telemetry_ifci_clients[0];
    }
}

bool AP_VertiqTelemetryManager::ProcessIquartTelemResponse()
{
    //We heard something. Go ahead and process it.
    if ((_active_ifci_client != nullptr) && (_active_ifci_client->telemetry_.IsFresh())) {
        //Grab the data
        IFCITelemetryData motor_telem = _active_ifci_client->telemetry_.get_reply();

        //Figure out who gave us this data
        uint8_t reporting_module_id = _active_ifci_client->obj_idn_;

        //Take the motor's speed in rad/s, and go to RPM. Then tell the flight controller about it
        float speed_rpm = abs(motor_telem.speed * M_1_PI * 0.5 * 60); //Note: ArduPilot Mavlink doesn't know how to handle negative RPM telem. Just make it positive
        update_rpm(reporting_module_id, speed_rpm, 0.0);

        //Give the flight controller the rest of the telemetry we got
        TelemetryData telem_for_fc{};

        telem_for_fc.temperature_cdeg = motor_telem.mcu_temp;
        telem_for_fc.motor_temp_cdeg = motor_telem.coil_temp;
        telem_for_fc.voltage = motor_telem.voltage * 0.01;
        telem_for_fc.current = motor_telem.current * 0.01;
        telem_for_fc.consumption_mah = motor_telem.consumption;
        telem_for_fc.usage_s = motor_telem.uptime;
        update_telem_data(
            reporting_module_id,
            telem_for_fc,
            TelemetryType::TEMPERATURE|
            TelemetryType::MOTOR_TEMPERATURE|
            TelemetryType::VOLTAGE|
            TelemetryType::CURRENT|
            TelemetryType::CONSUMPTION|
            TelemetryType::USAGE
        );

        //Make sure everyone knows we heard something
        return true;
    }

    return false;
}

void AP_VertiqTelemetryManager::UpdateTelemetryByte()
{
    if (_using_telemetry) {
        uint32_t time_now = AP_HAL::millis();

        //Check to see if we got a response from our target or if enough time has passed that we're giving up on them
        bool got_telem_response = ProcessIquartTelemResponse();
        bool timed_out = (time_now - _last_telem_request_time) > TELEM_RESPONSE_TIMEOUT_MS;

        if (timed_out || got_telem_response) {
            //It's time to move on. Update the index and time
            _current_telem_id_index = (_current_telem_id_index + 1) % _number_telem_ids;
            _last_telem_request_time = time_now;

            uint8_t new_module_id = _module_ids_in_use[_current_telem_id_index];

            _transmission_message->telem_byte = new_module_id;

            if ( _telemetry_ifci_clients[_current_telem_id_index] != nullptr) {
                _active_ifci_client = _telemetry_ifci_clients[_current_telem_id_index];
            }

            return;
        }
    }

    //We're still waiting for something to happen, make sure we're not overwhelming any one module ID
    _transmission_message->telem_byte = IMPOSSIBLE_MODULE_ID;
}