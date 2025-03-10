#include "AP_VertiqOutputManager.h"

AP_VertiqOutputManager::AP_VertiqOutputManager(GenericInterface * iquart_interface, AP_VertiqClientManager * client_manager):
    _iquart_interface(iquart_interface),
    _throttle_manager(&_transmission_message),
    _telemetry_manager(&_transmission_message, client_manager),
    _ifci_control(63)
{

}

void AP_VertiqOutputManager::Init(int32_t telem_bitmask, uint8_t number_cvs_in_use)
{
    _throttle_manager.Init(number_cvs_in_use);
    _telemetry_manager.Init(telem_bitmask);
}

void AP_VertiqOutputManager::Update()
{
    _throttle_manager.UpdateThrottleOutputs();
    _telemetry_manager.UpdateTelemetryByte();

    //We have all of our commands ready now, so let's put out our IFCI message
    _ifci_control.PackageIfciCommandsForTransmission(&_transmission_message, _output_message, &_output_len);

    //Our message is made, pass it over to our IQUART interface
    _ifci_control.packed_command_.set(*_iquart_interface, _output_message, _output_len);
}

