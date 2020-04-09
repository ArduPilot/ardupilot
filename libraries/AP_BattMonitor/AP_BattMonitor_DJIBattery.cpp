#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_DJIBattery.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>

/*
  "battery" monitor for DJI Battery via serial port.

   For future Referance:
   Total Voltage : 8,9
   Percentage : 17
   Cell1 :  20,21
   Cell2 :  22,23
   Cell3 :  24,25
   Cell4 :  26,27
   
 */
extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_DJIBattery::AP_BattMonitor_DJIBattery(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    
    if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DJIBattery, 0))) {
        last_send_us = AP_HAL::micros();
        delay_time_us = 1000000;
        Count = 0;
        _PortAvailable = true;
    } else { 
    _PortAvailable = false;
    gcs().send_text(MAV_SEVERITY_WARNING,"DJIBattery: Port not available or not configured properly");
    }
    // need to add check
    _state.healthy = false;
}

/*
  read - read the "voltage" and "current"
*/
void AP_BattMonitor_DJIBattery::read()
{
    if (_PortAvailable){
        uint32_t now = AP_HAL::micros();
        if (last_send_us != 0 && now - last_send_us > delay_time_us) {
           
            for (uint8_t i=0; i < Count; i++) {
                pktbuf[i] = 0;
            }
  
            uint8_t buf[14] {0xAB, 0x0E, 0x04, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA8};
            port->write(buf, sizeof(buf));
        
            last_send_us = now;
        }
    
        uint32_t n = port->available();
    
        for (uint8_t i=0; i<n; i++) {
            pktbuf[Count++] = port->read();
        }
        
        if (Count == 37){
                uint8_t pkt[Count -1];
                for (uint8_t i = 0; i < Count; i++) {
                    pkt[i] = pktbuf[i];
                }
                
                if ( pktbuf[Count - 1] == crc_crc8( pkt, 36)){
                    
                    _Percentage = pktbuf[17];
                    _UsedCapacity = 1 - (_Percentage / 100);  
                    _state.consumed_mah = _UsedCapacity * _params._pack_capacity;

                    char _temp;
                    snprintf(&_temp,5, "%02X%02X", pktbuf[8],pktbuf[9]);
                    _Volt = strtol(&_temp, NULL, 16); // Reading in millivolt
                    _state.voltage = _Volt / 1000;

                    _state.consumed_wh = 10;
                
                    Count = 0;
                    _state.last_time_micros = now;
                    _state.healthy = true;
                } else {
                    _state.healthy = false;
                    gcs().send_text(MAV_SEVERITY_WARNING,"Battery Serial CRC Mismatch");
                }
        }
        
    }
    
    
}