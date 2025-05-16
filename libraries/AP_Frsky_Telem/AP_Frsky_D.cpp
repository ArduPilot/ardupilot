#include "AP_Frsky_D.h"

#if AP_FRSKY_D_TELEM_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Baro/AP_Baro.h>


/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_D::send_byte(uint8_t byte)
{
    if (byte == START_STOP_D) {
        _port->write(0x5D);
        _port->write(0x3E);
    } else if (byte == BYTESTUFF_D) {
        _port->write(0x5D);
        _port->write(0x3D);
    } else {
        _port->write(byte);
    }
}

/*
 * send one uint16 frame of FrSky data - for FrSky D protocol (D-receivers)
 */
void AP_Frsky_D::send_uint16(uint16_t id, uint16_t data)
{
    _port->write(START_STOP_D);    // send a 0x5E start byte
    uint8_t *bytes = (uint8_t*)&id;
    send_byte(bytes[0]);
    bytes = (uint8_t*)&data;
    send_byte(bytes[0]); // LSB
    send_byte(bytes[1]); // MSB
}

/*
 * send frame1/2/3 telemetry data
 * for FrSky D protocol (D-receivers)
 */
void AP_Frsky_D::send(void)
{

    uint32_t now = AP_HAL::millis();
    uint8_t cells_count = 3; //hard coded for now
    uint16_t cell_id_voltage;
    
    // send fast FRAME1
    if (now - _D.last_fast_frame >= FAST_FRAME_INTERVAL) {
        _D.last_fast_frame = now;
        
        
        calc_acc();
 
        send_uint16(DATA_ID_ACC_X, (uint16_t)(_SPort_data.acc_x)); 
        send_uint16(DATA_ID_ACC_Y, (uint16_t)(_SPort_data.acc_y)); 
        send_uint16(DATA_ID_ACC_Z, (uint16_t)(_SPort_data.acc_z)); 

        calc_nav_alt();
        
        send_uint16(DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters);      
        send_uint16(DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm);          
        
        calc_temp();
        
        send_uint16(DATA_ID_TEMP1, _SPort_data.temp1);                   
        send_uint16(DATA_ID_TEMP2, _SPort_data.temp2); 

        calc_power(cells_count);

        //send per cell voltages
        for (uint8_t i = 0; i < cells_count; i++) {
            // make it FrSky style (id and volts in 2 bytes)
            cell_id_voltage = (uint16_t) (0x1000*i+(float)_SPort_data.cells_mvolts[i]/1000/4.2*0x0834); //magic number 0x0834 means 2100 ADC steps per 4.2V            
            // cell_id_voltage = (uint16_t) 0x1000*i+3.85/4.2*0x0834; //magic number 0x0834 = 2100 steps per 4.2V
            cell_id_voltage = (cell_id_voltage >> 8) | (cell_id_voltage << 8); //swap bytes

            send_uint16(DATA_ID_VOLTS, (uint16_t) cell_id_voltage);      //send cell voltages
        }
       
        send_uint16(DATA_ID_CURRENT, (uint16_t)_SPort_data.power_ampers); // send current consumption);      
        send_uint16(DATA_ID_VOLTAGE_BP, (uint16_t)(int)(_SPort_data.power_volts));        // 
        send_uint16(DATA_ID_VOLTAGE_AP, (uint16_t)(_SPort_data.power_volts-(int)_SPort_data.power_volts)*10);   // Has to be less 10 
        
        //end FRAME1 data
        
    }
    // send mid FRAME2
    if (now - _D.last_mid_frame >= MID_FRAME_INTERVAL) {
        
        _D.last_mid_frame = now;

        AP_AHRS &_ahrs = AP::ahrs();        
        send_uint16(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360));  // send heading in degree based on AHRS and not GPS
        send_uint16(DATA_ID_GPS_COURS_AP, (uint16_t)(0));                              // .0 
        
        if (AP::gps().status() >= 3) {
            
            calc_gps_position();        
        
            send_uint16(DATA_ID_GPS_LAT_BP, _SPort_data.latdddmm); // send gps latitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LAT_AP, _SPort_data.latmmmm); // send gps latitude minutes decimal part
            send_uint16(DATA_ID_GPS_LAT_NS, _SPort_data.lat_ns); // send gps North / South information
            send_uint16(DATA_ID_GPS_LONG_BP, _SPort_data.londddmm); // send gps longitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LONG_AP, _SPort_data.lonmmmm); // send gps longitude minutes decimal part
            send_uint16(DATA_ID_GPS_LONG_EW, _SPort_data.lon_ew); // send gps East / West information
            send_uint16(DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
            send_uint16(DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
            send_uint16(DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
            send_uint16(DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimal part
        }

        // uint8_t percentage = 0;
        // const AP_BattMonitor &_battery = AP::battery();
        // IGNORE_RETURN(_battery.capacity_remaining_pct(percentage));
        // uint8_t battery_fuel = ((percentage / 25) +1) * 25 ; // 0-25, 26-50, 51-75, 76-100
        // if (battery_fuel > 100) {
        //     battery_fuel = 100;
        // }   
        // send_uint16(DATA_ID_FUEL, (uint16_t) battery_fuel); // send battery remaining
        // // send_uint16(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage

    }
  
  
  
    // send slow FRAME3

    if ((now - _D.last_slow_frame >= SLOW_FRAME_INTERVAL)) {

        _D.last_slow_frame = now;
        
        calc_time();

        // send date - not used
        // send_uint16(DATA_ID_DAY_MONTH, (uint16_t)(0x0101));             // 01.01        
        // send_uint16(DATA_ID_YEAR_0, (uint16_t)(0x01));                  // 2001
       
        //  send time
        send_uint16(DATA_ID_HOURS_MINUTE, (uint16_t)((_SPort_data.minutes << 8) | _SPort_data.hours));    // send hours & mins       
        send_uint16(DATA_ID_SECONDS_0, (uint16_t)(_SPort_data.seconds));                                  // seconds

        uint8_t inst = 0;
        calc_rpm(inst,_SPort_data.rpm);
        
        // Baro alt as RPM (1000 is 0m)
        _SPort_data.rpm = 1000+(uint16_t)AP::baro().get_altitude(); 
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Baro:  %0.2f",(double)(_SPort_data.rpm));

       
        send_uint16(DATA_ID_RPM, (uint16_t)(_SPort_data.rpm));              
        
    }


        


}

#endif  // AP_FRSKY_D_TELEM_ENABLED
