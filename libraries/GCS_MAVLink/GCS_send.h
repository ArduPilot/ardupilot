// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.

#ifndef __GCS_send_H
#define __GCS_send_H

#include <AP_HAL.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <DataFlash.h>
#include <AP_Mission.h>
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>
#include <MAVLink_routing.h>
#include "../AP_SerialManager/AP_SerialManager.h"
#include "../AP_Mount/AP_Mount.h"
#include "GCS_private.h"


///
/// @class	GCS_MAVLINK
/// @brief	MAVLink transport control class
///
class GCS_send : public GCS_private {
public /*but should be protected*/:
    // setto true if this GCS link is active
    bool initialised;
    
public:
    // accessor for initialised
    bool is_initialized() const;
    
    void setup_uart(const AP_SerialManager& serial_manager, AP_SerialManager::SerialProtocol protocol, uint8_t instance);
    // accessor for uart
    AP_HAL::UARTDriver *get_uart() { return _port; }
    
    // call to reset the timeout window for entering the cli
    void set_snoop(void (*_msg_snoop)(const mavlink_message_t* msg));
    // return a bitmap of active channels. Used by libraries to loop
    // over active channels to send to all active channels    
    static uint8_t active_channel_mask(void) { return mavlink_active; }
    
public:    
    GCS_send();
    virtual void init(AP_HAL::UARTDriver *port, mavlink_channel_t mav_chan);
    
    // vehicle specific message send function
    virtual bool try_send_message(enum ap_message id) = 0;
    // vehicle specific message send function
    virtual void data_stream_send(void) = 0;
    
    void send_message(enum ap_message id);
    void send_text(gcs_severity severity, const char *str);
    void send_text_P(gcs_severity severity, const prog_char_t *str);
    void queued_param_send();
    void queued_waypoint_send();
    
    // common send functions
    void send_meminfo(void);
    void send_power_status(void);
    void send_ahrs2(AP_AHRS &ahrs);
    bool send_gps_raw(AP_GPS &gps);
    void send_system_time(AP_GPS &gps);
    void send_radio_in(uint8_t receiver_rssi);
    void send_raw_imu(const AP_InertialSensor &ins, const Compass &compass);
    void send_scaled_pressure(AP_Baro &barometer);
    void send_sensor_offsets(const AP_InertialSensor &ins, const Compass &compass, AP_Baro &barometer);
    void send_ahrs(AP_AHRS &ahrs);
    void send_battery2(const AP_BattMonitor &battery);
#if AP_AHRS_NAVEKF_AVAILABLE
    void send_opticalflow(AP_AHRS_NavEKF &ahrs, const OpticalFlow &optflow);
#endif
    void send_autopilot_version(void) const;
    void send_local_position(const AP_AHRS &ahrs) const;

    /*
      send a statustext message to all active MAVLink
      connections. This function is static so it can be called from
      any library
    */
    static void send_statustext_all(const prog_char_t *msg);

    /*
      send a MAVLink message to all components with this vehicle's system id
      This is a no-op if no routes to components have been learned
    */
    static void send_to_components(const mavlink_message_t* msg) { routing.send_to_components(msg); }
};

#endif // __GCS_H
