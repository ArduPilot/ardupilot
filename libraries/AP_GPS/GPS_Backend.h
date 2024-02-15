/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  GPS driver backend class
 */
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_config.h>
#include <AP_RTC/JitterCorrection.h>
#include "AP_GPS.h"
#include "AP_GPS_config.h"

#ifndef AP_GPS_DEBUG_LOGGING_ENABLED
// enable this to log all bytes from the GPS. Also needs a call to
// log_data() in each backend
#define AP_GPS_DEBUG_LOGGING_ENABLED 0
#endif

#ifndef AP_GPS_MB_MIN_LAG
#define AP_GPS_MB_MIN_LAG 0.05f
#endif

#ifndef AP_GPS_MB_MAX_LAG
#define AP_GPS_MB_MAX_LAG 0.25f
#endif

#if AP_GPS_DEBUG_LOGGING_ENABLED
#include <AP_HAL/utility/RingBuffer.h>
#endif

class AP_GPS_Backend
{
public:
    AP_GPS_Backend(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // we declare a virtual destructor so that GPS drivers can
    // override with a custom destructor if need be.
    virtual ~AP_GPS_Backend(void) {}

    // The read() method is the only one needed in each driver. It
    // should return true when the backend has successfully received a
    // valid packet from the GPS.
    virtual bool read() = 0;

    // Highest status supported by this GPS. 
    // Allows external system to identify type of receiver connected.
    virtual AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D; }

    virtual bool is_configured(void) const { return true; }

    virtual void inject_data(const uint8_t *data, uint16_t len);

#if HAL_GCS_ENABLED
    //MAVLink methods
    virtual bool supports_mavlink_gps_rtk_message() const { return false; }
    virtual void send_mavlink_gps_rtk(mavlink_channel_t chan);
    virtual void handle_msg(const mavlink_message_t &msg) { return ; }
#endif

    virtual void broadcast_configuration_failure_reason(void) const { return ; }

#if HAL_MSP_GPS_ENABLED
    virtual void handle_msp(const MSP::msp_gps_data_message_t &pkt) { return; }
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    virtual void handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt) { return; }
#endif
    
    // driver specific lag, returns true if the driver is confident in the provided lag
    virtual bool get_lag(float &lag) const { lag = 0.2f; return true; }

    // driver specific health, returns true if the driver is healthy
    virtual bool is_healthy(void) const { return true; }
    // returns true if the GPS is doing any logging it is expected to
    virtual bool logging_healthy(void) const { return true; }

    virtual const char *name() const = 0;

    void broadcast_gps_type() const;
    virtual void Write_AP_Logger_Log_Startup_messages() const;

    virtual bool prepare_for_arming(void) { return true; }

    // optional support for retrieving RTCMv3 data from a moving baseline base
    virtual bool get_RTCMV3(const uint8_t *&bytes, uint16_t &len) { return false; }
    virtual void clear_RTCMV3(void) {};

    virtual bool get_error_codes(uint32_t &error_codes) const { return false; }

    // return iTOW of last message, or zero if not supported
    uint32_t get_last_itow_ms(void) const;

    // check if an option is set
    bool option_set(const AP_GPS::DriverOptions option) const {
        return gps.option_set(option);
    }

protected:
    AP_HAL::UARTDriver *port;           ///< UART we are attached to
    AP_GPS &gps;                        ///< access to frontend (for parameters)
    AP_GPS::GPS_State &state;           ///< public state for this instance

    uint64_t _last_pps_time_us;
    JitterCorrection jitter_correction;
    uint32_t _last_itow_ms;
    bool _have_itow;

    /*
      fill in 3D velocity from 2D components
     */
    void fill_3d_velocity(void);

    /*
      fill ground course and speed from velocity
     */
    void velocity_to_speed_course(AP_GPS::GPS_State &s);

    /*
       fill in time_week_ms and time_week from BCD date and time components
       assumes MTK19 millisecond form of bcd_time
    */
    void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);

    void _detection_message(char *buffer, uint8_t buflen) const;

    bool should_log() const;

    /*
      set a timestamp based on arrival time on uart at current byte,
      assuming the message started nbytes ago
     */
    void set_uart_timestamp(uint16_t nbytes);

    void check_new_itow(uint32_t itow, uint32_t msg_length);

#if GPS_MOVING_BASELINE
    bool calculate_moving_base_yaw(const float reported_heading_deg, const float reported_distance, const float reported_D);
    bool calculate_moving_base_yaw(AP_GPS::GPS_State &interim_state, const float reported_heading_deg, const float reported_distance, const float reported_D);
#endif //GPS_MOVING_BASELINE

    // get GPS type, for subtype config
    AP_GPS::GPS_Type get_type() const {
        return gps.get_type(state.instance);
    }

    virtual void set_pps_desired_freq(uint8_t freq) {}

#if AP_GPS_DEBUG_LOGGING_ENABLED
    // log some data for debugging
    void log_data(const uint8_t *data, uint16_t length);
#endif

    // set alt in location, honouring GPS driver option for ellipsoid height
    void set_alt_amsl_cm(AP_GPS::GPS_State &_state, int32_t alt_amsl_cm);

private:
    // itow from previous message
    uint64_t _pseudo_itow;
    int32_t _pseudo_itow_delta_ms;
    uint32_t _last_ms;
    uint32_t _rate_ms;
    uint32_t _last_rate_ms;
    uint16_t _rate_counter;

#if AP_GPS_DEBUG_LOGGING_ENABLED
    // support raw GPS logging
    static struct loginfo {
        int fd = -1;
        ByteBuffer buf{16000};
    } logging[2];
    static bool log_thread_created;
    static void logging_loop(void);
    void logging_start(void);
#endif

};

#endif  // AP_GPS_ENABLED
