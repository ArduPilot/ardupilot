
#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_GPS_IN
#include <AP_GPS/AP_GPS_UAVCANARD.h>
#include <AP_RTC/AP_RTC.h>

void AP_Periph_FW::AP_Periph_GPS_In::uavcan_handle_fix(const uavcan_equipment_gnss_Fix &msg)
{
    if (seen_fix2) {
        // use Fix2 instead
        return;
    }
    bool process = false;

    if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_NO_FIX) {
        interim_state.status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_TIME_ONLY) {
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        } else if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_2D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_3D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        AP::rtc().set_utc_usec(msg.gnss_timestamp.usec, AP_RTC::SOURCE_GPS);
        uint64_t epoch_ms;
        AP::rtc().get_utc_usec(epoch_ms);
        if (epoch_ms != 0) {
            epoch_ms /= 1000;
            uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
            interim_state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
            interim_state.time_week_ms = (uint32_t)(gps_ms - (interim_state.time_week) * AP_MSEC_PER_WEEK);
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = msg.latitude_deg_1e8 / 10;
        loc.lng = msg.longitude_deg_1e8 / 10;
        loc.alt = msg.height_msl_mm / 10;
        interim_state.location = loc;

        if (!isnan(msg.ned_velocity[0])) {
            Vector3f vel(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);
            interim_state.velocity = vel;
            interim_state.ground_speed = norm(vel.x, vel.y);
            interim_state.ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
            interim_state.have_vertical_velocity = true;
        } else {
            interim_state.have_vertical_velocity = false;
        }

        float pos_cov[9];
        memcpy(&pos_cov[0], msg.position_covariance.data, ARRAY_SIZE(pos_cov));
        if (!isnan(pos_cov[8])) {
            if (pos_cov[8] > 0) {
                interim_state.vertical_accuracy = sqrtf(pos_cov[8]);
                interim_state.have_vertical_accuracy = true;
            } else {
                interim_state.have_vertical_accuracy = false;
            }
        } else {
            interim_state.have_vertical_accuracy = false;
        }

        const float horizontal_pos_variance = MAX(pos_cov[0], pos_cov[4]);
        if (!isnan(horizontal_pos_variance)) {
            if (horizontal_pos_variance > 0) {
                interim_state.horizontal_accuracy = sqrtf(horizontal_pos_variance);
                interim_state.have_horizontal_accuracy = true;
            } else {
                interim_state.have_horizontal_accuracy = false;
            }
        } else {
            interim_state.have_horizontal_accuracy = false;
        }

        float vel_cov[9];
        memcpy(&vel_cov[0], msg.position_covariance.data, ARRAY_SIZE(vel_cov));
        if (!isnan(vel_cov[0])) {
            interim_state.speed_accuracy = sqrtf((vel_cov[0] + vel_cov[4] + vel_cov[8]) / 3.0);
            interim_state.have_speed_accuracy = true;
        } else {
            interim_state.have_speed_accuracy = false;
        }

        interim_state.num_sats = msg.sats_used;
    } else {
        interim_state.have_vertical_velocity = false;
        interim_state.have_vertical_accuracy = false;
        interim_state.have_horizontal_accuracy = false;
        interim_state.have_speed_accuracy = false;
        interim_state.num_sats = 0;
    }

    if (!seen_aux) {
        // if we haven't seen an Aux message then populate vdop and
        // hdop from pdop. Some GPS modules don't provide the Aux message
        interim_state.hdop = interim_state.vdop = msg.pdop * 100.0;
    }

    interim_state.last_gps_time_ms = AP_HAL::millis();

    //_new_data = true;
    if (!seen_message) {
        if (interim_state.status == AP_GPS::GPS_Status::NO_GPS) {
            // the first time we see a fix message we change from
            // NO_GPS to NO_FIX, indicating to user that a UAVCAN GPS
            // has been seen
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        }
        seen_message = true;
    }

    AP_GPS_UAVCANARD::set_GPS_State(interim_state);
}


void AP_Periph_FW::AP_Periph_GPS_In::uavcan_handle_fix2(const uavcan_equipment_gnss_Fix2 &msg)
{
    bool process = false;
    seen_fix2 = true;

    if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX) {
        interim_state.status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY) {
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        } else if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        if (msg.gnss_time_standard == UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC) {
            AP::rtc().set_utc_usec(msg.gnss_timestamp.usec, AP_RTC::SOURCE_GPS);
            uint64_t epoch_ms;
            AP::rtc().get_utc_usec(epoch_ms);
            if (epoch_ms != 0) {
                epoch_ms /= 1000;
                uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
                interim_state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
                interim_state.time_week_ms = (uint32_t)(gps_ms - (interim_state.time_week) * AP_MSEC_PER_WEEK);
            }
        }

        if (interim_state.status == AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
            if (msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS) {
                interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS;
            } else if (msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK) {
                if (msg.sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT) {
                    interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT;
                } else if (msg.sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED) {
                    interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED;
                }
            }
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = msg.latitude_deg_1e8 / 10;
        loc.lng = msg.longitude_deg_1e8 / 10;
        loc.alt = msg.height_msl_mm / 10;
        interim_state.location = loc;

        if (!isnan(msg.ned_velocity[0])) {
            Vector3f vel(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);
            interim_state.velocity = vel;
            interim_state.ground_speed = norm(vel.x, vel.y);
            interim_state.ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
            interim_state.have_vertical_velocity = true;
        } else {
            interim_state.have_vertical_velocity = false;
        }

        if (msg.covariance.len == 6) {
            if (!isnan(msg.covariance.data[0])) {
                interim_state.horizontal_accuracy = sqrtf(msg.covariance.data[0]);
                interim_state.have_horizontal_accuracy = true;
            } else {
                interim_state.have_horizontal_accuracy = false;
            }
            if (!isnan(msg.covariance.data[2])) {
                interim_state.vertical_accuracy = sqrtf(msg.covariance.data[2]);
                interim_state.have_vertical_accuracy = true;
            } else {
                interim_state.have_vertical_accuracy = false;
            }
            if (!isnan(msg.covariance.data[3]) &&
                !isnan(msg.covariance.data[4]) &&
                !isnan(msg.covariance.data[5])) {
                interim_state.speed_accuracy = sqrtf((msg.covariance.data[3] + msg.covariance.data[4] + msg.covariance.data[5])/3);
                interim_state.have_speed_accuracy = true;
            } else {
                interim_state.have_speed_accuracy = false;
            }
        }

        interim_state.num_sats = msg.sats_used;
    } else {
        interim_state.have_vertical_velocity = false;
        interim_state.have_vertical_accuracy = false;
        interim_state.have_horizontal_accuracy = false;
        interim_state.have_speed_accuracy = false;
        interim_state.num_sats = 0;
    }

    if (!seen_aux) {
        // if we haven't seen an Aux message then populate vdop and
        // hdop from pdop. Some GPS modules don't provide the Aux message
        interim_state.hdop = interim_state.vdop = msg.pdop * 100.0;
    }
    
    interim_state.last_gps_time_ms = AP_HAL::millis();

    //_new_data = true;
    if (!seen_message) {
        if (interim_state.status == AP_GPS::GPS_Status::NO_GPS) {
            // the first time we see a fix message we change from
            // NO_GPS to NO_FIX, indicating to user that a UAVCAN GPS
            // has been seen
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        }
        seen_message = true;
    }

    AP_GPS_UAVCANARD::set_GPS_State(interim_state);
}

void AP_Periph_FW::AP_Periph_GPS_In::uavcan_handle_aux(const uavcan_equipment_gnss_Auxiliary &msg)
{
    if (!isnan(msg.hdop)) {
        seen_aux = true;
        interim_state.hdop = msg.hdop * 100.0;
    }

    if (!isnan(msg.vdop)) {
        seen_aux = true;
        interim_state.vdop = msg.vdop * 100.0;
    }

    AP_GPS_UAVCANARD::set_GPS_State(interim_state);
}

void AP_Periph_FW::AP_Periph_GPS_In::uavcan_handle_heading(const ardupilot_gnss_Heading &msg)
{
    if (interim_state.gps_yaw_configured == false) {
        interim_state.gps_yaw_configured = msg.heading_valid;
    }

    interim_state.have_gps_yaw = msg.heading_valid;
    interim_state.gps_yaw = degrees(msg.heading_rad);
    if (interim_state.have_gps_yaw) {
        interim_state.gps_yaw_time_ms = AP_HAL::millis();
    }

    interim_state.have_gps_yaw_accuracy = msg.heading_accuracy_valid;
    interim_state.gps_yaw_accuracy = degrees(msg.heading_accuracy_rad);

    AP_GPS_UAVCANARD::set_GPS_State(interim_state);
}

void AP_Periph_FW::AP_Periph_GPS_In::uavcan_handle_status(const ardupilot_gnss_Status &msg)
{
    AP_GPS_UAVCANARD::set_status(msg.status, msg.healthy);
}

#endif // HAL_PERIPH_ENABLE_GPS_IN
