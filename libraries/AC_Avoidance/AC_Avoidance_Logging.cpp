#include "AC_Avoid.h"
#include "AP_OADijkstra.h"
#include "AP_OABendyRuler.h"
#include <AP_Logger/AP_Logger.h>

void AP_OABendyRuler::Write_OABendyRuler(const uint8_t type, const bool active, const float target_yaw, const float target_pitch, const bool resist_chg, const float margin, const Location &final_dest, const Location &oa_dest) const
{
    int32_t oa_dest_alt, final_alt;
    const bool got_oa_dest = oa_dest.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, oa_dest_alt);
    const bool got_final_dest = final_dest.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, final_alt);
    
    const struct log_OABendyRuler pkt{
        LOG_PACKET_HEADER_INIT(LOG_OA_BENDYRULER_MSG),
        time_us     : AP_HAL::micros64(),
        type        : type,
        active      : active,
        target_yaw  : (uint16_t)wrap_360(target_yaw),
        yaw         : (uint16_t)wrap_360(AP::ahrs().yaw_sensor * 0.01f),
        target_pitch: (uint16_t)target_pitch,
        resist_chg  : resist_chg,
        margin      : margin,
        final_lat   : final_dest.lat,
        final_lng   : final_dest.lng,
        final_alt   : got_final_dest ? final_alt : final_dest.alt,
        oa_lat      : oa_dest.lat,
        oa_lng      : oa_dest.lng,
        oa_alt      : got_oa_dest ? oa_dest_alt : oa_dest.alt
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AP_OADijkstra::Write_OADijkstra(const uint8_t state, const uint8_t error_id, const uint8_t curr_point, const uint8_t tot_points, const Location &final_dest, const Location &oa_dest) const
{
    const struct log_OADijkstra pkt{
        LOG_PACKET_HEADER_INIT(LOG_OA_DIJKSTRA_MSG),
        time_us     : AP_HAL::micros64(),
        state       : state,
        error_id    : error_id,
        curr_point  : curr_point,
        tot_points  : tot_points,
        final_lat   : final_dest.lat,
        final_lng   : final_dest.lng,
        oa_lat      : oa_dest.lat,
        oa_lng      : oa_dest.lng
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AP_OADijkstra::Write_Visgraph_point(const uint8_t version, const uint8_t point_num, const int32_t Lat, const int32_t Lon) const
{
    const struct log_OD_Visgraph pkt{
        LOG_PACKET_HEADER_INIT(LOG_OD_VISGRAPH_MSG),
        time_us     : AP_HAL::micros64(),
        version     : version,
        point_num   : point_num,
        Lat         : Lat,
        Lon         : Lon,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}


void AC_Avoid::Write_SimpleAvoidance(const uint8_t state, const Vector3f& desired_vel, const Vector3f& modified_vel, const bool back_up) const
{
    const struct log_SimpleAvoid pkt{
        LOG_PACKET_HEADER_INIT(LOG_SIMPLE_AVOID_MSG),
        time_us         : AP_HAL::micros64(),
        state           : state,
        desired_vel_x   : desired_vel.x * 0.01f,
        desired_vel_y   : desired_vel.y * 0.01f,
        desired_vel_z   : desired_vel.z * 0.01f,
        modified_vel_x  : modified_vel.x * 0.01f,
        modified_vel_y  : modified_vel.y * 0.01f,
        modified_vel_z  : modified_vel.z * 0.01f,
        backing_up      : back_up,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
