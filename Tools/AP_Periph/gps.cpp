#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_GPS

/*
  GPS support
 */

#include <dronecan_msgs.h>
#include <AP_GPS/RTCM3_Parser.h>

#define DEBUG_PRINTS 0

#if DEBUG_PRINTS
 # define Debug(fmt, args ...)  do {can_printf(fmt "\n", ## args);} while(0)
#else
 # define Debug(fmt, args ...)
#endif

/*
  handle gnss::RTCMStream
 */
void AP_Periph_FW::handle_RTCMStream(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_gnss_RTCMStream req;
    if (uavcan_equipment_gnss_RTCMStream_decode(transfer, &req)) {
        return;
    }
    gps.handle_gps_rtcm_fragment(0, req.data.data, req.data.len);
}

/*
    handle gnss::MovingBaselineData
*/
#if GPS_MOVING_BASELINE
void AP_Periph_FW::handle_MovingBaselineData(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    ardupilot_gnss_MovingBaselineData msg;
    if (ardupilot_gnss_MovingBaselineData_decode(transfer, &msg)) {
        return;
    }
    gps.inject_MBL_data(msg.data.data, msg.data.len);
    Debug("MovingBaselineData: len=%u\n", msg.data.len);
}
#endif // GPS_MOVING_BASELINE

/*
  convert large values to NaN for float16
 */
static void check_float16_range(float *v, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        const float f16max = 65519;
        if (isinf(v[i]) || v[i] <= -f16max || v[i] >= f16max) {
            v[i] = nanf("");
        }
    }
}

/*
  update CAN GPS
 */
void AP_Periph_FW::can_gps_update(void)
{
    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    gps.update();
    send_moving_baseline_msg();
    send_relposheading_msg();
    if (last_gps_update_ms == gps.last_message_time_ms()) {
        return;
    }
    last_gps_update_ms = gps.last_message_time_ms();

    {
        /*
          send Fix2 packet
        */
        uavcan_equipment_gnss_Fix2 pkt {};
        const Location &loc = gps.location();
        const Vector3f &vel = gps.velocity();
        if (gps.status() < AP_GPS::GPS_OK_FIX_2D && !saw_gps_lock_once) {
            pkt.timestamp.usec = AP_HAL::micros64();
            pkt.gnss_timestamp.usec = 0;
        } else {
            saw_gps_lock_once = true;
            pkt.timestamp.usec = gps.time_epoch_usec();
            pkt.gnss_timestamp.usec = gps.last_message_epoch_usec();
        }
        if (pkt.gnss_timestamp.usec == 0) {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_NONE;
        } else {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_UTC;
        }
        pkt.longitude_deg_1e8 = uint64_t(loc.lng) * 10ULL;
        pkt.latitude_deg_1e8 = uint64_t(loc.lat) * 10ULL;
        pkt.height_ellipsoid_mm = loc.alt * 10;
        pkt.height_msl_mm = loc.alt * 10;
        float undulation;
        if (gps.get_undulation(undulation)) {
            pkt.height_ellipsoid_mm -= undulation*1000;
        }
        for (uint8_t i=0; i<3; i++) {
            pkt.ned_velocity[i] = vel[i];
        }
        pkt.sats_used = gps.num_sats();
        switch (gps.status()) {
        case AP_GPS::GPS_Status::NO_GPS:
        case AP_GPS::GPS_Status::NO_FIX:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
            break;
        }

        pkt.covariance.len = 6;

        float hacc;
        if (gps.horizontal_accuracy(hacc)) {
            pkt.covariance.data[0] = pkt.covariance.data[1] = sq(hacc);
        }
    
        float vacc;
        if (gps.vertical_accuracy(vacc)) {
            pkt.covariance.data[2] = sq(vacc);
        }

        float sacc;
        if (gps.speed_accuracy(sacc)) {
            float vc3 = sq(sacc);
            pkt.covariance.data[3] = pkt.covariance.data[4] = pkt.covariance.data[5] = vc3;
        }

        check_float16_range(pkt.covariance.data, pkt.covariance.len);

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_gnss_Fix2_encode(&pkt, buffer, !canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
    
    /*
      send aux packet
     */
    {
        uavcan_equipment_gnss_Auxiliary aux {};
        aux.hdop = gps.get_hdop() * 0.01;
        aux.vdop = gps.get_vdop() * 0.01;

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_AUXILIARY_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_gnss_Auxiliary_encode(&aux, buffer, !canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    // send the gnss status packet
    {
        ardupilot_gnss_Status status {};

        status.healthy = gps.is_healthy();
        if (gps.logging_present() && gps.logging_enabled() && !gps.logging_failed()) {
            status.status |= ARDUPILOT_GNSS_STATUS_STATUS_LOGGING;
        }
        uint8_t idx; // unused
        if (status.healthy && !gps.first_unconfigured_gps(idx)) {
            status.status |= ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE;
        }

        uint32_t error_codes;
        if (gps.get_error_codes(error_codes)) {
            status.error_codes = error_codes;
        }

        uint8_t buffer[ARDUPILOT_GNSS_STATUS_MAX_SIZE] {};
        const uint16_t total_size = ardupilot_gnss_Status_encode(&status, buffer, !canfdout());
        canard_broadcast(ARDUPILOT_GNSS_STATUS_SIGNATURE,
                        ARDUPILOT_GNSS_STATUS_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);

    }

    // send Heading message if we are not sending RelPosHeading messages and have yaw
    if (gps.have_gps_yaw() && last_relposheading_ms == 0) {
        float yaw_deg, yaw_acc_deg;
        uint32_t yaw_time_ms;
        if (gps.gps_yaw_deg(yaw_deg, yaw_acc_deg, yaw_time_ms) && yaw_time_ms != last_gps_yaw_ms) {
            last_gps_yaw_ms = yaw_time_ms;
            
            ardupilot_gnss_Heading heading {};
            heading.heading_valid = true;
            heading.heading_accuracy_valid = is_positive(yaw_acc_deg);
            heading.heading_rad = radians(yaw_deg);
            heading.heading_accuracy_rad = radians(yaw_acc_deg);
            uint8_t buffer[ARDUPILOT_GNSS_HEADING_MAX_SIZE] {};
            const uint16_t total_size = ardupilot_gnss_Heading_encode(&heading, buffer, !canfdout());
            canard_broadcast(ARDUPILOT_GNSS_HEADING_SIGNATURE,
                             ARDUPILOT_GNSS_HEADING_ID,
                             CANARD_TRANSFER_PRIORITY_LOW,
                             &buffer[0],
                             total_size);
        }
    }
}


void AP_Periph_FW::send_moving_baseline_msg()
{
#if GPS_MOVING_BASELINE
    const uint8_t *data = nullptr;
    uint16_t len = 0;
    if (!gps.get_RTCMV3(data, len)) {
        return;
    }
    if (len == 0 || data == nullptr) {
        return;
    }
    // send the packet from Moving Base to be used RelPosHeading calc by GPS module
    ardupilot_gnss_MovingBaselineData mbldata {};
    // get the data from the moving base
    static_assert(sizeof(ardupilot_gnss_MovingBaselineData::data.data) == RTCM3_MAX_PACKET_LEN, "Size of Moving Base data is wrong");
    mbldata.data.len = len;
    memcpy(mbldata.data.data, data, len);
    uint8_t buffer[ARDUPILOT_GNSS_MOVINGBASELINEDATA_MAX_SIZE] {};
    const uint16_t total_size = ardupilot_gnss_MovingBaselineData_encode(&mbldata, buffer, !canfdout());

#if HAL_NUM_CAN_IFACES >= 2
    if (gps_mb_can_port != -1 && (gps_mb_can_port < HAL_NUM_CAN_IFACES)) {
        uint8_t *tid_ptr = get_tid_ptr(MAKE_TRANSFER_DESCRIPTOR(ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE, ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID, 0, CANARD_BROADCAST_NODE_ID));
        canardBroadcast(&dronecan.canard,
                        ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE,
                        ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID,
                        tid_ptr,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size
#if CANARD_MULTI_IFACE
                        ,(1U<<gps_mb_can_port)
#endif
#if HAL_CANFD_SUPPORTED
                       ,canfdout()
#endif
                        );
    } else 
#endif
    {
        // we use MEDIUM priority on this data as we need to get all
        // the data through for RTK moving baseline yaw to work
        canard_broadcast(ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE,
                        ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID,
                        CANARD_TRANSFER_PRIORITY_MEDIUM,
                        &buffer[0],
                        total_size);
    }
    gps.clear_RTCMV3();
#endif // GPS_MOVING_BASELINE
}

void AP_Periph_FW::send_relposheading_msg() {
#if GPS_MOVING_BASELINE
    float reported_heading;
    float relative_distance;
    float relative_down_pos;
    float reported_heading_acc;
    uint32_t curr_timestamp = 0;
    gps.get_RelPosHeading(curr_timestamp, reported_heading, relative_distance, relative_down_pos, reported_heading_acc);
    if (last_relposheading_ms == curr_timestamp) {
        return;
    }
    last_relposheading_ms = curr_timestamp;
    ardupilot_gnss_RelPosHeading relpos {};
    relpos.timestamp.usec = uint64_t(curr_timestamp)*1000LLU;
    relpos.reported_heading_deg = reported_heading;
    relpos.relative_distance_m = relative_distance;
    relpos.relative_down_pos_m = relative_down_pos;
    relpos.reported_heading_acc_deg = reported_heading_acc;
    relpos.reported_heading_acc_available = !is_zero(relpos.reported_heading_acc_deg);
    uint8_t buffer[ARDUPILOT_GNSS_RELPOSHEADING_MAX_SIZE] {};
    const uint16_t total_size = ardupilot_gnss_RelPosHeading_encode(&relpos, buffer, !canfdout());
    canard_broadcast(ARDUPILOT_GNSS_RELPOSHEADING_SIGNATURE,
                    ARDUPILOT_GNSS_RELPOSHEADING_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // GPS_MOVING_BASELINE
}

#endif // HAL_PERIPH_ENABLE_GPS
