/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

/*
    Implemented Digital Sky RFM features as required for Indian Drone Certification.
*/

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_DIGITAL_SKY_RFM

#pragma GCC diagnostic ignored "-Wunused-function"
#include <AP_Math/AP_Math.h>
#include "AP_DigitalSky.h"
#include "KeyManager.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <StorageManager/StorageManager.h>
#include <time.h>
#include <GCS_MAVLink/GCS.h>
#include <npnt.h>
#include <AP_Terrain/AP_Terrain.h>

extern const AP_HAL::HAL& hal;

static npnt_s npnt_handle;

#define STORAGE_PREVLOGHASH_OFF 0
#define STORAGE_UIN_OFF 32
#define STORAGE_OID_OFF 64
static StorageAccess dsnpnt_data(StorageManager::StorageDSNPNT);

//Setup DSNPNT at the initialisation step
AP_DSNPNT::AP_DSNPNT()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many DSNPNT modules");
        return;
    }
    _singleton = this;
}


bool AP_DSNPNT::load_permission() {
    //Initialise NPNT Handler
    npnt_init_handle(&npnt_handle);
    if (_check_npnt_permission()) {
        permission_granted = true;
        if (!verify_permission_reg) {
            hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_DSNPNT::verify_permission, void));
            verify_permission_reg = true;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Permission Granted!");
        return true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Valid Permart not found!\n");
    return false;
}

void AP_DSNPNT::verify_permission() {
    if (permission_granted && ((AP_HAL::millis()-last_verify_time_ms) > 1000)) {
        //Permission granted
        last_verify_time_ms = AP_HAL::millis();
        Vector2f curr_loc;
        time_t t_now;
        float alt;
        update_permission(true, curr_loc, t_now, alt);
        if (hal.util->get_soft_armed()) {
            int ret = npnt_start_logger(&npnt_handle, t_now, curr_loc.x, curr_loc.y, alt);
            if (ret < 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "DigitalSky: Logging Failed: %d", ret);
            }
        } else {
            int ret = npnt_stop_logger(&npnt_handle, t_now, curr_loc.x, curr_loc.y, alt);
            if (ret < 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "DigitalSky: Log File Close Failed: %d", ret);
            }
            permission_granted = false;
        }
    }
}

bool AP_DSNPNT::update_permission(bool do_log, Vector2f &curr_loc, time_t &t_now, float &alt)
{
    //Check if we are breaching fence before giving permission
    Location loc;
    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix);

    t_now = (time_t)(time_unix/1000000);
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    time_t t_start = mktime(&npnt_handle.flight_params.flightStartTime);
    time_t t_end = mktime(&npnt_handle.flight_params.flightEndTime);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    time_t t_start = timegm(&npnt_handle.flight_params.flightStartTime);
    time_t t_end = timegm(&npnt_handle.flight_params.flightEndTime);
#endif

    auto terrain = AP::terrain();
    if (!terrain || !terrain->height_above_terrain(alt, true)) {
        if (do_log) {
            npnt_log_gps_fail_event(&npnt_handle, t_now, 0.0f, 0.0f, 0.0f);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Bad Altitude!");
        }
        goto fail;
    }

    if (!AP::ahrs().get_position(loc)) {
        if (do_log) {
            npnt_log_gps_fail_event(&npnt_handle, t_now, 0.0f, 0.0f, 0.0f);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Bad GPS!");
        }
        goto fail;
    }

    curr_loc.x = (float)loc.lat / 10000000.0f;
    curr_loc.y = (float)loc.lng / 10000000.0f;
    if (Polygon_outside(curr_loc, fence_verts, npnt_handle.fence.nverts)) {
        if (do_log) {
            npnt_log_fence_breach_event(&npnt_handle, t_now, curr_loc.x, curr_loc.y, alt);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Horizontal Fence Breached!");
        }
        goto fail;
    }

    if (alt > npnt_handle.fence.maxAltitude) {
        if (do_log) {
            npnt_log_fence_breach_event(&npnt_handle, t_now, curr_loc.x, curr_loc.y, alt);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Altitude Fence Breached!");
        }
        goto fail;
    }

    if(difftime(t_now, t_start)<0 || difftime(t_end, t_now)<0){
        if (do_log) {
            npnt_log_time_breach_event(&npnt_handle, t_now, curr_loc.x, curr_loc.y, alt);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Time Breached!");
        }
        goto fail;
    }
    return true;

fail:
    return false;
}

//Load permissionArtefacts
bool AP_DSNPNT::_check_npnt_permission()
{
    struct stat st;
    if (AP::FS().stat(AP_NPNT_PERMART_FILE, &st) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Unable to find Permission Artifact: %s\n", strerror(errno));
        return false;
    }
    int fd = AP::FS().open(AP_NPNT_PERMART_FILE, O_RDONLY);
    uint8_t* permart = (uint8_t*)malloc(st.st_size + 1);
    if (!permart) {
        return false;
    }
    
    uint32_t permart_len;
    permart_len = AP::FS().read(fd, permart, st.st_size);
    if ((off_t)permart_len != st.st_size) {
        free(permart);
        return false;
    }
    //reset npnt handle
    npnt_reset_handle(&npnt_handle);

    //append with NULL character
    permart[st.st_size] = '\0';
    permart_len++;
    int ret = npnt_set_permart(&npnt_handle, permart, permart_len, 0);
    if (ret < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Unable to load Permission Artifact, Err: %d\n", ret);
        return false;
    }

    char UIN[32];
    char OID[64];
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    uint8_t drone_id[12];
    uint8_t drone_id_len = 12;
#else
    uint8_t drone_id[13];
    uint8_t drone_id_len = 13;
#endif
    char* hex_drone_id;
    dsnpnt_data.read_block(UIN, STORAGE_UIN_OFF, 32);
    dsnpnt_data.read_block(OID, STORAGE_OID_OFF, 64);

    if (strcmp(npnt_handle.pa_params.operator_id, OID) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Bad Operator ID!");
        return false;
    }

    if (strcmp(npnt_handle.flight_params.uinNo, UIN) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Bad UIN!");
        return false;
    }

    if (!hal.util->get_system_id_unformatted(drone_id, drone_id_len)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to get System ID");
        return false;
    }

    hex_drone_id = hexify((const uint8_t*)drone_id, drone_id_len);
    if (hex_drone_id) {
        if (strcmp(npnt_handle.flight_params.drone_id, hex_drone_id) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Bad Drone ID %d %s %s", drone_id_len, hex_drone_id, npnt_handle.flight_params.drone_id);
            return false;
        }
        free(hex_drone_id);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Failed to get Drone ID");
        return false;
    }

    // Load authorised fence
    if (fence_verts != nullptr) {
        delete fence_verts;
        fence_verts = nullptr;
    }
    if (npnt_handle.fence.nverts < 3 || npnt_handle.fence.vertlat == nullptr || npnt_handle.fence.vertlon == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Received Bad Fence from Permission Artifact\n");
        return false;
    }
    fence_verts = new Vector2f[npnt_handle.fence.nverts + 1];
    if (fence_verts == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Failed to load Fence from Permission Artifact\n");
        return false;
    }
    for (uint8_t i=0; i<npnt_handle.fence.nverts; i++) {
        fence_verts[i].x = npnt_handle.fence.vertlat[i];
        fence_verts[i].y = npnt_handle.fence.vertlon[i];
    }
    fence_verts[npnt_handle.fence.nverts].x = npnt_handle.fence.vertlat[0];
    fence_verts[npnt_handle.fence.nverts].y = npnt_handle.fence.vertlon[0];
    Vector2f curr_loc;
    time_t t_now;
    float alt;
    if (!update_permission(false, curr_loc, t_now, alt)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DigitalSky: Outside Permission!");
        return false;
    }

    return true;
}

void AP_DSNPNT::handle_set_dsnpnt_params(const mavlink_message_t &msg)
{
    mavlink_dsnpnt_params_t dsnpnt_params;
    mavlink_msg_dsnpnt_params_decode(&msg, &dsnpnt_params);
    if (strlen(dsnpnt_params.UIN)) {
        dsnpnt_data.write_block(STORAGE_UIN_OFF, dsnpnt_params.UIN, strlen(dsnpnt_params.UIN)+1);
    }
    if (strlen(dsnpnt_params.OID)) {
        dsnpnt_data.write_block(STORAGE_OID_OFF, dsnpnt_params.OID, strlen(dsnpnt_params.OID)+1);
    }
}

void AP_DSNPNT::handle_get_dsnpnt_params(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_dsnpnt_params_t dsnpnt_params;
    dsnpnt_params.target_system = msg.sysid;
    dsnpnt_params.target_component = msg.compid;
    dsnpnt_data.read_block(dsnpnt_params.UIN, STORAGE_UIN_OFF, 32);
    dsnpnt_data.read_block(dsnpnt_params.OID, STORAGE_OID_OFF, 64);
    mavlink_msg_dsnpnt_params_send_struct(chan, &dsnpnt_params);
}

// singleton instance
AP_DSNPNT *AP_DSNPNT::_singleton;

namespace AP {

AP_DSNPNT &dsnpnt()
{
    return *AP_DSNPNT::get_singleton();
}

}


//NPNT HELPERS
extern "C" {

void reset_sha256(npnt_sha_t *sha_handler)
{
    AP::keymgr().reset_sha256(sha_handler);
}

void update_sha256(npnt_sha_t *sha_handler, const char* data, uint16_t data_len)
{
    AP::keymgr().update_sha256(sha_handler, data, data_len);
}

void final_sha256(npnt_sha_t *sha_handler, uint8_t* hash)
{
    AP::keymgr().final_sha256(sha_handler, hash);
}

int npnt_check_authenticity(npnt_s *handle, const char* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len)
{
    int ret = AP::keymgr().verify_hash_with_server_pkey(hashed_data, hashed_data_len, signature, signature_len);
    return ret;
}

bool open_logfile(npnt_s *handle)
{
    // Load last loghash from storage
    if (!dsnpnt_data.read_block(handle->logger.last_loghash, STORAGE_PREVLOGHASH_OFF, 32)) {
        return false;
    }
    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix);
    time_t t_now = (time_t)(time_unix/1000000);
    struct tm *_tm = gmtime(&t_now);

    // Load logfile
    char file_name[50];
    snprintf(file_name, 50, HAL_BOARD_STORAGE_DIRECTORY "/npntlog_%04d-%02d-%02dT%02d:%02d:%02d.json", 
                        _tm->tm_year, _tm->tm_mon+1, _tm->tm_mday, _tm->tm_hour, _tm->tm_min, _tm->tm_sec);
    hal.console->printf("DigitalSky: Creating logfile: %s\n", file_name);
    handle->logger.log_fd = AP::FS().open(file_name, O_WRONLY|O_CREAT|O_TRUNC);
    if (handle->logger.log_fd < 0) {
        hal.console->printf("DigitalSky: Logging failed: %d %s\n", errno, strerror(errno));
        return false;
    }
    return true;
}

bool write_logfile(npnt_s *handle, const char* data, uint32_t len)
{
    if (data == nullptr) {
        return false;
    }
	if (AP::FS().write(handle->logger.log_fd, data, len) < 0) {
        return false;
    }
    return true;
}

bool close_logfile(npnt_s *handle)
{
    AP::FS().close(handle->logger.log_fd);
    handle->logger.log_fd = 0;
    return true;
}

bool record_lastloghash(uint8_t* data, uint8_t data_len)
{
    if (data == nullptr) {
        return false;
    }
    return dsnpnt_data.write_block(STORAGE_PREVLOGHASH_OFF, data, data_len);
}

} //extern "C"

#endif //HAL_DIGITAL_SKY_RFM
