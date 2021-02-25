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

extern const AP_HAL::HAL& hal;

static AP_DSNPNT _dsnpnt;


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
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Permission Granted!");
        return true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Valid Permart not found!\n");
    return false;
}

bool AP_DSNPNT::update_permission()
{
    //Check if we are breaching fence before giving permission
    Location loc;
    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix);

    time_t t_now = (time_t)(time_unix/1000000);
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    time_t t_start = mktime(&npnt_handle.flight_params.flightStartTime);
    time_t t_end = mktime(&npnt_handle.flight_params.flightEndTime);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    time_t t_start = timegm(&npnt_handle.flight_params.flightStartTime);
    time_t t_end = timegm(&npnt_handle.flight_params.flightEndTime);
#endif
    Vector2f curr_loc;

    if (!AP::ahrs().get_position(loc)) {
        npnt_log_gps_fail_event(&npnt_handle, t_now, 0.0f, 0.0f, 0.0f);
        goto fail;
    }

    curr_loc.x = (float)loc.lat / 10000000.0f;
    curr_loc.y = (float)loc.lng / 10000000.0f;
    if (Polygon_outside(curr_loc, fence_verts, npnt_handle.fence.nverts)) {
        npnt_log_fence_breach_event(&npnt_handle, t_now, curr_loc.x, curr_loc.y, loc.alt/100.0f);
        goto fail;
    }

    if (loc.alt > (npnt_handle.fence.maxAltitude*100)) {
        npnt_log_fence_breach_event(&npnt_handle, t_now, curr_loc.x, curr_loc.y, loc.alt/100.0);
        goto fail;
    }

    if(difftime(t_now, t_start)<0 || difftime(t_end, t_now)<0){
        npnt_log_time_breach_event(&npnt_handle, t_now, curr_loc.x, curr_loc.y, loc.alt/100.0f);
        goto fail;
    }

    if (hal.util->get_soft_armed()) {
        npnt_start_logger(&npnt_handle, t_now, curr_loc.x, curr_loc.y, loc.alt/100.0f);
    } else {
        npnt_stop_logger(&npnt_handle, t_now, curr_loc.x, curr_loc.y, loc.alt/100.0f);
    }
    return true;

fail:
    if (!hal.util->get_soft_armed()) {
        npnt_stop_logger(&npnt_handle, t_now, curr_loc.x, curr_loc.y, loc.alt/100.0f);
    }
    return false;
}

//Load permissionArtefacts
bool AP_DSNPNT::_check_npnt_permission()
{
    struct stat st;
    if (AP::FS().stat(AP_NPNT_PERMART_FILE, &st) != 0) {
        hal.console->printf("Unable to find Permission Artifact: %s\n", strerror(errno));
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
        hal.console->printf("Unable to load Permission Artifact, Err: %d\n", ret);
        return false;
    }

    // Load authorised fence
    if (fence_verts != nullptr) {
        delete fence_verts;
        fence_verts = nullptr;
    }
    if (npnt_handle.fence.nverts < 3 || npnt_handle.fence.vertlat == nullptr || npnt_handle.fence.vertlon == nullptr) {
        hal.console->printf("Received Bad Fence from Permission Artifact\n");
        return false;
    }
    fence_verts = new Vector2f[npnt_handle.fence.nverts + 1];
    if (fence_verts == nullptr) {
        hal.console->printf("Failed to load Fence from Permission Artifact\n");
        return false;
    }
    for (uint8_t i=0; i<npnt_handle.fence.nverts; i++) {
        fence_verts[i].x = npnt_handle.fence.vertlat[i];
        fence_verts[i].y = npnt_handle.fence.vertlon[i];
    }
    fence_verts[npnt_handle.fence.nverts].x = npnt_handle.fence.vertlat[0];
    fence_verts[npnt_handle.fence.nverts].y = npnt_handle.fence.vertlon[0];

    if (!update_permission()) {
        hal.console->printf("Outside Permission!\n");
        return false;
    }

    return true;
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

void final_sha256(npnt_sha_t *sha_handler, char* hash)
{
    AP::keymgr().final_sha256(sha_handler, (uint8_t*)hash);
}

int npnt_check_authenticity(npnt_s *handle, const char* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len)
{
    int ret = AP::keymgr().verify_hash_with_server_pkey(hashed_data, hashed_data_len, signature, signature_len);
    return ret;
}

static StorageAccess lastloghash(StorageManager::StorageDSNPNT);

bool open_logfile(npnt_s *handle)
{
    // Load last loghash from storage
    if (!lastloghash.read_block(handle->logger.last_loghash, 0, 32)) {
        return false;
    }
    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix);
    time_t t_now = (time_t)(time_unix/1000000);
    struct tm *_tm = gmtime(&t_now);

    // Load logfile
    char file_name[40];
    snprintf(file_name, 40, HAL_BOARD_STORAGE_DIRECTORY "/npntlog_%04d-%02d-%02dT%02d:%02d:%02d.json", 
                        _tm->tm_year, _tm->tm_mon+1, _tm->tm_mday, _tm->tm_hour, _tm->tm_min, _tm->tm_sec);
    handle->logger.log_fd = AP::FS().open(file_name, O_WRONLY|O_APPEND|O_CREAT);
    if (handle->logger.log_fd < 0) {
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

bool sign_data_with_self_key(uint8_t* data, uint32_t len, uint8_t* signature)
{
    return AP::keymgr().sign_data_with_ap_key(data, len, signature);
}

bool record_lastloghash(uint8_t* data, uint8_t data_len)
{
    if (data == nullptr) {
        return false;
    }
    return lastloghash.write_block(0, data, data_len);
}

} //extern "C"

#endif //HAL_DIGITAL_SKY_RFM
