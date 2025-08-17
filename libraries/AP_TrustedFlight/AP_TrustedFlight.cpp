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
 * Author: @rhythmize
 */

#include "AP_TrustedFlight_Config.h"

#if AP_TRUSTED_FLIGHT_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include "AP_TrustedFlight.h"
#include "LogStructure.h"

extern const AP_HAL::HAL& hal;

AP_TrustedFlight::AP_TrustedFlight()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many TrustedFlight modules");
        return;
    }

    _singleton = this;
}

AP_TrustedFlight::~AP_TrustedFlight()
{
    if (public_key)
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        delete public_key;
#else
        AP_ROMFS::free(public_key);
#endif
    if (token_issuer)
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        delete token_issuer;
#else
        AP_ROMFS::free(token_issuer);
#endif
}

// Aerobridge Trusted Flight module init
void AP_TrustedFlight::init()
{
    if (init_done)
        return;

    // Trusted Flight validation happens pre-arm, it's good to enable log_disarmed if the feature is enabled.
    // NOTE: only override .log_disarmed if defaults at unset
    if (AP::logger()._params.log_disarmed == AP_Logger::LogDisarmed::NONE)
        AP::logger()._params.log_disarmed.set(AP_Logger::LogDisarmed::LOG_WHILE_DISARMED); //AP_Logger::LogDisarmed::LOG_WHILE_DISARMED_NOT_USB

    // read public key
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if(!read_from_file(public_key_path, &public_key, &public_key_length))
#else
    public_key = AP_ROMFS::find_decompress(public_key_path, public_key_length);
    if(!public_key)
#endif
    {
        log_message("Failed to read public key\n");
        return;
    }

    // read token issuer
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if(!read_from_file(token_issuer_path, &token_issuer, &token_issuer_length))
#else 
    token_issuer = AP_ROMFS::find_decompress(token_issuer_path, token_issuer_length);
    if(!token_issuer)
#endif
    {
        log_message("Failed to read token issuer\n");
        return;
    }

    init_done = true;
}

// entry method to check if trusted flight artifacts are valid or not
bool AP_TrustedFlight::is_trusted(char *buffer, size_t buflen)
{
    if (!init_done) {
        hal.util->snprintf(buffer, buflen, "Initialization is not done yet");
        log_message("Initialization is not done yet");
        return false;
    }

    uint64_t time;
    if (!AP::rtc().get_utc_usec(time)) {
        hal.util->snprintf(buffer, buflen, "RTC not available");
        log_message("RTC not available");
        return false;
    } else {
        char msg[100];
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        hal.util->snprintf(msg, 100, "RTC is available. Current utc sec: %ld", time/1000000U);
#else
        hal.util->snprintf(msg, 100, "RTC is available. Current utc sec: %lld", time/1000000U);
#endif
        log_message(msg);
    }

    uint8_t *token;
    uint32_t token_length;

    if(!read_from_file(token_file_path, &token, &token_length))
    {
        hal.util->snprintf(buffer, buflen, "Unable to read token");
        log_message("Failed to read token from file system");
        return false;
    }
    
    log_message("Token is valid.");
    return true;
}

// read file contents
bool AP_TrustedFlight::read_from_file(const char *filepath, uint8_t **outbuf, uint32_t *outsize)
{
    FileData *filedata = AP::FS().load_file(filepath);
    if (filedata == nullptr || filedata->data == nullptr || filedata->length <= 0) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Cannot read file: %s", filepath);
        log_message(msg);
        return false;
    }

    *outbuf = (uint8_t *)malloc(filedata->length + 1);
    if (*outbuf == nullptr) {
        log_message("Cannot allocate buffer for token");
        return false;
    }

    memcpy(*outbuf, filedata->data, filedata->length);
    (*outbuf)[filedata->length]='\0';
    *outsize = filedata->length;

    delete filedata;
    return true;
}

// write log message
void AP_TrustedFlight::log_message(const char *message)
{
    struct log_Message pkt{
        LOG_PACKET_HEADER_INIT(LOG_TRUSTED_FLIGHT_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };

    strncpy_noterm(pkt.msg, message, sizeof(pkt.msg));
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

AP_TrustedFlight *AP_TrustedFlight::_singleton;

namespace AP
{
AP_TrustedFlight &trusted_flight()
{
    return *AP_TrustedFlight::get_singleton();
}
};

#endif // AP_TRUSTED_FLIGHT_ENABLED
