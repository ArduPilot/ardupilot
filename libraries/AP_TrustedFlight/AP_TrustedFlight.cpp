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
#include "AP_TrustedFlight.h"
#include "LogStructure.h"
#include "AP_JWT.h"

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
    if (public_key) {
        delete public_key;
    }
    if (token_issuer) {
        delete token_issuer;
    }
    public_key = nullptr;
    token_issuer = nullptr;
}

// Aerobridge Trusted Flight module init
void AP_TrustedFlight::init()
{
    if (init_done) {
        return;
    }

    // Trusted Flight validation happens pre-arm, it's good to enable log_disarmed if the feature is enabled.
    // NOTE: only override .log_disarmed if defaults at unset
    if (AP::logger()._params.log_disarmed == AP_Logger::LogDisarmed::NONE) {
        AP::logger()._params.log_disarmed.set(AP_Logger::LogDisarmed::LOG_WHILE_DISARMED);    //AP_Logger::LogDisarmed::LOG_WHILE_DISARMED_NOT_USB
    }

    // read public key + token
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if ((public_key = read_from_file(public_key_path, &public_key_length)) == nullptr) {
        log_message("Failed to read public key\n");
        return;
    }
    if ((token_issuer = read_from_file(token_issuer_path, &token_issuer_length)) == nullptr) {
        log_message("Failed to read token issuer\n");
        return;
    }
#else
    if (app_descriptor.key_type != AP_TrustedFlight::PUBLIC_KEY_EdDSA) {
        log_message("Unsupported Public Key Type in app descriptor\n");
        return;
    }
    public_key_length = app_descriptor.key_len;
    public_key = app_descriptor.key;
    token_issuer_length = app_descriptor.issuer_len;
    token_issuer = app_descriptor.issuer;
#endif
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

    const uint8_t *token;
    uint32_t token_length;

    if ((token = read_from_file(token_file_path, &token_length)) == nullptr) {
        hal.util->snprintf(buffer, buflen, "Unable to read token");
        log_message("Failed to read token from file system");
        return false;
    }

    AP_JWT jwt(token, token_length);
    AP_JWT::TokenValidationResult result = jwt.validate(public_key, token_issuer, token_issuer_length);

    delete token;

    switch (result) {
    case AP_JWT::TOKEN_VALID:
        log_message("Token is valid");
        return true;
    case AP_JWT::INVALID_TYP_CLAIM:
        hal.util->snprintf(buffer, buflen, "Invalid token type");
        log_message("Invalid token type");
        return false;
    case AP_JWT::INVALID_ALG_CLAIM:
        hal.util->snprintf(buffer, buflen, "Invalid token algorithm");
        log_message("Invalid token algorithm");
        return false;
    case AP_JWT::INVALID_ISS_CLAIM:
        hal.util->snprintf(buffer, buflen, "Invalid token issuer");
        log_message("Invalid token issuer");
        return false;
    case AP_JWT::INVALID_FORMAT:
        hal.util->snprintf(buffer, buflen, "Invalid token format");
        log_message("Invalid token format");
        return false;
    case AP_JWT::INVALID_IAT_CLAIM:
        hal.util->snprintf(buffer, buflen, "Invalid token iat claim");
        log_message("Invalid token iat claim.");
        return false;
    case AP_JWT::INVALID_EXP_CLAIM:
        hal.util->snprintf(buffer, buflen, "Invalid token exp claim");
        log_message("Invalid token exp claim. Token is probably expired");
        return false;
    case AP_JWT::INVALID_NBF_CLAIM:
        hal.util->snprintf(buffer, buflen, "Invalid token nbf claims");
        log_message("Invalid token nbf claim.");
        return false;
    case AP_JWT::INVALID_SIGNATURE:
        hal.util->snprintf(buffer, buflen, "Invalid token signature");
        log_message("Invalid token signature.");
        return false;
    default:
        hal.util->snprintf(buffer, buflen, "Invalid JWT token");
        log_message("Invalid token, unknown error");
        return false;
    }
}

// read file contents
const uint8_t* AP_TrustedFlight::read_from_file(const char *filepath, uint32_t *outsize)
{
    FileData *filedata = AP::FS().load_file(filepath);
    if (filedata == nullptr || filedata->data == nullptr || filedata->length <= 0) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Cannot read file: %s", filepath);
        log_message(msg);
        return nullptr;
    }

    *outsize = filedata->length;
    return filedata->data;
}

// write log message
void AP_TrustedFlight::log_message(const char *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRUSTED_FLIGHT_MSG),
        time_us   : AP_HAL::micros64(),
        msg       : {}
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
