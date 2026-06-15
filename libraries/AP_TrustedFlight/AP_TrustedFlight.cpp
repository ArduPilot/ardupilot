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

#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Logger/AP_Logger.h>
#if AP_SIM_ENABLED
#include <SITL/SITL.h>
#endif
#include "AP_TrustedFlight.h"

extern const AP_HAL::HAL& hal;

/*
  app descriptor for public key and token issuer
 */
extern const app_descriptor_t app_descriptor;

AP_TrustedFlight::AP_TrustedFlight() :
    init_done(false),
    public_key(nullptr),
    token_issuer(nullptr),
    token(nullptr),
    public_key_length(0),
    token_issuer_length(0),
    token_length(0),
    _last_validation_time_ms(0),
    _cached_validation_result(AP_JWT::INVALID_FORMAT)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many TrustedFlight modules");
        return;
    }

    _singleton = this;
}

AP_TrustedFlight::~AP_TrustedFlight()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    delete[] public_key;
    delete[] token_issuer;
#endif
    delete[] token;
    public_key = token_issuer = token = nullptr;
    token_length = 0;
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
        AP::logger()._params.log_disarmed.set_and_save(AP_Logger::LogDisarmed::LOG_WHILE_DISARMED);    //AP_Logger::LogDisarmed::LOG_WHILE_DISARMED_NOT_USB
    }

    // read public key + token
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    uint8_t *public_key_buffer = const_cast<uint8_t *>(public_key);
    if (!read_from_file(public_key_path, public_key_buffer, public_key_length)) {
        log_message("Failed to read public key");
        return;
    }
    public_key = public_key_buffer;

    uint8_t *token_issuer_buffer = const_cast<uint8_t *>(token_issuer);
    if (!read_from_file(token_issuer_path, token_issuer_buffer, token_issuer_length)) {
        log_message("Failed to read token issuer");
        return;
    }
    token_issuer = token_issuer_buffer;
#else
    if (app_descriptor.auth_params.key_type != AP_TrustedFlight::PUBLIC_KEY_EdDSA) {
        log_message("Unsupported Public Key Type in app descriptor");
        return;
    }
    public_key_length = app_descriptor.auth_params.key_len;
    public_key = app_descriptor.auth_params.key;
    token_issuer_length = app_descriptor.auth_params.issuer_len;
    token_issuer = app_descriptor.auth_params.issuer;
#endif

    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_TrustedFlight::validate, void));

    init_done = true;
}

// scheduled method to periodically validate trusted flight artifacts
void AP_TrustedFlight::validate()
{
    {
        WITH_SEMAPHORE(_validation_sem);
        if (!AP_HAL::timeout_expired(_last_validation_time_ms, AP_HAL::millis(), validate_frequency_ms)) {
            return;
        }
    }

#if AP_SIM_ENABLED
    if (AP::sitl()->trusted_flight_validate_pause > 0) {
        return;
    }
#endif

    if (!read_from_file(token_file_path, token, token_length)) {
        log_message("Unable to read token from file system");

        WITH_SEMAPHORE(_validation_sem);
        // reset last validation time to invalidate
        _last_validation_time_ms = 0;
        return;
    }

    AP_JWT::TokenValidationResult result = _jwt_parser.validate(token, token_length, public_key, token_issuer, token_issuer_length);

    log_message(AP_JWT::validation_result_to_string(result));

    WITH_SEMAPHORE(_validation_sem);
    _cached_validation_result = result;
    _last_validation_time_ms = AP_HAL::millis();
}

// Perform the pre-arm checks and prevent arming if they are not satisifed
bool AP_TrustedFlight::pre_arm_check(char *buffer, size_t buflen)
{
    WITH_SEMAPHORE(_validation_sem);

    if (_last_validation_time_ms == 0) {
        hal.util->snprintf(buffer, buflen, "Token validation pending");
        log_message(buffer);
        return false;
    }

    uint32_t cached_validation_age_ms = AP_HAL::millis() - _last_validation_time_ms;
    if (cached_validation_age_ms > max_validation_stale_time_ms) {
        hal.util->snprintf(buffer, buflen, "Validation result is stale (%ld ms)", (unsigned long)cached_validation_age_ms);
        log_message(buffer);
        return false;
    }

    hal.util->snprintf(buffer, buflen, "%s", AP_JWT::validation_result_to_string((AP_JWT::TokenValidationResult)_cached_validation_result));

    return _cached_validation_result == AP_JWT::TOKEN_VALID;
}

// read file contents into a buffer
bool AP_TrustedFlight::read_from_file(const char *filepath, uint8_t *&buffer, uint32_t &size)
{
    struct stat file_stat;
    if (AP::FS().stat(filepath, &file_stat) != 0 || file_stat.st_size == 0) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "File not found or empty: %s", filepath);
        log_message(msg);
        return false;
    }

    const uint32_t file_size = file_stat.st_size;
    if (buffer == nullptr || size != file_size) {
        uint8_t *new_buffer = NEW_NOTHROW uint8_t[file_size + 1U];
        if (new_buffer == nullptr) {
            log_message("Failed to allocate memory for data");
            return false;
        }
        delete[] buffer;
        buffer = new_buffer;
    }

    const int fd = AP::FS().open(filepath, O_RDONLY);
    if (fd == -1) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Cannot open file: %s", filepath);
        log_message(msg);
        return false;
    }

    const int32_t ret = AP::FS().read(fd, buffer, file_size);
    AP::FS().close(fd);
    if (ret != int32_t(file_size)) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Cannot read file: %s", filepath);
        log_message(msg);
        return false;
    }

    buffer[file_size] = '\0';
    size = file_size;
    return true;
}

// write log message
void AP_TrustedFlight::log_message(const char *message)
{
    AP::logger().Write_MessageF("[TFL] %s", message);
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
