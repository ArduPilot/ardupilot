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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <string.h>
#endif

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include <AP_Logger/AP_Logger.h>
#if AP_SIM_ENABLED
#include <SITL/SITL.h>
#endif
#include "AP_TrustedFlight.h"

extern const AP_HAL::HAL& hal;

AP_TrustedFlight::AP_TrustedFlight() :
    init_done(false),
    public_key(nullptr),
    token_issuer(nullptr),
    token(nullptr),
    token_issuer_length(0),
    token_length(0),
    _last_result_ms(0),
    _cached_validation_result(AP_JWT::INVALID_FORMAT),
    _last_logged_result(LogState::NONE)
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
    _singleton = nullptr;
}

// Aerobridge Trusted Flight module init
void AP_TrustedFlight::init()
{
    if (init_done) {
        return;
    }

    // validation runs while disarmed; force disarmed logging
    AP::logger().set_force_log_disarmed(true);

    // read public key + issuer
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    uint8_t *public_key_buffer = nullptr;
    uint32_t public_key_length = 0;
    if (!read_from_file(public_key_path, public_key_buffer, public_key_length)) {
        log_message("Failed to read public key");
        delete[] public_key_buffer;
        return;
    }

    if (public_key_length != AP_JWT::ed25519_public_key_length) {
        log_message("Public key has unexpected size");
        delete[] public_key_buffer;
        return;
    }
    public_key = public_key_buffer;

    uint8_t *token_issuer_buffer = nullptr;
    if (!read_from_file(token_issuer_path, token_issuer_buffer, token_issuer_length)) {
        log_message("Failed to read token issuer");
        delete[] token_issuer_buffer;
        return;
    }

    token_issuer_length = strcspn((const char *)token_issuer_buffer, "\r\n");
    if (token_issuer_length == 0) {
        log_message("Token issuer is empty");
        delete[] token_issuer_buffer;
        return;
    }
    token_issuer = token_issuer_buffer;
#else
    const trusted_flight_artifacts *artifacts = read_from_romfs();
    if (artifacts == nullptr) {
        // reason already logged; leave init_done false so arming is refused
        return;
    }
    if (artifacts->key_type != AP_TrustedFlight::PUBLIC_KEY_EdDSA_Blake2b) {
        log_message("Unsupported Public Key Type in trusted flight artifacts");
        AP_ROMFS::free(reinterpret_cast<const uint8_t *>(artifacts));
        return;
    }
    public_key = artifacts->key;
    token_issuer = artifacts->issuer;
    token_issuer_length = sizeof(artifacts->issuer);
#endif

    // separate thread: ed25519 verify needs more stack than the shared IO thread has
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_TrustedFlight::validate, void),
                                      "trusted_flight", validate_thread_stack_size,
                                      AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        log_message("Failed to start validation thread");
        return;
    }

    init_done = true;
}

// dedicated thread that periodically validates trusted flight artifacts
void AP_TrustedFlight::validate()
{
    while (true) {
        hal.scheduler->delay(validate_frequency_ms);

        // do not validate if already armed
        if (hal.util->get_soft_armed()) {
            continue;
        }

#if AP_SIM_ENABLED
        const SITL::SIM *sitl = AP::sitl();
        if (sitl != nullptr && sitl->trusted_flight_validate_pause > 0) {
            continue;
        }
#endif

        const uint32_t now_ms = AP_HAL::millis();

        if (!read_from_file(token_file_path, token, token_length)) {
            WITH_SEMAPHORE(_validation_sem);
            // no result to report until the token can be read again
            _last_result_ms = 0;
            continue;
        }

        AP_JWT::TokenValidationResult result = _jwt_parser.validate(token, token_length, public_key, token_issuer, token_issuer_length);

        log_message(AP_JWT::validation_result_to_string(result), (LogState)result);

        WITH_SEMAPHORE(_validation_sem);
        _cached_validation_result = result;
        _last_result_ms = now_ms;
    }
}

// Perform the pre-arm checks and prevent arming if they are not satisifed
bool AP_TrustedFlight::pre_arm_check(char *buffer, size_t buflen)
{
    if (!init_done) {
        hal.util->snprintf(buffer, buflen, "Not initialised");
        return false;
    }

    WITH_SEMAPHORE(_validation_sem);

    if (_last_result_ms == 0) {
        hal.util->snprintf(buffer, buflen, "Token validation pending");
        return false;
    }

    const uint32_t cached_validation_age_ms = AP_HAL::millis() - _last_result_ms;
    if (cached_validation_age_ms > max_validation_stale_time_ms) {
        hal.util->snprintf(buffer, buflen, "Validation result is stale (%lu ms)", (unsigned long)cached_validation_age_ms);
        log_message(buffer);
        return false;
    }

    hal.util->snprintf(buffer, buflen, "%s", AP_JWT::validation_result_to_string((AP_JWT::TokenValidationResult)_cached_validation_result));

    return _cached_validation_result == AP_JWT::TOKEN_VALID;
}

#if !(CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
// read contents from ROMFS
const trusted_flight_artifacts *AP_TrustedFlight::read_from_romfs()
{
    uint32_t length = 0;
    const uint8_t *data = AP_ROMFS::find_decompress(artifacts_romfs_name, length);
    if (data == nullptr) {
        log_message("Trusted flight artifacts not found in ROMFS");
        return nullptr;
    }

    if (length != sizeof(trusted_flight_artifacts)) {
        log_message("Trusted flight artifacts have unexpected size");
        AP_ROMFS::free(data);
        return nullptr;
    }

    return reinterpret_cast<const trusted_flight_artifacts *>(data);
}
#endif

// read file contents into a buffer, NUL terminated
bool AP_TrustedFlight::read_from_file(const char *filepath, uint8_t *&buffer, uint32_t &size)
{
    const uint32_t last_size = size;
    size = 0;

    struct stat file_stat;
    if (AP::FS().stat(filepath, &file_stat) != 0 || file_stat.st_size <= 0) {
        char msg[64];
        hal.util->snprintf(msg, sizeof(msg), "File not found or empty: %s", filepath);
        log_message(msg, LogState::FILE_NOT_FOUND);
        return false;
    }

    if (file_stat.st_size > max_file_size) {
        char msg[64];
        hal.util->snprintf(msg, sizeof(msg), "File too large: %s", filepath);
        log_message(msg, LogState::FILE_TOO_LARGE);
        return false;
    }

    const uint32_t file_size = file_stat.st_size;
    if (buffer == nullptr || last_size != file_size) {
        uint8_t *new_buffer = NEW_NOTHROW uint8_t[file_size + 1U];
        if (new_buffer == nullptr) {
            log_message("Failed to allocate memory for data", LogState::NO_MEMORY);
            return false;
        }
        delete[] buffer;
        buffer = new_buffer;
    }

    const int fd = AP::FS().open(filepath, O_RDONLY);
    if (fd == -1) {
        char msg[64];
        hal.util->snprintf(msg, sizeof(msg), "Cannot open file: %s", filepath);
        log_message(msg, LogState::FILE_OPEN_FAILED);
        return false;
    }

    const int32_t ret = AP::FS().read(fd, buffer, file_size);
    AP::FS().close(fd);
    if (ret != int32_t(file_size)) {
        char msg[64];
        hal.util->snprintf(msg, sizeof(msg), "Cannot read file: %s", filepath);
        log_message(msg, LogState::FILE_READ_FAILED);
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

// write log message, unless `state` is the one that was logged last
void AP_TrustedFlight::log_message(const char *message, LogState state)
{
    if (state == _last_logged_result) {
        return;
    }
    _last_logged_result = state;
    log_message(message);
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
