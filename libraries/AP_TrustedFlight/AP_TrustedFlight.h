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

#pragma once

#include "AP_TrustedFlight_Config.h"

#if AP_TRUSTED_FLIGHT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_JWT.h"

#if !(CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
struct PACKED trusted_flight_artifacts {
    uint32_t key_type;
    uint8_t key[AP_JWT::ed25519_public_key_length];
    uint8_t issuer[AP_TRUSTED_FLIGHT_ISSUER_LENGTH];
};

static_assert(sizeof(trusted_flight_artifacts) == sizeof(uint32_t) + AP_JWT::ed25519_public_key_length + AP_TRUSTED_FLIGHT_ISSUER_LENGTH, "incorrect trusted_flight_artifacts length");
#endif

class AP_TrustedFlight
{
public:
    enum KeyType {
        PUBLIC_KEY_None,
        // curve25519 + Blake2b (monocypher), not standard RFC 8032 Ed25519 (SHA-512)
        PUBLIC_KEY_EdDSA_Blake2b
    };

    // constructor
    AP_TrustedFlight();

    // destructor
    ~AP_TrustedFlight();

    // Aerobridge Trusted Flight module init
    void init();

    // get singleton instance of AP_TrustedFlight
    static AP_TrustedFlight *get_singleton()
    {
        return _singleton;
    }

    /**
     * perform the pre-arm checks and prevent arming if they are not satisifed
     * @param buffer output message buffer
     * @param buflen output message buffer length
     * @returns true if artifacts are valid, false otherwise
     */
    bool pre_arm_check(char *buffer, size_t buflen);

private:
    enum class LogState : uint8_t {
        NONE = AP_JWT::JWT_LAST,
        FILE_NOT_FOUND,
        FILE_TOO_LARGE,
        NO_MEMORY,
        FILE_OPEN_FAILED,
        FILE_READ_FAILED,
    };

    // method to periodically validate trusted flight artifacts
    void validate(void);

    /**
     * write log message
     * @param message message to log
     */
    void log_message(const char *message);

    /**
     * write log message, unless `state` is the one that was logged last
     * @param message message to log
     * @param state outcome being logged
     */
    void log_message(const char *message, LogState state);

    /**
     * read file contents into a buffer, NUL terminated
     * @param filepath path of the file to read
     * @param buffer reference to put the file content; reallocated when the size changes
     * @param size reference to put size of the file content; zero on failure
     * @returns true on success, false otherwise
     */
    bool read_from_file(const char *filepath, uint8_t *&buffer, uint32_t &size);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    const char *public_key_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/key.pub";
    const char *token_issuer_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token_issuer";
#else
    /**
     * read contents from ROMFS
     * @returns reference to artifacts buffer on success, nullptr otherwise
     */
    const trusted_flight_artifacts *read_from_romfs();

    const char *artifacts_romfs_name = "trusted_flight/artifacts";
#endif

    const char *token_file_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token";

    // flag to determine module initialization
    bool init_done;

    // Semaphore to protect validation results
    HAL_Semaphore _validation_sem;
    // largest file read_from_file() accepts
    static constexpr uint32_t max_file_size = 2048U;
    // period between validation passes on the dedicated thread
    static constexpr uint32_t validate_frequency_ms = 1000U;
    // Maximum time (in milliseconds) before cached validation results are considered stale
    static constexpr uint32_t max_validation_stale_time_ms = validate_frequency_ms * 2;
    // validation thread stack size
    static constexpr uint32_t validate_thread_stack_size = 4096U;

    const uint8_t* public_key;
    const uint8_t* token_issuer;
    uint8_t *token;
    uint32_t token_issuer_length;
    uint32_t token_length;
    AP_JWT _jwt_parser;
    uint32_t _last_result_ms;
    uint8_t _cached_validation_result;
    // last outcome logged, to avoid re-logging an unchanged one
    LogState _last_logged_result;

    static AP_TrustedFlight *_singleton;
};

namespace AP
{
AP_TrustedFlight &trusted_flight();
};

#endif // AP_TRUSTED_FLIGHT_ENABLED
