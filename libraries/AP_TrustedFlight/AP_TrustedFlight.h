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

struct trusted_flight_artifacts {
    uint32_t key_type;
    uint32_t key_len;
    uint8_t key[32];
    uint32_t issuer_len;
    uint8_t issuer[AP_TRUSTED_FLIGHT_ISSUER_LENGTH];
};

class AP_TrustedFlight
{
public:
    enum KeyType {
        PUBLIC_KEY_None,
        PUBLIC_KEY_EdDSA
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
    // method to periodically validate trusted flight artifacts
    void validate(void);

    /**
     * write log message
     * @param message message to log
     */
    void log_message(const char *message);

    /**
     * read file contents into a buffer
     * @param filepath reference to path of the file to read
     * @param buffer reference to put the file content
     * @param size reference to put size of the file content
     * @returns true on success, false otherwise
     */
    bool read_from_file(const char *filepath, uint8_t *&buffer, uint32_t &size);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    const char *public_key_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/key.pub";
    const char *token_issuer_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token_issuer";
#endif

    const char *token_file_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token";

    // flag to determine module initialization
    bool init_done;

    // Semaphore to protect validation results
    HAL_Semaphore _validation_sem;
    // frequency of running trusted flight validate method
    const uint32_t validate_frequency_ms = 1000U;
    // Maximum time (in milliseconds) before cached validation results are considered stale
    const uint32_t max_validation_stale_time_ms = validate_frequency_ms + 50;

    const uint8_t* public_key;
    const uint8_t* token_issuer;
    uint8_t *token;
    uint32_t public_key_length;
    uint32_t token_issuer_length;
    uint32_t token_length;
    AP_JWT _jwt_parser;
    uint32_t _last_validation_time_ms;
    uint8_t _cached_validation_result;

    static AP_TrustedFlight *_singleton;
};

namespace AP
{
AP_TrustedFlight &trusted_flight();
};

#endif // AP_TRUSTED_FLIGHT_ENABLED
