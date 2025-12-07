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
#include <AP_CheckFirmware/AP_CheckFirmware.h>

/*
  app descriptor for public key and token issuer
 */
extern const app_descriptor_t app_descriptor;

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
     * entry method to check if trusted flight artifacts are valid or not
     * @param buffer output message buffer
     * @param buflen output message buffer length
     * @returns true if artifacts are valid, false otherwise
     */
    bool pre_arm_check(char *buffer, size_t buflen);

private:
    /**
     * write log message
     * @param message message to log
     */
    void log_message(const char *message);

    /**
     * read file contents
     * @param filename reference to path of the file to read
     * @param outsize reference to put size of the file content
     * @returns pointer to buffer of contents read from file
     */
    const uint8_t *read_from_file(const char *filename, uint32_t *outsize);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const char *public_key_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/key.pub";
    const char *token_issuer_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token_issuer";
#endif

    const char *token_file_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token";

    // flag to determine module initialization
    bool init_done = false;

    const uint8_t* public_key;
    const uint8_t* token_issuer;
    uint32_t public_key_length;
    uint32_t token_issuer_length;

    static AP_TrustedFlight *_singleton;
};

namespace AP
{
AP_TrustedFlight &trusted_flight();
};

#endif // AP_TRUSTED_FLIGHT_ENABLED
