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

#if AP_JWT_ENABLED

#include <tiny-json.h>

class AP_JWT
{
public:
    enum TokenValidationResult {
        INVALID_FORMAT,
        INVALID_SIGNATURE,
        INVALID_TYP_CLAIM,
        INVALID_ALG_CLAIM,
        INVALID_ISS_CLAIM,
        INVALID_IAT_CLAIM,
        INVALID_NBF_CLAIM,
        INVALID_EXP_CLAIM,
        RTC_NOT_AVAILABLE,
        TOKEN_VALID,
        JWT_LAST
    };

    AP_JWT();
    ~AP_JWT();

    // function to validate JWT token
    TokenValidationResult validate(const uint8_t *token, uint32_t token_length, const uint8_t *public_key, const uint8_t *issuer, uint32_t issuer_length);

    // function to get string representation of validation result
    static const char* validation_result_to_string(TokenValidationResult result);

private:

    struct JsonSection {
        uint8_t *decoded_buf;
        json_t *json_pool;
        const json_t *root;
    };

    enum claim_status {
        missing,
        valid,
        invalid
    };

    // clear attributes previously parsed `token`
    void clear_last_parse();

    // parse `new_token`, if it's different from previously cached `token`
    bool refresh_last_parse(const uint8_t *new_token, uint32_t new_token_length);

    // parse JWT section to JSON
    const uint8_t *parse_json(const uint8_t *section_begin, JsonSection &section);

    // decode JWT section
    const uint8_t *decode_section(const uint8_t *section_begin, uint8_t *&out, uint16_t &out_len, bool is_sig);

    // validate TYP claim
    bool validate_type();

    // validate ALG claim
    bool validate_algorithm();

    // validate ISS claim
    bool validate_issuer(const uint8_t *issuer, uint32_t issuer_length);

    // validate IAT claim
    bool validate_issued_at(uint32_t now_seconds);

    // validate EXP claim
    bool validate_expiration(uint32_t now_seconds);

    // validate NBF claim
    bool validate_not_before(uint32_t now_seconds);

    // get jwt claim with `name`
    claim_status get_claim(jsonType_t type, const JsonSection &section, const char *name, const json_t *&prop) const;

    uint8_t *token;
    uint32_t token_length;
    uint32_t token_length_without_sign;

    JsonSection header_section;
    JsonSection payload_section;
    uint8_t *decoded_signature;
    uint16_t decoded_signature_len;
};

#endif // AP_JWT_ENABLED
