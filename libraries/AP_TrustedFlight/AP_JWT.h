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

#include<AP_JSON/AP_JSON.h>

uint8_t* base64_decode(const uint8_t *src, uint16_t len, uint16_t *out_len);
uint8_t* base64url_decode(const uint8_t *src, uint16_t len, uint16_t *out_len);

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
        TOKEN_VALID
    };

    // constructor
    AP_JWT(const uint8_t *_token, uint32_t _size) : token(_token), token_length(_size) {}

    // function to validate JWT token
    TokenValidationResult validate(const uint8_t *public_key, const uint8_t *issuer, uint32_t issuer_length);

private:

    // parse JWT section to JSON
    const uint8_t *parse_json(const uint8_t *section_begin, AP_JSON::value &out);

    // decode JWT section
    const uint8_t *decode_section(const uint8_t *section_begin, const uint8_t **out, uint16_t *out_len, bool is_sig);

    // entry method for parsing JWT token
    bool parse();

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

    const uint8_t *token;
    uint32_t token_length;
    uint32_t token_length_without_sign;

    AP_JSON::value header_json;
    AP_JSON::value payload_json;
    const uint8_t *decoded_signature;
    uint16_t decoded_signature_len;
};

#endif // AP_JWT_ENABLED
