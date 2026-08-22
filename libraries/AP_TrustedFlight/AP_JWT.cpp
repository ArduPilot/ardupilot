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

#if AP_JWT_ENABLED

#include <AP_CheckFirmware/monocypher.h>
#include <AP_Common/base64.h>
#include <AP_RTC/AP_RTC.h>
#include "AP_JWT.h"

static const char * const jwt_validation_result_strings[] = {
    "Invalid token format",                              // INVALID_FORMAT
    "Invalid token signature",                           // INVALID_SIGNATURE
    "Invalid token type",                                // INVALID_TYP_CLAIM
    "Invalid token algorithm",                           // INVALID_ALG_CLAIM
    "Invalid token issuer",                              // INVALID_ISS_CLAIM
    "Invalid token iat claim",                           // INVALID_IAT_CLAIM
    "Invalid token nbf claim",                           // INVALID_NBF_CLAIM
    "Invalid token exp claim (token expired)",           // INVALID_EXP_CLAIM
    "RTC not available for validation",                  // RTC_NOT_AVAILABLE
    "Token is valid"                                     // TOKEN_VALID
};

static_assert(ARRAY_SIZE(jwt_validation_result_strings) == AP_JWT::TokenValidationResult::JWT_LAST,
              "Mismatched JWT validation results and descriptions");

// function to get string representation of validation result
const char* AP_JWT::validation_result_to_string(TokenValidationResult result)
{
    uint8_t index = (uint8_t)result;
    if (index < ARRAY_SIZE(jwt_validation_result_strings)) {
        return jwt_validation_result_strings[index];
    }
    return "Unknown token validation result";
}

AP_JWT::AP_JWT() :
    token(nullptr),
    token_length(0),
    token_length_without_sign(0),
    header_section{},
    payload_section{},
    decoded_signature(nullptr),
    decoded_signature_len(0) { }

AP_JWT::~AP_JWT()
{
    clear_last_parse();
    delete[] token;
    token = nullptr;
}

// function to validate JWT token
AP_JWT::TokenValidationResult AP_JWT::validate(const uint8_t *new_token, uint32_t new_token_length, const uint8_t *public_key, const uint8_t *issuer, uint32_t issuer_length)
{
    // parse jwt token
    if (!refresh_last_parse(new_token, new_token_length)) {
        return AP_JWT::INVALID_FORMAT;
    }

    if (!validate_type()) {
        return AP_JWT::INVALID_TYP_CLAIM;
    }
    if (!validate_algorithm()) {
        return AP_JWT::INVALID_ALG_CLAIM;
    }

    // validate signature
    if (crypto_check(decoded_signature, public_key, token, token_length_without_sign) != 0) {
        return AP_JWT::INVALID_SIGNATURE;
    }

    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        return AP_JWT::RTC_NOT_AVAILABLE;
    }
    uint32_t utc_sec = utc_usec/1000000U;

    // validate claims
    if (!validate_issued_at(utc_sec)) {
        return AP_JWT::INVALID_IAT_CLAIM;
    }
    if (!validate_not_before(utc_sec)) {
        return AP_JWT::INVALID_NBF_CLAIM;
    }
    if (!validate_expiration(utc_sec)) {
        return AP_JWT::INVALID_EXP_CLAIM;
    }
    if (!validate_issuer(issuer, issuer_length)) {
        return AP_JWT::INVALID_ISS_CLAIM;
    }

    return AP_JWT::TOKEN_VALID;
}

// validate TYP claim
bool AP_JWT::validate_type()
{
    const json_t *prop = nullptr;
    claim_status status = get_claim(JSON_TEXT, header_section, "typ", prop);
    return status == claim_status::valid ? strncmp(json_getValue(prop), "JWT", 3) == 0 : false;
}

// validate ALG claim
bool AP_JWT::validate_algorithm()
{
    const json_t *prop = nullptr;
    claim_status status = get_claim(JSON_TEXT, header_section, "alg", prop);
    return status == claim_status::valid ? strncmp(json_getValue(prop), "EdDSA", 5) == 0 : false;
}

// validate ISS claim
bool AP_JWT::validate_issuer(const uint8_t *issuer, uint32_t issuer_length)
{
    const json_t *prop = nullptr;
    claim_status status = get_claim(JSON_TEXT, payload_section, "iss", prop);
    return status == claim_status::valid ? strncmp(json_getValue(prop), reinterpret_cast<const char *>(issuer), issuer_length) == 0 : false;
}

// validate IAT claim
bool AP_JWT::validate_issued_at(uint32_t now_seconds)
{
    const json_t *prop = nullptr;
    claim_status status = get_claim(JSON_INTEGER, payload_section, "iat", prop);
    return status == claim_status::valid ? now_seconds > json_getInteger(prop) : false;
}

// validate EXP claim
bool AP_JWT::validate_expiration(uint32_t now_seconds)
{
    const json_t *prop = nullptr;
    claim_status status = get_claim(JSON_INTEGER, payload_section, "exp", prop);
    return status == claim_status::valid ? now_seconds < json_getInteger(prop) : false;
}

// validate NBF claim
bool AP_JWT::validate_not_before(uint32_t now_seconds)
{
    const json_t *prop = nullptr;
    claim_status status = get_claim(JSON_INTEGER, payload_section, "nbf", prop);
    // missing nbf claim is also valid
    return status == claim_status::valid ? now_seconds > json_getInteger(prop) : status == claim_status::missing;
}

// clear attributes previously parsed `token`
void AP_JWT::clear_last_parse()
{
    delete[] header_section.decoded_buf;
    delete[] header_section.json_pool;
    delete[] payload_section.decoded_buf;
    delete[] payload_section.json_pool;
    delete[] decoded_signature;

    header_section = {};
    payload_section = {};
    decoded_signature = nullptr;
    token_length_without_sign = 0;
    decoded_signature_len = 0;
}

// parse `new_token`, if it's different from previously cached `token`
bool AP_JWT::refresh_last_parse(const uint8_t *new_token, uint32_t new_token_length)
{
    if (new_token_length == 0) {
        return false;
    }

    if (token != nullptr && token_length == new_token_length && memcmp(token, new_token, new_token_length) == 0) {
        // token unchanged, use previously parsed attributes
        return header_section.root != nullptr && payload_section.root != nullptr && decoded_signature != nullptr;
    }

    if (token_length != new_token_length) {
        uint8_t *new_token_buffer = NEW_NOTHROW uint8_t[new_token_length];
        if (new_token_buffer == nullptr) {
            return false;
        }
        delete[] token;
        token = new_token_buffer;
    }

    memcpy(token, new_token, new_token_length);
    token_length = new_token_length;

    clear_last_parse();

    // parse the new token and store parsing results
    const uint8_t *next;
    if ((next = parse_json(token, header_section)) == nullptr) {
        return false;
    }
    if ((next = parse_json(next + 1, payload_section)) == nullptr) {
        return false;
    }

    // next marks the end of payload here
    token_length_without_sign = next - token;

    next = decode_section(next + 1, decoded_signature, decoded_signature_len, true);
    if (next != nullptr || decoded_signature == nullptr) {
        return false;
    }

    return true;
}

// parse JWT section to JSON
const uint8_t *AP_JWT::parse_json(const uint8_t *section_begin, JsonSection &section)
{
    uint16_t current_len;

    const uint8_t *next = decode_section(section_begin, section.decoded_buf, current_len, false);
    if (next == nullptr || section.decoded_buf == nullptr) {
        return nullptr;
    }

    uint16_t pool_size = 0;
    for (uint16_t i = 0; i < current_len; ++i) {
        if (section.decoded_buf[i] == '{' || section.decoded_buf[i] == '[' || section.decoded_buf[i] == ':') {
            pool_size++;
        }
    }

    section.json_pool = NEW_NOTHROW json_t[pool_size];
    if (section.json_pool == nullptr) {
        return nullptr;
    }

    section.root = json_create((char *)section.decoded_buf, section.json_pool, pool_size);
    return section.root != nullptr ? next : nullptr;
}

// decode JWT section
const uint8_t *AP_JWT::decode_section(const uint8_t *section_begin, uint8_t *&out, uint16_t &out_len, bool is_sig)
{
    const uint8_t *current = section_begin;
    const uint8_t *next = (uint8_t *)memchr(current, '.', token + token_length - current);

    // not signature section but does not have another delimiter '.'
    if (!is_sig && next == nullptr) {
        return nullptr;
    }

    // signature section but has another delimiter '.'
    if (is_sig && next != nullptr) {
        return nullptr;
    }

    uint16_t section_len = (next == nullptr ? token + token_length : next) - section_begin;
    out = base64url_decode(current, section_len, &out_len);

    return next;
}

// get jwt claim with `name`
AP_JWT::claim_status AP_JWT::get_claim(jsonType_t type, const JsonSection &section, const char *name, const json_t *&prop) const
{
    prop = (section.root == nullptr) ? nullptr : json_getProperty(section.root, name);
    if (prop == nullptr || json_getType(prop) == JSON_NULL) {
        return claim_status::missing;
    }
    return json_getType(prop) == type ? claim_status::valid : claim_status::invalid;
}

#endif // AP_JWT_ENABLED
