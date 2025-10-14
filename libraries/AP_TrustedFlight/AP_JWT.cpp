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
#include <AP_RTC/AP_RTC.h>
#include "AP_JWT.h"

// function to validate JWT token
AP_JWT::TokenValidationResult AP_JWT::validate(const uint8_t *public_key, const uint8_t *issuer, uint32_t issuer_length)
{
    // parse jwt token
    if (!parse()) {
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
    AP_JSON::value val = header_json.get("typ");
    if (val.is<AP_JSON::null>() || !val.is<std::string>()) {
        return false;
    }
    return strncmp(val.get<std::string>().c_str(), "JWT", 3) == 0;
}

// validate ALG claim
bool AP_JWT::validate_algorithm()
{
    AP_JSON::value val = header_json.get("alg");
    if (val.is<AP_JSON::null>() || !val.is<std::string>()) {
        return false;
    }
    return strncmp(val.get<std::string>().c_str(), "EdDSA", 5) == 0;
}

// validate ISS claim
bool AP_JWT::validate_issuer(const uint8_t *issuer, uint32_t issuer_length)
{
    AP_JSON::value val = payload_json.get("iss");
    if (val.is<AP_JSON::null>() || !val.is<std::string>()) {
        return false;
    }
    return strncmp(val.get<std::string>().c_str(), (char *)issuer, issuer_length) == 0;
}

// validate IAT claim
bool AP_JWT::validate_issued_at(uint32_t now_seconds)
{
    AP_JSON::value val = payload_json.get("iat");
    if (val.is<AP_JSON::null>() || !val.is<double>()) {
        return false;
    }
    return now_seconds > val.get<double>();
}

// validate EXP claim
bool AP_JWT::validate_expiration(uint32_t now_seconds)
{
    AP_JSON::value val = payload_json.get("exp");
    if (val.is<AP_JSON::null>() || !val.is<double>()) {
        return false;
    }
    return now_seconds < val.get<double>();
}

// validate NBF claim
bool AP_JWT::validate_not_before(uint32_t now_seconds)
{
    AP_JSON::value val = payload_json.get("nbf");
    // claim not available, ignore
    if (val.is<AP_JSON::null>()) {
        return true;
    }
    // invalid nbf type
    else if (!val.is<double>()) {
        return false;
    }
    return now_seconds > val.get<double>();
}

// entry method for parsing JWT token
bool AP_JWT::parse()
{
    const uint8_t *next;
    if ((next = parse_json(token, header_json)) == nullptr) {
        return false;
    }
    if ((next = parse_json(next + 1, payload_json)) == nullptr) {
        return false;
    }

    // next marks the end of payload here
    token_length_without_sign = next - token;

    next = decode_section(next + 1, &decoded_signature, &decoded_signature_len, true);
    if (next != nullptr || decoded_signature == nullptr) {
        return false;
    }

    return true;
}

// parse JWT section to JSON
const uint8_t *AP_JWT::parse_json(const uint8_t *section_begin, AP_JSON::value &out)
{
    const uint8_t *current;
    uint16_t current_len;

    const uint8_t *next = decode_section(section_begin, &current, &current_len, false);
    if (next == nullptr || current == nullptr) {
        return nullptr;
    }

    std::string err = AP_JSON::parse(out, (char *)current);
    if (!err.empty()) {
        return nullptr;
    }
    return next;
}

// decode JWT section
const uint8_t *AP_JWT::decode_section(const uint8_t *section_begin, const uint8_t **out, uint16_t *out_len, bool is_sig)
{
    const uint8_t *current = section_begin;
    const uint8_t *next = (uint8_t *)strchr((char *)current, '.');

    // not signature section but does not have another delimiter '.'
    if (!is_sig && next == nullptr) {
        return nullptr;
    }

    // signature section but has another delimiter '.'
    if (is_sig && next != nullptr) {
        return nullptr;
    }

    uint16_t section_len = (next == nullptr ? token + token_length : next) - section_begin;
    *out = base64url_decode(current, section_len, out_len);

    return next;
}

#endif // AP_JWT_ENABLED
