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
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include <mbedtls/pem.h>
#include "AP_AerobridgeTrustedFlight.h"

extern const AP_HAL::HAL& hal;

AP_AerobridgeTrustedFlight::AP_AerobridgeTrustedFlight()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many AerobridgeTrustedFlight modules");
        return;
    }

    _singleton = this;
}

bool AP_AerobridgeTrustedFlight::is_trusted()
{
    l8w8jwt_decoding_params_init(&params);
    params.alg = L8W8JWT_ALG_RS256;
    params.validate_exp = 1;
    params.exp_tolerance_seconds = 60;

    mbedtls_x509_crt trusted_certificate;
    mbedtls_x509_crt certificate;

    mbedtls_x509_crt_init(&trusted_certificate);
    mbedtls_x509_crt_init(&certificate);

    if (!read_from_file(token_issuer_path, &(params.validate_iss), &(params.validate_iss_length))) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Cannot read trusted token issuer", tag);
        return false;
    }

    if (!read_certificate_from_file(trusted_certificate_path, &trusted_certificate)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Cannot read trusted certificate", tag);
        return false;
    }

    return validate_certificate_chain(&trusted_certificate, &certificate, params.validate_iss) && validate_token(&certificate);
}

bool AP_AerobridgeTrustedFlight::validate_certificate_chain(mbedtls_x509_crt* trusted_certificate,
        mbedtls_x509_crt* certificate_chain, const char *cn)
{
    if (!read_certificate_from_file(certificate_chain_path, certificate_chain) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Cannot read untrusted certificate chain", tag);
        return false;
    }

    uint32_t flags;
    if (mbedtls_x509_crt_verify(certificate_chain, trusted_certificate, NULL, cn, &flags, NULL, NULL) != 0) {
        char verify_info[512];
        mbedtls_x509_crt_verify_info(verify_info, sizeof(verify_info), "", flags );
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Invalid certificate chain", tag);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Verification error: %s", tag, verify_info);
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s: Certificate chain is valid", tag);
    return true;
}

bool AP_AerobridgeTrustedFlight::validate_token(mbedtls_x509_crt* certificate)
{
    if (!read_from_file(token_file_path, &(params.jwt), &(params.jwt_length)) ||
        !write_pem_certificate(certificate, &(params.verification_key), &(params.verification_key_length))) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Token verification failed", tag);
        return false;
    }

    enum l8w8jwt_validation_result validation_result;
    int decode_result = l8w8jwt_decode(&params, &validation_result, NULL, NULL);

    free(params.jwt);
    free(params.validate_iss);
    free(params.verification_key);

    if (decode_result == L8W8JWT_DECODE_FAILED_INVALID_TOKEN_FORMAT) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Invalid token format, verification failed", tag);
        return false;
    }
    if (decode_result == L8W8JWT_KEY_PARSE_FAILURE) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Invalid public key format, verification failed", tag);
        return false;
    }
    if (validation_result == L8W8JWT_ISS_FAILURE) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Invalid issuer, verification failed", tag);
        return false;
    }
    if (validation_result == L8W8JWT_EXP_FAILURE) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Token expired, verification failed", tag);
        return false;
    }
    if (decode_result != L8W8JWT_SUCCESS || validation_result != L8W8JWT_VALID) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s: decode_result: %d, validation_result: %d", tag, decode_result, validation_result);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Token verification failed", tag);
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s: Token verification successful", tag);
    return true;
}

bool AP_AerobridgeTrustedFlight::write_pem_certificate(mbedtls_x509_crt* certificate, unsigned char **outbuf, size_t *outsize)
{
    size_t size;
    mbedtls_pem_write_buffer(public_key_header, public_key_footer, certificate->pk_raw.p, certificate->pk_raw.len, NULL, 0, &size);

    *outbuf = (unsigned char *)malloc(size);
    if (*outbuf == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s: Cannot allocate buffer for verification key", tag);
        return false;
    }

    if (mbedtls_pem_write_buffer(public_key_header, public_key_footer, certificate->pk_raw.p, certificate->pk_raw.len,
                                 *outbuf, size, outsize) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s: Cannot write public key pem to verification key", tag);
        return false;
    }

    return true;
}

bool AP_AerobridgeTrustedFlight::read_certificate_from_file(const char *filepath, mbedtls_x509_crt* certificate)
{
    char *certificate_data;
    size_t certificate_size;

    if (!read_from_file(filepath, &certificate_data, &certificate_size)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Cannot read file %s", tag, filepath);
        return false;
    }

    int parse_res = mbedtls_x509_crt_parse(certificate, (unsigned char *)(certificate_data),
                                           certificate_size + 1); // +1 to include NULL byte at the end

    free(certificate_data);
    if (parse_res != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s: Certificate parsing failed", tag);
        return false;
    }
    return true;
}

bool AP_AerobridgeTrustedFlight::read_from_file(const char *filepath, char **outbuf, size_t *outsize)
{
    FileData *filedata = AP::FS().load_file(filepath);
    if (filedata->data == nullptr || filedata->length <= 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s: Cannot read file %s", tag, filepath);
        return false;
    }

    *outbuf = (char *)malloc(filedata->length + 1);
    if (*outbuf == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s: Cannot allocate buffer for token", tag);
        return false;
    }

    memcpy(*outbuf, filedata->data, filedata->length);
    (*outbuf)[filedata->length]='\0';
    *outsize = filedata->length;

    delete filedata;
    return true;
}

AP_AerobridgeTrustedFlight *AP_AerobridgeTrustedFlight::_singleton;

namespace AP
{
AP_AerobridgeTrustedFlight &aerobridge_trusted_flight()
{
    return *AP_AerobridgeTrustedFlight::get_singleton();
}
};
