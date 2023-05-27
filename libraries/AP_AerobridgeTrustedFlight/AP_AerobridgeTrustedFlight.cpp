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
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <mbedtls/pem.h>
#include "AP_AerobridgeTrustedFlight.h"
#include "LogStructure.h"

extern const AP_HAL::HAL& hal;

// Constructor
AP_AerobridgeTrustedFlight::AP_AerobridgeTrustedFlight()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many AerobridgeTrustedFlight modules");
        return;
    }

    _singleton = this;
}

// Aerobridge Trusted Flight module init
void AP_AerobridgeTrustedFlight::init()
{
    if (init_done)
        return;

    // Trusted Flight validation happens pre-arm, it's good to enable log_disarmed if the feature is enabled.
    // NOTE: only override .log_disarmed if defaults at unset
    if (AP::logger()._params.log_disarmed == AP_Logger::LogDisarmed::NONE)
        AP::logger()._params.log_disarmed.set(AP_Logger::LogDisarmed::LOG_WHILE_DISARMED); //AP_Logger::LogDisarmed::LOG_WHILE_DISARMED_NOT_USB
    
    mbedtls_x509_crt_init(&trusted_certificate);
    if (!read_certificate_from_file(trusted_certificate_path, &trusted_certificate)) {
        log_message("Cannot read root trusted certificate from ROMFS");
        init_done = false;
        return;
    }

    init_done = true;
}

// entry method to check if trusted flight artifacts are valid or not
bool AP_AerobridgeTrustedFlight::is_trusted(char *buffer, size_t buflen)
{
    if (!init_done) {
        hal.util->snprintf(buffer, buflen, "Initialization is not done yet");
        log_message("Initialization is not done yet");
        return false;
    }

    l8w8jwt_decoding_params_init(&params);
    params.alg = L8W8JWT_ALG_RS256;
    params.validate_exp = 1;
    params.exp_tolerance_seconds = 60;

    if (!read_from_file(token_issuer_path, &(params.validate_iss), &(params.validate_iss_length))) {
        log_message("Cannot read trusted token issuer from ROMFS");
        return false;
    }

    mbedtls_x509_crt certificate;
    mbedtls_x509_crt_init(&certificate);

    if (!validate_certificate_chain(&certificate, params.validate_iss)) {
        hal.util->snprintf(buffer, buflen, "Invalid certificate chain");
        return false;
    }

    if (!validate_token(&certificate)) {
        hal.util->snprintf(buffer, buflen, "Token verification failed");
        return false;
    }

    return true;
}

// validate certificate chain against root certificate from ROMFS
bool AP_AerobridgeTrustedFlight::validate_certificate_chain(mbedtls_x509_crt* certificate_chain, const char *cn)
{
    if (!read_certificate_from_file(certificate_chain_path, certificate_chain) != 0) {
        log_message("Cannot read untrusted certificate chain");
        return false;
    }

    uint32_t flags;
    if (mbedtls_x509_crt_verify(certificate_chain, &trusted_certificate, NULL, cn, &flags, NULL, NULL) != 0) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Verification error. flags: %lu", (unsigned long int)flags);
        log_message(msg);

        log_message("Invalid certificate chain");
        return false;
    }

    log_message("Certificate chain is valid");
    return true;
}

// validate JWT token against leaf certificate from certificate chain
bool AP_AerobridgeTrustedFlight::validate_token(mbedtls_x509_crt* certificate)
{
    if (!read_from_file(token_file_path, &(params.jwt), &(params.jwt_length)) ||
        !write_pem_certificate(certificate, &(params.verification_key), &(params.verification_key_length))) {
        log_message("Token verification failed");
        return false;
    }

    enum l8w8jwt_validation_result validation_result;
    int decode_result = l8w8jwt_decode(&params, &validation_result, NULL, NULL);

    free(params.jwt);
    free(params.validate_iss);
    free(params.verification_key);

    if (decode_result == L8W8JWT_DECODE_FAILED_INVALID_TOKEN_FORMAT) {
        log_message("Invalid token format, verification failed");
        return false;
    }
    if (decode_result == L8W8JWT_KEY_PARSE_FAILURE) {
        log_message("Invalid public key format, verification failed");
        return false;
    }
    if (validation_result == L8W8JWT_ISS_FAILURE) {
        log_message("Invalid issuer, verification failed");
        return false;
    }
    if (validation_result == L8W8JWT_EXP_FAILURE) {
        log_message("Token expired, verification failed");
        return false;
    }
    if (decode_result != L8W8JWT_SUCCESS || validation_result != L8W8JWT_VALID) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Token decode_result: %d, validation_result: %d", decode_result, validation_result);
        log_message(msg);
        
        log_message("Token verification failed");
        return false;
    }

    log_message("Token verification successful");
    return true;
}

// write X509 certificate to PEM buffer
bool AP_AerobridgeTrustedFlight::write_pem_certificate(mbedtls_x509_crt* certificate, unsigned char **outbuf, size_t *outsize)
{
    size_t size;
    mbedtls_pem_write_buffer(public_key_header, public_key_footer, certificate->pk_raw.p, certificate->pk_raw.len, NULL, 0, &size);

    *outbuf = (unsigned char *)malloc(size);
    if (*outbuf == nullptr) {
        log_message("Cannot allocate buffer for verification key");
        return false;
    }

    if (mbedtls_pem_write_buffer(public_key_header, public_key_footer, certificate->pk_raw.p, certificate->pk_raw.len,
                                 *outbuf, size, outsize) != 0) {
        log_message("Cannot write public key pem to verification key");
        return false;
    }

    return true;
}

// read certificate from FS
bool AP_AerobridgeTrustedFlight::read_certificate_from_file(const char *filepath, mbedtls_x509_crt* certificate)
{
    char *certificate_data;
    size_t certificate_size;

    if (!read_from_file(filepath, &certificate_data, &certificate_size)) {
        return false;
    }

    int parse_res = mbedtls_x509_crt_parse(certificate, (unsigned char *)(certificate_data),
                                           certificate_size + 1); // +1 to include NULL byte at the end

    free(certificate_data);
    if (parse_res != 0) {
        log_message("Certificate parsing failed");
        return false;
    }
    return true;
}

// read file contents
bool AP_AerobridgeTrustedFlight::read_from_file(const char *filepath, char **outbuf, size_t *outsize)
{
    FileData *filedata = AP::FS().load_file(filepath);
    if (filedata->data == nullptr || filedata->length <= 0) {
        char msg[50];
        hal.util->snprintf(msg, sizeof(msg), "Cannot read file: %s", filepath);
        log_message(msg);
        return false;
    }

    *outbuf = (char *)malloc(filedata->length + 1);
    if (*outbuf == nullptr) {
        log_message("Cannot allocate buffer for token");
        return false;
    }

    memcpy(*outbuf, filedata->data, filedata->length);
    (*outbuf)[filedata->length]='\0';
    *outsize = filedata->length;

    delete filedata;
    return true;
}

// write log message
void AP_AerobridgeTrustedFlight::log_message(const char *message)
{
    struct log_Message pkt{
        LOG_PACKET_HEADER_INIT(LOG_TRUSTED_FLIGHT_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };

    strncpy_noterm(pkt.msg, message, sizeof(pkt.msg));
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

AP_AerobridgeTrustedFlight *AP_AerobridgeTrustedFlight::_singleton;

namespace AP
{
AP_AerobridgeTrustedFlight &aerobridge_trusted_flight()
{
    return *AP_AerobridgeTrustedFlight::get_singleton();
}
};
