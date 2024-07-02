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

#include <AP_HAL/AP_HAL.h>
#include <mbedtls/x509_crt.h>
#include <l8w8jwt/decode.h>

class AP_AerobridgeTrustedFlight
{
public:
    /**
     * Aerobridge Trusted Flight module init
     */
    void init();
    
    /**
     * get singleton instance of AP_AerobridgeTrustedFlight
     */
    static AP_AerobridgeTrustedFlight *get_singleton()
    {
        return _singleton;
    }

    /**
     *  Constructor
     */
    AP_AerobridgeTrustedFlight();

    /**
     * entry method to check if trusted flight artifacts are valid or not
     * @param buffer output message buffer
     * @param buflen output message buffer length
     * @returns true if artifacts are valid, false otherwise
     */ 
    bool is_trusted(char *buffer, size_t buflen);

private:
    /**
     * write log message
     * @param message message to log
     */
    void log_message(const char *message);

    /**
     * validate JWT token against leaf certificate from certificate chain
     * @param certificate X509 certificate reference to validate token against
     * @returns true if token is valid, false otherwise
     */
    bool validate_token(mbedtls_x509_crt *certificate);

    /**
     * validate certificate chain against root certificate from ROMFS
     * @param certificate_chain reference to list of certificates starting from leaf certificate
     * @param cn Comman Name for the leaf certificate
     * @returns true if certificate chain is valid, false otherwise
     */
    bool validate_certificate_chain(mbedtls_x509_crt* certificate_chain, const char *cn);

    /**
     * read certificate from FS
     * @param filepath reference to path of the file to read
     * @param certificate reference to Certificate type to put parsed certificate 
     * @returns true if certificate was parsed successfully, false otherwise
     */
    bool read_certificate_from_file(const char *filepath, mbedtls_x509_crt* certificate);

    /**
     * read file contents
     * @param filename reference to path of the file to read
     * @param outbuf reference to buffer to put file contents
     * @param outsize reference to put size of the file contentx
     * @returns true if read was successful, false otherwise
     */
    bool read_from_file(const char *filename, char **outbuf, size_t *outsize);

    /**
     * write X509 certificate to PEM buffer
     * @param certificate reference to X509 certificate to read
     * @param outbuf reference to buffer to put PEM contents
     * @param outsize reference to put size of the PEM contentx
     * @returns true if certificate was parsed successfully, false otherwise
     */
    bool write_pem_certificate(mbedtls_x509_crt* certificate, unsigned char **outbuf, size_t *outsize);

    const char *token_issuer_path = "@ROMFS/trusted_flight/token_issuer";
    const char *trusted_certificate_path = "@ROMFS/trusted_flight/root_ca.crt";

    const char *token_file_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/token";
    const char *certificate_chain_path = HAL_BOARD_STORAGE_DIRECTORY "/trusted_flight/ca_chain.crt";
    const char *public_key_header = "-----BEGIN PUBLIC KEY-----\n";
    const char *public_key_footer = "-----END PUBLIC KEY-----\n";

    bool init_done = false;

    struct l8w8jwt_decoding_params params;
    struct mbedtls_x509_crt trusted_certificate;

    static AP_AerobridgeTrustedFlight *_singleton;
};

namespace AP
{
AP_AerobridgeTrustedFlight &aerobridge_trusted_flight();
};
