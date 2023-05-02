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
    static AP_AerobridgeTrustedFlight *get_singleton()
    {
        return _singleton;
    }
    AP_AerobridgeTrustedFlight();
    bool is_trusted();

private:
    bool validate_token(mbedtls_x509_crt *certificate);
    bool validate_certificate_chain(mbedtls_x509_crt* trusted_certificate, mbedtls_x509_crt* certificate_chain, const char *cn);
    bool read_certificate_from_file(const char *filepath, mbedtls_x509_crt* certificate);
    bool read_from_file(const char *filename, char **outbuf, size_t *outsize);
    bool write_pem_certificate(mbedtls_x509_crt* certificate, unsigned char **outbuf, size_t *outsize);

    const char *token_issuer_path = "@ROMFS/trusted_flight/token_issuer";
    const char *trusted_certificate_path = "@ROMFS/trusted_flight/root_ca.crt";

    const char *token_file_path = "trusted_flight/token";
    const char *certificate_chain_path = "trusted_flight/ca_chain.crt";
    const char *public_key_header = "-----BEGIN PUBLIC KEY-----\n";
    const char *public_key_footer = "-----END PUBLIC KEY-----\n";
    const char *tag = "Aerobridge Trusted Flight";
    struct l8w8jwt_decoding_params params;

    static AP_AerobridgeTrustedFlight *_singleton;
};

namespace AP
{
AP_AerobridgeTrustedFlight &aerobridge_trusted_flight();
};
