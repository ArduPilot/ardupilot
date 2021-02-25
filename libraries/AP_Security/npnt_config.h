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
 * Code by Siddharth Bharat Purohit
 */
#pragma once

#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/sha256.h>

typedef wc_Sha256 npnt_sha_t;

#define DIGEST_VALUE_LEN 32 // For SHA256
#define SIGNATURE_BYTE_LEN 256
//TODO: Remove this once test app adds this
#define PERMART_SKIP_ALT
#define PERMART_SKIP_FPARAMS

#define MXML_REALLOC(p,n) mxml_realloc( (p) , (n) )
