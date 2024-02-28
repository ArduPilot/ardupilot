/*
 ---------------------------------------------------------------------------
 Copyright (c) 2002, Dr Brian Gladman, Worcester, UK.   All rights reserved.

 LICENSE TERMS

 The free distribution and use of this software in both source and binary
 form is allowed (with or without changes) provided that:

   1. distributions of this source code include the above copyright
      notice, this list of conditions and the following disclaimer;

   2. distributions in binary form include the above copyright
      notice, this list of conditions and the following disclaimer
      in the documentation and/or other associated materials;

   3. the copyright holder's name is not used to endorse products
      built using this software without specific written permission.

 ALTERNATIVELY, provided that this notice is retained in full, this product
 may be distributed under the terms of the GNU General Public License (GPL),
 in which case the provisions of the GPL apply INSTEAD OF those given above.

 DISCLAIMER

 This software is provided 'as is' with no explicit or implied warranties
 in respect of its properties, including, but not limited to, correctness
 and/or fitness for purpose.
 ---------------------------------------------------------------------------
 Issue Date: 01/08/2005
*/

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Size of SHA1 digest */

#define SHA1_DIGEST_SIZE 20


#ifdef __cplusplus
extern "C" {
#endif

/* type to hold the SHA1 context  */

typedef struct
{   uint32_t count[2];
    uint32_t hash[5];
    uint32_t wbuf[16];
} sha1_ctx;

void sha1_begin(sha1_ctx ctx[1]);
void sha1_hash(const unsigned char data[], unsigned long len, sha1_ctx ctx[1]);
void sha1_end(unsigned char hval[], sha1_ctx ctx[1]);
void sha1(unsigned char hval[], const unsigned char data[], unsigned long len);
uint8_t* base64_encode(const uint8_t *src, uint16_t len, uint16_t *out_len);
uint8_t* base64_decode(const uint8_t *src, uint16_t len, uint16_t *out_len);
uint8_t* base64url_encode(const uint8_t *src, uint16_t len, uint16_t *out_len);

#ifdef __cplusplus
}
#endif
