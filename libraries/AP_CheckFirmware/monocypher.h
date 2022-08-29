// Monocypher version 3.1.2
//
// This file is dual-licensed.  Choose whichever licence you want from
// the two licences listed below.
//
// The first licence is a regular 2-clause BSD licence.  The second licence
// is the CC-0 from Creative Commons. It is intended to release Monocypher
// to the public domain.  The BSD licence serves as a fallback option.
//
// SPDX-License-Identifier: BSD-2-Clause OR CC0-1.0
//
// ------------------------------------------------------------------------
//
// Copyright (c) 2017-2019, Loup Vaillant
// All rights reserved.
//
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// ------------------------------------------------------------------------
//
// Written in 2017-2019 by Loup Vaillant
//
// To the extent possible under law, the author(s) have dedicated all copyright
// and related neighboring rights to this software to the public domain
// worldwide.  This software is distributed without any warranty.
//
// You should have received a copy of the CC0 Public Domain Dedication along
// with this software.  If not, see
// <https://creativecommons.org/publicdomain/zero/1.0/>

#ifndef MONOCYPHER_H
#define MONOCYPHER_H

#include <stddef.h>
#include <stdint.h>

////////////////////////
/// Type definitions ///
////////////////////////

// Vtable for EdDSA with a custom hash.
// Instantiate it to define a custom hash.
// Its size, contents, and layout, are part of the public API.
typedef struct {
    void (*hash)(uint8_t hash[64], const uint8_t *message, size_t message_size);
    void (*init  )(void *ctx);
    void (*update)(void *ctx, const uint8_t *message, size_t message_size);
    void (*final )(void *ctx, uint8_t hash[64]);
    size_t ctx_size;
} crypto_sign_vtable;

// Do not rely on the size or contents of any of the types below,
// they may change without notice.

// Poly1305
typedef struct {
    uint32_t r[4];   // constant multiplier (from the secret key)
    uint32_t h[5];   // accumulated hash
    uint32_t c[5];   // chunk of the message
    uint32_t pad[4]; // random number added at the end (from the secret key)
    size_t   c_idx;  // How many bytes are there in the chunk.
} crypto_poly1305_ctx;

// Hash (Blake2b)
typedef struct {
    uint64_t hash[8];
    uint64_t input_offset[2];
    uint64_t input[16];
    size_t   input_idx;
    size_t   hash_size;
} crypto_blake2b_ctx;

// Signatures (EdDSA)
typedef struct {
    const crypto_sign_vtable *hash;
    uint8_t buf[96];
    uint8_t pk [32];
} crypto_sign_ctx_abstract;
typedef crypto_sign_ctx_abstract crypto_check_ctx_abstract;

typedef struct {
    crypto_sign_ctx_abstract ctx;
    crypto_blake2b_ctx       hash;
} crypto_sign_ctx;
typedef crypto_sign_ctx crypto_check_ctx;

////////////////////////////
/// High level interface ///
////////////////////////////

// Constant time comparisons
// -------------------------

// Return 0 if a and b are equal, -1 otherwise
int crypto_verify16(const uint8_t a[16], const uint8_t b[16]);
int crypto_verify32(const uint8_t a[32], const uint8_t b[32]);
int crypto_verify64(const uint8_t a[64], const uint8_t b[64]);

// Erase sensitive data
// --------------------

// Please erase all copies
void crypto_wipe(void *secret, size_t size);


// Authenticated encryption
// ------------------------
void crypto_lock(uint8_t        mac[16],
                 uint8_t       *cipher_text,
                 const uint8_t  key[32],
                 const uint8_t  nonce[24],
                 const uint8_t *plain_text, size_t text_size);
int crypto_unlock(uint8_t       *plain_text,
                  const uint8_t  key[32],
                  const uint8_t  nonce[24],
                  const uint8_t  mac[16],
                  const uint8_t *cipher_text, size_t text_size);

// With additional data
void crypto_lock_aead(uint8_t        mac[16],
                      uint8_t       *cipher_text,
                      const uint8_t  key[32],
                      const uint8_t  nonce[24],
                      const uint8_t *ad        , size_t ad_size,
                      const uint8_t *plain_text, size_t text_size);
int crypto_unlock_aead(uint8_t       *plain_text,
                       const uint8_t  key[32],
                       const uint8_t  nonce[24],
                       const uint8_t  mac[16],
                       const uint8_t *ad         , size_t ad_size,
                       const uint8_t *cipher_text, size_t text_size);


// General purpose hash (Blake2b)
// ------------------------------

// Direct interface
void crypto_blake2b(uint8_t hash[64],
                    const uint8_t *message, size_t message_size);

void crypto_blake2b_general(uint8_t       *hash   , size_t hash_size,
                            const uint8_t *key    , size_t key_size, // optional
                            const uint8_t *message, size_t message_size);

// Incremental interface
void crypto_blake2b_init  (crypto_blake2b_ctx *ctx);
void crypto_blake2b_update(crypto_blake2b_ctx *ctx,
                           const uint8_t *message, size_t message_size);
void crypto_blake2b_final (crypto_blake2b_ctx *ctx, uint8_t *hash);

void crypto_blake2b_general_init(crypto_blake2b_ctx *ctx, size_t hash_size,
                                 const uint8_t      *key, size_t key_size);

// vtable for signatures
extern const crypto_sign_vtable crypto_blake2b_vtable;


// Password key derivation (Argon2 i)
// ----------------------------------
void crypto_argon2i(uint8_t       *hash,      uint32_t hash_size,     // >= 4
                    void          *work_area, uint32_t nb_blocks,     // >= 8
                    uint32_t       nb_iterations,                     // >= 3
                    const uint8_t *password,  uint32_t password_size,
                    const uint8_t *salt,      uint32_t salt_size);    // >= 8

void crypto_argon2i_general(uint8_t       *hash,      uint32_t hash_size,// >= 4
                            void          *work_area, uint32_t nb_blocks,// >= 8
                            uint32_t       nb_iterations,                // >= 3
                            const uint8_t *password,  uint32_t password_size,
                            const uint8_t *salt,      uint32_t salt_size,// >= 8
                            const uint8_t *key,       uint32_t key_size,
                            const uint8_t *ad,        uint32_t ad_size);


// Key exchange (x25519 + HChacha20)
// ---------------------------------
#define crypto_key_exchange_public_key crypto_x25519_public_key
void crypto_key_exchange(uint8_t       shared_key      [32],
                         const uint8_t your_secret_key [32],
                         const uint8_t their_public_key[32]);


// Signatures (EdDSA with curve25519 + Blake2b)
// --------------------------------------------

// Generate public key
void crypto_sign_public_key(uint8_t        public_key[32],
                            const uint8_t  secret_key[32]);

// Direct interface
void crypto_sign(uint8_t        signature [64],
                 const uint8_t  secret_key[32],
                 const uint8_t  public_key[32], // optional, may be 0
                 const uint8_t *message, size_t message_size);
int crypto_check(const uint8_t  signature [64],
                 const uint8_t  public_key[32],
                 const uint8_t *message, size_t message_size);

////////////////////////////
/// Low level primitives ///
////////////////////////////

// For experts only.  You have been warned.

// Chacha20
// --------

// Specialised hash.
// Used to hash X25519 shared secrets.
void crypto_hchacha20(uint8_t       out[32],
                      const uint8_t key[32],
                      const uint8_t in [16]);

// Unauthenticated stream cipher.
// Don't forget to add authentication.
void crypto_chacha20(uint8_t       *cipher_text,
                     const uint8_t *plain_text,
                     size_t         text_size,
                     const uint8_t  key[32],
                     const uint8_t  nonce[8]);
void crypto_xchacha20(uint8_t       *cipher_text,
                      const uint8_t *plain_text,
                      size_t         text_size,
                      const uint8_t  key[32],
                      const uint8_t  nonce[24]);
void crypto_ietf_chacha20(uint8_t       *cipher_text,
                          const uint8_t *plain_text,
                          size_t         text_size,
                          const uint8_t  key[32],
                          const uint8_t  nonce[12]);
uint64_t crypto_chacha20_ctr(uint8_t       *cipher_text,
                             const uint8_t *plain_text,
                             size_t         text_size,
                             const uint8_t  key[32],
                             const uint8_t  nonce[8],
                             uint64_t       ctr);
uint64_t crypto_xchacha20_ctr(uint8_t       *cipher_text,
                              const uint8_t *plain_text,
                              size_t         text_size,
                              const uint8_t  key[32],
                              const uint8_t  nonce[24],
                              uint64_t       ctr);
uint32_t crypto_ietf_chacha20_ctr(uint8_t       *cipher_text,
                                  const uint8_t *plain_text,
                                  size_t         text_size,
                                  const uint8_t  key[32],
                                  const uint8_t  nonce[12],
                                  uint32_t       ctr);

// Poly 1305
// ---------

// This is a *one time* authenticator.
// Disclosing the mac reveals the key.
// See crypto_lock() on how to use it properly.

// Direct interface
void crypto_poly1305(uint8_t        mac[16],
                     const uint8_t *message, size_t message_size,
                     const uint8_t  key[32]);

// Incremental interface
void crypto_poly1305_init  (crypto_poly1305_ctx *ctx, const uint8_t key[32]);
void crypto_poly1305_update(crypto_poly1305_ctx *ctx,
                            const uint8_t *message, size_t message_size);
void crypto_poly1305_final (crypto_poly1305_ctx *ctx, uint8_t mac[16]);


// X-25519
// -------

// Shared secrets are not quite random.
// Hash them to derive an actual shared key.
void crypto_x25519_public_key(uint8_t       public_key[32],
                              const uint8_t secret_key[32]);
void crypto_x25519(uint8_t       raw_shared_secret[32],
                   const uint8_t your_secret_key  [32],
                   const uint8_t their_public_key [32]);

// "Dirty" versions of x25519_public_key()
// Only use to generate ephemeral keys you want to hide.
// Note that those functions leaks 3 bits of the private key.
void crypto_x25519_dirty_small(uint8_t pk[32], const uint8_t sk[32]);
void crypto_x25519_dirty_fast (uint8_t pk[32], const uint8_t sk[32]);

// scalar "division"
// Used for OPRF.  Be aware that exponential blinding is less secure
// than Diffie-Hellman key exchange.
void crypto_x25519_inverse(uint8_t       blind_salt [32],
                           const uint8_t private_key[32],
                           const uint8_t curve_point[32]);


// EdDSA to X25519
// ---------------
void crypto_from_eddsa_private(uint8_t x25519[32], const uint8_t eddsa[32]);
void crypto_from_eddsa_public (uint8_t x25519[32], const uint8_t eddsa[32]);


// EdDSA -- Incremental interface
// ------------------------------

// Signing (2 passes)
// Make sure the two passes hash the same message,
// else you might reveal the private key.
void crypto_sign_init_first_pass(crypto_sign_ctx_abstract *ctx,
                                 const uint8_t  secret_key[32],
                                 const uint8_t  public_key[32]);
void crypto_sign_update(crypto_sign_ctx_abstract *ctx,
                        const uint8_t *message, size_t message_size);
void crypto_sign_init_second_pass(crypto_sign_ctx_abstract *ctx);
// use crypto_sign_update() again.
void crypto_sign_final(crypto_sign_ctx_abstract *ctx, uint8_t signature[64]);

// Verification (1 pass)
// Make sure you don't use (parts of) the message
// before you're done checking it.
void crypto_check_init  (crypto_check_ctx_abstract *ctx,
                         const uint8_t signature[64],
                         const uint8_t public_key[32]);
void crypto_check_update(crypto_check_ctx_abstract *ctx,
                         const uint8_t *message, size_t message_size);
int crypto_check_final  (crypto_check_ctx_abstract *ctx);

// Custom hash interface
void crypto_sign_public_key_custom_hash(uint8_t       public_key[32],
                                        const uint8_t secret_key[32],
                                        const crypto_sign_vtable *hash);
void crypto_sign_init_first_pass_custom_hash(crypto_sign_ctx_abstract *ctx,
                                             const uint8_t secret_key[32],
                                             const uint8_t public_key[32],
                                             const crypto_sign_vtable *hash);
void crypto_check_init_custom_hash(crypto_check_ctx_abstract *ctx,
                                   const uint8_t signature[64],
                                   const uint8_t public_key[32],
                                   const crypto_sign_vtable *hash);

// Elligator 2
// -----------

// Elligator mappings proper
void crypto_hidden_to_curve(uint8_t curve [32], const uint8_t hidden[32]);
int  crypto_curve_to_hidden(uint8_t hidden[32], const uint8_t curve [32],
                            uint8_t tweak);

// Easy to use key pair generation
void crypto_hidden_key_pair(uint8_t hidden[32], uint8_t secret_key[32],
                            uint8_t seed[32]);


#endif // MONOCYPHER_H
