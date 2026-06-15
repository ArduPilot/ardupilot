#pragma once

#include <AP_HAL/AP_HAL.h>

/**
 * base64_encode - Base64 encode
 * @src: Data to be encoded
 * @len: Length of the data to be encoded
 * @out_len: Pointer to output length variable, or %nullptr if not used
 * Returns: Allocated buffer of out_len bytes of encoded data, or %nullptr on failure
 *
 * Caller is responsible for freeing the returned buffer. Returned buffer is
 * nul terminated to make it easier to use as a C string. The nul terminator is
 * not included in out_len.
 */
uint8_t* base64_encode(const uint8_t *src, uint16_t len, uint16_t *out_len);
uint8_t* base64url_encode(const uint8_t *src, uint16_t len, uint16_t *out_len);

/**
 * base64_decode - Base64 decode
 * @src: Data to be decoded
 * @len: Length of the data to be decoded
 * @out_len: Pointer to output length variable
 * Returns: Allocated buffer of out_len bytes of decoded data, or %nullptr on failure
 *
 * Caller is responsible for freeing the returned buffer.
 */
uint8_t* base64_decode(const uint8_t *src, uint16_t len, uint16_t *out_len);
uint8_t* base64url_decode(const uint8_t *src, uint16_t len, uint16_t *out_len);
