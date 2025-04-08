/*
 * Base64 encoding/decoding (RFC1341)
 * Copyright (c) 2005-2011, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */
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

#include "onvifhelpers.h"

static const uint8_t base64_table[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static const uint8_t base64url_table[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";


/**
 * base64_encode - Base64 encode
 * @src: Data to be encoded
 * @len: Length of the data to be encoded
 * @out_len: Pointer to output length variable, or %nullptr if not used
 * Returns: Allocated buffer of out_len bytes of encoded data,
 * or %nullptr on failure
 *
 * Caller is responsible for freeing the returned buffer. Returned buffer is
 * nul terminated to make it easier to use as a C string. The nul terminator is
 * not included in out_len.
 */
static uint8_t* base64_encode_global(const uint8_t *src, uint16_t len, uint16_t *out_len, const uint8_t* table, bool urlsafe)
{
	uint8_t *out, *pos;
	const uint8_t *end, *in;
	uint16_t olen;
	int16_t line_len;

	olen = len * 4 / 3 + 4; /* 3-byte blocks to 4-byte */
	if (!urlsafe) {
		olen += olen / 72; /* line feeds */
	}

	olen++; /* nul termination */
	if (olen < len) {
		return nullptr; /* integer overflow */
	}
	out = (uint8_t*)malloc(olen);
	if (out == nullptr) {
		return nullptr;
	}

	end = src + len;
	in = src;
	pos = out;
	line_len = 0;
	while (end - in >= 3) {
		*pos++ = table[in[0] >> 2];
		*pos++ = table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
		*pos++ = table[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
		*pos++ = table[in[2] & 0x3f];
		in += 3;
		// if (!urlsafe) {
		// 	line_len += 4;
		// 	if (line_len >= 72) {
		// 		*pos++ = '\n';
		// 		line_len = 0;
		// 	}
		// }
	}

	if (end - in) {
		*pos++ = table[in[0] >> 2];
		if (end - in == 1) {
			*pos++ = table[(in[0] & 0x03) << 4];
			if(!urlsafe) {
				*pos++ = '=';
			}
		} else {
			*pos++ = table[((in[0] & 0x03) << 4) |
					      (in[1] >> 4)];
			*pos++ = table[(in[1] & 0x0f) << 2];
		}
		if (!urlsafe) {
			*pos++ = '=';
		}
		line_len += 4;
	}

	// if (line_len && !urlsafe) {
	// 	*pos++ = '\n';
	// }

	*pos = '\0';
	if (out_len) {
		*out_len = pos - out;
	}
	return out;
}


uint8_t* base64_encode(const uint8_t *src, uint16_t len, uint16_t *out_len)
{
	return base64_encode_global(src, len, out_len, base64_table, false);
}

//Reference: https://tools.ietf.org/html/rfc4648#section-5
uint8_t* base64url_encode(const uint8_t *src, uint16_t len, uint16_t *out_len)
{
	return base64_encode_global(src, len, out_len, base64url_table, true);
}



/**
 * base64_decode - Base64 decode
 * @src: Data to be decoded
 * @len: Length of the data to be decoded
 * @out_len: Pointer to output length variable
 * Returns: Allocated buffer of out_len bytes of decoded data,
 * or %nullptr on failure
 *
 * Caller is responsible for freeing the returned buffer.
 */
uint8_t* base64_decode(const uint8_t *src, uint16_t len, uint16_t *out_len)
{
	uint8_t dtable[256], *out, *pos, block[4], tmp;
	uint16_t i, count, olen;
	int16_t pad = 0;

	memset(dtable, 0x80, 256);
	for (i = 0; i < sizeof(base64_table) - 1; i++) {
		dtable[base64_table[i]] = (uint8_t) i;
	}
	dtable['='] = 0;

	count = 0;
	for (i = 0; i < len; i++) {
		if (dtable[src[i]] != 0x80) {
			count++;
		}
	}

	if (count == 0 || count % 4) {
		return nullptr;
	}

	olen = count / 4 * 3;
	pos = out = (uint8_t*)malloc(olen);
	if (out == nullptr) {
		return nullptr;
	}

	count = 0;
	for (i = 0; i < len; i++) {
		tmp = dtable[src[i]];
		if (tmp == 0x80) {
			continue;
		}

		if (src[i] == '=') {
			pad++;
		}
		block[count] = tmp;
		count++;
		if (count == 4) {
			*pos++ = (block[0] << 2) | (block[1] >> 4);
			*pos++ = (block[1] << 4) | (block[2] >> 2);
			*pos++ = (block[2] << 6) | block[3];
			count = 0;
			if (pad) {
				if (pad == 1) {
					pos--;
				} else if (pad == 2) {
					pos -= 2;
				} else {
					/* Invalid padding */
					free(out);
					return nullptr;
				}
				break;
			}
		}
	}

	*out_len = pos - out;
	return out;
}
