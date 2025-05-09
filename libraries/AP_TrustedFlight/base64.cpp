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
 * Code by Rhythm Chopra
 */
#include "AP_TrustedFlight_Config.h"

#if AP_JWT_ENABLED

#include <string.h>
#include "AP_Jwt.h"

static const uint8_t TABLE[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static const uint8_t URL_SAFE_TABLE[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

static uint8_t* base64_decode_global(const uint8_t* data, const size_t data_length, uint16_t* out_length, bool urlsafe)
{
    uint8_t *out;
	if (data == NULL || out_length == NULL)
    {
        return nullptr;
    }

    size_t in_length = data_length;

    if (in_length == 0)
    {
        return nullptr;
    }

    if (*(data + in_length - 1) == '\0')
    {
        in_length--;
    }

    size_t i;
    size_t count = 0;
    uint8_t dtable[256];
    const uint8_t* table = urlsafe ? URL_SAFE_TABLE : TABLE;

    memset(dtable, 0x80, 256);

    for (i = 0; i < sizeof(TABLE) - 1; ++i)
    {
        dtable[table[i]] = (uint8_t)i;
    }

    dtable['='] = 0;

    for (i = 0; i < in_length; ++i)
    {
        if (dtable[(unsigned char)data[i]] != 0x80)
            count++;
    }

    int r = (int)(count % 4);

    if (count == 0 || r == 1 || (!urlsafe && r > 0)) // Invalid input string (format or padding).
        return nullptr;

    if (r == 3)
        r = 1;

    out = (uint8_t *)malloc(count / 4 * 3 + 16);
    if (out == nullptr)
    {
        return nullptr;
    }

    count = 0;
    int pad = 0;
    uint8_t tmp;
    uint8_t block[4];
    uint8_t* pos = out;

    for (i = 0; i < in_length + r; ++i)
    {
        const unsigned char c = i < in_length ? data[i] : '=';

        tmp = dtable[c];

        if (tmp == 0x80)
            continue;

        if (c == '=')
            pad++;

        block[count] = tmp;
        count++;

        if (count == 4)
        {
            *pos++ = (block[0] << 2) | (block[1] >> 4);
            *pos++ = (block[1] << 4) | (block[2] >> 2);
            *pos++ = (block[2] << 6) | block[3];
            count = 0;
            if (pad)
            {
                if (pad == 1)
                {
                    pos--;
                }
                else if (pad == 2)
                {
                    pos -= 2;
                }
                else
                {
                    free(out);
                    return nullptr; // Invalid padding...
                }
                break;
            }
        }
    }

    *out_length = pos - out;

    return out;
}

uint8_t* base64_decode(const uint8_t *src, uint16_t len, uint16_t *out_len)
{
	return base64_decode_global(src, len, out_len, false);
}

uint8_t* base64url_decode(const uint8_t *src, uint16_t len, uint16_t *out_len)
{
	return base64_decode_global(src, len, out_len, true);
}

#endif // AP_JWT_ENABLED