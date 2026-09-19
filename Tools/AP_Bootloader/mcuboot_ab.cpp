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
 * Code by @davidbuzz and Claude
 */

#include "mcuboot_ab.h"

#if AP_BOOTLOADER_MCUBOOT_AB

#include "support.h"
#include <string.h>

/* The slot-1 check is SHA-256 integrity only: a full MCUBoot image walk needs an
 * Ed25519 verifier this tree does not carry. */

/* Slot geometry in flash_func_* address space (offset 0 = start of the app). */
#ifndef AP_MCUBOOT_SLOT_SIZE
// 3 MB per slot, matching the board DTS slot0/slot1_partition layout
// (slot1_partition@320000 = app-region offset 0x300000) and zephyr.py's app
// .img --slot-size. Was 2 MB before 2026-08-14, chosen independently of the
// DTS; the DTS geometry (flush against storage_partition@620000) wins.
#define AP_MCUBOOT_SLOT_SIZE (3U * 1024U * 1024U)   // 3 MB per slot
#endif
#define SLOT0_OFFSET 0U
#define SLOT1_OFFSET AP_MCUBOOT_SLOT_SIZE


// ---- compact SHA-256 (public domain, Brad Conte's implementation) ----
struct sha256_ctx {
    uint8_t data[64];
    uint32_t datalen;
    uint64_t bitlen;
    uint32_t state[8];
};

#define ROTR(a, b) (((a) >> (b)) | ((a) << (32 - (b))))
#define CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x)  (ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define EP1(x)  (ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SIG0(x) (ROTR(x, 7) ^ ROTR(x, 18) ^ ((x) >> 3))
#define SIG1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

static const uint32_t sha256_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

static void sha256_transform(sha256_ctx *ctx, const uint8_t data[])
{
    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
    for (i = 0, j = 0; i < 16; ++i, j += 4) {
        m[i] = (data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3]);
    }
    for (; i < 64; ++i) {
        m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];
    }
    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];
    for (i = 0; i < 64; ++i) {
        t1 = h + EP1(e) + CH(e, f, g) + sha256_k[i] + m[i];
        t2 = EP0(a) + MAJ(a, b, c);
        h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init(sha256_ctx *ctx)
{
    ctx->datalen = 0; ctx->bitlen = 0;
    ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
}

static void sha256_update(sha256_ctx *ctx, const uint8_t data[], size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        ctx->data[ctx->datalen] = data[i];
        ctx->datalen++;
        if (ctx->datalen == 64) {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

static void sha256_final(sha256_ctx *ctx, uint8_t hash[])
{
    uint32_t i = ctx->datalen;
    ctx->data[i++] = 0x80;
    if (ctx->datalen < 56) {
        while (i < 56) {
            ctx->data[i++] = 0x00;
        }
    } else {
        while (i < 64) {
            ctx->data[i++] = 0x00;
        }
        sha256_transform(ctx, ctx->data);
        memset(ctx->data, 0, 56);
    }
    ctx->bitlen += ctx->datalen * 8;
    for (int k = 7; k >= 0; --k) {
        ctx->data[56 + (7 - k)] = (uint8_t)(ctx->bitlen >> (k * 8));
    }
    sha256_transform(ctx, ctx->data);
    for (i = 0; i < 4; ++i) {
        for (int k = 0; k < 8; ++k) {
            hash[i + k * 4] = (uint8_t)((ctx->state[k] >> (24 - i * 8)) & 0xff);
        }
    }
}

// ---- slot access (flash_func_* address space) ----
static void slot_read(uint32_t slot_off, uint32_t rel, void *dst, uint32_t len)
{
    uint8_t *d = (uint8_t *)dst;
    uint32_t off = slot_off + rel;
    while (len >= 4) {
        uint32_t w = flash_func_read_word(off);
        memcpy(d, &w, 4);
        d += 4; off += 4; len -= 4;
    }
    if (len) {
        uint32_t w = flash_func_read_word(off);
        memcpy(d, &w, len);
    }
}

// Feed the whole hashed span (header+image) of a slot into SHA-256, in
// page-sized reads so no large buffer is needed.
static void slot_sha256(uint32_t slot_off, uint32_t hashed_len, uint8_t out[32])
{
    sha256_ctx c;
    sha256_init(&c);
    uint8_t buf[256];
    uint32_t done = 0;
    while (done < hashed_len) {
        uint32_t n = hashed_len - done;
        if (n > sizeof(buf)) {
            n = sizeof(buf);
        }
        slot_read(slot_off, done, buf, n);
        sha256_update(&c, buf, n);
        done += n;
    }
    sha256_final(&c, out);
}

/* Validate the imgtool image staged in slot 1: header magic, then walk the TLVs. */
static bool slot1_image_valid(uint32_t &total_len)
{
    struct mcuboot_image_header hdr;
    slot_read(SLOT1_OFFSET, 0, &hdr, sizeof(hdr));
    if (hdr.magic != MCUBOOT_IMAGE_MAGIC) {
        return false;
    }
    const uint32_t hashed_len = hdr.hdr_size + hdr.img_size;
    if (hashed_len == 0 || hashed_len > AP_MCUBOOT_SLOT_SIZE) {
        return false;
    }

    uint8_t computed[32];
    slot_sha256(SLOT1_OFFSET, hashed_len, computed);

    // TLV info header follows the protected region
    uint32_t tlv_off = hashed_len;
    uint16_t info[2];
    slot_read(SLOT1_OFFSET, tlv_off, info, sizeof(info));
    const uint16_t tlv_magic = info[0];
    const uint16_t tlv_total = info[1];
    if (tlv_magic != MCUBOOT_TLV_INFO_MAGIC && tlv_magic != MCUBOOT_TLV_PROT_INFO_MAGIC) {
        return false;
    }

    bool sha_ok = false;
    uint32_t p = tlv_off + 4;
    const uint32_t tlv_end = tlv_off + tlv_total;
    while (p + 4 <= tlv_end) {
        uint16_t th[2];
        slot_read(SLOT1_OFFSET, p, th, sizeof(th));
        const uint16_t type = th[0];
        const uint16_t len = th[1];
        const uint32_t val = p + 4;
        if (type == MCUBOOT_TLV_SHA256 && len == 32) {
            uint8_t stored[32];
            slot_read(SLOT1_OFFSET, val, stored, 32);
            sha_ok = (memcmp(stored, computed, 32) == 0);
        }
        p = val + len;
    }

    if (!sha_ok) {
        return false;
    }
    total_len = hashed_len + tlv_total;
    return true;
}

// Erase every slot-0 sector spanned by len, then program it page-by-page
// from slot 1. Erase-then-program-once, the pattern the NOR tolerates.
static bool copy_slot1_to_slot0(uint32_t len)
{
    // sector indices are relative to flash_base; slot 0 starts at sector 0
    const uint32_t sec_size = flash_func_sector_size(0);
    if (sec_size == 0) {
        return false;
    }
    const uint32_t nsec = (len + sec_size - 1) / sec_size;
    for (uint32_t s = 0; s < nsec; s++) {
        if (!flash_func_erase_sector(s, true)) {
            return false;
        }
    }
    uint8_t page[256];
    uint32_t done = 0;
    while (done < len) {
        uint32_t n = len - done;
        if (n > sizeof(page)) {
            n = sizeof(page);
        }
        // pad the tail to a word boundary with 0xFF (programs no bits)
        uint32_t words = (n + 3) / 4;
        memset(page, 0xFF, sizeof(page));
        slot_read(SLOT1_OFFSET, done, page, n);
        if (!flash_func_write_words(SLOT0_OFFSET + done, (uint32_t *)page, (uint8_t)words)) {
            return false;
        }
        done += n;
    }
    return true;
}

bool mcuboot_ab_update(void)
{
    uint32_t total_len = 0;
    if (!slot1_image_valid(total_len)) {
        return false;
    }
    if (!copy_slot1_to_slot0(total_len)) {
        return false;
    }
    // Erase slot 1's first sector so the image is not re-applied next boot
    // (overwrite-only: no revert, no confirm handshake).
    const uint32_t sec_size = flash_func_sector_size(0);
    const uint32_t slot1_first_sector = SLOT1_OFFSET / sec_size;
    (void)flash_func_erase_sector(slot1_first_sector, true);
    return true;
}

#endif  // AP_BOOTLOADER_MCUBOOT_AB
