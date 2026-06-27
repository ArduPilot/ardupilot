#!/usr/bin/env python3
'''
Decrypt a raw image of an AES-256-XTS full-disk-encrypted ArduPilot SD card.

When a board is built with "define AP_FATFS_CRYPTO_ENABLED 1" (see
libraries/AP_DiskCrypto), every 512-byte SD card sector is encrypted on the
media with AES-256-XTS, using the absolute sector number (LBA) as the XTS
tweak. Such a card cannot be mounted directly on a PC. This tool reverses that:
given the 64-byte key, it turns a raw card dump into a plaintext FAT image you
can loop-mount and analyse.

Make a raw dump of the card first, e.g.:
    sudo dd if=/dev/sdX of=card.img bs=1M

Then:
    Tools/scripts/decrypt_sdcard.py --key <128-hex-chars> card.img card.plain.img
    # mount the result (Linux):
    sudo mount -o loop,ro card.plain.img /mnt

The key is the same 64 bytes (32-byte data key followed by 32-byte tweak key)
that the firmware was built with (AP_DISKCRYPTO_KEY), given as 128 hex chars.

Requires the 'cryptography' package (pip install cryptography).

AP_FLAKE8_CLEAN
'''

import argparse
import sys

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

SECTOR_SIZE = 512
KEY_LEN = 64


def parse_key(text):
    text = text.strip().lower().replace(' ', '').replace(':', '')
    if text.startswith('0x'):
        text = text[2:]
    key = bytes.fromhex(text)
    if len(key) != KEY_LEN:
        raise ValueError("key must be %u bytes (%u hex chars), got %u bytes" %
                         (KEY_LEN, KEY_LEN * 2, len(key)))
    return key


def crypt_sector(key, data, lba, decrypt):
    tweak = lba.to_bytes(16, 'little')
    cipher = Cipher(algorithms.AES(key), modes.XTS(tweak))
    ctx = cipher.decryptor() if decrypt else cipher.encryptor()
    return ctx.update(data) + ctx.finalize()


def process(key, infile, outfile, decrypt, start_lba):
    lba = start_lba
    short = 0
    with open(infile, 'rb') as fin, open(outfile, 'wb') as fout:
        while True:
            block = fin.read(SECTOR_SIZE)
            if not block:
                break
            if len(block) < SECTOR_SIZE:
                # trailing partial sector: pass through untouched (not a full
                # XTS data unit) and warn.
                short = len(block)
                fout.write(block)
                break
            fout.write(crypt_sector(key, block, lba, decrypt))
            lba += 1
    sectors = lba - start_lba
    sys.stderr.write("processed %u sectors (%u bytes)%s\n" %
                     (sectors, sectors * SECTOR_SIZE,
                      "" if short == 0 else ", plus %u trailing bytes copied verbatim" % short))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--key', required=True,
                    help='64-byte AES-256-XTS key as %u hex chars' % (KEY_LEN * 2))
    ap.add_argument('--encrypt', action='store_true',
                    help='encrypt instead of decrypt (e.g. to prepare a test image)')
    ap.add_argument('--start-lba', type=int, default=0,
                    help='absolute sector number of the first sector in the image (default 0)')
    ap.add_argument('input', help='raw input image')
    ap.add_argument('output', help='output image')
    args = ap.parse_args()

    try:
        key = parse_key(args.key)
    except ValueError as e:
        ap.error(str(e))

    process(key, args.input, args.output, decrypt=not args.encrypt, start_lba=args.start_lba)


if __name__ == '__main__':
    main()
