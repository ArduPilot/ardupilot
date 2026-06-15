#!/usr/bin/env python3

"""Decrypt AP_Logger encrypted .BIN containers to standard DataFlash .BIN."""

import argparse
import ctypes
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[2]
MONOCYPHER_DIR = ROOT / "libraries" / "AP_CheckFirmware"
MONOCYPHER_CPP = MONOCYPHER_DIR / "monocypher.cpp"

HEADER_MAGIC = b"APLOGE1\0"
FRAME_MAGIC = b"APLF"
HEADER_LEN = 152
FRAME_HEADER_LEN = 16
TAG_LEN = 16
VERSION = 1
AEAD_XCHACHA20_POLY1305 = 1
WRAP_X25519_BLAKE2B = 1
WRAP_CONTEXT = b"ArduPilot AP_Logger key wrap v1"

WRAPPER_CPP = r"""
#include <stddef.h>
#include <stdint.h>

#include "monocypher.h"

extern "C" {

void ap_crypto_x25519_public_key(uint8_t *public_key, const uint8_t *secret_key)
{
    crypto_x25519_public_key(public_key, secret_key);
}

void ap_crypto_x25519(uint8_t *shared_secret, const uint8_t *secret_key, const uint8_t *public_key)
{
    crypto_x25519(shared_secret, secret_key, public_key);
}

void ap_crypto_blake2b_general(uint8_t *hash, size_t hash_size,
                               const uint8_t *key, size_t key_size,
                               const uint8_t *message, size_t message_size)
{
    crypto_blake2b_general(hash, hash_size, key, key_size, message, message_size);
}

int ap_crypto_unlock_aead(uint8_t *plain_text,
                          const uint8_t *key,
                          const uint8_t *nonce,
                          const uint8_t *mac,
                          const uint8_t *ad,
                          size_t ad_size,
                          const uint8_t *cipher_text,
                          size_t text_size)
{
    return crypto_unlock_aead(plain_text, key, nonce, mac, ad, ad_size, cipher_text, text_size);
}

}
"""


class Crypto:
    def __init__(self):
        self.lib = self._load_library()
        void_p = ctypes.c_void_p
        size_t = ctypes.c_size_t
        self.lib.ap_crypto_x25519_public_key.argtypes = [void_p, void_p]
        self.lib.ap_crypto_x25519_public_key.restype = None
        self.lib.ap_crypto_x25519.argtypes = [void_p, void_p, void_p]
        self.lib.ap_crypto_x25519.restype = None
        self.lib.ap_crypto_blake2b_general.argtypes = [void_p, size_t, void_p, size_t, void_p, size_t]
        self.lib.ap_crypto_blake2b_general.restype = None
        self.lib.ap_crypto_unlock_aead.argtypes = [
            void_p, void_p, void_p, void_p, void_p, size_t, void_p, size_t
        ]
        self.lib.ap_crypto_unlock_aead.restype = ctypes.c_int

    def _load_library(self):
        if not MONOCYPHER_CPP.exists():
            raise RuntimeError(f"Monocypher source not found: {MONOCYPHER_CPP}")

        digest = hashlib.sha256(MONOCYPHER_CPP.read_bytes() + WRAPPER_CPP.encode("utf-8")).hexdigest()[:16]
        suffix = ".dylib" if sys.platform == "darwin" else ".so"
        cache_dir = Path(tempfile.gettempdir()) / "ap_logger_decrypt_monocypher"
        cache_dir.mkdir(parents=True, exist_ok=True)
        lib_path = cache_dir / f"libaplogger_monocypher_{digest}{suffix}"

        if not lib_path.exists():
            compiler = os.environ.get("CXX") or shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
            if compiler is None:
                raise RuntimeError("No C++ compiler found; set CXX or install clang++/g++")
            wrapper_path = cache_dir / f"aplogger_monocypher_wrapper_{digest}.cpp"
            wrapper_path.write_text(WRAPPER_CPP)
            cmd = [
                compiler,
                "-std=c++11",
                "-shared",
                "-fPIC",
                "-I",
                str(MONOCYPHER_DIR),
                str(wrapper_path),
                str(MONOCYPHER_CPP),
                "-o",
                str(lib_path),
            ]
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            except subprocess.CalledProcessError as ex:
                raise RuntimeError(f"failed to build Monocypher shim: {ex.stderr}") from ex

        return ctypes.CDLL(str(lib_path))

    @staticmethod
    def _buf(data):
        return (ctypes.c_uint8 * len(data)).from_buffer_copy(data)

    @staticmethod
    def _out(size):
        return (ctypes.c_uint8 * size)()

    @staticmethod
    def _ptr(buf):
        return ctypes.cast(buf, ctypes.c_void_p)

    def x25519_public_key(self, private_key):
        out = self._out(32)
        private = self._buf(private_key)
        self.lib.ap_crypto_x25519_public_key(self._ptr(out), self._ptr(private))
        return bytes(out)

    def x25519(self, private_key, public_key):
        out = self._out(32)
        private = self._buf(private_key)
        public = self._buf(public_key)
        self.lib.ap_crypto_x25519(self._ptr(out), self._ptr(private), self._ptr(public))
        return bytes(out)

    def blake2b_general(self, size, key, message):
        out = self._out(size)
        message_buf = self._buf(message)
        if key:
            key_buf = self._buf(key)
            key_ptr = self._ptr(key_buf)
            key_size = len(key)
        else:
            key_buf = None
            key_ptr = None
            key_size = 0
        self.lib.ap_crypto_blake2b_general(
            self._ptr(out),
            size,
            key_ptr,
            key_size,
            self._ptr(message_buf),
            len(message),
        )
        return bytes(out)

    def unlock_aead(self, key, nonce, ad, mac, ciphertext):
        out = self._out(len(ciphertext))
        key_buf = self._buf(key)
        nonce_buf = self._buf(nonce)
        ad_buf = self._buf(ad)
        mac_buf = self._buf(mac)
        ciphertext_buf = self._buf(ciphertext)
        rc = self.lib.ap_crypto_unlock_aead(
            self._ptr(out),
            self._ptr(key_buf),
            self._ptr(nonce_buf),
            self._ptr(mac_buf),
            self._ptr(ad_buf),
            len(ad),
            self._ptr(ciphertext_buf),
            len(ciphertext),
        )
        if rc != 0:
            raise ValueError("authentication failed")
        return bytes(out)


def read_private_key(args):
    if bool(args.private_key_hex) == bool(args.private_key_file):
        raise ValueError("provide exactly one of --private-key-hex or --private-key-file")

    if args.private_key_hex:
        key_text = args.private_key_hex
    else:
        raw = Path(args.private_key_file).read_bytes()
        if len(raw) == 32:
            return raw
        key_text = raw.decode("ascii").strip()

    key_text = key_text.replace("0x", "").replace(" ", "").replace("\n", "")
    private_key = bytes.fromhex(key_text)
    if len(private_key) != 32:
        raise ValueError("private key must be 32 bytes")
    return private_key


def read_exact(handle, size, what):
    data = handle.read(size)
    if len(data) != size:
        raise EOFError(f"truncated encrypted log while reading {what}")
    return data


def parse_header(header):
    if header[:8] != HEADER_MAGIC:
        raise ValueError("input is not an AP_Logger encrypted log")
    version = header[8]
    aead_id = header[9]
    wrap_id = header[10]
    flags = header[11]
    header_len = int.from_bytes(header[12:14], "little")
    if version != VERSION:
        raise ValueError(f"unsupported encrypted log version {version}")
    if aead_id != AEAD_XCHACHA20_POLY1305:
        raise ValueError(f"unsupported AEAD id {aead_id}")
    if wrap_id != WRAP_X25519_BLAKE2B:
        raise ValueError(f"unsupported wrap id {wrap_id}")
    if flags != 0:
        raise ValueError(f"unsupported encrypted log flags {flags}")
    if header_len != HEADER_LEN:
        raise ValueError(f"unexpected encrypted log header length {header_len}")
    return {
        "key_id": header[16:32],
        "ephemeral_public_key": header[32:64],
        "wrap_nonce": header[64:88],
        "stream_nonce_prefix": header[88:104],
        "wrapped_key": header[104:136],
        "wrapped_key_tag": header[136:152],
    }


def derive_wrap_key(crypto, shared_secret, key_id, ephemeral_public_key, recipient_public_key):
    message = WRAP_CONTEXT + key_id + ephemeral_public_key + recipient_public_key
    return crypto.blake2b_general(32, shared_secret, message)


def decrypt_log(input_path, output_path, private_key):
    crypto = Crypto()
    recipient_public_key = crypto.x25519_public_key(private_key)
    recipient_key_id = crypto.blake2b_general(16, b"", recipient_public_key)

    frames = 0
    plaintext_bytes = 0
    with Path(input_path).open("rb") as inf, Path(output_path).open("wb") as outf:
        header = read_exact(inf, HEADER_LEN, "header")
        fields = parse_header(header)
        if fields["key_id"] != recipient_key_id:
            raise ValueError("private key does not match this log recipient key id")

        shared_secret = crypto.x25519(private_key, fields["ephemeral_public_key"])
        wrap_key = derive_wrap_key(
            crypto,
            shared_secret,
            fields["key_id"],
            fields["ephemeral_public_key"],
            recipient_public_key,
        )
        data_key = crypto.unlock_aead(
            wrap_key,
            fields["wrap_nonce"],
            header[:104],
            fields["wrapped_key_tag"],
            fields["wrapped_key"],
        )

        while True:
            frame_header = inf.read(FRAME_HEADER_LEN)
            if frame_header == b"":
                break
            if len(frame_header) != FRAME_HEADER_LEN:
                raise EOFError("truncated encrypted log frame header")
            if frame_header[:4] != FRAME_MAGIC:
                raise ValueError("bad encrypted log frame magic")

            sequence = int.from_bytes(frame_header[4:12], "little")
            plaintext_len = int.from_bytes(frame_header[12:14], "little")
            if frame_header[14] != 0 or frame_header[15] != 0:
                raise ValueError("unsupported encrypted log frame flags")
            if sequence != frames:
                raise ValueError(f"unexpected encrypted log frame sequence {sequence}, expected {frames}")

            tag = read_exact(inf, TAG_LEN, "frame tag")
            ciphertext = read_exact(inf, plaintext_len, "frame ciphertext")
            nonce = fields["stream_nonce_prefix"] + sequence.to_bytes(8, "little")
            plaintext = crypto.unlock_aead(data_key, nonce, frame_header, tag, ciphertext)
            outf.write(plaintext)
            frames += 1
            plaintext_bytes += len(plaintext)

    return frames, plaintext_bytes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", help="encrypted AP_Logger .BIN")
    parser.add_argument("output", nargs="?", help="decrypted standard DataFlash .BIN")
    parser.add_argument("--private-key-hex", help="32-byte X25519 private key as hex")
    parser.add_argument("--private-key-file", help="file containing a 32-byte private key, raw or hex")
    parser.add_argument("--show-public-key", action="store_true", help="print the public key for the supplied private key")
    args = parser.parse_args()

    try:
        private_key = read_private_key(args)
        if args.show_public_key:
            print(Crypto().x25519_public_key(private_key).hex())
            return 0
        if args.input is None or args.output is None:
            parser.error("input and output are required unless --show-public-key is used")
        frames, plaintext_bytes = decrypt_log(args.input, args.output, private_key)
    except Exception as ex:
        print(f"ap_logger_decrypt: {ex}", file=sys.stderr)
        return 1

    print(f"decrypted {frames} frames, {plaintext_bytes} bytes -> {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
