#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Select, download, validate and cache firmware ELF files for Renode."""

import argparse
import gzip
import hashlib
import io
import json
import os
import re
import sys
import tempfile
import urllib.parse
import urllib.request

from pathlib import Path

MANIFEST_URL = 'https://firmware.ardupilot.org/manifest.json.gz'
FIRMWARE_HOST = 'firmware.ardupilot.org'
MAX_MANIFEST_DOWNLOAD = 16 * 1024 * 1024
MAX_MANIFEST_SIZE = 128 * 1024 * 1024
MAX_FIRMWARE_SIZE = 128 * 1024 * 1024
SAFE_COMPONENT_RE = re.compile(r'^[A-Za-z0-9_.+-]+$')


def default_cache():
    root = os.environ.get('XDG_CACHE_HOME')
    if root:
        return Path(root).expanduser() / 'ardupilot' / 'renode' / 'firmware'
    return Path.home() / '.cache' / 'ardupilot' / 'renode' / 'firmware'


def _read_limited(response, maximum, progress=None):
    expected = response.headers.get('Content-Length')
    expected = int(expected) if expected is not None else None
    if expected is not None and not 0 <= expected <= maximum:
        raise ValueError('download size %u is outside the allowed range' % expected)
    chunks = []
    received = 0
    while True:
        chunk = response.read(1024 * 1024)
        if not chunk:
            break
        received += len(chunk)
        if received > maximum:
            raise ValueError('download exceeds %u bytes' % maximum)
        chunks.append(chunk)
        if progress is not None:
            progress(received, expected or 0)
    if expected is not None and received != expected:
        raise ValueError('downloaded %u bytes; expected %u' %
                         (received, expected))
    return b''.join(chunks)


def fetch_manifest(url=MANIFEST_URL):
    with urllib.request.urlopen(url, timeout=30) as response:
        compressed = _read_limited(response, MAX_MANIFEST_DOWNLOAD)
    try:
        chunks = []
        expanded = 0
        with gzip.GzipFile(fileobj=io.BytesIO(compressed)) as stream:
            while True:
                chunk = stream.read(min(1024 * 1024,
                                        MAX_MANIFEST_SIZE - expanded + 1))
                if not chunk:
                    break
                expanded += len(chunk)
                if expanded > MAX_MANIFEST_SIZE:
                    raise ValueError('expanded firmware manifest is too large')
                chunks.append(chunk)
        data = b''.join(chunks)
    except (gzip.BadGzipFile, EOFError, OSError) as error:
        raise ValueError('firmware manifest is not valid gzip') from error
    manifest = json.loads(data)
    if (not isinstance(manifest, dict) or
            manifest.get('format-version') != '1.0.0' or
            not isinstance(manifest.get('firmware'), list)):
        raise ValueError('unsupported firmware manifest format')
    return manifest


def _validate_entry(entry):
    required = ('platform', 'vehicletype', 'git-sha', 'url')
    if (not isinstance(entry, dict) or
            any(not isinstance(entry.get(name), str) or not entry[name]
                for name in required)):
        raise ValueError('incomplete firmware manifest entry')
    if entry.get('format') != 'elf':
        raise ValueError('firmware manifest entry is not an ELF')
    for name in ('platform', 'vehicletype'):
        if (entry[name] in ('.', '..') or
                not SAFE_COMPONENT_RE.fullmatch(entry[name])):
            raise ValueError('unsafe firmware %s' % name)
    if not re.fullmatch(r'[0-9a-f]{40}', entry['git-sha']):
        raise ValueError('unsafe firmware git-sha')
    parsed = urllib.parse.urlparse(entry['url'])
    if (parsed.scheme != 'https' or parsed.hostname != FIRMWARE_HOST or
            parsed.username is not None or parsed.password is not None or
            parsed.query or parsed.fragment or not parsed.path.endswith('.elf')):
        raise ValueError('unsafe firmware URL')
    return parsed


def select_firmware(manifest, board, vehicle, channel):
    """Select the server alias entry for an exact board, vehicle and channel."""
    if channel not in ('stable', 'latest'):
        raise ValueError('firmware channel must be stable or latest')
    candidates = []
    for entry in manifest.get('firmware', []):
        if (entry.get('platform') != board or
                entry.get('vehicletype') != vehicle or
                entry.get('format') != 'elf'):
            continue
        parsed = _validate_entry(entry)
        parts = [urllib.parse.unquote(part) for part in parsed.path.split('/')]
        if len(parts) < 4 or parts[-2] != board or parts[-3] != channel:
            continue
        if channel == 'latest' and entry.get('latest') != 1:
            continue
        candidates.append(entry)
    if len(candidates) != 1:
        raise ValueError('found %u %s ELF images for %s/%s' %
                         (len(candidates), channel, vehicle, board))
    return candidates[0]


def _elf_is_arm32(path):
    try:
        header = path.read_bytes()[:20]
    except OSError:
        return False
    return (len(header) == 20 and header[:4] == b'\x7fELF' and
            header[4:6] == b'\x01\x01' and
            int.from_bytes(header[18:20], 'little') == 40)


def _cache_paths(cache, entry):
    _validate_entry(entry)
    directory = (Path(cache).expanduser() / entry['vehicletype'] /
                 entry['platform'] / entry['git-sha'])
    return directory / 'firmware.elf', directory / 'firmware.json'


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def cached_firmware(cache, entry):
    firmware, metadata_path = _cache_paths(cache, entry)
    try:
        metadata = json.loads(metadata_path.read_text())
        if (metadata.get('url') != entry['url'] or
                metadata.get('git-sha') != entry['git-sha'] or
                metadata.get('size') != firmware.stat().st_size or
                metadata.get('sha256') != _sha256(firmware) or
                not _elf_is_arm32(firmware)):
            return None
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return firmware


def install_firmware(cache, entry, progress=None):
    """Return (ELF path, downloaded), installing atomically when necessary."""
    current = cached_firmware(cache, entry)
    if current is not None:
        return current, False
    firmware, metadata_path = _cache_paths(cache, entry)
    firmware.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with urllib.request.urlopen(entry['url'], timeout=60) as response:
            data = _read_limited(response, MAX_FIRMWARE_SIZE, progress)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix='.firmware-', dir=firmware.parent)
        temporary = Path(temporary_name)
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if not _elf_is_arm32(temporary):
            raise ValueError('download is not a 32-bit ARM ELF')
        metadata = {
            'url': entry['url'],
            'git-sha': entry['git-sha'],
            'version': entry.get('mav-firmware-version-str'),
            'size': temporary.stat().st_size,
            'sha256': _sha256(temporary),
        }
        temporary.replace(firmware)
        metadata_tmp = metadata_path.with_suffix('.json.tmp')
        metadata_tmp.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + '\n')
        metadata_tmp.replace(metadata_path)
        return firmware, True
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board')
    parser.add_argument('--vehicle', default='Copter')
    parser.add_argument('--channel', choices=('stable', 'latest'),
                        default='latest')
    parser.add_argument('--cache', default=str(default_cache()))
    args = parser.parse_args()
    try:
        manifest = fetch_manifest()
        selected = select_firmware(
            manifest, args.board, args.vehicle, args.channel)
        path, downloaded = install_firmware(args.cache, selected)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print('%s%s' % (path, ' (downloaded)' if downloaded else ' (cached)'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
