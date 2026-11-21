#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Download and cache binary data used by the ArduPilot Renode models."""

import hashlib
import os
import tempfile
import urllib.parse
import urllib.request

from pathlib import Path

DATA_DOWNLOAD_BASE = 'https://firmware.ardupilot.org/Tools/Renode/data/'
DATA_FILES = {
    'SVD/STM32F103.svd.gz': {
        'size': 32649,
        'sha256': '995bce40d496c566fda7725578cb41f8bbe0848ad252a88ad6f2752d6d906c07',
    },
    'SVD/STM32F105.svd.gz': {
        'size': 37574,
        'sha256': 'ac8eaaa663d77ad65327b213d79d56b0691e8ef5d5bd714e010ccebb958afbd3',
    },
    'SVD/STM32F303.svd.gz': {
        'size': 52613,
        'sha256': '198cde683ea1a8e4f7102f4a5d5fed924051e9882177cd0731938cfe83435a4b',
    },
    'SVD/STM32F405.svd.gz': {
        'size': 78888,
        'sha256': 'b0fc9efe65831f1086c60cc5ac76ab65aea3a2e5dceb71e17c701c1a0b9101d3',
    },
    'SVD/STM32F407.svd.gz': {
        'size': 81728,
        'sha256': '54e7169a1481ef82d62a8b64f01b1ae47154d84313b57a03f205ff647eb48ee1',
    },
    'SVD/STM32F412.svd.gz': {
        'size': 59979,
        'sha256': '355689a753662e64acf934ebd18ea72825edbfa2fc3a12a0e1bda099e4c31550',
    },
    'SVD/STM32F427.svd.gz': {
        'size': 86427,
        'sha256': '1ced53d7a27f7b51c1b8b80da688a86ebe771cf2e8aee1b81592407fae716a2f',
    },
    'SVD/STM32F732.svd.gz': {
        'size': 87283,
        'sha256': 'e6f3fb9145b64a9fcfea44403110a0efb0c226dfc44df1e1be4270362f04ea77',
    },
    'SVD/STM32F767.svd.gz': {
        'size': 118748,
        'sha256': '17616e71497e77f079e1465ebfaf73389511019a01bcfdcb9d7f575f8065fdf2',
    },
    'SVD/STM32G441.svd.gz': {
        'size': 81720,
        'sha256': '14a5744d2ad8079109c6752447a2de7f999da2d9d15439d4fb35af632263808b',
    },
    'SVD/STM32G474.svd.gz': {
        'size': 118641,
        'sha256': '7cec39911505a6b23db1038d376e67b0e25ed8cfade240ddc33e385287c2d7f8',
    },
    'SVD/STM32G491.svd.gz': {
        'size': 82513,
        'sha256': 'c23dd506885baccb6d7615f833d65d8572c3785048f979a710e161d35fbf27ef',
    },
    'SVD/STM32H723.svd.gz': {
        'size': 201668,
        'sha256': '1e67df5a3c03e9b723b06b38b57a86563d72dc81d32f39b977a87e132e69d81d',
    },
    'SVD/STM32H743.svd.gz': {
        'size': 194463,
        'sha256': '65591459bc16c7ae8fb7e04b6dfe0d9a17d2b5e9cb80eb58e629b78d49a80ff7',
    },
    'SVD/STM32H757.svd.gz': {
        'size': 202699,
        'sha256': 'dc69af15ca0698382726ffa6ee52acdc23da53f01be6a92480790ecb2cc539d3',
    },
    'SVD/STM32L431.svd.gz': {
        'size': 65830,
        'sha256': '79748e22a1a878468b2712c54eae98d8f470b2aebc44105a4d7d45544fcc7f77',
    },
    'SVD/STM32L4R5.svd.gz': {
        'size': 123282,
        'sha256': 'd4a64abd8673edfd57829ad732846806f1ad5f885a39171846db0c062d42bbfc',
    },
    'SVD/STM32L4x6.svd.gz': {
        'size': 83727,
        'sha256': '83ee109be901f48c398ac8f4018b5b3fc36001b86cd8accc0cfefd1d1b05a36b',
    },
}
MCU_DATA_FILES = {
    'CKS32F407xx': {'svd': 'SVD/STM32F407.svd.gz'},
    'STM32F103xB': {'svd': 'SVD/STM32F103.svd.gz'},
    'STM32F105xC': {'svd': 'SVD/STM32F105.svd.gz'},
    'STM32F303xC': {'svd': 'SVD/STM32F303.svd.gz'},
    'STM32F405xx': {'svd': 'SVD/STM32F405.svd.gz'},
    'STM32F407xx': {'svd': 'SVD/STM32F407.svd.gz'},
    'STM32F412Rx': {'svd': 'SVD/STM32F412.svd.gz'},
    'STM32F427xx': {'svd': 'SVD/STM32F427.svd.gz'},
    'STM32F732xx': {'svd': 'SVD/STM32F732.svd.gz'},
    'STM32F767xx': {'svd': 'SVD/STM32F767.svd.gz'},
    'STM32G441xx': {'svd': 'SVD/STM32G441.svd.gz'},
    'STM32G474xx': {'svd': 'SVD/STM32G474.svd.gz'},
    'STM32G491xx': {'svd': 'SVD/STM32G491.svd.gz'},
    'STM32H723xx': {'svd': 'SVD/STM32H723.svd.gz'},
    'STM32H743xx': {'svd': 'SVD/STM32H743.svd.gz'},
    'STM32H757xx': {'svd': 'SVD/STM32H757.svd.gz'},
    'STM32L431xx': {'svd': 'SVD/STM32L431.svd.gz'},
    'STM32L476xx': {'svd': 'SVD/STM32L4x6.svd.gz'},
    'STM32L496xx': {'svd': 'SVD/STM32L4x6.svd.gz'},
    'STM32L4R5xx': {'svd': 'SVD/STM32L4R5.svd.gz'},
}


def default_cache():
    configured = os.environ.get('RENODE_DATA_CACHE')
    if configured:
        return Path(configured).expanduser()
    root = os.environ.get('XDG_CACHE_HOME')
    if root:
        return Path(root).expanduser() / 'ardupilot' / 'renode' / 'data'
    return Path.home() / '.cache' / 'ardupilot' / 'renode' / 'data'


def cache_path(cache=None):
    return Path(cache).expanduser() if cache is not None else default_cache()


def valid_cached_file(path, metadata):
    try:
        if path.stat().st_size != metadata['size']:
            return False
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        return digest.hexdigest() == metadata['sha256']
    except OSError:
        return False


def ensure_data_file(filename, cache=None, opener=None, base_url=None):
    metadata = DATA_FILES.get(filename)
    if metadata is None:
        raise ValueError('unknown Renode data file %s' % filename)
    relative = Path(filename)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('invalid Renode data file path %s' % filename)
    cache = cache_path(cache)
    destination = cache / filename
    if valid_cached_file(destination, metadata):
        return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    opener = opener or urllib.request.urlopen
    base_url = (base_url or os.environ.get('RENODE_DATA_BASE_URL') or
                DATA_DOWNLOAD_BASE)
    url = urllib.parse.urljoin(base_url.rstrip('/') + '/',
                               urllib.parse.quote(filename))
    request = urllib.request.Request(
        url, headers={'Cache-Control': 'no-cache'})
    descriptor, temporary_name = tempfile.mkstemp(
        prefix='.%s.' % destination.name, dir=destination.parent)
    os.close(descriptor)
    temporary = Path(temporary_name)
    digest = hashlib.sha256()
    received = 0
    try:
        with opener(request, timeout=60) as response, temporary.open('wb') as output:
            while True:
                block = response.read(1024 * 1024)
                if not block:
                    break
                output.write(block)
                digest.update(block)
                received += len(block)
        if received != metadata['size']:
            raise RuntimeError(
                '%s download is %u bytes; expected %u' %
                (filename, received, metadata['size']))
        if digest.hexdigest() != metadata['sha256']:
            raise RuntimeError('%s download SHA-256 does not match' % filename)
        temporary.chmod(0o644)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination


def ensure_mcu_data(mcu_type, cache=None, opener=None, base_url=None):
    return {
        role: ensure_data_file(filename, cache, opener, base_url)
        for role, filename in MCU_DATA_FILES.get(mcu_type, {}).items()
    }
