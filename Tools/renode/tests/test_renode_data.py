#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for downloading and caching Renode model data."""

import hashlib
import importlib.util
import io

from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'renode_data.py'
SPEC = importlib.util.spec_from_file_location('renode_data', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
renode_data = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(renode_data)

GEN_BOARD_PATH = Path(__file__).resolve().parents[1] / 'gen_board.py'
GEN_BOARD_SPEC = importlib.util.spec_from_file_location(
    'gen_board_for_data_test', GEN_BOARD_PATH)
assert GEN_BOARD_SPEC is not None and GEN_BOARD_SPEC.loader is not None
gen_board = importlib.util.module_from_spec(GEN_BOARD_SPEC)
GEN_BOARD_SPEC.loader.exec_module(gen_board)


def test_mcu_data_is_verified_and_cached(tmp_path, monkeypatch):
    payload = b'test SVD data'
    monkeypatch.setattr(renode_data, 'DATA_FILES', {
        'SVD/test.svd.gz': {
            'size': len(payload),
            'sha256': hashlib.sha256(payload).hexdigest(),
        },
    })
    monkeypatch.setattr(renode_data, 'MCU_DATA_FILES', {
        'TEST_MCU': {'svd': 'SVD/test.svd.gz'},
    })
    requests = []

    def download(request, timeout):
        requests.append(request.full_url)
        assert timeout == 60
        return io.BytesIO(payload)

    paths = renode_data.ensure_mcu_data(
        'TEST_MCU', tmp_path, opener=download,
        base_url='https://example.invalid/data')

    expected = tmp_path / 'SVD' / 'test.svd.gz'
    assert paths == {'svd': expected}
    assert expected.read_bytes() == payload
    assert requests == ['https://example.invalid/data/SVD/test.svd.gz']

    def no_download(_request, _timeout):
        raise AssertionError('valid cached data should not be downloaded')

    assert renode_data.ensure_mcu_data(
        'TEST_MCU', tmp_path, opener=no_download) == {'svd': expected}

    expected.write_bytes(b'corrupt')
    assert renode_data.ensure_mcu_data(
        'TEST_MCU', tmp_path, opener=download) == {'svd': expected}
    assert expected.read_bytes() == payload
    assert len(requests) == 2


def test_unknown_mcu_does_not_create_cache(tmp_path):
    cache = tmp_path / 'data'

    assert renode_data.ensure_mcu_data('UNKNOWN_MCU', cache) == {}
    assert not cache.exists()


@pytest.mark.parametrize(('mcu_type', 'filename'), [
    ('CKS32F407xx', 'SVD/STM32F407.svd.gz'),
    ('STM32F103xB', 'SVD/STM32F103.svd.gz'),
    ('STM32F105xC', 'SVD/STM32F105.svd.gz'),
    ('STM32F303xC', 'SVD/STM32F303.svd.gz'),
    ('STM32F405xx', 'SVD/STM32F405.svd.gz'),
    ('STM32F407xx', 'SVD/STM32F407.svd.gz'),
    ('STM32F412Rx', 'SVD/STM32F412.svd.gz'),
    ('STM32F427xx', 'SVD/STM32F427.svd.gz'),
    ('STM32F732xx', 'SVD/STM32F732.svd.gz'),
    ('STM32F767xx', 'SVD/STM32F767.svd.gz'),
    ('STM32G441xx', 'SVD/STM32G441.svd.gz'),
    ('STM32G474xx', 'SVD/STM32G474.svd.gz'),
    ('STM32G491xx', 'SVD/STM32G491.svd.gz'),
    ('STM32H723xx', 'SVD/STM32H723.svd.gz'),
    ('STM32H743xx', 'SVD/STM32H743.svd.gz'),
    ('STM32H757xx', 'SVD/STM32H757.svd.gz'),
    ('STM32L431xx', 'SVD/STM32L431.svd.gz'),
    ('STM32L476xx', 'SVD/STM32L4x6.svd.gz'),
    ('STM32L496xx', 'SVD/STM32L4x6.svd.gz'),
    ('STM32L4R5xx', 'SVD/STM32L4R5.svd.gz'),
])
def test_mcu_uses_matching_svd(monkeypatch, mcu_type, filename):
    def resolve(requested, _cache, _opener, _base_url):
        return Path(requested)

    monkeypatch.setattr(renode_data, 'ensure_data_file', resolve)

    assert renode_data.ensure_mcu_data(mcu_type, Path('cache')) == {
        'svd': Path(filename),
    }


def test_every_supported_mcu_has_data_mapping():
    assert set(renode_data.MCU_DATA_FILES) == set(gen_board.FAMILIES)
    referenced = {
        filename
        for files in renode_data.MCU_DATA_FILES.values()
        for filename in files.values()
    }
    assert referenced == set(renode_data.DATA_FILES)


def test_every_supported_mcu_script_applies_cached_svd():
    scripts = {
        family['script']
        for family in gen_board.FAMILIES.values()
    }
    for script in scripts:
        text = (MODULE_PATH.parent / 'scripts' / script).read_text()
        assert 'sysbus ApplySVD $mcu_svd' in text


def test_bad_download_is_not_cached(tmp_path, monkeypatch):
    expected = b'expected data'
    monkeypatch.setattr(renode_data, 'DATA_FILES', {
        'SVD/test.svd.gz': {
            'size': len(expected),
            'sha256': hashlib.sha256(expected).hexdigest(),
        },
    })

    def corrupt_download(_request, timeout):
        assert timeout == 60
        return io.BytesIO(b'x' * len(expected))

    with pytest.raises(RuntimeError, match='SHA-256 does not match'):
        renode_data.ensure_data_file(
            'SVD/test.svd.gz', tmp_path, opener=corrupt_download)

    assert not (tmp_path / 'SVD' / 'test.svd.gz').exists()
    assert not [path for path in tmp_path.rglob('*') if path.is_file()]
