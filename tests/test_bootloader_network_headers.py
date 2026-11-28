# AP_FLAKE8_CLEAN
"""Host regression test for the network bootloader's HTTP header reader."""

import re
import subprocess

from pathlib import Path

import pytest

HARNESS_PREFIX = r"""
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <string>

class SocketAPM {
public:
    explicit SocketAPM(const std::string &request) : request(request) {}

    int recv(char *data, size_t length, int timeout) {
        (void)timeout;
        if (length == 0 || bytes_read == request.size()) {
            return 0;
        }
        *data = request[bytes_read++];
        return 1;
    }

    const std::string &request;
    size_t bytes_read = 0;
};

class BL_Network {
public:
    char *read_headers(SocketAPM *sock);
};

bool fail_allocation = false;

void *header_malloc(size_t size) {
    return fail_allocation ? nullptr : std::malloc(size);
}

#define malloc header_malloc
"""

HARNESS_SUFFIX = r"""
#undef malloc

int main(int argc, char **argv) {
    if (argc != 2) {
        return 2;
    }

    const std::string request(
        (std::istreambuf_iterator<char>(std::cin)),
        std::istreambuf_iterator<char>());
    SocketAPM sock(request);
    BL_Network reader;
    fail_allocation = std::strcmp(argv[1], "alloc_failure") == 0;
    char *headers = reader.read_headers(&sock);

    if (std::strcmp(argv[1], "reject") == 0 || fail_allocation) {
        if (headers != nullptr) {
            std::free(headers);
            std::fprintf(stderr, "invalid header was accepted\n");
            return 1;
        }
        if (fail_allocation && sock.bytes_read != 0) {
            std::fprintf(stderr, "read after allocation failed\n");
            return 1;
        }
        return 0;
    }

    if (headers == nullptr || sock.bytes_read != request.size() ||
        std::memcmp(headers, request.data(), request.size()) != 0 ||
        headers[request.size()] != '\0') {
        std::free(headers);
        std::fprintf(stderr, "valid header was not read\n");
        return 1;
    }
    std::free(headers);
    return 0;
}
"""


def _read_actual_reader():
    network_cpp = Path(__file__).resolve().parents[1] / "Tools/AP_Bootloader/network.cpp"
    source = network_cpp.read_text(encoding="utf-8")
    signature = re.search(r"char \*BL_Network::read_headers\(SocketAPM \*sock\)\s*\{", source)
    assert signature is not None, "could not locate BL_Network::read_headers()"

    depth = 0
    for position in range(signature.end() - 1, len(source)):
        if source[position] == "{":
            depth += 1
        elif source[position] == "}":
            depth -= 1
            if depth == 0:
                return source[signature.start():position + 1]
    raise AssertionError("could not locate the end of BL_Network::read_headers()")


@pytest.fixture(scope="module")
def header_reader(tmp_path_factory):
    build_dir = tmp_path_factory.mktemp("bootloader_headers")
    harness = build_dir / "header_reader.cpp"
    binary = build_dir / "header_reader"
    harness.write_text(HARNESS_PREFIX + _read_actual_reader() + HARNESS_SUFFIX, encoding="utf-8")
    command = [
        "g++", "-std=c++17", "-O1", "-g", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-fsanitize=address,undefined",
        "-fno-sanitize-recover=all",
        str(harness), "-o", str(binary),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    return binary


def _run_reader(binary, request, mode):
    return subprocess.run([str(binary), mode], input=request, capture_output=True, check=False)


def test_valid_header_is_read(header_reader):
    request = b"GET / HTTP/1.1\r\nHost: x\r\n\r\n"
    result = _run_reader(header_reader, request, "valid")
    assert result.returncode == 0, result.stderr.decode(errors="replace")


def test_largest_valid_header_is_read(header_reader):
    prefix = b"GET / HTTP/1.1\r\nX-Test: "
    request = prefix + b"A" * (1023 - len(prefix) - 4) + b"\r\n\r\n"
    result = _run_reader(header_reader, request, "valid")
    assert result.returncode == 0, result.stderr.decode(errors="replace")


def test_header_with_no_room_for_terminator_is_rejected(header_reader):
    prefix = b"GET / HTTP/1.1\r\nX-Test: "
    request = prefix + b"A" * (1024 - len(prefix) - 4) + b"\r\n\r\n"
    result = _run_reader(header_reader, request, "reject")
    assert result.returncode == 0, result.stderr.decode(errors="replace")


@pytest.mark.parametrize("payload", [b"GET / HTTP/1.1\r\nHost: x\r\n", b"A" * 1023])
def test_incomplete_header_is_rejected(header_reader, payload):
    result = _run_reader(header_reader, payload, "reject")
    assert result.returncode == 0, result.stderr.decode(errors="replace")


def test_allocation_failure_is_rejected(header_reader):
    request = b"GET / HTTP/1.1\r\n\r\n"
    result = _run_reader(header_reader, request, "alloc_failure")
    assert result.returncode == 0, result.stderr.decode(errors="replace")


def test_oversized_header_is_rejected(header_reader):
    request = b"GET / HTTP/1.1\r\nX-Test: " + b"A" * 2048 + b"\r\n\r\n"
    result = _run_reader(header_reader, request, "reject")
    assert result.returncode == 0, result.stderr.decode(errors="replace")
