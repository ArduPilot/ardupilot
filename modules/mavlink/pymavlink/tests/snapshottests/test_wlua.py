#!/usr/bin/env python3

"""
Test that the wlua generator works in Wireshark
"""
import os, re
import shutil
import subprocess

import pytest
from pymavlink.generator import mavgen


def generate_wlua(xml_filepath, dest_dir):
    """
    Create the dissector script
    """
    lua_filepath = dest_dir / "mavlink.lua"
    assert not lua_filepath.is_file()
    mavgen.mavgen(
        mavgen.Opts(output=str(lua_filepath), language="wlua"), [xml_filepath]
    )
    assert lua_filepath.is_file()
    return lua_filepath


try:
    from syrupy import snapshot
except ImportError:

    @pytest.fixture
    def snapshot(*args, **kwargs):
        pytest.skip("syrupy snapshot plugin not available")


@pytest.mark.parametrize(
    ("mdef", "pcap"),
    [
        # simple baseline tests
        ("common.xml", "common.pcapng"),
        # tests both command_long and that we can interpret tcp packets
        ("common.xml", "command_long_over_tcp.pcapng"),
        # test that output is okay when we encounter messages not defined in the xml
        ("minimal.xml", "common.pcapng"),
        # test that 64-bit bitfield in AUTOPILOT_VERSION is correctly handled
        ("common.xml", "autopilot_version.pcapng"),
        # test showing units and decoded values
        ("common.xml", "gps_global_origin.pcapng"),
        # test command specific params in MISSION_ITEM_INT are shown
        ("common.xml", "mission_item_int.pcapng"),
        # test multipliers on eph and epv params of GPS_RAW_INT are handled
        ("common.xml", "gps_raw_int.pcapng"),
        # test custom dissector for FILE_TRANSFER_PROTOCOL
        ("common.xml", "file_transfer_protocol.pcapng"),
    ],
)
def test_wlua(request, tmp_path, snapshot, mdef, pcap):
    files = request.path.parent / "resources"
    lua_filepath = generate_wlua(files / mdef, tmp_path)

    packet_file = files / pcap
    no_such_dir = tmp_path / "empty"
    tshark = shutil.which("tshark")
    if tshark is None:
        pytest.skip("tshark (Wireshark CLI) not found")

    env = {
        # suppress system-wide wireshark config (if any)
        "WIRESHARK_CONFIG_DIR": str(no_such_dir),
        # suppress user wireshark config on macOS (and I think Linux)
        "HOME": str(no_such_dir),
    }
    try:
        # needed on Windows to initialize GnuTLS
        env["SYSTEMROOT"] = os.environ["SYSTEMROOT"]
    except KeyError:
        pass

    actual = subprocess.run(
        [
            tshark,
            "-Ttext",
            "-r" + str(packet_file),
            "-Xlua_script:" + str(lua_filepath),
            "-Omavlink_proto",
            "-n",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    # note that, with text output, tshark truncates hex dump lines at 80-ish or 100-ish characters.
    # Truncate them preemptively so this isn't reflected in the diff.
    # This doesn't lose any info we really care about.
    stdout_lines = actual.stdout.splitlines()
    for i in range(len(stdout_lines)):
        if re.search(r"[0-9a-f]{60}", stdout_lines[i]):
            stdout_lines[i] = stdout_lines[i][:72]
    truncated_stdout = '\n'.join(stdout_lines) + '\n'
    props_to_match = {
        "stdout": truncated_stdout,
        "stderr": actual.stderr,
        "returncode": actual.returncode,
    }

    assert props_to_match == snapshot()


if __name__ == "__main__":
    pytest.main([__file__])
