# Wireshark Mavlink Telemetry Log (TLOG) file reader

This is a Wireshark LUA plugin, which allows Mavlink TLOG files to be read in Wireshark.
TLOG files are typically created by tools such as [MavProxy](https://ardupilot.org/mavproxy/),
[Mission Planner](https://ardupilot.org/planner/) and others, to provide a log of the
Mavlink communication between the tool and the attached vehicle.

Note that this LUA script provides the capability to read TLOG files, while the message
decoding is provided by the [MAVLink Lua Plugin](https://mavlink.io/en/guide/wireshark.html)
which should also be installed to make use of this plugin.

## Installation

To install this plugin, download the [mavlink-tlog-reader.lua](mavlink-tlog-reader.lua) file,
and place it in your LUA plugins folder.
You can find your global and personal LUA plugins folders from the Folders tab on the
Wireshark About dialog.

Follow the build and installation instructions on the [MAVLink Lua Plugin](https://mavlink.io/en/guide/wireshark.html)
page, to install the associated Mavlink message decoder.

## Usage

Within the Wireshark GUI, select File/Open, and choose any .tlog file to load. Note that
the default "All Capture Files" filter does not include .tlog files, so you will need to
select "All Files" to see them. The file should then load, showing all Mavlink messages
recorded in the file.

Note that if the MAVLink Lua Plugin is not also installed, then messages with show with
"UNKNOWN" in the protocol column, and "WTAP_ENCAP = 45" in the Info column.

## File Format

TLOG files consist of a stream of [Mavlink](https://mavlink.io/en/) messages, including
their complete [header, checksum and signature](https://mavlink.io/en/guide/serialization.html)
(if applicable). Each message is preceeded by a 8-byte [Unix time](https://www.unixtimestamp.com/)
timestamp in microseconds.
