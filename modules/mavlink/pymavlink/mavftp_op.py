#!/usr/bin/env python3

"""
MAVLink File Transfer Protocol support - https://mavlink.io/en/services/ftp.html

SPDX-FileCopyrightText: 2011-2024 Andrew Tridgell, 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
"""

# FLAKE_CLEAN

import struct
from typing import Generator, Optional, Tuple, Union


# pylint: disable=invalid-name

# opcodes
OP_None = 0
OP_TerminateSession = 1
OP_ResetSessions = 2
OP_ListDirectory = 3
OP_OpenFileRO = 4
OP_ReadFile = 5
OP_CreateFile = 6
OP_WriteFile = 7
OP_RemoveFile = 8
OP_CreateDirectory = 9
OP_RemoveDirectory = 10
OP_OpenFileWO = 11
OP_TruncateFile = 12
OP_Rename = 13
OP_CalcFileCRC32 = 14
OP_BurstReadFile = 15
OP_Ack = 128
OP_Nack = 129


class FTP_OP:  # pylint: disable=invalid-name, too-many-instance-attributes
    """
    Represents an operation in the MAVLink File Transfer Protocol (FTP).

    This class encapsulates the details of a single FTP operation such as reading, writing, listing directories, etc.,
    including the necessary parameters and payload for the operation.
    """

    def __init__(  # pylint: disable=too-many-arguments
        self,
        seq: int,
        session: int,
        opcode: int,
        size: int,
        req_opcode: int,
        burst_complete: int,
        offset: int,
        payload: Optional[bytearray] = None,
    ) -> None:
        self.seq: int = seq  # Sequence number for the operation.
        self.session: int = session  # Session identifier.
        self.opcode: int = opcode  # Operation code indicating the type of FTP operation.
        self.size: int = size  # Size of the operation.
        self.req_opcode: int = req_opcode  # Request operation code.
        self.burst_complete: int = burst_complete  # (int) Flag indicating if the burst transfer is complete.
        self.offset: int = offset  # Offset for read/write operations.
        self.payload: Optional[bytearray] = payload  # (bytes) Payload for the operation.

    def pack(self) -> bytearray:
        """Pack message."""
        ret = struct.pack(
            "<HBBBBBBI",
            self.seq,
            self.session,
            self.opcode,
            self.size,
            self.req_opcode,
            self.burst_complete,
            0,
            self.offset,
        )
        if self.payload is not None:
            ret += self.payload
        return bytearray(ret)

    def __str__(self) -> str:
        """String representation of the FTP_OP."""
        plen = 0
        if self.payload is not None:
            plen = len(self.payload)
        ret = f"OP seq:{self.seq} sess:{self.session} opcode:{self.opcode} req_opcode:{self.req_opcode} size:{self.size}" \
              f" bc:{self.burst_complete} ofs:{self.offset} plen={plen}"
        if self.payload is not None and plen > 0:
            ret += f" [{self.payload[0]}]"
        return ret

    def items(self) -> Generator[Tuple[str, Union[int, bool, bytes, Optional[bytearray]]], None, None]:
        """Yield each attribute and its value for the FTP_OP instance. For debugging purposes."""
        yield "seq", self.seq
        yield "session", self.session
        yield "opcode", self.opcode
        yield "size", self.size
        yield "req_opcode", self.req_opcode
        yield "burst_complete", self.burst_complete
        yield "offset", self.offset
        yield "payload", self.payload
