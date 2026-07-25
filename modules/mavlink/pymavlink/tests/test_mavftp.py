#!/usr/bin/env python3

'''
MAVLink File Transfer Protocol support test - https://mavlink.io/en/services/ftp.html

SPDX-FileCopyrightText: 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
'''

import unittest
#from unittest.mock import patch
from io import StringIO
import logging
from pymavlink import mavutil
from pymavlink.mavftp import FTP_OP, MAVFTP, MAVFTPReturn
from pymavlink.mavftp import FtpError
from pymavlink.mavftp import OP_ListDirectory
from pymavlink.mavftp import OP_ReadFile
from pymavlink.mavftp import OP_Ack
from pymavlink.mavftp import OP_Nack

class TestMAVFTPPayloadDecoding(unittest.TestCase):
    """Test MAVFTP payload decoding"""

    def setUp(self):
        self.log_stream = StringIO()
        handler = logging.StreamHandler(self.log_stream)
        formatter = logging.Formatter('%(levelname)s: %(message)s')
        handler.setFormatter(formatter)
        logger = logging.getLogger()
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

        # Mock mavutil.mavlink_connection to simulate a connection
        self.mock_master = mavutil.mavlink_connection(device="udp:localhost:14550", source_system=1)

        # Initialize MAVFTP instance for testing
        self.mav_ftp = MAVFTP(self.mock_master, target_system=1, target_component=1)

    def tearDown(self):
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

    def test_logging(self):
        # Code that triggers logging
        logging.info("This is a test log message")

        # Flush and get log output
        log_output = self.log_stream.getvalue()

        # Assert to check if the expected log is in log_output
        self.assertIn("This is a test log message", log_output)

    @staticmethod
    def ftp_operation(seq: int, opcode: int, req_opcode: int, payload: bytearray) -> FTP_OP:
        return FTP_OP(seq=seq, session=1, opcode=opcode, size=0, req_opcode=req_opcode, burst_complete=0, offset=0,
                      payload=payload)

    def test_decode_ftp_ack_and_nack(self):
        # Test cases grouped by expected outcome
        # pylint: disable=line-too-long
        test_cases = [
            {
                "name": "Successful Operation",
                "op": self.ftp_operation(seq=1, opcode=OP_Ack, req_opcode=OP_ListDirectory, payload=None),
                "expected_message": "ListDirectory succeeded"
            },
            {
                "name": "Generic Failure",
                "op": self.ftp_operation(seq=2, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.Fail])),
                "expected_message": "ListDirectory failed, generic error"
            },
            {
                "name": "System Error",
                "op": self.ftp_operation(seq=3, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FailErrno, 1])),  # System error 1
                "expected_message": "ListDirectory failed, system error 1"
            },
            {
                "name": "Invalid Data Size",
                "op": self.ftp_operation(seq=4, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.InvalidDataSize])),
                "expected_message": "ListDirectory failed, invalid data size"
            },
            {
                "name": "Invalid Session",
                "op": self.ftp_operation(seq=5, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.InvalidSession])),
                "expected_message": "ListDirectory failed, session is not currently open"
            },
            {
                "name": "No Sessions Available",
                "op": self.ftp_operation(seq=6, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.NoSessionsAvailable])),
                "expected_message": "ListDirectory failed, no sessions available"
            },
            {
                "name": "End of File",
                "op": self.ftp_operation(seq=7, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.EndOfFile])),
                "expected_message": "ListDirectory failed, offset past end of file"
            },
            {
                "name": "Unknown Command",
                "op": self.ftp_operation(seq=8, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.UnknownCommand])),
                "expected_message": "ListDirectory failed, unknown command"
            },
            {
                "name": "File Exists",
                "op": self.ftp_operation(seq=9, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FileExists])),
                "expected_message": "ListDirectory failed, file/directory already exists"
            },
            {
                "name": "File Protected",
                "op": self.ftp_operation(seq=10, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FileProtected])),
                "expected_message": "ListDirectory failed, file/directory is protected"
            },
            {
                "name": "File Not Found",
                "op": self.ftp_operation(seq=11, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FileNotFound])),
                "expected_message": "ListDirectory failed, file/directory not found"
            },
            {
                "name": "No Error Code in Payload",
                "op": self.ftp_operation(seq=12, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=None),
                "expected_message": "ListDirectory failed, payload contains no error code"
            },
            {
                "name": "No Error Code in Nack",
                "op": self.ftp_operation(seq=13, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.Success])),
                "expected_message": "ListDirectory failed, no error code"
            },
            {
                "name": "No Filesystem Error in Payload",
                "op": self.ftp_operation(seq=14, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FailErrno])),
                "expected_message": "ListDirectory failed, file-system error missing in payload"
            },
            {
                "name": "Invalid Error Code",
                "op": self.ftp_operation(seq=15, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.InvalidErrorCode])),
                "expected_message": "ListDirectory failed, invalid error code"
            },
            {
                "name": "Payload Too Large",
                "op": self.ftp_operation(seq=16, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([0, 0, 0])),
                "expected_message": "ListDirectory failed, payload is too long"
            },
            {
                "name": "Invalid Opcode",
                "op": self.ftp_operation(seq=17, opcode=126, req_opcode=OP_ListDirectory, payload=None),
                "expected_message": "ListDirectory failed, invalid opcode 126"
            },
            {
                "name": "Unknown Opcode in Request",
                "op": self.ftp_operation(seq=19, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.UnknownCommand])),  # Assuming 100 is an unknown opcode
                "expected_message": "ListDirectory failed, unknown command"
            },
            {
                "name": "Payload with System Error",
                "op": self.ftp_operation(seq=20, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([FtpError.FailErrno, 2])),  # System error 2
                "expected_message": "ListDirectory failed, system error 2"
            },
            {
                "name": "Invalid Error Code in Payload",
                "op": self.ftp_operation(seq=21, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([105])),  # Assuming 105 is an invalid error code
                "expected_message": "ListDirectory failed, invalid error code 105"
            },
            {
                "name": "Invalid Opcode with Payload",
                "op": self.ftp_operation(seq=23, opcode=126, req_opcode=OP_ReadFile, payload=bytes([1, 1])),  # Invalid opcode with payload
                "expected_message": "ReadFile failed, invalid opcode 126"
            },
            # Add more test cases as needed...
        ]
        # pylint: enable=line-too-long

        for case in test_cases:
            ret = self.mav_ftp._MAVFTP__decode_ftp_ack_and_nack(case['op'])  # pylint: disable=protected-access
            ret.display_message()
            log_output = self.log_stream.getvalue().strip()
            self.assertIn(case["expected_message"], log_output,
                          f"Test {case['name']}: Expected {case['expected_message']} but got {log_output}")
            self.log_stream.seek(0)
            self.log_stream.truncate(0)

        # Invalid Arguments
        ret = MAVFTPReturn("Command arguments", FtpError.InvalidArguments)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Command arguments failed, invalid arguments", log_output, "Expected invalid arguments message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Test for unknown error code in display_message
        op = self.ftp_operation(seq=22, opcode=OP_Nack, req_opcode=OP_ListDirectory, payload=bytes([255]))
        ret = self.mav_ftp._MAVFTP__decode_ftp_ack_and_nack(op, "ListDirectory")  # pylint: disable=protected-access
        ret.error_code = 125  # Set error code to 125 to trigger unknown error message
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("ListDirectory failed, unknown error 125 in display_message()", log_output,
                      "Expected unknown error message for unknown error code")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Put already in progress
        ret = MAVFTPReturn("Put", FtpError.PutAlreadyInProgress)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Put failed, put already in progress", log_output, "Expected put already in progress message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Fail to open local file
        ret = MAVFTPReturn("Put", FtpError.FailToOpenLocalFile)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Put failed, failed to open local file", log_output, "Expected fail to open local file message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)

        # Remote Reply Timeout
        ret = MAVFTPReturn("Put", FtpError.RemoteReplyTimeout)
        ret.display_message()
        log_output = self.log_stream.getvalue().strip()
        self.assertIn("Put failed, remote reply timeout", log_output, "Expected remote reply timeout message")
        self.log_stream.seek(0)
        self.log_stream.truncate(0)


if __name__ == '__main__':
    unittest.main()
