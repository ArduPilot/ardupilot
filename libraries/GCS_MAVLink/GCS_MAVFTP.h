/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"


#ifndef HAL_WITH_MAVFTP
  #if HAL_OS_POSIX_IO || HAL_OS_FATFS_IO
    #define HAL_WITH_MAVFTP (!HAL_MINIMIZE_FEATURES)
  #else
    #define HAL_WITH_MAVFTP 0
  #endif
#endif

#define MAX_FTP_DATA_LENGTH (MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - 12)

#if HAL_WITH_MAVFTP
class GCS_MAVFTP {
    //MAVFTP Private members
    struct {
        uint8_t		stream_target_system_id;
        bool		stream_download;
        int		    fd;
        unsigned	stream_chunk_transmitted;
        uint16_t	stream_seq_number;
        uint32_t	stream_offset;
        uint32_t	file_size;
    } _session_info;

    char *_ftp_current_abs_path;
    /// This is the payload which is in mavlink_file_transfer_protocol_t.payload.
    /// This needs to be packed, because it's typecasted from mavlink_file_transfer_protocol_t.payload, which starts
    /// at a 3 byte offset, causing an unaligned access to seq_number and offset
    struct __attribute__((__packed__)) FTPPayloadHeader {
        uint16_t	seq_number;	///< sequence number for message
        uint8_t		session;	///< Session id for read and write commands
        uint8_t		opcode;		///< Command opcode
        uint8_t		size;		///< Size of data
        uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
        uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
        uint8_t		padding;        ///< 32 bit aligment padding
        uint32_t	offset;		///< Offsets for List and Read commands
        uint8_t		data[MAX_FTP_DATA_LENGTH];		///< command data, varies by Opcode
    };

    /// Command opcodes
    enum FTPOpcode : uint8_t {
        kCmdNone,		///< ignored, always acked
        kCmdTerminateSession,	///< Terminates open Read session
        kCmdResetSessions,	///< Terminates all open Read sessions
        kCmdListDirectory,	///< List files in <path> from <offset>
        kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
        kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
        kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
        kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
        kCmdRemoveFile,		///< Remove file at <path>
        kCmdCreateDirectory,	///< Creates directory at <path>
        kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
        kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
        kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
        kCmdRename,		///< Rename <path1> to <path2>
        kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
        kCmdBurstReadFile,	///< Burst download session file

        kRspAck = 128,		///< Ack response
        kRspNak			///< Nak response
    };

    /// @brief Error codes returned in Nak response FTPPayloadHeader.data[0].
    enum FTPErrorCode : uint8_t {
        kErrNone,
        kErrFail,			///< Unknown failure
        kErrFailErrno,			///< Command failed, errno sent back in FTPPayloadHeader.data[1]
        kErrInvalidDataSize,		///< FTPPayloadHeader.size is invalid
        kErrInvalidSession,		///< Session is not currently open
        kErrNoSessionsAvailable,	///< All available Sessions in use
        kErrEOF,			///< Offset past end of file for List and Read commands
        kErrUnknownCommand,		///< Unknown command opcode
        kErrFailFileExists,		///< File exists already
        kErrFailFileProtected,		///< File is write protected
        kErrBusy
    };

    mavlink_file_transfer_protocol_t _ftp_response;
    mavlink_file_transfer_protocol_t _ftp_request;

    FTPErrorCode	_ftp_List(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_Open(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload, int oflag);
    FTPErrorCode	_ftp_Read(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_Burst(FTPPayloadHeader *request_payload, uint8_t target_system_id);
    FTPErrorCode	_ftp_Write(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_Terminate(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_Reset(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_RemoveDirectory(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_CreateDirectory(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_RemoveFile(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_TruncateFile(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_Rename(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    FTPErrorCode	_ftp_CalcFileCRC32(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload);
    char *_data_as_cstring(FTPPayloadHeader *payload);
    bool _set_current_abs_path(char* rel_path);
    void _ftp_burst_send(void);
    void _ftp_timer(void);
    void _ftp_loop(void);
    void _ftp_wait_until_buffer_free(FTPPayloadHeader *payload);

    static int8_t _ftp_open_on_iface;
    int _ftp_errno;
    uint8_t _ftp_target_system_id = 255;
    bool _ftp_timer_registered;
    bool _last_pkt_resp_pend;
    uint32_t _ftp_stream_chunk_size;
    HAL_Semaphore _ftp_sem;
    bool _ftp_initialised;
    mavlink_channel_t _ftp_chan;
public:
    void handle_ftp_messages(mavlink_channel_t chan, mavlink_message_t *msg);
    void ftp_init(void);
};
#endif
