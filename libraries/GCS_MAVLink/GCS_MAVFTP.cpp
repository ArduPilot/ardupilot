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
/*
  handle FTP operations over MAVLink
 */

#include <AP_HAL/AP_HAL.h>
#include "GCS_MAVFTP.h"
#include <AP_Math/crc.h>
#include "GCS.h"

#if HAL_WITH_MAVFTP

#if HAL_OS_POSIX_IO
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#endif

#if HAL_OS_FATFS_IO
#include <stdio.h>
#endif


extern const AP_HAL::HAL& hal;

static const char   kDirentFile = 'F';	///< Identifies File returned from List command
static const char   kDirentDir = 'D';	///< Identifies Directory returned from List command
static const char   kDirentSkip = 'S';	///< Identifies Skipped entry from List command
#if 1
#define debug(str) hal.console->printf("%s\n", str)
#else
#define debug(str)
#endif

int8_t GCS_MAVFTP::_ftp_open_on_iface = -1;

#define ABS_PATH_MAX_LEN (100U + sizeof(HAL_BOARD_STORAGE_DIRECTORY))

void GCS_MAVFTP::ftp_init()
{
    _ftp_current_abs_path = new char[ABS_PATH_MAX_LEN];
    if (_ftp_current_abs_path == nullptr) {
        return;
    }
    strcpy(_ftp_current_abs_path, HAL_BOARD_STORAGE_DIRECTORY);
    _session_info.fd = -1;
    _ftp_initialised = true;
}

char *GCS_MAVFTP::_data_as_cstring(FTPPayloadHeader *payload)
{
    // guarantee null termination
    // even if the payload length is greater than MAX_FTP_DATA_LENGTH
    if (payload->size < MAX_FTP_DATA_LENGTH) {
        ((char*)payload->data)[payload->size] = '\0';
    } else {
        payload->data[MAX_FTP_DATA_LENGTH - 1] = '\0';
    }

    // and return data
    return (char *) & (payload->data[0]);
}

bool GCS_MAVFTP::_set_current_abs_path(char* rel_path)
{
    if (rel_path[0] == '/' && rel_path[1] == '\0') {
        _ftp_current_abs_path[strlen(HAL_BOARD_STORAGE_DIRECTORY)] = '\0';
    } else if (strlen(rel_path) < (ABS_PATH_MAX_LEN-strlen(HAL_BOARD_STORAGE_DIRECTORY))) {
        strcpy(&_ftp_current_abs_path[strlen(HAL_BOARD_STORAGE_DIRECTORY)], rel_path);
    }  else {
        return false;
    }
    return true;
}

void GCS_MAVFTP::handle_ftp_messages(mavlink_channel_t chan, mavlink_message_t *msg)
{
    if (!_ftp_initialised) {
        gcs().send_statustext(MAV_SEVERITY_ERROR, 1 << chan, "MAVFTP Failed to intialise.");
        return;
    }

    if (!_ftp_timer_registered) {
        //First FTP command received, register MAV FTP worker method
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&GCS_MAVFTP::_ftp_loop, void), "MAVFTP", 768, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
            gcs().send_statustext(MAV_SEVERITY_ERROR, 1 << chan, "MAVFTP: Thread Create Failed.");
            return;
        }
        _ftp_timer_registered = true;
    }
    if (!_last_pkt_resp_pend) {
        if (_ftp_sem.take_nonblocking()) {  //Do a nonblocking semaphore take, to ensure ftp worker is not active
            //Update ftp_request structure
            mavlink_msg_file_transfer_protocol_decode(msg, &_ftp_request);
            _ftp_target_system_id = msg->sysid;
            _ftp_sem.give();
            _last_pkt_resp_pend = true;
            _ftp_chan  = chan;
            return;
        }
    }
    //Last Packet is still being processed, send a NACK
    mavlink_file_transfer_protocol_t ftp_request;
    mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);
    FTPPayloadHeader *payload = reinterpret_cast<FTPPayloadHeader *>(&ftp_request.payload[0]);
    payload->req_opcode = payload->opcode;
    payload->opcode = kRspNak;
    payload->size = 1;
    memset(payload->data, 0, MAX_FTP_DATA_LENGTH);
    payload->data[0] = kErrBusy;
    ftp_request.target_system = msg->sysid;
    //Send Error Response
    mavlink_msg_file_transfer_protocol_send_struct(chan, &ftp_request);
}

void GCS_MAVFTP::_ftp_loop()
{
    while(true) {
        if (_last_pkt_resp_pend) {
            _ftp_timer();
        } else {
            _ftp_burst_send();
        }
        hal.scheduler->delay(1);
    }
}

//MAV FTP worker method, We do all FTP related processing and response in IO thread
void GCS_MAVFTP::_ftp_timer()
{
    _ftp_sem.take(0);
    FTPPayloadHeader *payload = reinterpret_cast<FTPPayloadHeader *>(&_ftp_request.payload[0]);
    FTPPayloadHeader *response_payload = reinterpret_cast<FTPPayloadHeader *>(&_ftp_response.payload[0]);
    FTPErrorCode errorCode = kErrNone;
    _ftp_errno = 0;
    // basic sanity checks; must validate length before use
    if (payload->size > MAX_FTP_DATA_LENGTH) {
        errorCode = kErrInvalidDataSize;
        goto out;
    }

    // check the sequence number: if this is a resent request, resend the last response
    if (payload->seq_number + 1 == response_payload->seq_number) {
        // this is the same request as the one we replied to last. It means the (n)ack got lost, and the GCS
        // re-send the request
        // We take over the transmit link when sending FTP message,
        // To avoid buffer corruption
        GCS_MAVLINK::lock_channel(_ftp_chan, false, true);
        _ftp_wait_until_buffer_free(response_payload);
        mavlink_msg_file_transfer_protocol_send_struct(_ftp_chan, &_ftp_response);
        GCS_MAVLINK::lock_channel(_ftp_chan, false, false);
        _last_pkt_resp_pend = false;
        _ftp_sem.give();
        return;
    }
    memset(response_payload->data,0,239);
    switch (payload->opcode) {
    case kCmdNone:
        break;

    case kCmdTerminateSession:
        errorCode = _ftp_Terminate(payload, response_payload);
        break;

    case kCmdResetSessions:
        errorCode = _ftp_Reset(payload, response_payload);
        break;

    case kCmdListDirectory:
        errorCode = _ftp_List(payload, response_payload);
        break;

    case kCmdOpenFileRO:
        errorCode = _ftp_Open(payload, response_payload, O_RDONLY);
        break;

    case kCmdCreateFile:
        errorCode = _ftp_Open(payload, response_payload, O_CREAT | O_EXCL | O_WRONLY);
        break;

    case kCmdOpenFileWO:
        errorCode = _ftp_Open(payload, response_payload, O_CREAT | O_WRONLY);
        break;

    case kCmdReadFile:
        errorCode = _ftp_Read(payload, response_payload);
        break;

    case kCmdBurstReadFile:
        response_payload->offset = 0;
        response_payload->size = 0;
        response_payload->burst_complete = false;
        errorCode = _ftp_Burst(payload, _ftp_target_system_id);
        break;

    case kCmdWriteFile:
        errorCode = _ftp_Write(payload, response_payload);
        break;

    case kCmdRemoveFile:
        errorCode = _ftp_RemoveFile(payload, response_payload);
        break;

    case kCmdRename:
        errorCode = _ftp_Rename(payload, response_payload);
        break;

    case kCmdTruncateFile:
        errorCode = _ftp_TruncateFile(payload, response_payload);
        break;

    case kCmdCreateDirectory:
        errorCode = _ftp_CreateDirectory(payload, response_payload);
        break;

    case kCmdRemoveDirectory:
        errorCode = _ftp_RemoveDirectory(payload, response_payload);
        break;

    case kCmdCalcFileCRC32:
        errorCode = _ftp_CalcFileCRC32(payload, response_payload);
        break;

    default:
        errorCode = kErrUnknownCommand;
        break;
    }

out:
    response_payload->seq_number = payload->seq_number + 1;

    // handle success vs. error
    if (errorCode == kErrNone) {
        response_payload->req_opcode = payload->opcode;
        response_payload->opcode = kRspAck;
    } else {
        memset(response_payload->data,0,239);
        response_payload->req_opcode = payload->opcode;
        response_payload->opcode = kRspNak;
        response_payload->size = 1;
        response_payload->data[0] = errorCode;
        if (errorCode == kErrFailErrno) {
            response_payload->size = 2;
            response_payload->data[1] = _ftp_errno;
        }
    }

    //Send Response
    _ftp_response.target_system = _ftp_target_system_id;

    // We take over the transmit link when sending FTP message,
    // To avoid buffer corruption
    GCS_MAVLINK::lock_channel(_ftp_chan, false, true);
    _ftp_wait_until_buffer_free(response_payload);
    //Lock Channel for transmit so that other mavlink packets don't get in the way
    mavlink_msg_file_transfer_protocol_send_struct(_ftp_chan, &_ftp_response);
    GCS_MAVLINK::lock_channel(_ftp_chan, false, false);

    _last_pkt_resp_pend = false;
    _ftp_sem.give();
}

void GCS_MAVFTP::_ftp_wait_until_buffer_free(FTPPayloadHeader *payload)
{
    //Ensure the Buffer is free enough for transmitting FTP message,
    //We never drop a response due to lack of buffer length, we flush it
    while (mavlink_comm_port[_ftp_chan]->txspace() <= (uint32_t)(PAYLOAD_SIZE(_ftp_chan, FILE_TRANSFER_PROTOCOL) - (MAX_FTP_DATA_LENGTH - payload->size) + PAYLOAD_SIZE(_ftp_chan, HEARTBEAT))) {
        hal.scheduler->delay(10);
    }
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_List(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    FTPErrorCode errorCode = kErrNone;
    unsigned offset = 0;
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }
    DIR *dp = opendir(_ftp_current_abs_path);

    if (dp == nullptr) {
        debug("FTP: can't open path (file system corrupted?)");
        debug(_ftp_current_abs_path);
        _ftp_errno = errno;
        // this is not an FTP error, abort directory by simulating eof
        return kErrEOF;
    }

    struct dirent *result = nullptr;

    // move to the requested offset
    int requested_offset = request_payload->offset;
    response_payload->offset = request_payload->offset;
    while (requested_offset-- > 0) {
        if (readdir(dp) == nullptr) {
            _ftp_errno = errno;
            closedir(dp);
            return kErrFailErrno;
        }
    }

    while ((offset + 2) < MAX_FTP_DATA_LENGTH) {
        errno = 0;
        result = readdir(dp);
        // read the directory entry
        if (result == nullptr) {
            if (errno) {
                debug("FTP: list readdir failure");
                debug(_ftp_current_abs_path);
                response_payload->data[offset++] = kDirentSkip;
                *((char *)&response_payload->data[offset]) = '\0';
                offset++;
                response_payload->size = offset;
                _ftp_errno = errno;
                closedir(dp);

                return kErrFailErrno;
            }

            // no more entries?
            if (request_payload->offset != 0 && offset == 0) {
                // User is requesting subsequent dir entries but there were none. This means the user asked
                // to seek past EOF.
                errorCode = kErrEOF;
            }

            // Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
            break;
        }

        uint64_t fileSize = 0;


        // Do we have room for the name, the one char directory identifier and the null terminator?
        int remaining_space = MAX_FTP_DATA_LENGTH - 2 - offset;
        if ((((unsigned)remaining_space) < strlen(result->d_name)) || (remaining_space <= 0)) {
            break;
        }
        // Determine the directory entry type
        if (result->d_type == DT_REG) {
            // For files we get the file size as well
            response_payload->data[offset++] = kDirentFile;
            int ret = snprintf((char*)&response_payload->data[offset], remaining_space, "%s/%s", _ftp_current_abs_path, result->d_name);
            bool buf_is_ok = ((ret > 0) && (ret < remaining_space));

            if (buf_is_ok) {
                struct stat st;

                if (stat((char*)&response_payload->data[offset], &st) == 0) {
                    fileSize = st.st_size;
                } else {
                    //skip file
                    offset--;
                    continue;
                }
            } else if (!buf_is_ok && (ret < (int)MAX_FTP_DATA_LENGTH)) {
                //do retry
                offset--;
                break;
            } else if (!buf_is_ok) {
                //skip file
                offset--;
                continue;
            }
            
        } else if (result->d_type == DT_DIR) {
            response_payload->data[offset++] = kDirentDir;
        } else {
            // We only send back file and diretory entries, skip everything else
            response_payload->data[offset++] = kDirentSkip;
        }

        if (response_payload->data[offset - 1] == kDirentSkip) {
            // Skip send only dirent identifier
            response_payload->data[offset++] = '\0';
        } else if (response_payload->data[offset - 1] == kDirentFile) {
            // Files send filename and file length
            int ret = snprintf((char*)&response_payload->data[offset], remaining_space, "%s\t%llu", result->d_name, fileSize);
            bool buf_is_ok = ((ret > 0) && (ret < remaining_space));
            if (!buf_is_ok && (ret < (int)MAX_FTP_DATA_LENGTH)) {
                //do retry
                offset--;
                break;
            } else if (!buf_is_ok) {
                //skip file
                offset--;
                continue;
            }
            offset += MIN(ret, remaining_space);
            response_payload->data[offset++] = '\0';
        } else {
            // Everything else just sends name
            strncpy((char*)&response_payload->data[offset], result->d_name, remaining_space);
            response_payload->data[MAX_FTP_DATA_LENGTH - 1] = '\0';
            offset += MIN(strlen(result->d_name), (unsigned)remaining_space);
            response_payload->data[offset++] = '\0';
        }
    }
    closedir(dp);
    response_payload->size = offset;
    _ftp_errno = errno;
    return errorCode;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Open(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload, int oflag)
{
    if (_session_info.fd >= 0) {
        return kErrNoSessionsAvailable;
    }

    uint64_t fileSize = 0;
    struct stat st;
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }

    if (stat(_ftp_current_abs_path, &st)) {
        // fail only if requested open for read
        if (oflag & O_RDONLY) {
            _ftp_errno = errno;
            return kErrFailErrno;
        } else {
            st.st_size = 0;
        }
    }
    fileSize = st.st_size;
    if ( fileSize > 0x7FFFFFFF ) {
        return kErrFail;
    }
#if HAL_OS_POSIX_IO
    int fd = ::open(_ftp_current_abs_path, oflag, 0777);
#else
    int fd = ::open(_ftp_current_abs_path, oflag);
#endif
    if (fd < 0) {
        _ftp_errno = errno;
        return kErrFailErrno;
    }

    _session_info.fd = fd;
    _session_info.file_size = fileSize;
    _session_info.stream_download = false;

    response_payload->session = 0;
    response_payload->size = sizeof(uint32_t);
    memcpy(response_payload->data, &fileSize, response_payload->size);
    return kErrNone;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Read(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (request_payload->session != 0 || _session_info.fd < 0) {
        return kErrInvalidSession;
    }
    response_payload->offset = 0;
    // We have to test seek past EOF ourselves, lseek will allow seek past EOF
    if (request_payload->offset >= _session_info.file_size) {
        debug("request past EOF");
        return kErrEOF;
    }

    if (lseek(_session_info.fd, request_payload->offset, SEEK_SET) < 0) {
        debug("seek fail");
        _ftp_errno = errno;
        return kErrFailErrno;
    }

    int bytes_read = ::read(_session_info.fd, &response_payload->data[0], MAX_FTP_DATA_LENGTH);

    if (bytes_read < 0) {
        // Negative return indicates error other than eof
        debug("read fail");
        _ftp_errno = errno;
        return kErrFailErrno;
    }

    response_payload->size = bytes_read;
    response_payload->offset = request_payload->offset;
    return kErrNone;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Burst(FTPPayloadHeader *request_payload, uint8_t target_system_id)
{
    if (request_payload->session != 0 && _session_info.fd < 0) {
        return kErrInvalidSession;
    }
    // Setup for streaming sends
    _session_info.stream_download = true;
    _session_info.stream_offset = request_payload->offset;
    _session_info.stream_chunk_transmitted = 0;
    _session_info.stream_seq_number = request_payload->seq_number + 2;
    _session_info.stream_target_system_id = target_system_id;
    return kErrNone;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Write(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (request_payload->session != 0 && _session_info.fd < 0) {
        return kErrInvalidSession;
    }
    response_payload->offset = request_payload->offset;
    if (lseek(_session_info.fd, request_payload->offset, SEEK_SET) < 0) {
        // Unable to see to the specified location
        debug("seek fail");
        _ftp_errno = errno;
        return kErrFailErrno;
    }

    int bytes_written = ::write(_session_info.fd, &request_payload->data[0], request_payload->size);

    if (bytes_written < 0) {
        // Negative return indicates error other than eof
        debug("write fail");
        _ftp_errno = errno;
        return kErrFailErrno;
    }

    response_payload->size = sizeof(uint32_t);
    memcpy(response_payload->data, &bytes_written, response_payload->size);

    return kErrNone;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Terminate(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (request_payload->session != 0 || _session_info.fd < 0) {
        return kErrInvalidSession;
    }

    ::close(_session_info.fd);
    _session_info.fd = -1;
    _session_info.stream_download = false;

    response_payload->size = 0;

    return kErrNone;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Reset(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (_session_info.fd != -1) {
        ::close(_session_info.fd);
        _session_info.fd = -1;
        _session_info.stream_download = false;
    }

    response_payload->size = 0;
    return kErrNone;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_RemoveDirectory(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }
    if (rmdir(_ftp_current_abs_path) == 0) {
        response_payload->size = 0;
        return kErrNone;
    } else {
        _ftp_errno = errno;
        return kErrFailErrno;
    }
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_CreateDirectory(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }

    if (mkdir(_ftp_current_abs_path, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
        response_payload->size = 0;
        return kErrNone;
    } else {
        _ftp_errno = errno;
        return kErrFailErrno;
    }
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_RemoveFile(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }

    if (unlink(_ftp_current_abs_path) == 0) {
        response_payload->size = 0;
        return kErrNone;
    } else {
        _ftp_errno = errno;
        return kErrFailErrno;
    }
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_TruncateFile(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }

    int ret = truncate(_ftp_current_abs_path, request_payload->offset);

    if (ret == 0) {
        return kErrNone;
    }
    _ftp_errno = errno;
    return kErrFailErrno;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_Rename(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    char *ptr = _data_as_cstring(request_payload);
    size_t oldpath_sz = strlen(ptr);

    if (oldpath_sz >= request_payload->size) {
        // no newpath
        _ftp_errno = EINVAL;
        return kErrFailErrno;
    }

    if (!_set_current_abs_path(ptr)) {
        return kErrFail;
    }
    char* current_path = new char[strlen(_ftp_current_abs_path)];
    if (current_path == nullptr) {
        return kErrFail;
    }
    strcpy(current_path, _ftp_current_abs_path);

    if (!_set_current_abs_path(ptr + oldpath_sz + 1)) {
        delete current_path;
        return kErrFail;
    }
    _ftp_current_abs_path[ABS_PATH_MAX_LEN - 1] = '\0'; // ensure termination

    if (rename(current_path, _ftp_current_abs_path) == 0) {
        response_payload->size = 0;
        delete current_path;
        return kErrNone;

    }
    _ftp_errno = errno;
    delete current_path;
    return kErrFailErrno;
}

GCS_MAVFTP::FTPErrorCode GCS_MAVFTP::_ftp_CalcFileCRC32(FTPPayloadHeader *request_payload, FTPPayloadHeader *response_payload)
{
    uint32_t checksum = 0;
    ssize_t bytes_read;
    if (!_set_current_abs_path(_data_as_cstring(request_payload))) {
        return kErrFail;
    }
#if HAL_OS_POSIX_IO
    int fd = ::open(_ftp_current_abs_path, O_RDONLY, 0777);
#else
    int fd = ::open(_ftp_current_abs_path, O_RDONLY);
#endif
    if (fd < 0) {
        _ftp_errno = errno;
        return kErrFailErrno;
    }

    do {
        bytes_read = ::read(fd, response_payload->data, MAX_FTP_DATA_LENGTH);

        if (bytes_read < 0) {
            _ftp_errno = errno;
            ::close(fd);
            return kErrFailErrno;
        }
        checksum = crc_crc32(checksum, (uint8_t *)response_payload->data, bytes_read);
    } while (bytes_read == MAX_FTP_DATA_LENGTH);

    ::close(fd);

    response_payload->size = sizeof(uint32_t);
    memcpy(response_payload->data, &checksum, response_payload->size);
    return kErrNone;
}

void GCS_MAVFTP::_ftp_burst_send()
{
    // Anything to stream?
    if (!_session_info.stream_download) {
        return;
    }
    FTPPayloadHeader *stream_payload = reinterpret_cast<FTPPayloadHeader *>(&_ftp_response.payload[0]);

    bool more_data;

    if (mavlink_comm_port[_ftp_chan] != nullptr) {
        if (mavlink_comm_port[_ftp_chan]->is_usbserial()) {
            //have high stream chunk size for USB Serial Ports
            _ftp_stream_chunk_size = 350000; //350KB/chunk
        } else {
            _ftp_stream_chunk_size = 10000; //10KB/chunk
        }
    } else {
        return;
    }
    // Exclusively send Burst Data,
    // We take over the transmit part of the link when sending FTP message,
    // To avoid buffer corruption and maximise throughput
    GCS_MAVLINK::lock_channel(_ftp_chan, false, true);
    do {
        more_data = false;
        FTPErrorCode error_code = kErrNone;

        stream_payload->seq_number = _session_info.stream_seq_number;
        stream_payload->session = 0;
        stream_payload->opcode = kRspAck;
        stream_payload->req_opcode = kCmdBurstReadFile;
        stream_payload->offset = _session_info.stream_offset;
        //reset the data
        memset(stream_payload->data, 0, MAX_FTP_DATA_LENGTH);
        _session_info.stream_seq_number++;


        // We have to test seek past EOF ourselves, lseek will allow seek past EOF
        if (_session_info.stream_offset >= _session_info.file_size) {
            error_code = kErrEOF;
        }

        if (error_code == kErrNone) {
            if (lseek(_session_info.fd, stream_payload->offset, SEEK_SET) < 0) {
                _ftp_errno = errno;
                error_code = kErrFailErrno;
            }
        }

        if (error_code == kErrNone) {
            int bytes_read = ::read(_session_info.fd, &stream_payload->data[0], MAX_FTP_DATA_LENGTH);

            if (bytes_read < 0) {
                // Negative return indicates error other than eof
                _ftp_errno = errno;
                error_code = kErrFailErrno;
            } else {
                stream_payload->size = bytes_read;
                _session_info.stream_offset += bytes_read;
                _session_info.stream_chunk_transmitted += bytes_read;
            }
        }

        if (error_code != kErrNone) {
            stream_payload->opcode = kRspNak;
            stream_payload->size = 1;
            stream_payload->data[0] = error_code;

            if (error_code == kErrFailErrno) {
                stream_payload->size = 2;
                stream_payload->data[1] = _ftp_errno;
            }

            _session_info.stream_download = false;

        } else {
            more_data = true;
            stream_payload->burst_complete = false;
        }

        if (_session_info.stream_chunk_transmitted > _ftp_stream_chunk_size) {
            stream_payload->burst_complete = true;
            _session_info.stream_download = false;
            _session_info.stream_chunk_transmitted = 0;
            more_data = false;
        }
        _ftp_response.target_system = _session_info.stream_target_system_id;
        _ftp_wait_until_buffer_free(stream_payload);
        mavlink_msg_file_transfer_protocol_send_struct(_ftp_chan, &_ftp_response);
        //also send heartbeat every 10 pkts arbitrarily to keep GCS's happy
        if (!(_session_info.stream_chunk_transmitted % 10)) {
            gcs().chan(_ftp_chan).send_heartbeat();
        }
    } while (more_data && _session_info.stream_download);
    GCS_MAVLINK::lock_channel(_ftp_chan, false, false);
}
#endif //#if HAL_WITH_MAVFTP
