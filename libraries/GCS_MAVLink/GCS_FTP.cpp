/*
   GCS MAVLink functions related to FTP

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

#include "GCS_config.h"

#if AP_MAVLINK_FTP_ENABLED

#include "GCS_FTP.h"

#include <AP_HAL/AP_HAL.h>

#include "GCS.h"

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

GCS_FTP *GCS_FTP::ftp;

// timeout for session inactivity, when we will kill the session if
// the session slot is needed
#define FTP_SESSION_TIMEOUT 3000

// timeout for session inactivity, when we will kill an idle session
#define FTP_SESSION_KILL_TIMEOUT 20000

bool GCS_FTP::init(void)
{
    if (initialised) {
        return true;
    }

    initialised = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&GCS_FTP::worker, void),
                                               "FTP", 2560, AP_HAL::Scheduler::PRIORITY_IO, 0);
    if (!initialised) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed to initialize MAVFTP");
    }

    return initialised;
}

/*
  handle a FILE_TRANSFER_PROTOCOL message
 */
void GCS_FTP::handle_file_transfer_protocol(const mavlink_message_t &msg, mavlink_channel_t chan)
{
#if !defined(HAL_BUILD_AP_PERIPH)
    // check if ftp is disabled for memory savings
    if (AP_BoardConfig::ftp_disabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FTP disabled");
        return;
    }
#endif
    if (ftp == nullptr) {
        ftp = NEW_NOTHROW GCS_FTP;
        if (ftp == nullptr) {
            return;
        }
    }
    if (ftp->init()) {
        mavlink_file_transfer_protocol_t packet;
        mavlink_msg_file_transfer_protocol_decode(&msg, &packet);

        Transaction request;

        request.chan = chan;
        request.seq_number = le16toh_ptr(packet.payload);

        request.session = packet.payload[2];
        request.opcode = static_cast<FTP_OP>(packet.payload[3]);
        request.size = packet.payload[4];
        request.req_opcode = static_cast<FTP_OP>(packet.payload[5]);
        request.burst_complete = packet.payload[6];
        request.offset = le32toh_ptr(&packet.payload[8]);
        request.sysid = msg.sysid;
        request.compid = msg.compid;
        memcpy(request.data, &packet.payload[12], sizeof(packet.payload) - 12);

        // if the push fails we drop the message
        // we could NACK it, but that can lead to GCS
        // confusion, so we're treating it like lost data
        ftp->requests.push(request);
    }
}

bool GCS_FTP::send_reply(const Transaction &reply)
{
    if (!GCS_MAVLINK::last_txbuf_is_greater(33)) { // It helps avoid GCS timeout if this is less than the threshold where we slow down normal streams (<=49)
        return false;
    }
    WITH_SEMAPHORE(comm_chan_lock(reply.chan));
    if (!HAVE_PAYLOAD_SPACE(reply.chan, FILE_TRANSFER_PROTOCOL)) {
        return false;
    }
    mavlink_file_transfer_protocol_t pkt {};
    pkt.target_network = 0;
    pkt.target_system = reply.sysid;
    pkt.target_component = reply.compid;
    uint8_t *payload = pkt.payload;
    put_le16_ptr(payload, reply.seq_number);
    payload[2] = reply.session;
    payload[3] = static_cast<uint8_t>(reply.opcode);
    payload[4] = reply.size;
    payload[5] = static_cast<uint8_t>(reply.req_opcode);
    payload[6] = reply.burst_complete ? 1 : 0;
    put_le32_ptr(&payload[8], reply.offset);
    memcpy(&pkt.payload[12], reply.data, sizeof(reply.data));
    mavlink_msg_file_transfer_protocol_send_struct(reply.chan, &pkt);
    return true;
}

/*
  check a name length for validity
 */
bool GCS_FTP::Session::check_name_len(const Transaction &request)
{
    const size_t file_name_len = strnlen((char *)request.data, sizeof(request.data));
    if (request.size == 0) {
        return false;
    }
    if (file_name_len == request.size) {
        return true;
    }
    return (request.size - file_name_len == 1) && (request.data[sizeof(request.data) - 1] == 0);
}

// send our response back out to the system
void GCS_FTP::Session::push_reply(Transaction &reply)
{
    last_send_ms = AP_HAL::millis(); // Used to detect active FTP session

    while (!send_reply(reply)) {
        hal.scheduler->delay_microseconds(100);
    }

    if (reply.req_opcode == FTP_OP::TerminateSession) {
        last_send_ms = 0;
    }
}

// calculates how much string length is needed to fit this in a list response
int GCS_FTP::Session::gen_dir_entry(char *dest, size_t space, const char *path, const struct dirent * entry)
{
#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
    const bool is_file = entry->d_type == DT_REG || entry->d_type == DT_LNK;
#else
    // assume true initially, then handle below
    const bool is_file = true;
#endif

    if (space < 3) {
        return -1;
    }
    dest[0] = 0;

#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
    if (!is_file && entry->d_type != DT_DIR) {
        return -1; // this just forces it so we can't send this back, it's easier then sending skips to a GCS
    }
#endif

    if (is_file) {
#ifdef MAX_NAME_LEN
        const uint8_t max_name_len = MIN(unsigned(MAX_NAME_LEN), 255U);
#else
        const uint8_t max_name_len = 255U;
#endif
        const size_t full_path_len = strlen(path) + strnlen(entry->d_name, max_name_len);
        char full_path[full_path_len + 2];
        hal.util->snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        struct stat st;
        if (AP::FS().stat(full_path, &st)) {
            return -1;
        }

#if !AP_FILESYSTEM_HAVE_DIRENT_DTYPE
        if (S_ISDIR(st.st_mode)) {
            return hal.util->snprintf(dest, space, "D%s%c", entry->d_name, (char)0);
        }
#endif
        return hal.util->snprintf(dest, space, "F%s\t%u%c", entry->d_name, (unsigned)st.st_size, (char)0);
    } else {
        return hal.util->snprintf(dest, space, "D%s%c", entry->d_name, (char)0);
    }
}

// list the contents of a directory, skip the offset number of entries before providing data
void GCS_FTP::Session::list_dir(Transaction &request, Transaction &response)
{
    response.offset = request.offset; // this should be set for any failure condition for debugging

    // sanity check that our the request looks well formed
    if (!check_name_len(request)) {
        GCS_FTP::error(response, FTP_ERROR::InvalidDataSize);
        return;
    }

    request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

    // Strip trailing /
    const size_t dir_len = strlen((char *)request.data);
    if ((dir_len > 1) && (request.data[dir_len - 1] == '/')) {
        request.data[dir_len - 1] = 0;
    }

    // open the dir
    auto *dir = AP::FS().opendir((char *)request.data);
    if (dir == nullptr) {
        GCS_FTP::error(response, FTP_ERROR::FailErrno);
        return;
    }

    // burn the entries we don't care about
    while (request.offset > 0) {
        const struct dirent *entry = AP::FS().readdir(dir);
        if(entry == nullptr) {
            GCS_FTP::error(response, FTP_ERROR::EndOfFile);
            AP::FS().closedir(dir);
            return;
        }

        // check how much space would be needed to emit the listing
        const int needed_space = gen_dir_entry((char *)response.data, sizeof(request.data), (char *)request.data, entry);

        if (needed_space < 0 || needed_space > (int)sizeof(request.data)) {
            continue;
        }

        request.offset--;
    }

    // start packing in entries that fit
    uint8_t index = 0;
    struct dirent *entry;
    while ((entry = AP::FS().readdir(dir))) {
        // figure out if we can fit the file
        const int required_space = gen_dir_entry((char *)(response.data + index), sizeof(response.data) - index, (char *)request.data, entry);

        // couldn't ever send this so drop it
        if (required_space < 0) {
            continue;
        }

        // can't fit it in this one, leave it for the next list to send
        if ((required_space + index) >= (int)sizeof(request.data)) {
            break;
        }

        // step the index forward and keep going
        index += required_space + 1;
    }

    if (index == 0) {
        GCS_FTP::error(response, FTP_ERROR::EndOfFile);
        AP::FS().closedir(dir);
        return;
    }
    
    // strip any bad temp data from our response as it can confuse a GCS, and defeats 0 trimming
    if (index < sizeof(response.data)) {
        memset(response.data + index, 0, MAX(0, (int)(sizeof(response.data)) - index));
    }

    response.opcode = FTP_OP::Ack;
    response.size = index;

    AP::FS().closedir(dir);
}

/*
  close a session

  returns the error code friom the underlying close() call, or zero (no error) if the
  file was closed already
 */
int GCS_FTP::Session::close(void)
{
    int result = 0;

    if (fd != -1) {
        result = AP::FS().close(fd);
        fd = -1;
    }
    last_send_ms = 0;

    return result;
}

/*
  handle one request on a session

  return true if the main loop should skip the push_replies() call
 */
bool GCS_FTP::Session::handle_request(Transaction &request, Transaction &reply)
{
    bool skip_push_reply = false;

    // sanity check the request size
    if (request.size > sizeof(request.data)) {
        GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
        return false;
    }

    const uint32_t now = AP_HAL::millis();

    // dispatch the command as needed
    switch (request.opcode) {
    case FTP_OP::None:
        reply.opcode = FTP_OP::Ack;
        break;
    case FTP_OP::TerminateSession:
        if (close()) {
            // close() operation indicated an error, errno
            // was set by close() itself
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
        } else {
            reply.opcode = FTP_OP::Ack;
        }
        break;
    case FTP_OP::ListDirectory:
        list_dir(request, reply);
        break;
    case FTP_OP::OpenFileRO:
    {
        // only allow one file to be open per session
        if (fd != -1 && now - last_send_ms > FTP_SESSION_TIMEOUT) {
            // no activity for 3s, assume client has
            // timed out receiving open reply, close
            // the file
            close();    // error code ignored
            fd = -1;
        }
        if (fd != -1) {
            GCS_FTP::error(reply, FTP_ERROR::Fail);
            break;
        }

        // sanity check that the request looks well formed
        if (!check_name_len(request)) {
            GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
            break;
        }

        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

        // get the file size
        struct stat st;
        if (AP::FS().stat((char *)request.data, &st)) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }
        const size_t file_size = st.st_size;

        // actually open the file
        fd = AP::FS().open((char *)request.data, O_RDONLY);
        if (fd == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }
        mode = FTP_FILE_MODE::Read;

        reply.opcode = FTP_OP::Ack;
        reply.size = sizeof(uint32_t);
        put_le32_ptr(reply.data, (uint32_t)file_size);

        // provide compatibility with old protocol banner download
        if (strncmp((const char *)request.data, "@PARAM/param.pck", 16) == 0) {
            auto *gchan = gcs().chan(reply.chan);
            if (gchan != nullptr) {
                gchan->send_banner();
            }
        }
        break;
    }
    case FTP_OP::ReadFile:
    {
        // must actually be working on a file
        if (fd == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FileNotFound);
            break;
        }

        // must have the file in read mode
        if ((mode != FTP_FILE_MODE::Read)) {
            GCS_FTP::error(reply, FTP_ERROR::Fail);
            break;
        }

        // seek to requested offset
        if (AP::FS().lseek(fd, request.offset, SEEK_SET) == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        // fill the buffer
        const ssize_t read_bytes = AP::FS().read(fd, reply.data, MIN(sizeof(reply.data),request.size));
        if (read_bytes == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }
        if (read_bytes == 0) {
            GCS_FTP::error(reply, FTP_ERROR::EndOfFile);
            break;
        }

        reply.opcode = FTP_OP::Ack;
        reply.offset = request.offset;
        reply.size = (uint8_t)read_bytes;
        break;
    }
    case FTP_OP::Ack:
    case FTP_OP::Nack:
        // eat these, we just didn't expect them
        return true;

    case FTP_OP::OpenFileWO:
    case FTP_OP::CreateFile:
    {
        // only allow one file to be open per session
        if (fd != -1) {
            GCS_FTP::error(reply, FTP_ERROR::Fail);
            break;
        }

        // sanity check that our the request looks well formed
        if (!check_name_len(request)) {
            GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
            break;
        }

        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

        // actually open the file
        fd = AP::FS().open((char *)request.data,
                           (request.opcode == FTP_OP::CreateFile) ? O_WRONLY|O_CREAT|O_TRUNC : O_WRONLY|O_CREAT);
        if (fd == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }
        mode = FTP_FILE_MODE::Write;

        reply.opcode = FTP_OP::Ack;
        break;
    }
    case FTP_OP::WriteFile:
    {
        // must actually be working on a file
        if (fd == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FileNotFound);
            break;
        }

        // must have the file in write mode
        if ((mode != FTP_FILE_MODE::Write)) {
            GCS_FTP::error(reply, FTP_ERROR::Fail);
            break;
        }

        // seek to requested offset
        if (AP::FS().lseek(fd, request.offset, SEEK_SET) == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        // fill the buffer
        const ssize_t write_bytes = AP::FS().write(fd, request.data, request.size);
        if (write_bytes == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        reply.opcode = FTP_OP::Ack;
        reply.offset = request.offset;
        break;
    }
    case FTP_OP::CreateDirectory:
    {
        // sanity check that our the request looks well formed
        if (!check_name_len(request)) {
            GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
            break;
        }

        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

        // actually make the directory
        if (AP::FS().mkdir((char *)request.data) == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        reply.opcode = FTP_OP::Ack;
        break;
    }
    case FTP_OP::RemoveDirectory:
    case FTP_OP::RemoveFile:
    {
        // sanity check that our the request looks well formed
        if (!check_name_len(request)) {
            GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
            break;
        }

        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

        // remove the file/dir
        if (AP::FS().unlink((char *)request.data) == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        reply.opcode = FTP_OP::Ack;
        break;
    }
    case FTP_OP::CalcFileCRC32:
    {
        // sanity check that our the request looks well formed
        if (!check_name_len(request)) {
            GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
            break;
        }

        request.data[sizeof(request.data) - 1] = 0; // ensure the path is null terminated

        uint32_t checksum = 0;
        if (!AP::FS().crc32((char *)request.data, checksum)) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        // reset our scratch area so we don't leak data, and can leverage trimming
        memset(reply.data, 0, sizeof(reply.data));
        reply.size = sizeof(uint32_t);
        put_le32_ptr(reply.data, checksum);
        reply.opcode = FTP_OP::Ack;
        break;
    }
    case FTP_OP::BurstReadFile:
    {
        const uint16_t max_read = (request.size == 0?sizeof(reply.data):request.size);
        // must actually be working on a file
        if (fd == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FileNotFound);
            break;
        }

        // must have the file in read mode
        if ((mode != FTP_FILE_MODE::Read)) {
            GCS_FTP::error(reply, FTP_ERROR::Fail);
            break;
        }

        // seek to requested offset
        if (AP::FS().lseek(fd, request.offset, SEEK_SET) == -1) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }

        /*
          calculate a burst delay so that FTP burst
          transfer doesn't use more than 1/3 of
          available bandwidth on links that don't have
          flow control. This reduces the chance of
          lost packets a lot, which results in overall
          faster transfers
        */
        uint32_t burst_delay_ms = 0;
        if (valid_channel(request.chan)) {
            auto *port = mavlink_comm_port[request.chan];
            if (port != nullptr && port->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE) {
                const uint32_t bw = port->bw_in_bytes_per_second();
                const uint16_t pkt_size = PAYLOAD_SIZE(request.chan, FILE_TRANSFER_PROTOCOL) - (sizeof(reply.data) - max_read);
                burst_delay_ms = 3000 * pkt_size / bw;
            }
        }

        // this transfer size is enough for a full parameter file with max parameters
        const uint32_t transfer_size = 2000;
        for (uint32_t i = 0; (i < transfer_size); i++) {
            // fill the buffer
            const ssize_t read_bytes = AP::FS().read(fd, reply.data, MIN(sizeof(reply.data), max_read));
            if (read_bytes == -1) {
                GCS_FTP::error(reply, FTP_ERROR::FailErrno);
                break;
            }

            if (read_bytes != sizeof(reply.data)) {
                // don't send any old data
                memset(reply.data + read_bytes, 0, sizeof(reply.data) - read_bytes);
            }

            if (read_bytes == 0) {
                GCS_FTP::error(reply, FTP_ERROR::EndOfFile);
                break;
            }

            reply.opcode = FTP_OP::Ack;
            reply.offset = request.offset + i * max_read;
            reply.burst_complete = ((read_bytes < max_read) || (i == (transfer_size - 1)));
            reply.size = (uint8_t)read_bytes;

            push_reply(reply);

            if (read_bytes < max_read) {
                // ensure the NACK which we send next is at the right offset
                reply.offset += read_bytes;
            }

            // prep the reply to be used again
            reply.seq_number++;

            if (burst_delay_ms > 0) {
                hal.scheduler->delay(burst_delay_ms);
            }
        }

        if (reply.opcode != FTP_OP::Nack) {
            // prevent a duplicate packet send for
            // normal replies of burst reads
            skip_push_reply = true;
        }
        break;
    }

    case FTP_OP::Rename: {
        // sanity check that the request looks well formed
        const char *filename1 = (char*)request.data;
        const size_t len1 = strnlen(filename1, sizeof(request.data)-2);
        const char *filename2 = (char*)&request.data[len1+1];
        const size_t len2 = strnlen(filename2, sizeof(request.data)-(len1+1));
        const bool is_req_size_consider_tnull = (request.size - (len1+len2) == 2 &&
                                                 request.data[sizeof(request.data) - 1] == 0);
        if (filename1[len1] != 0 || ((len1+len2+1 != request.size) && !is_req_size_consider_tnull) || (request.size == 0)) {
            GCS_FTP::error(reply, FTP_ERROR::InvalidDataSize);
            break;
        }
        request.data[sizeof(request.data) - 1] = 0; // ensure the 2nd path is null terminated
        // remove the file/dir
        if (AP::FS().rename(filename1, filename2) != 0) {
            GCS_FTP::error(reply, FTP_ERROR::FailErrno);
            break;
        }
        reply.opcode = FTP_OP::Ack;
        break;
    }

    case FTP_OP::TruncateFile:
    default:
        // this was bad data, just nack it
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Unsupported FTP: %d", static_cast<int>(request.opcode));
        GCS_FTP::error(reply, FTP_ERROR::Fail);
        break;
    }

    return skip_push_reply;
}

/*
  get the time of the last send for a channel
 */
uint32_t GCS_FTP::get_last_send_ms(mavlink_channel_t chan)
{
    if (ftp == nullptr) {
        return 0;
    }
    uint32_t ret = 0;
    for (const auto &s : ftp->sessions) {
        // using a comparison will be briefly wrong every 49 days, but
        // this is non-critical and getting it perfect would be
        // expensive in a hot path
        if (s.chan == chan && s.last_send_ms > ret) {
            ret = s.last_send_ms;
        }
    }
    return ret;
}

/*
  fill in a reply with an error code
 */
void GCS_FTP::error(Transaction &response, FTP_ERROR error)
{
    response.opcode = FTP_OP::Nack;
    response.data[0] = static_cast<uint8_t>(error);
    response.size = 1;

    // FIXME: errno's are not thread-local as they should be on ChibiOS
    if (error == FTP_ERROR::FailErrno) {
        // translate the errno's that we have useful messages for
        switch (errno) {
            case EEXIST:
                response.data[0] = static_cast<uint8_t>(FTP_ERROR::FileExists);
                break;
            case ENOENT:
                response.data[0] = static_cast<uint8_t>(FTP_ERROR::FileNotFound);
                break;
            default:
                response.data[1] = static_cast<uint8_t>(errno);
                response.size = 2;
                break;
        }
    }
}

/*
  setup reply packet to reply to the request
*/
void GCS_FTP::setup_reply(const Transaction &request, Transaction &reply)
{
    memset(&reply, 0, sizeof(reply));
    reply.req_opcode = request.opcode;
    reply.session = request.session;
    reply.seq_number = request.seq_number + 1;
    reply.chan = request.chan;
    reply.sysid = request.sysid;
    reply.compid = request.compid;
}

/*
  main FTP thread
 */
void GCS_FTP::worker(void)
{
    Transaction request;
    Transaction reply {};
    reply.session = -1; // flag the reply as invalid for any reuse

    while (true) {
        while (!requests.pop(request)) {
            // nothing to handle, delay ourselves a bit then check again. Ideally we'd use conditional waits here
            hal.scheduler->delay(2);

            // kill any dead sessions
            const uint32_t now = AP_HAL::millis();
            for (auto &s : sessions) {
                if (s.last_send_ms != 0 &&
                    now - s.last_send_ms > FTP_SESSION_KILL_TIMEOUT) {
                    s.close();   // error code ignored
                }
            }
        }

        if (request.opcode == FTP_OP::ResetSessions) {
            /*
              close all sessions for this channel, compid and sysid
             */
            for (auto &s : sessions) {
                if (request.sysid == s.sysid &&
                    request.compid == s.compid &&
                    request.chan == s.chan) {
                    // close this session
                    s.close();   // error code ignored
                }
            }
            // always ACK, even if no sessions were closed
            setup_reply(request, reply);
            reply.opcode = FTP_OP::Ack;
            send_reply(reply);
            continue;
        }

        Session *session = nullptr;
        for (uint8_t i=0; i<ARRAY_SIZE(sessions); i++) {
            auto &s = sessions[i];
            if (request.sysid == s.sysid &&
                request.compid == s.compid &&
                request.chan == s.chan &&
                request.session == s.session_id) {
                // found the session
                session = &s;
                break;
            }
        }

        if (session == nullptr) {
            /*
              find the oldest session to possibly reuse
             */
            const uint32_t now = AP_HAL::millis();
            session = &sessions[0];
            for (uint8_t i=1; i<ARRAY_SIZE(sessions); i++) {
                auto &s = sessions[i];
                if ((now - s.last_send_ms) > (now - session->last_send_ms)) {
                    session = &s;
                }
            }

            // only reuse the session if it is not active
            auto &s = *session;
            if (s.last_send_ms != 0 &&
                now - s.last_send_ms < FTP_SESSION_TIMEOUT) {
                // the oldest session is still active, reject the request
                setup_reply(request, reply);
                error(reply, FTP_ERROR::NoSessionsAvailable);
                send_reply(reply);
                continue;
            }
            // claim the session
            s.close();   // error code ignored
            s.session_id = request.session;
            s.sysid = request.sysid;
            s.compid = request.compid;
            s.chan = request.chan;
        }

        // if it's a rerequest and we still have the last response then send it
        if ((request.sysid == reply.sysid) && (request.compid == reply.compid) &&
            (request.session == reply.session) && (request.seq_number + 1 == reply.seq_number) &&
            reply.data[0] != uint8_t(FTP_ERROR::NoSessionsAvailable)) {
            session->push_reply(reply);
            continue;
        }

        setup_reply(request, reply);

        bool skip_push_reply = session->handle_request(request, reply);

        if (!skip_push_reply) {
            session->push_reply(reply);
        }
    }
}

#endif  // AP_MAVLINK_FTP_ENABLED
