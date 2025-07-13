/*
  implementation of FILE_TRANSFER_PROTOCOL MAVLink sub-protocol
 */

#pragma once

#include "GCS_config.h"

#if AP_MAVLINK_FTP_ENABLED

#include "GCS.h"

class GCS_FTP {
public:
    void handle_file_transfer_protocol(const mavlink_message_t &msg, mavlink_channel_t chan);
    uint32_t get_last_send_ms(void) const { return last_send_ms; }

private:
    enum class FTP_OP : uint8_t {
        None = 0,
        TerminateSession = 1,
        ResetSessions = 2,
        ListDirectory = 3,
        OpenFileRO = 4,
        ReadFile = 5,
        CreateFile = 6,
        WriteFile = 7,
        RemoveFile = 8,
        CreateDirectory = 9,
        RemoveDirectory = 10,
        OpenFileWO = 11,
        TruncateFile = 12,
        Rename = 13,
        CalcFileCRC32 = 14,
        BurstReadFile = 15,
        Ack = 128,
        Nack = 129,
    };

    enum class FTP_ERROR : uint8_t {
        None = 0,
        Fail = 1,
        FailErrno = 2,
        InvalidDataSize = 3,
        InvalidSession = 4,
        NoSessionsAvailable = 5,
        EndOfFile = 6,
        UnknownCommand = 7,
        FileExists = 8,
        FileProtected = 9,
        FileNotFound = 10,
    };

    struct pending {
        uint32_t offset;
        mavlink_channel_t chan;        
        uint16_t seq_number;
        FTP_OP opcode;
        FTP_OP req_opcode;
        bool  burst_complete;
        uint8_t size;
        uint8_t session;
        uint8_t sysid;
        uint8_t compid;
        uint8_t data[239];
    };

    enum class FTP_FILE_MODE {
        Read,
        Write,
    };

    ObjectBuffer<pending> requests{5};

    // session specific info, currently only support a single session over all links
    bool initialised;
    int fd = -1;
    FTP_FILE_MODE mode; // work around AP_Filesystem not supporting file modes
    int16_t current_session;
    uint32_t last_send_ms;
    uint8_t need_banner_send_mask;

    bool init(void);
    void error(struct pending &response, FTP_ERROR error); // FTP helper method for packing a NAK
    bool check_name_len(const struct pending &request);
    int gen_dir_entry(char *dest, size_t space, const char * path, const struct dirent * entry); // FTP helper for emitting a dir response
    void list_dir(struct pending &request, struct pending &response);

    bool send_reply(const pending &reply);
    void worker(void);
    void push_replies(pending &reply);
};

#endif  // AP_MAVLINK_FTP_ENABLED
