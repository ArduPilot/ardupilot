/*
  implementation of FILE_TRANSFER_PROTOCOL MAVLink sub-protocol
 */

#pragma once

#include "GCS_config.h"

#if AP_MAVLINK_FTP_ENABLED

#include "GCS.h"

#ifndef AP_MAVLINK_FTP_MAX_SESSIONS
#define AP_MAVLINK_FTP_MAX_SESSIONS 5
#endif

class GCS_FTP {
public:
    static void handle_file_transfer_protocol(const mavlink_message_t &msg, mavlink_channel_t chan);
    static uint32_t get_last_send_ms(mavlink_channel_t chan);

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

    struct Transaction {
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

    ObjectBuffer<Transaction> requests{AP_MAVLINK_FTP_MAX_SESSIONS};

    bool initialised;

    // session specific info
    class Session {
    public:
        int fd = -1;
        uint32_t last_send_ms;
        int16_t session_id;
        FTP_FILE_MODE mode; // work around AP_Filesystem not supporting file modes
        mavlink_channel_t chan;
        uint8_t sysid;
        uint8_t compid;

        bool check_name_len(const Transaction &request);
        int gen_dir_entry(char *dest, size_t space, const char * path, const struct dirent * entry); // FTP helper for emitting a dir response
        void list_dir(Transaction &request, Transaction &response);
        void push_reply(Transaction &reply);
        bool handle_request(Transaction &request, Transaction &reply);

        int close(void);
    };
    Session sessions[AP_MAVLINK_FTP_MAX_SESSIONS];

    bool init(void);

    static bool send_reply(const Transaction &reply);
    static void error(Transaction &response, FTP_ERROR error);

    /*
      setup reply packet to reply to the request
     */
    void setup_reply(const Transaction &request, Transaction &reply);

    void worker(void);

    // GCS_FTP instance created by static handle_file_transfer_protocol()
    static GCS_FTP *ftp;
};

#endif  // AP_MAVLINK_FTP_ENABLED
