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
        None = MAV_FTP_OPCODE_NONE,
        TerminateSession = MAV_FTP_OPCODE_TERMINATESESSION,
        ResetSessions = MAV_FTP_OPCODE_RESETSESSION,
        ListDirectory = MAV_FTP_OPCODE_LISTDIRECTORY,
        OpenFileRO = MAV_FTP_OPCODE_OPENFILERO,
        ReadFile = MAV_FTP_OPCODE_READFILE,
        CreateFile = MAV_FTP_OPCODE_CREATEFILE,
        WriteFile = MAV_FTP_OPCODE_WRITEFILE,
        RemoveFile = MAV_FTP_OPCODE_REMOVEFILE,
        CreateDirectory = MAV_FTP_OPCODE_CREATEDIRECTORY,
        RemoveDirectory = MAV_FTP_OPCODE_REMOVEDIRECTORY,
        OpenFileWO = MAV_FTP_OPCODE_OPENFILEWO,
        TruncateFile = MAV_FTP_OPCODE_TRUNCATEFILE,
        Rename = MAV_FTP_OPCODE_RENAME,
        CalcFileCRC32 = MAV_FTP_OPCODE_CALCFILECRC,
        BurstReadFile = MAV_FTP_OPCODE_BURSTREADFILE,
        // ListDirectoryWithTime: like ListDirectory, but each entry also
        // carries its last-modification time. The opcode is upstream, but is
        // not in the bundled mavlink definitions yet, so its value (16) is
        // hardcoded here; switch to MAV_FTP_OPCODE_LISTDIRECTORYWITHTIME
        // when modules/mavlink is next updated.
        ListDirectoryWithTime = 16,
        Ack = MAV_FTP_OPCODE_ACK,
        Nack = MAV_FTP_OPCODE_NAK,
    };

    enum class FTP_ERROR : uint8_t {
        None = MAV_FTP_ERR_NONE,
        Fail = MAV_FTP_ERR_FAIL,
        FailErrno = MAV_FTP_ERR_FAILERRNO,
        InvalidDataSize = MAV_FTP_ERR_INVALIDDATASIZE,
        InvalidSession = MAV_FTP_ERR_INVALIDSESSION,
        NoSessionsAvailable = MAV_FTP_ERR_NOSESSIONSAVAILABLE,
        EndOfFile = MAV_FTP_ERR_EOF,
        UnknownCommand = MAV_FTP_ERR_UNKNOWNCOMMAND,
        FileExists = MAV_FTP_ERR_FILEEXISTS,
        FileProtected = MAV_FTP_ERR_FILEPROTECTED,
        FileNotFound = MAV_FTP_ERR_FILENOTFOUND,
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
        uint32_t sysid;
        uint8_t compid;
        uint8_t data[239];
    };

    enum class FTP_FILE_MODE {
        Read,
        Write,
    };

    ObjectBuffer<Transaction> requests{AP_MAVLINK_FTP_MAX_SESSIONS};

    // signalled when a request is queued, so the worker wakes for it
    HAL_BinarySemaphore *requests_sem;

    /* Push/drop counts for the STATUSTEXT below. The printk FTPDIAG lines are
       invisible whenever a GCS holds the USB CDC console, which is always, so
       the same facts have to reach MAVLink to be readable at all. */
    static uint32_t dbg_pushes;
    static uint32_t dbg_drops;

    /* FTPDIAG counters, written by worker() and read from the receive path -
       which runs even when worker() does not, so they are visible whether or
       not the worker is being scheduled. Distinguishes three causes that all
       present as "MAVFTP times out":
         dbg_spins   frozen  -> worker never scheduled
         dbg_spins   rising, dbg_pops 0 -> worker runs but sees an EMPTY queue
                                           while the producer reports it FULL
         dbg_pops    rising, dbg_replies 0 -> the reply path
       dbg_spins also gives the real poll rate: the idle loop is delay(2), so
       it should climb about 500/s if the worker is healthy. */
    static volatile uint32_t dbg_spins;
    static volatile uint32_t dbg_pops;
    static volatile uint32_t dbg_replies;
    /* send_reply() bracketing: pinpoints which statement the worker parks on.
       enter > lock   -> blocked acquiring comm_chan_lock(chan)
       lock  > ok     -> HAVE_PAYLOAD_SPACE never true, or stuck in the send
       txbuf_fail     -> the radio flow-control gate is rejecting (should not
                         happen on USB, where the stale-report path returns true) */
    static volatile uint32_t dbg_send_enter;
    static volatile uint32_t dbg_send_txbuf_fail;
    static volatile uint32_t dbg_send_lock;
    static volatile uint32_t dbg_send_nospace;
    static volatile uint32_t dbg_send_ok;

    bool initialised;

    // session specific info
    class Session {
    public:
        int fd = -1;
        uint32_t last_send_ms;
        int16_t session_id;
        FTP_FILE_MODE mode; // work around AP_Filesystem not supporting file modes
        mavlink_channel_t chan;
        uint32_t sysid;
        uint8_t compid;

        bool check_name_len(const Transaction &request);
        int gen_dir_entry(char *dest, size_t space, const char * path, const struct dirent * entry, bool with_time); // FTP helper for emitting a dir response
        void list_dir(Transaction &request, Transaction &response, bool with_time);
        bool push_reply(Transaction &reply);
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
