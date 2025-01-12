#include "AP_Networking_Config.h"

#if AP_NETWORKING_FILESYSTEM_ENABLED

class NineP2000 {
public:
    /* Do not allow copies */
    CLASS_NO_COPY(NineP2000);

    NineP2000() {};

    void init();

    static const struct AP_Param::GroupInfo var_info[];
    AP_Int8 enabled;
    AP_Networking_IPV4 ip {"0.0.0.0"};
    AP_Int32 port;
    SocketAPM *sock;

    // Return true if connected and mounted
    bool mounted();

    // Return true if there is a response for the given tag
    bool tag_response(const uint16_t tag);

    // Called when a command is timed out
    void clear_tag(const uint16_t tag);

    // Destination of walk
    enum walkType {
        File,
        Directory,
        Any,
    };

    // Walk to a new file or directory, return tag, NOTAG if failed
    uint16_t request_walk(const char* path, const walkType type);

    // Check if the walk result is valid for a directory or file
    uint32_t walk_result(const uint16_t tag);

    // Return the file id to the server for re-use
    void free_file_id(const uint32_t id);

    // Request read of given file or directory with given flags
    uint16_t request_open(const uint32_t id, const int flags);

    // Return true if open success
    bool open_result(const uint16_t tag);

    // Request read of a file, return tag, NOTAG if failed
    uint16_t request_file_read(const uint32_t id, const uint64_t offset, const uint32_t count, void *buf);

    // Request read of a directory, return tag, NOTAG if failed
    uint16_t request_dir_read(const uint32_t id, const uint64_t offset, struct dirent *de);

    // Get result of read request, Return the number of bytes read, -1 for error
    int32_t read_result(const uint16_t tag, const bool is_dir);

    // Request stat for a given file id, return tag, NOTAG if failed
    uint16_t request_stat(const uint32_t id, struct stat *stbuf);

    // Get stat result
    bool stat_result(const uint16_t tag);

    // Request write for given file id, return tag
    uint16_t request_write(const uint32_t id, const uint64_t offset, uint32_t count, const void *buf);

    // Return the number of bytes written, -1 for error
    int32_t write_result(const uint16_t tag);

    // Request create for given directory id, return tag
    uint16_t request_create(const uint32_t id, const char*name, const bool dir);

    // Get create result
    bool create_result(const uint16_t tag);

    // Request remove for given id, return tag
    uint16_t request_remove(const uint32_t id);

    // Get remove result
    bool remove_result(const uint16_t tag);

    // Request rename for given id, return tag
    uint16_t request_rename(const uint32_t id, const char*name);

    // Request mtime update for given id, return tag
    uint16_t request_set_mtime(const uint32_t id, const uint32_t mtime);

    // Get rename result and mtime result
    bool stat_update_result(const uint16_t tag);

    // Magic value for invalid tag
    static constexpr uint16_t NOTAG = 0xFFFF;

    // Return the maximum length that can be read in a single packet
    // This is only valid if the file system is mounted as bufferlength is negotiated
    uint32_t max_read_len() const;

    // Return the maximum length that can be written in a single packet
    // This is only valid if the file system is mounted as bufferlength is negotiated
    uint32_t max_write_len() const;

private:
    void loop();
    bool connected;

    bool update();
    void parse(const uint32_t len);

    // State of connection process
    enum class State {
        Version,
        Attach,
        Mounted,
    } state;

    // qid structure is used in several messages
    struct PACKED qid_t {
        uint8_t type;
        uint32_t vers;
        uint64_t path;
    };

    // Static part of stat structure, followed by four variable length strings
    struct PACKED stat_t {
        uint16_t msg_size;
        uint16_t type;
        uint32_t dev;
        qid_t qid;
        uint32_t mode;
        uint32_t atime;
        uint32_t mtime;
        uint64_t length;
        // name string with length
        // uid string with length
        // gid string with length
        // muid string with length
    };

    // Message shape
    union PACKED Message {
        uint8_t buffer[1024];

        struct PACKED {
            // Header is used on all messages
            struct PACKED {
                uint32_t length;
                uint8_t type;
                uint16_t tag;
            } header;

            // Message payloads differ
            union PACKED {

                // Version request and response
                struct PACKED {
                    uint32_t msize;
                    // version string with length
                } Tversion;

                struct PACKED {
                    uint32_t msize;
                    uint16_t version_string_len;
                    // version string of length version_string_len
                } Rversion;

                // Attach request and response
                struct PACKED {
                    uint32_t fid;
                    uint32_t afid;
                    // uname string with length
                    // aname string with length
                } Tattach;

                struct PACKED {
                    qid_t qid;
                } Rattach;

                // Clunk request
                struct PACKED {
                    uint32_t fid;
                } Tclunk;

                // Error response
                struct PACKED {
                    uint16_t ename_string_len;
                    // ename string with length
                } Rerror;

                // Open request and response
                struct PACKED {
                    uint32_t fid;
                    uint8_t mode;
                } Topen;

                struct PACKED {
                    qid_t qid;
                    uint32_t iounit;
                } Ropen;

                // Create request and response
                struct PACKED {
                    uint32_t fid;
                    // name string with length
                    // uint32_t perm
                    // uint8_t mode
                } Tcreate;

                struct PACKED {
                    qid_t qid;
                    uint32_t iounit;
                } Rcreate;

                // Read request and response
                struct PACKED {
                    uint32_t fid;
                    uint64_t offset;
                    uint32_t count;
                } Tread;

                struct PACKED {
                    uint32_t count;
                    // count * data
                } Rread;

                // Write request and response
                struct PACKED {
                    uint32_t fid;
                    uint64_t offset;
                    uint32_t count;
                    // count * data
                } Twrite;

                struct PACKED {
                    uint32_t count;
                } Rwrite;

                // Remove request
                struct PACKED {
                    uint32_t fid;
                } Tremove;

                // Stat request and response
                struct PACKED {
                    uint32_t fid;
                } Tstat;

                struct PACKED {
                    uint16_t stat_len;
                    stat_t stat;
                    // note stat has variable length strings!
                } Rstat;

                // Stat write request
                struct PACKED {
                    uint32_t fid;
                    uint16_t nstat;
                    stat_t stat;
                    // note stat has variable length strings!
                } Twstat;

                // Walk request and response
                struct PACKED {
                    uint32_t fid;
                    uint32_t newfid;
                    uint16_t nwname;
                    // nwname * wname string with length
                } Twalk;

                struct PACKED {
                    uint16_t nwqid;
                    // nwqid * qid_t
                } Rwalk;

            };
        };
    };

    enum qidType {
        QTDIR    = (1 << 7), // DMDIR bit 31, directory
        QTAPPEND = (1 << 6), // DMAPPEND bit 30, append only
        QTEXCL   = (1 << 5), // DMEXCL bit 29, exclusive use
        // Not used
        QTAUTH   = (1 << 3), // DMAUTH bit 27, authentication file
        QTTMP    = (1 << 2), // DMTMP bit 26, temporay
        QTFILE   = 0,
    };

    enum openMode {
        OREAD   = 0, // Read only
        OWRITE  = 1, // Write only
        ORDWR   = 2, // Read and write
        OEXEC   = 3, // Execute
        NONE    = 4,
        OTRUNC  = 0x10,
        ORCLOSE = 0x40,
    };

    enum class Type: uint8_t {
        Tversion = 100,
        Rversion = 101,
        Tauth    = 102,
        Rauth    = 103,
        Tattach  = 104,
        Rattach  = 105,
        Rerror   = 107,
        Tflush   = 108,
        Rflush   = 109,
        Twalk    = 110,
        Rwalk    = 111,
        Topen    = 112,
        Ropen    = 113,
        Tcreate  = 114,
        Rcreate  = 115,
        Tread    = 116,
        Rread    = 117,
        Twrite   = 118,
        Rwrite   = 119,
        Tclunk   = 120,
        Rclunk   = 121,
        Tremove  = 122,
        Rremove  = 123,
        Tstat    = 124,
        Rstat    = 125,
        Twstat   = 126,
        Rwstat   = 127,
    };

    // Object for holding responses
    // Tag is used as index into array
    // Array length is the max number of concurrent operations
    struct Request {
        bool active;
        bool pending;
        Type expectedType;

        // Union stores variables useful for each message type
        union {
            struct {
                uint32_t fileId;
                walkType type;
            } walk;

            struct {
                uint32_t fileId;
            } clunk;

            struct {
                bool result;
            } open;

            struct {
                bool is_dir;
                int32_t count;
                union {
                    struct dirent *dir;
                    void *buf;
                };
            } read;

            struct {
                bool result;
                struct stat *stbuf;
            } stat;

            struct {
                bool result;
            } remove;

            struct {
                bool result;
            } rwstat;

            struct {
                bool result;
            } create;

            struct {
                int32_t count;
            } write;

        };
    } request[8];

    // Send/Receive buffer
    Message buffer;

    // Buffer length must be negotiated.
    uint16_t bufferLen;

    // Add a string to a message
    bool add_string(Message &msg, const char *str) const WARN_IF_UNUSED;

    // Request version and message size
    void request_version();

    // Handle version response
    void handle_version();

    // Request attach
    void request_attach();

    // Handle attach response
    void handle_attach();

    // Handle walk response
    void handle_rwalk(Request& result);

    // Handle directory read response
    void handle_dir_Rread(Request& result);

    // Handle file read response
    void handle_file_Rread(Request& result);

    // Handle a stat response
    void handle_Rstat(Request& result);

    // Return true if there is a response for the given tag with type
    bool tag_response_type(const uint16_t tag, const Type type);

    // Active file IDs, cannot used concurrently
    // 0 is always root and means unused
    struct {
        bool active;
        bool clunked;
    } fileIds[ARRAY_SIZE(request)];

    // Generate a new unique file id
    uint32_t generate_unique_file_id();

    // Clear a file id now the file has been closed
    void clear_file_id(const uint32_t fileId);

    // Check if a given ID active
    bool valid_file_id(const uint32_t fileId);

    // Return the next available tag, NOTAG is none free
    uint16_t get_free_tag();

    // Decode error messaged print
    void handle_error(Request& result);

    // Semaphore should be take any time the request array is used
    HAL_Semaphore request_sem;
};

#endif // AP_NETWORKING_FILESYSTEM_ENABLED