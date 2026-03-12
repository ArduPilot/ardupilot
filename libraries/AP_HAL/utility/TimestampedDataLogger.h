#include <AP_HAL/AP_HAL_Boards.h>

#if AP_RINGBUFFER_CONTENT_LOGGING_ENABLED
class TimestampedDataLogger {

public:
    void log_data(uint8_t channel, const uint8_t *data, uint16_t length);
    bool set_enabled(bool value) { enabled = value; }

private:
    bool enabled = true;

    // support raw serial logging
    struct LogInfo {
        struct LogInfo *next;
        int fd = -1;
        ByteBuffer buf{16000};
        uint8_t instance;
        bool bytes_requiring_fsync;
        uint32_t last_fsync_ms;
    };

    static LogInfo *backend_loginfos;  // linked list of all backend log infos

    // thread-related members:
    static bool log_thread_created;
    static void logging_loop(void);
    void logging_start(void);
};

#endif  // AP_RINGBUFFER_CONTENT_LOGGING_ENABLED
