/*
 * Support for logging all incoming/outgoing data from a serial port
 */

#if HAL_UART_DEBUG_LOGGING_ENABLED

/*
  log some data for debugging
*/
void AP_HAL::UARTDriver::log_data(uint8_t channel, const uint8_t *data, uint16_t length)
{
    // first see if we actually do want to log:
    if (!enabled) {
        return;
    }

    // find logging instance corresponding to this UART:
    if (loginfo == nullptr) {
        WITH_SEMAPHORE(loginfo_creation_sem);

        loginfo = NEW_NOTHROW LogInfo();
        if (loginfo == nullptr) {
            return;
        }

        loginfo->instance = instance;
        loginfo->next = backend_loginfos;
        backend_loginfos = loginfo;

        if (!log_thread_created) {
            log_thread_created = true;
            hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_HAL::UARTDriver::logging_start, void), "uart_log", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0);
        }
    }

    // write packet into buffer:
    struct PACKED Header {
        enum class Flag {
            IS_WRITTEN_DATA = 1,
        };
        uint32_t magic = 0xEAEF0D0F;
        uint32_t time_ms;
        uint16_t length;
        uint8_t flags;
    };

    // note that we are assuming we can do this writing atomically.
    // If there are multiple writers to the serial port that won't be
    // the case.  We could reserve, write to and commit an iovec
    // instead of this:

    const auto space_required = sizeof(Header) + length;

    if (loginfo->buf.space() < space_required) {
        return;
    }

    Header header;
    header.time_ms = AP_HAL::millis();
    header.length = length;
    if (is_written_data) {
        header.flags |= uint8_t(Header::Flag::IS_WRITTEN_DATA);
    }

    loginfo->buf.write((uint8_t*)&header, sizeof(header));
    loginfo->buf.write(data, length);
}

// a linked list of backends which are currently logging:
AP_HAL::UARTDriver::LogInfo *AP_HAL::UARTDriver::backend_loginfos;
bool AP_HAL::UARTDriver::log_thread_created;

// logging loop, needs to be static as it loops over backends created
// by all all backends
void AP_HAL::UARTDriver::logging_loop(void)
{
    while (true) {
        hal.scheduler->delay(10);
        for (auto *loginfo_ptr = backend_loginfos; loginfo_ptr != nullptr; loginfo_ptr = loginfo_ptr->next) {
            auto &loginfo = *loginfo_ptr;
            if (loginfo.fd == -2) {
                // previously failed; ignore.
                continue;
            }
            // open logfile if required:
            if (loginfo.fd == -1) {
                char fname[20];
                for (uint32_t lognum=0; lognum<1000; lognum++) {
                    hal.util->snprintf(fname, ARRAY_SIZE(fname), "uart%u_%03u.log", loginfo.instance, (unsigned)lognum);
                    struct stat st;
                    if (AP::FS().stat(fname, &st) != 0) {
                        break;
                    }
                }
                loginfo.fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_APPEND);
                if (loginfo.fd == -1) {
                    // convert into a permanent failure:
                    loginfo.fd = -2;
                    continue;
                }
            }
            uint32_t n = 0;
            const uint8_t *p;
            while ((p = loginfo.buf.readptr(n)) != nullptr && n != 0) {
                // short writes are unlikely and are ignored (only FS full errors)
                AP::FS().write(loginfo.fd, p, n);
                loginfo.buf.advance(n);
                loginfo.bytes_requiring_fsync = true;
            }
            const auto now_ms = AP_HAL::millis();
            if (loginfo.bytes_requiring_fsync > 0 &&
                now_ms - loginfo.last_fsync_ms > 1000) {
                AP::FS().fsync(loginfo.fd);
                loginfo.bytes_requiring_fsync = false;
                loginfo.last_fsync_ms = now_ms;
            }
        }
    }
}

// logging thread start, needs to be non-static for thread_create
void AP_HAL::UARTDriver::logging_start(void)
{
    logging_loop();
}
#endif // HAL_UART_DEBUG_LOGGING_ENABLED
