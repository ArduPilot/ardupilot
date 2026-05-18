#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_CAPTURE_ENABLED

#include <AP_HAL/Semaphores.h>
#include <stdint.h>
#include <stddef.h>

/*
  Simple pcap packet capture to file.
  Each instance writes to a separate file.
  Thread-safe via internal semaphore.
*/
class AP_Networking_Capture
{
public:
    // Start capture to the given filename (e.g. "eth0.cap")
    // If already capturing, flushes the file instead
    void start(const char *filename);

    // Stop capture and close file
    void stop();

    // Write a frame to the pcap file (thread-safe)
    void capture_frame(const uint8_t *frame, size_t len);

    // Check if currently capturing
    bool is_active() const { return fd != -1; }

private:
    HAL_Semaphore sem;
    int fd = -1;
};

#endif // AP_NETWORKING_CAPTURE_ENABLED
