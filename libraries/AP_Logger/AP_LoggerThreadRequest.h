#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Logger_config.h"

// TODO: optimise RAM taken up by these structures
class AP_LoggerThreadRequest {
public:

    // State - request lifecycle state
    enum class State {
        FREE       = 0,  // FREE must be value 0
        CLAIMED    = 1,
        PENDING    = 2,  // request filled by main thread, as-yet unseen by logger thread
        PROCESSING = 3,  // now owned by the logger thread
        ABANDONED  = 4,  // main thread has abandoned this request; logger thread will free
        PROCESSED  = 5,  // logger thread has completed this request
    };

    // Type - request type.  This determines what action the
    // LoggerThread will undertake, and which member of the Parameters
    // union is valid.
    enum class Type {
#if HAL_LOGGER_FLUSH_SUPPORTED
        Flush,                    // flush pending writes to storage
#endif
        HandleLogRequest_List,    // process GCS request for log list
        HandleLogRequest_Data,    // process GCS request for log data
        HandleLogRequest_Erase,
        HandleLogRequest_End,
        StartWriteEntireMission,
        StartWriteEntireRally,
        PrepForArming,
        VehicleWasDisarmed,  // FIXME: remove this and just look at the armed state in the thread
        HandleRemoteLogBlockStatus,

        // FindLastLog,
        // FindOldestLog,
        // GetLogBoundaries,
        // GetLogData,
        // GetLogInfo,
        // GetNumLogs,
        StopLogging,
        // IDs for file backend:
        KillWriteFD,
    };

    // each Type of request has a unique set of parameters.  This is a
    // union containing those parameters.  If a Request obviously
    // requires a return value of some description then it will be
    // present in these parameters.
    union Parameters {
        struct {
            class GCS_MAVLINK *link;
            uint16_t start;
            uint16_t end;
        } HandleLogRequest_List;
        struct {
            GCS_MAVLINK *link;
            uint32_t ofs;
            uint32_t count;
            uint16_t id;
        } HandleLogRequest_Data;
        struct {
            GCS_MAVLINK *link;
        } HandleLogRequest_Erase;
        struct {
            GCS_MAVLINK *link;
        } HandleLogRequest_End;
        struct {
            GCS_MAVLINK *link;
            uint8_t sysid;
            uint8_t compid;
            uint32_t seqno;
            uint8_t status;
        } HandleRemoteLogBlockStatus;
    } parameters;

    Type type;

    // starts in FREE state
    State state;

    // if process who requested this action will not wait for the
    // process to complete then this MUST be set
    bool free_after_processing;
};

class AP_LoggerThreadRequestQueue {
public:

    AP_LoggerThreadRequest *claim_thread_request(AP_LoggerThreadRequest::Type type);

    AP_LoggerThreadRequest *claim_thread_request();

    void free_request(AP_LoggerThreadRequest &r) {
        r.state = AP_LoggerThreadRequest::State::FREE;
        requests_count--;
    }
    void mark_as_pending(AP_LoggerThreadRequest &r) {
        r.state = AP_LoggerThreadRequest::State::PENDING;
    }

    bool complete_simple_thread_request(AP_LoggerThreadRequest::Type type, const char *name);
    bool complete_thread_request(AP_LoggerThreadRequest &request);

    uint8_t requests_count;

    HAL_Semaphore requests_semaphore;

    AP_LoggerThreadRequest requests[5];

private:

};
