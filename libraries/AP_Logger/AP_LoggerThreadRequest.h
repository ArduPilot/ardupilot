#pragma once

// TODO: optimise RAM taken up by these structures
class PACKED LoggerThreadRequest {
public:

    // State - request lifecycle state
    enum class State : uint8_t {
        FREE       = 0,  // FREE must be value 0
        PENDING    = 1,  // request filled by main thread, as-yet unseen by logger thread
        PROCESSING = 2,  // now owned by the logger thread
        ABANDONED  = 3,  // main thread has abandoned this request; logger thread will free
        PROCESSED  = 4,  // logger thread has completed this request
    };

    // Type - request type.  This determines what action the
    // LoggerThread will undertake, and which member of the Parameters
    // union is valid.
    enum class Type : uint8_t {
        EraseAll,                 // Erase all log files in storage
        Flush,                    // flush pending writes to storage
        HandleLogRequest_List,    // process GCS request for log list
        HandleLogRequest_Data,    // process GCS request for log data
        HandleLogRequest_Erase,
        HandleLogRequest_End,
        StartWriteEntireMission,
        StartWriteEntireRally,
        PrepForArming,
        VehicleWasDisarmed,  // FIXME: remove this and just look at the armed state in the thread
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

    union Parameters {
        struct PACKED {
            GCS_MAVLINK *link;
            uint16_t start;
            uint16_t end;
        } HandleLogRequest_List;
        struct PACKED {
            GCS_MAVLINK *link;
            uint32_t ofs;
            uint32_t count;
            uint16_t id;
        } HandleLogRequest_Data;
        struct PACKED {
            GCS_MAVLINK *link;
        } HandleLogRequest_Erase;
        struct PACKED {
            GCS_MAVLINK *link;
        } HandleLogRequest_End;
    } parameters;

    // union Results {
    //     struct SomeTypeThingy {
    //         uint8_t return_value;
    //     };
    // };

    Type type;

    // starts in FREE state
    State state;

    // if process who requested this action will not wait for the
    // process to complete then this MUST be set
    bool free_after_processing;
};
