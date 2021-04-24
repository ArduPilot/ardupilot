#pragma once

// TODO: optimise RAM taken up by these structures
class PACKED LoggerThreadRequest {
public:
    enum class State : uint8_t {
        FREE = 0,  // FREE must be value 0
        PENDING = 1,
        PROCESSING = 2,
        ABANDONED = 3,
        PROCESSED = 4,
    };
    enum class Type : uint8_t {
        // IDs for all backends:
        EraseAll,
        Flush,
        HandleLogRequest_List,
        HandleLogRequest_Data,
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
