/*
 * Copyright (c) 2016-2019 UAVCAN Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Contributors: https://github.com/UAVCAN/libcanard/contributors
 *
 * Documentation: http://uavcan.org/Implementations/Libcanard
 */

#ifndef CANARD_H
#define CANARD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

/// Build configuration header. Use it to provide your overrides.
#if defined(CANARD_ENABLE_CUSTOM_BUILD_CONFIG) && CANARD_ENABLE_CUSTOM_BUILD_CONFIG
# include "canard_build_config.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/// Libcanard version. API will be backwards compatible within the same major version.
#define CANARD_VERSION_MAJOR                        0
#define CANARD_VERSION_MINOR                        2


#ifndef CANARD_ENABLE_CANFD
#define CANARD_ENABLE_CANFD                         0
#endif

#ifndef CANARD_MULTI_IFACE
#define CANARD_MULTI_IFACE                          0
#endif

#ifndef CANARD_ENABLE_DEADLINE
#define CANARD_ENABLE_DEADLINE                      0
#endif

#ifndef CANARD_ENABLE_TAO_OPTION
#if CANARD_ENABLE_CANFD
#define CANARD_ENABLE_TAO_OPTION                    1
#else
#define CANARD_ENABLE_TAO_OPTION                    0
#endif
#endif

/// By default this macro resolves to the standard assert(). The user can redefine this if necessary.
#ifndef CANARD_ASSERT
#ifdef CANARD_ENABLE_ASSERTS
# define CANARD_ASSERT(x) assert(x)
#else
# define CANARD_ASSERT(x)
#endif
#endif // CANARD_ASSERT

#define CANARD_GLUE(a, b)           CANARD_GLUE_IMPL_(a, b)
#define CANARD_GLUE_IMPL_(a, b)     a##b

/// By default this macro expands to static_assert if supported by the language (C11, C++11, or newer).
/// The user can redefine this if necessary.
#ifndef CANARD_STATIC_ASSERT
# if (defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)) ||\
     (defined(__cplusplus) && (__cplusplus >= 201103L))
#  define CANARD_STATIC_ASSERT(...) static_assert(__VA_ARGS__)
# else
#  define CANARD_STATIC_ASSERT(x, ...) typedef char CANARD_GLUE(_static_assertion_, __LINE__)[(x) ? 1 : -1]
# endif
#endif

#ifndef CANARD_ALLOCATE_SEM
#define CANARD_ALLOCATE_SEM 0
#endif
/// Error code definitions; inverse of these values may be returned from API calls.
#define CANARD_OK                                      0
// Value 1 is omitted intentionally, since -1 is often used in 3rd party code
#define CANARD_ERROR_INVALID_ARGUMENT                  2
#define CANARD_ERROR_OUT_OF_MEMORY                     3
#define CANARD_ERROR_NODE_ID_NOT_SET                   4
#define CANARD_ERROR_INTERNAL                          9
#define CANARD_ERROR_RX_INCOMPATIBLE_PACKET            10
#define CANARD_ERROR_RX_WRONG_ADDRESS                  11
#define CANARD_ERROR_RX_NOT_WANTED                     12
#define CANARD_ERROR_RX_MISSED_START                   13
#define CANARD_ERROR_RX_WRONG_TOGGLE                   14
#define CANARD_ERROR_RX_UNEXPECTED_TID                 15
#define CANARD_ERROR_RX_SHORT_FRAME                    16
#define CANARD_ERROR_RX_BAD_CRC                        17

/// The size of a memory block in bytes.
#if CANARD_ENABLE_CANFD
#define CANARD_MEM_BLOCK_SIZE                       128U
#elif CANARD_ENABLE_DEADLINE
#define CANARD_MEM_BLOCK_SIZE                       40U
#else
#define CANARD_MEM_BLOCK_SIZE                       32U
#endif

#define CANARD_CAN_FRAME_MAX_DATA_LEN               8U
#if CANARD_ENABLE_CANFD
#define CANARD_CANFD_FRAME_MAX_DATA_LEN             64U
#endif

/// Node ID values. Refer to the specification for more info.
#define CANARD_BROADCAST_NODE_ID                    0
#define CANARD_MIN_NODE_ID                          1
#define CANARD_MAX_NODE_ID                          127

/// Refer to the type CanardRxTransfer
#define CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE      (CANARD_MEM_BLOCK_SIZE - offsetof(CanardRxState, buffer_head))

/// Refer to the type CanardBufferBlock
#define CANARD_BUFFER_BLOCK_DATA_SIZE               (CANARD_MEM_BLOCK_SIZE - offsetof(CanardBufferBlock, data))

/// Refer to canardCleanupStaleTransfers() for details.
#define CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC     1000000U

/// Transfer priority definitions
#define CANARD_TRANSFER_PRIORITY_HIGHEST            0
#define CANARD_TRANSFER_PRIORITY_HIGH               8
#define CANARD_TRANSFER_PRIORITY_MEDIUM             16
#define CANARD_TRANSFER_PRIORITY_LOW                24
#define CANARD_TRANSFER_PRIORITY_LOWEST             31

/// Related to CanardCANFrame
#define CANARD_CAN_EXT_ID_MASK                      0x1FFFFFFFU
#define CANARD_CAN_STD_ID_MASK                      0x000007FFU
#define CANARD_CAN_FRAME_EFF                        (1UL << 31U)         ///< Extended frame format
#define CANARD_CAN_FRAME_RTR                        (1UL << 30U)         ///< Remote transmission (not used by UAVCAN)
#define CANARD_CAN_FRAME_ERR                        (1UL << 29U)         ///< Error frame (not used by UAVCAN)

#define CANARD_TRANSFER_PAYLOAD_LEN_BITS            10U
#define CANARD_MAX_TRANSFER_PAYLOAD_LEN             ((1U << CANARD_TRANSFER_PAYLOAD_LEN_BITS) - 1U)

#ifndef CANARD_64_BIT
#ifdef __WORDSIZE
#define CANARD_64_BIT (__WORDSIZE == 64)
#else
#define CANARD_64_BIT 0
#endif
#endif

/*
  canard_buffer_idx_t is used to avoid pointers in data structures
  that have to have the same size on both 32 bit and 64 bit
  platforms. It is an index into mem_arena passed to canardInit
  treated as a uint8_t array

  A value of CANARD_BUFFER_IDX_NONE means a NULL pointer
 */
#if CANARD_64_BIT
typedef uint32_t canard_buffer_idx_t;
#define CANARD_BUFFER_IDX_NONE 0U
#else
typedef void* canard_buffer_idx_t;
#define CANARD_BUFFER_IDX_NONE NULL
#endif


    
/**
 * This data type holds a standard CAN 2.0B data frame with 29-bit ID.
 */
typedef struct
{
    /**
     * Refer to the following definitions:
     *  - CANARD_CAN_FRAME_EFF
     *  - CANARD_CAN_FRAME_RTR
     *  - CANARD_CAN_FRAME_ERR
     */
    uint32_t id;
#if CANARD_ENABLE_DEADLINE
    uint64_t deadline_usec;
#endif
#if CANARD_ENABLE_CANFD
    uint8_t data[CANARD_CANFD_FRAME_MAX_DATA_LEN];
#else
    uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
#endif
    uint8_t data_len;
    uint8_t iface_id;
#if CANARD_MULTI_IFACE
    uint8_t iface_mask;
#endif
#if CANARD_ENABLE_CANFD
    bool canfd;
#endif
} CanardCANFrame;

/**
 * Transfer types are defined by the UAVCAN specification.
 */
typedef enum
{
    CanardTransferTypeResponse  = 0,
    CanardTransferTypeRequest   = 1,
    CanardTransferTypeBroadcast = 2
} CanardTransferType;

/**
 * Types of service transfers. These are not applicable to message transfers.
 */
typedef enum
{
    CanardResponse,
    CanardRequest
} CanardRequestResponse;

/*
 * Forward declarations.
 */
typedef struct CanardInstance CanardInstance;
typedef struct CanardRxTransfer CanardRxTransfer;
typedef struct CanardRxState CanardRxState;
typedef struct CanardTxQueueItem CanardTxQueueItem;

/**
 * This struture provides information about encoded dronecan frame that needs
 * to be put on the wire.
 * 
 * In case of broadcast or request pointer to the Transfer ID should point to a persistent variable
 * (e.g. static or heap allocated, not on the stack); it will be updated by the library after every transmission. 
 * The Transfer ID value cannot be shared between transfers that have different descriptors!
 * More on this in the transport layer specification.
 * 
 * For the case of response, the pointer to the Transfer ID is treated as const and generally points to transfer id
 * in CanardRxTransfer structure.
 * 
 */
typedef struct {
    CanardTransferType transfer_type; ///< Type of transfer: CanardTransferTypeBroadcast, CanardTransferTypeRequest, CanardTransferTypeResponse
    uint64_t data_type_signature; ///< Signature of the message/service
    uint16_t data_type_id; ///< ID of the message/service
    uint8_t* inout_transfer_id; ///< Transfer ID reference
    uint8_t priority; ///< Priority of the transfer
    const uint8_t* payload; ///< Pointer to the payload
    uint16_t payload_len; ///< Length of the payload
#if CANARD_ENABLE_CANFD
    bool canfd; ///< True if CAN FD is enabled
#endif
#if CANARD_ENABLE_DEADLINE
    uint64_t deadline_usec; ///< Deadline in microseconds
#endif
#if CANARD_MULTI_IFACE
    uint8_t iface_mask; ///< Bitmask of interfaces to send the transfer on
#endif
#if CANARD_ENABLE_TAO_OPTION
    bool tao; ///< True if tail array optimization is enabled
#endif
} CanardTxTransfer;

struct CanardTxQueueItem
{
    CanardTxQueueItem* next;
    CanardCANFrame frame;
};
CANARD_STATIC_ASSERT(sizeof(CanardTxQueueItem) <= CANARD_MEM_BLOCK_SIZE, "Unexpected memory block size");
/**
 * The application must implement this function and supply a pointer to it to the library during initialization.
 * The library calls this function to determine whether the transfer should be received.
 *
 * If the application returns true, the value pointed to by 'out_data_type_signature' must be initialized with the
 * correct data type signature, otherwise transfer reception will fail with CRC mismatch error. Please refer to the
 * specification for more details about data type signatures. Signature for any data type can be obtained in many
 * ways; for example, using the command line tool distributed with Libcanard (see the repository).
 */
typedef bool (* CanardShouldAcceptTransfer)(const CanardInstance* ins,          ///< Library instance
                                            uint64_t* out_data_type_signature,  ///< Must be set by the application!
                                            uint16_t data_type_id,              ///< Refer to the specification
                                            CanardTransferType transfer_type,   ///< Refer to CanardTransferType
                                            uint8_t source_node_id);            ///< Source node ID or Broadcast (0)

/**
 * This function will be invoked by the library every time a transfer is successfully received.
 * If the application needs to send another transfer from this callback, it is highly recommended
 * to call canardReleaseRxTransferPayload() first, so that the memory that was used for the block
 * buffer can be released and re-used by the TX queue.
 */
typedef void (* CanardOnTransferReception)(CanardInstance* ins,                 ///< Library instance
                                           CanardRxTransfer* transfer);         ///< Ptr to temporary transfer object

/**
 * INTERNAL DEFINITION, DO NOT USE DIRECTLY.
 * A memory block used in the memory block allocator.
 */
typedef union CanardPoolAllocatorBlock_u
{
    char bytes[CANARD_MEM_BLOCK_SIZE];
    union CanardPoolAllocatorBlock_u* next;
} CanardPoolAllocatorBlock;

/**
 * This structure provides usage statistics of the memory pool allocator.
 * This data helps to evaluate whether the allocated memory is sufficient for the application.
 */
typedef struct
{
    uint16_t capacity_blocks;               ///< Pool capacity in number of blocks
    uint16_t current_usage_blocks;          ///< Number of blocks that are currently allocated by the library
    uint16_t peak_usage_blocks;             ///< Maximum number of blocks used since initialization
} CanardPoolAllocatorStatistics;

/**
 * INTERNAL DEFINITION, DO NOT USE DIRECTLY.
 * Buffer block for received data.
 */
typedef struct CanardBufferBlock
{
    struct CanardBufferBlock* next;
    uint8_t data[];
} CanardBufferBlock;

/**
 * INTERNAL DEFINITION, DO NOT USE DIRECTLY.
 */
typedef struct
{
    // user should initialize semaphore after the canardInit
    // or at first call of canard_allocate_sem_take
    void *semaphore;
    CanardPoolAllocatorBlock* free_list;
    CanardPoolAllocatorStatistics statistics;
    void *arena;
} CanardPoolAllocator;


/**
 * INTERNAL DEFINITION, DO NOT USE DIRECTLY.
 */
struct CanardRxState
{
    canard_buffer_idx_t next;
    canard_buffer_idx_t buffer_blocks;

    uint64_t timestamp_usec;

    const uint32_t dtid_tt_snid_dnid;

    // We're using plain 'unsigned' here, because C99 doesn't permit explicit field type specification
    unsigned calculated_crc : 16;
    unsigned payload_len    : CANARD_TRANSFER_PAYLOAD_LEN_BITS;
    unsigned transfer_id    : 5;
    unsigned next_toggle    : 1;    // 16+10+5+1 = 32, aligned.

    uint16_t payload_crc;
    uint8_t  iface_id;
    uint8_t buffer_head[];
};
CANARD_STATIC_ASSERT(offsetof(CanardRxState, buffer_head) <= 27, "Invalid memory layout");
CANARD_STATIC_ASSERT(CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE >= 5, "Invalid memory layout");

/**
 * This is the core structure that keeps all of the states and allocated resources of the library instance.
 * The application should never access any of the fields directly! Instead, API functions should be used.
 */
struct CanardInstance
{
    uint8_t node_id;                                ///< Local node ID; may be zero if the node is anonymous

    CanardShouldAcceptTransfer should_accept;       ///< Function to decide whether the application wants this transfer
    CanardOnTransferReception on_reception;         ///< Function the library calls after RX transfer is complete

    CanardPoolAllocator allocator;                  ///< Pool allocator

    CanardRxState* rx_states;                       ///< RX transfer states
    CanardTxQueueItem* tx_queue;                    ///< TX frames awaiting transmission

    void* user_reference;                           ///< User pointer that can link this instance with other objects

#if CANARD_ENABLE_TAO_OPTION
    bool tao_disabled;                              ///< True if TAO is disabled
#endif
};

/**
 * This structure represents a received transfer for the application.
 * An instance of it is passed to the application via callback when the library receives a new transfer.
 * Pointers to the structure and all its fields are invalidated after the callback returns.
 */
struct CanardRxTransfer
{
    /**
     * Timestamp at which the first frame of this transfer was received.
     */
    uint64_t timestamp_usec;

    /**
     * Payload is scattered across three storages:
     *  - Head points to CanardRxState.buffer_head (length of which is up to CANARD_PAYLOAD_HEAD_SIZE), or to the
     *    payload field (possibly with offset) of the last received CAN frame.
     *
     *  - Middle is located in the linked list of dynamic blocks (only for multi-frame transfers).
     *
     *  - Tail points to the payload field (possibly with offset) of the last received CAN frame
     *    (only for multi-frame transfers).
     *
     * The tail offset depends on how much data of the last frame was accommodated in the last allocated block.
     *
     * For single-frame transfers, middle and tail will be NULL, and the head will point at first byte
     * of the payload of the CAN frame.
     *
     * In simple cases it should be possible to get data directly from the head and/or tail pointers.
     * Otherwise it is advised to use canardDecodeScalar().
     */
    const uint8_t* payload_head;            ///< Always valid, i.e. not NULL.
                                            ///< For multi frame transfers, the maximum size is defined in the constant
                                            ///< CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE.
                                            ///< For single-frame transfers, the size is defined in the
                                            ///< field payload_len.
    CanardBufferBlock* payload_middle;      ///< May be NULL if the buffer was not needed. Always NULL for single-frame
                                            ///< transfers.
    const uint8_t* payload_tail;            ///< Last bytes of multi-frame transfers. Always NULL for single-frame
                                            ///< transfers.
    uint16_t payload_len;                   ///< Effective length of the payload in bytes.

    /**
     * These fields identify the transfer for the application.
     */
    uint16_t data_type_id;                  ///< 0 to 255 for services, 0 to 65535 for messages
    uint8_t transfer_type;                  ///< See CanardTransferType
    uint8_t transfer_id;                    ///< 0 to 31
    uint8_t priority;                       ///< 0 to 31
    uint8_t source_node_id;                 ///< 1 to 127, or 0 if the source is anonymous
#if CANARD_ENABLE_TAO_OPTION
    bool tao;
#endif
#if CANARD_ENABLE_CANFD
    bool canfd;                             ///< frame canfd
#endif
};

/**
 * Initializes a library instance.
 * Local node ID will be set to zero, i.e. the node will be anonymous.
 *
 * Typically, size of the memory pool should not be less than 1K, although it depends on the application. The
 * recommended way to detect the required pool size is to measure the peak pool usage after a stress-test. Refer to
 * the function canardGetPoolAllocatorStatistics().
 */
void canardInit(CanardInstance* out_ins,                    ///< Uninitialized library instance
                void* mem_arena,                            ///< Raw memory chunk used for dynamic allocation
                size_t mem_arena_size,                      ///< Size of the above, in bytes
                CanardOnTransferReception on_reception,     ///< Callback, see CanardOnTransferReception
                CanardShouldAcceptTransfer should_accept,   ///< Callback, see CanardShouldAcceptTransfer
                void* user_reference);                      ///< Optional pointer for user's convenience, can be NULL

/**
 * Returns the value of the user pointer.
 * The user pointer is configured once during initialization.
 * It can be used to store references to any user-specific data, or to link the instance object with C++ objects.
 */
void* canardGetUserReference(const CanardInstance* ins);

/**
 * Assigns a new node ID value to the current node.
 * Node ID can be assigned only once.
 */
void canardSetLocalNodeID(CanardInstance* ins,
                          uint8_t self_node_id);

/**
 * Returns node ID of the local node.
 * Returns zero (broadcast) if the node ID is not set, i.e. if the local node is anonymous.
 */
uint8_t canardGetLocalNodeID(const CanardInstance* ins);

/**
 * Forgets the current node ID value so that a new Node ID can be assigned.
 */
void canardForgetLocalNodeID(CanardInstance* ins);

/**
 * Initialise TX transfer object.
 * Should be called at least once before using transfer object to send transmissions.
*/
void canardInitTxTransfer(CanardTxTransfer* transfer);

/**
 * Sends a broadcast transfer.
 * If the node is in passive mode, only single frame transfers will be allowed (they will be transmitted as anonymous).
 *
 * For anonymous transfers, maximum data type ID (CanardTxTransfer::data_type_id) is limited to 3 (see specification for details).
 *
 * Please refer to the specification for more details about data type signatures (CanardTxTransfer::data_type_signature). Signature for 
 * any data type can be obtained in many ways; for example, using the generated code generated using dronecan_dsdlc (see the repository).
 *
 * Use CanardTxTransfer structure to pass the transfer parameters. The structure is initialized by the
 * canardInitTxTransfer() function.
 * 
 * Pointer to the Transfer ID (CanardTxTransfer::inout_transfer_id) should point to a persistent variable
 * (e.g. static or heap allocated, not on the stack); it will be updated by the library after every transmission. 
 * The Transfer ID value cannot be shared between transfers that have different descriptors!
 * More on this in the transport layer specification.
 *
 * Returns the number of frames enqueued, or negative error code.
 */

int16_t canardBroadcastObj(CanardInstance* ins,            ///< Library instance
                           CanardTxTransfer* transfer      ///< Transfer object
                          );

// Legacy API, try to avoid using it, as this will not be extended with new features
int16_t canardBroadcast(CanardInstance* ins,            ///< Library instance
                        uint64_t data_type_signature,   ///< See above
                        uint16_t data_type_id,          ///< Refer to the specification
                        uint8_t* inout_transfer_id,     ///< Pointer to a persistent variable containing the transfer ID
                        uint8_t priority,               ///< Refer to definitions CANARD_TRANSFER_PRIORITY_*
                        const void* payload,            ///< Transfer payload
                        uint16_t payload_len            ///< Length of the above, in bytes
#if CANARD_ENABLE_DEADLINE
                        ,uint64_t tx_deadline           ///< Transmission deadline, microseconds
#endif
#if CANARD_MULTI_IFACE
                        ,uint8_t iface_mask               ///< Bitmask of interfaces to transmit on
#endif
#if CANARD_ENABLE_CANFD
                        ,bool canfd                      ///< Is the frame canfd
#endif
                        );
/**
 * Sends a request or a response transfer.
 * Fails if the node is in passive mode.
 *
 * Please refer to the specification for more details about data type signatures (CanardTxTransfer::data_type_signature). Signature for 
 * any data type can be obtained in many ways; for example, using the generated code generated using dronecan_dsdlc (see the repository).
 *
 * Pointer to the Transfer ID (CanardTxTransfer::inout_transfer_id) should point to a persistent variable
 * (e.g. static or heap allocated, not on the stack); it will be updated by the library after every request.
 * The Transfer ID value cannot be shared between requests that have different descriptors!
 * More on this in the transport layer specification.
 *
 * For Response transfers, the pointer to the Transfer ID(CanardTxTransfer::inout_transfer_id) will be treated as const (i.e. read-only),
 * and normally it should point to the transfer_id field of the structure CanardRxTransfer.
 *
 * Returns the number of frames enqueued, or negative error code.
 */

int16_t canardRequestOrRespondObj(CanardInstance* ins,             ///< Library instance
                                  uint8_t destination_node_id,     ///< Node ID of the server/client
                                  CanardTxTransfer* transfer       ///< Transfer object
                                );
// Legacy API, try to avoid using it, as this will not be extended with new features
int16_t canardRequestOrRespond(CanardInstance* ins,             ///< Library instance
                               uint8_t destination_node_id,     ///< Node ID of the server/client
                               uint64_t data_type_signature,    ///< See above
                               uint8_t data_type_id,            ///< Refer to the specification
                               uint8_t* inout_transfer_id,      ///< Pointer to a persistent variable with transfer ID
                               uint8_t priority,                ///< Refer to definitions CANARD_TRANSFER_PRIORITY_*
                               CanardRequestResponse kind,      ///< Refer to CanardRequestResponse
                               const void* payload,             ///< Transfer payload
                               uint16_t payload_len             ///< Length of the above, in bytes
#if CANARD_ENABLE_DEADLINE
                               ,uint64_t tx_deadline            ///< Transmission deadline, microseconds
#endif
#if CANARD_MULTI_IFACE
                               ,uint8_t iface_mask               ///< Bitmask of interfaces to transmit on
#endif
#if CANARD_ENABLE_CANFD
                                ,bool canfd                     ///< Is the frame canfd
#endif
                            );
/**
 * Returns a pointer to the top priority frame in the TX queue.
 * Returns NULL if the TX queue is empty.
 * The application will call this function after canardBroadcast() or canardRequestOrRespond() to transmit generated
 * frames over the CAN bus.
 */
CanardCANFrame* canardPeekTxQueue(const CanardInstance* ins);

/**
 * Returns the timeout for the frame on top of TX queue.
 * Returns zero if the TX queue is empty.
 * The application will call this function after canardPeekTxQueue() to determine when to call canardPopTxQueue(), if
 * the frame is not transmitted.
 */
#if CANARD_ENABLE_DEADLINE
uint64_t canardPeekTxQueueDeadline(const CanardInstance* ins);
#endif
/**
 * Removes the top priority frame from the TX queue.
 * The application will call this function after canardPeekTxQueue() once the obtained frame has been processed.
 * Calling canardBroadcast() or canardRequestOrRespond() between canardPeekTxQueue() and canardPopTxQueue()
 * is NOT allowed, because it may change the frame at the top of the TX queue.
 */
void canardPopTxQueue(CanardInstance* ins);

/**
 * Processes a received CAN frame with a timestamp.
 * The application will call this function when it receives a new frame from the CAN bus.
 *
 * Return value will report any errors in decoding packets.
 */
int16_t canardHandleRxFrame(CanardInstance* ins,
                            const CanardCANFrame* frame,
                            uint64_t timestamp_usec);

/**
 * Traverses the list of transfers and removes those that were last updated more than timeout_usec microseconds ago.
 * This function must be invoked by the application periodically, about once a second.
 * Also refer to the constant CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC.
 */
void canardCleanupStaleTransfers(CanardInstance* ins,
                                 uint64_t current_time_usec);

/**
 * This function can be used to extract values from received UAVCAN transfers. It decodes a scalar value -
 * boolean, integer, character, or floating point - from the specified bit position in the RX transfer buffer.
 * Simple single-frame transfers can also be parsed manually.
 *
 * Returns the number of bits successfully decoded, which may be less than requested if operation ran out of
 * buffer boundaries, or negated error code, such as invalid argument.
 *
 * Caveat:  This function works correctly only on platforms that use two's complement signed integer representation.
 *          I am not aware of any modern microarchitecture that uses anything else than two's complement, so it should
 *          not affect portability in any way.
 *
 * The type of value pointed to by 'out_value' is defined as follows:
 *
 *  | bit_length | value_is_signed | out_value points to                      |
 *  |------------|-----------------|------------------------------------------|
 *  | 1          | false           | bool (may be incompatible with uint8_t!) |
 *  | 1          | true            | N/A                                      |
 *  | [2, 8]     | false           | uint8_t, or char                         |
 *  | [2, 8]     | true            | int8_t, or char                          |
 *  | [9, 16]    | false           | uint16_t                                 |
 *  | [9, 16]    | true            | int16_t                                  |
 *  | [17, 32]   | false           | uint32_t                                 |
 *  | [17, 32]   | true            | int32_t, or 32-bit float                 |
 *  | [33, 64]   | false           | uint64_t                                 |
 *  | [33, 64]   | true            | int64_t, or 64-bit float                 |
 */
int16_t canardDecodeScalar(const CanardRxTransfer* transfer,    ///< The RX transfer where the data will be copied from
                           uint32_t bit_offset,                 ///< Offset, in bits, from the beginning of the transfer
                           uint8_t bit_length,                  ///< Length of the value, in bits; see the table
                           bool value_is_signed,                ///< True if the value can be negative; see the table
                           void* out_value);                    ///< Pointer to the output storage; see the table

/**
 * This function can be used to encode values for later transmission in a UAVCAN transfer. It encodes a scalar value -
 * boolean, integer, character, or floating point - and puts it to the specified bit position in the specified
 * contiguous buffer.
 * Simple single-frame transfers can also be encoded manually.
 *
 * Caveat:  This function works correctly only on platforms that use two's complement signed integer representation.
 *          I am not aware of any modern microarchitecture that uses anything else than two's complement, so it should
 *          not affect portability in any way.
 *
 * The type of value pointed to by 'value' is defined as follows:
 *
 *  | bit_length | value points to                          |
 *  |------------|------------------------------------------|
 *  | 1          | bool (may be incompatible with uint8_t!) |
 *  | [2, 8]     | uint8_t, int8_t, or char                 |
 *  | [9, 16]    | uint16_t, int16_t                        |
 *  | [17, 32]   | uint32_t, int32_t, or 32-bit float       |
 *  | [33, 64]   | uint64_t, int64_t, or 64-bit float       |
 */
void canardEncodeScalar(void* destination,      ///< Destination buffer where the result will be stored
                        uint32_t bit_offset,    ///< Offset, in bits, from the beginning of the destination buffer
                        uint8_t bit_length,     ///< Length of the value, in bits; see the table
                        const void* value);     ///< Pointer to the value; see the table

/**
 * This function can be invoked by the application to release pool blocks that are used
 * to store the payload of the transfer.
 *
 * If the application needs to send new transfers from the transfer reception callback, this function should be
 * invoked right before calling canardBroadcast() or canardRequestOrRespond(). Not releasing the buffers before
 * transmission may cause higher peak usage of the memory pool.
 *
 * If the application didn't call this function before returning from the callback, the library will do that,
 * so it is guaranteed that the memory will not leak.
 */
void canardReleaseRxTransferPayload(CanardInstance* ins,
                                    CanardRxTransfer* transfer);

/**
 * Returns a copy of the pool allocator usage statistics.
 * Refer to the type CanardPoolAllocatorStatistics.
 * Use this function to determine worst case memory needs of your application.
 */
CanardPoolAllocatorStatistics canardGetPoolAllocatorStatistics(CanardInstance* ins);

/**
 * Float16 marshaling helpers.
 * These functions convert between the native float and 16-bit float.
 * It is assumed that the native float is IEEE 754 single precision float, otherwise results will be unpredictable.
 * Vast majority of modern computers and microcontrollers use IEEE 754, so this limitation should not affect
 * portability.
 */
uint16_t canardConvertNativeFloatToFloat16(float value);
float canardConvertFloat16ToNativeFloat(uint16_t value);

uint16_t extractDataType(uint32_t id);
CanardTransferType extractTransferType(uint32_t id);

/// Abort the build if the current platform is not supported.
#if CANARD_ENABLE_CANFD
CANARD_STATIC_ASSERT(((uint32_t)CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE) < 128,
                     "Please define CANARD_64_BIT=1 for 64 bit builds");
#else
CANARD_STATIC_ASSERT(((uint32_t)CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE) < 32,
                     "Please define CANARD_64_BIT=1 for 64 bit builds");
#endif

#if CANARD_ALLOCATE_SEM
// user implemented functions for taking and freeing semaphores
void canard_allocate_sem_take(CanardPoolAllocator *allocator);
void canard_allocate_sem_give(CanardPoolAllocator *allocator);
#endif

#ifdef __cplusplus
}
#endif
#endif
