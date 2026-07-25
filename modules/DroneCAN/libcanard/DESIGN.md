# libcanard design document

## Design goals

The following list contains main design goals in the order of importance.

* *Small ROM footprint*.
    A minimal node that periodically publishes `uavcan.protocol.NodeStatus` and responds to `uavcan.protocol.GetNodeInfo` should not require more than 4K of ROM.
    For reference, a similar application based on libuavcan requires 19K of ROM (LPC11C24).
* *Small RAM footprint*.
    A node like in the example above should not require more than 4K of RAM, including the stack and buffers.
    For reference, a similar application based on libuavcan requires about 6K of RAM.
* *Determinism*.
    Worst case execution time of all code paths should be predictable, which precludes use of heap.
* *Portability*.
    The library should not present any specific requirements to the underlying hardware or OS, and it must be coded in standard C99.
    However, the portability requirement can be superseded by other design goals, such as small memory footprint.
* *Simplicity*.
    Unlike libuavcan, which somewhat resembles a framework rather than just a library, this project should not attempt to implement all of the high-level functionality of UAVCAN.


## Feature set

According to the core design goals defined above, the functionality of the library should be restricted to the bare minimum.
The following features are considered to comprise the bare minimum:

* Publication and reception of message transfers.
* Publication and reception of service request and service response transfers.
* Configurable Data Type ID of supported messages and services, at run time.
* Support for vendor-specific data types.

The following features are intentionally not supported by the library:

* Support for redundant physical interfaces.
    Leaving out redundant interfaces allows to significantly simplify the implementation of the transport layer.
* RPC-like abstraction on top of service request/response exchanges.
    This means that services will be supported simply as independent request and response transfers, unlike the way it is implemented in libuavcan, where a convenient high-level abstraction is provided.
* Time synchronization master.
    This feature requires some special logic to be supported by the CAN driver and the library itself.
    Applications that require this functionality are likely to be able to tolerate the memory requirements of libuavcan.
* Multithreaded nodes.
    Again, this is implemented in libuavcan and is unlikely to be demanded by low-end applications.
* Support for platforms with 64-bit pointers.
    Vast majority of deeply embedded systems use 32/16/8-bit CPU.
    Systems that are based on AMD64 can still be supported by means of x86 compatibility mode (although it is recommended to use libuavcan instead).

# Architecture

## Memory management

### Library state
Entire state of the library should be kept in one instance of a C structure.
Every API call of the library that depends on the state will be accepting the aforementioned instance as its first argument.

### Dynamic memory pool
The library should implement a block memory allocator that will be used by the following subsystems (each is described below):

* Incoming transfer buffers.
* Incoming transfer states.
* Prioritized TX queue.

The number of blocks in the pool will be defined at compile time.
32 bytes is probably the optimal choice considering typical object sizes (see below).
For reference, libuavcan uses 64-byte blocks.

Implementation of the block allocation algorithm can be borrowed from libuavcan.


### Transfer buffers
Transfer buffers should be implemented as a singly-linked lists of blocks, where every block is an instance of the following structure:

```c
typedef struct CanardBufferBlock
{
    struct CanardBufferBlock* next;
    uint8_t data[];
} CanardBufferBlock;

#define CANARD_BUFFER_BLOCK_DATA_SIZE (CANARD_MEM_BLOCK_SIZE - sizeof(CanardBufferBlock))
```

Where `CANARD_MEM_BLOCK_SIZE` is the allocation size (32 bytes).

According to the transport layer specification, the following operations must be defined for buffers:

* Push bytes.
    This operation adds a number of bytes (1 to 8 or 1 to 64, depending on the CAN standard used) to the end of the buffer.
    It will be invoked during reception of multi-frame transfers.
* Pop bits.
    This operation is used during deserialization of received payload.
    It returns a number of bits (1 to 64, depending on the type of the field being deserialized) from the buffer to the caller.

New blocks should be allocated ad hoc, however their removal should happen at once, after the data is processed upon completion of transfer reception.

### Buffer Indexes

To allow for building on both 32 bit and 64 bit systems we use a
canard_buffer_idx_t to represent an index into the memory pool. The
functions CanardBufferFromIdx() CanardBufferToIdx(), CanardRxFromIdx()
and CanardRxToIdx() are used to convert from and to indexes and the
associated pointers.

### RX transfer states

The library will have to keep some state associated with every unique incoming transfer.
The concept of unique transfer is explained in the specification here.
Every unique incoming transfer can be identified with the following four values:

* Data type ID
* Transfer type
* Source node ID
* Destination node ID (zero for broadcast transfers)

Upon reception of a CAN frame, the library will have to check whether the frame should be accepted and how it should be processed.
Detailed description of the logic is available in the specification, here we just define a C structure that keeps all of the relevant states.
Such a structure will have to be instantiated and maintained for every unique incoming transfer that the library is interested in.
Since the number of unique incoming transfers cannot be determined statically, these structures will be allocated at run time using the memory pool.
Hence, size of the structure must not exceed the size of the allocatable block (32 bytes).

```c
typedef struct CanardRxState
{
    canard_buffer_idx_t next_idx;
    canard_buffer_idx_t buffer_blocks_idx;

    uint64_t timestamp_usec;

    const uint32_t dtid_tt_snid_dnid;

    uint16_t payload_crc;
    uint16_t payload_len : 10;
    uint16_t transfer_id : 5;
    uint16_t next_toggle : 1;

    uint8_t buffer_head[];
} CanardRxState;

#define CANARD_MULTIFRAME_RX_PAYLOAD_HEAD_SIZE (CANARD_MEM_BLOCK_SIZE - sizeof(CanardRxState))
```

Few things to note:

* `timestamp_usec` keeps the timestamp at which the transfer that is currently being received was started, i.e. when the first frame of it was received.
    This value is needed for detection and removal of stale RX state objects, and it is also passed to the application as a transfer reception timestamp.
* The last field of the structure is buffer_head, size of which is 32 - sizeof(CanardRxState).
    It is intended for keeping first few bytes of incoming transfers.

Value of the field dtid_tt_snid_dnid can be computed with the following helper macro:

```c
#define CANARD_MAKE_TRANSFER_DESCRIPTOR(data_type_id, transfer_type, \
                                        src_node_id, dst_node_id) \
    ((data_type_id) | ((transfer_type) << 16) | \
     ((src_node_id) << 18) | ((dst_node_id) << 25))
```

Keeping the transfer ID in a single scalar rather than in separate fields should be beneficial in terms of ROM footprint and linked list search speed.

Using the concepts defined above, the frame reception procedure can be defined roughly as follows:


* Check if the application is interested in receiving a transfer with data type ID and transfer type of the newly received frame.
    If not, exit.
* Check if there’s an RX state instance for this combination of (data type ID, transfer type, source node ID, destination node ID).
    If not, instantiate one. If instantiation fails due to lack of memory, exit.
* Check if the frame matches the expectations of the RX state instance (e.g. toggle bit, transfer ID, payload size, etc - refer to the specification for details).
    If not, exit.
* Update the RX state instance with the new frame, append the payload to the buffer if necessary.
* If the frame is the last one in the transfer, report to the application, then remove all buffered data and prepare the RX state instance for the next transfer.
    Exit.

### TX queue

Frames that are scheduled for transmission should be stored in a prioritized TX queue.
When the CAN driver is ready to accept a frame, the application will take the highest priority frame from the TX queue and pass it to the driver.
The TX queue should be stored as a singly-linked list of CAN frames sorted in the descending order of priority (so that the highest priority frame is always at the root of the list).
There’s an implementation of a function that compares priorities of two CAN frames according to the [CAN specification](https://github.com/UAVCAN/libuavcan/blob/ec9006381b9ed0848eb12704a2beb9e74aea74bf/libuavcan/src/driver/uc_can.cpp).

```c
#define CANARD_CAN_FRAME_MAX_DATA_LEN  8

/**
 * CAN driver IO is based on this type.
 */
typedef struct
{
    uint32_t id;
    uint8_t data[UC_CAN_FRAME_MAX_DATA_LEN];
    uint8_t data_len;
} CanardCANFrame;

#define CANARD_CAN_FRAME_EFF        (1U << 31) ///< Extended frame format
#define CANARD_CAN_FRAME_RTR        (1U << 30) ///< Remote transmission request
#define CANARD_CAN_FRAME_ERR        (1U << 29) ///< Error frame

/**
 * This type is internal to the library, it should not be exposed in the header file.
 */
typedef struct CanardTxQueueItem
{
    struct CanardTxQueueItem* next;
    CanardCANFrame frame;
} CanardTxQueueItem;
```

### Threading model

The library should be single-threaded, not thread-aware.
Hence the API will be not thread-safe, which is OK as most applications will likely be running all of the UAVCAN-related logic in one thread.

The documentation should provide advices about how to integrate the library in a multithreaded environment.

### API

The following list provides a high-level description of the major use cases:

* *Frame reception*.
    On a reception of a frame, the application passes it to the library via a dedicated API function.
    Upon reception of a frame, the library detects its transfer parameters, such as data type ID, transfer type and source node ID.
    If this is the first frame of a transfer, the library then requests the application via a function pointer on whether the transfer should be received.
    If the application reports that the transfer is not of interest, the frame will be ignored.
    Otherwise, the library finds the receiver state instance for the transfer (or creates one if it couldn’t be found), then updates it according to the rules defined in the specification.
    If the newly received frame was accepted and it was the last frame of the transfer, the library will invoke the appropriate callback (via a function pointer) so that the application can execute the associated business logic.
    Upon return from the callback, all of the dynamic memory that was allocated for this transfer, if any, will be automatically released, although the RX state instance will be kept.
* *Transfer transmission*.
    When the application needs to send a transfer, it invokes one of the corresponding functions depending on the type of the transfer (broadcast, request, or response).
    The library breaks the transfer down into independent CAN frames and stores them into the internal TX queue, using the memory pool.
    The application then unloads the frames from the TX queue into the CAN driver using two library calls: peek and pop.
* *Cleanup of stale transfers*.
    The user should periodically invoke an API call that will traverse the list of receiver state instances and remove those that were last updated more than T seconds ago.
    Note that the traversing is computationally inexpensive, as it requires only two operations: 1) switch to the next item; 2) check if the last update timestamp is lower than the current time minus T.
    Recommended value of T is 3 seconds.
* *Data structure serialization and deserialization*.
    Unlike in libuavcan, this functionality should be completely decoupled from the rest of the library.
    Note that from the application side, transfer reception and transmission deals with raw binary chunks rather than with structured data.
    This allows applications to operate on raw data, which is likely to be beneficial in terms of memory footprint and processing time.
    Those applications that are interested in automatic serialization and deserialization should use independently autogenerated serialization and deserialization code, where serialization and deserialization of every data type will be implemented in a tiny header-only library of its own.

The library should provide the following functions for the application:

* *Send a transfer*.
    Encoded transfer will be stored in the prioritized TX queue (as described above).
    This function should accept serialized payload, i.e. not a message object.
* *Peek one frame from the TX queue*.
    This function will be used by the application to move frames from the TX queue into the CAN driver.
* *Remove one frame from the TX queue*.
    This function will be used by the application to remove the frame from the queue once the driver confirmed that it has been accepted for transmission.
* *Process one received frame*.
    This function will be invoked by the application once the CAN driver reports about reception of a frame.
    This function will also contain an argument for the RX timestamp of the frame.

The application should provide the following functions for the library (the application will bind the library to these functions dynamically by means of function pointers):

* *Process received transfer*.
    The library will be passing information about the received transfer and the raw payload (non-deserialized) into the callback so the application can execute the associated business logic.
* *Determine if the transfer should be received*.
    The library will be invoking this function on every reception of a CAN frame to check if it should proceed on reception of the transfer.
    If the application does not need this transfer, the frame will be discarded.

Note that the proposed API does not make any assumptions about the CAN driver interface or its implementation.
A rough draft of the API definitions is provided below:

```c
#define CANARD_BROADCAST_NODE_ID    0
#define CANARD_MIN_NODE_ID          1
#define CANARD_MAX_NODE_ID          127

#define CANARD_MEM_BLOCK_SIZE       32

typedef enum
{
    CanardResponse,
    CanardRequest
} CanardRequestResponse;

typedef enum
{
    CanardTransferTypeResponse  = 0,
    CanardTransferTypeRequest   = 1,
    CanardTransferTypeBroadcast = 2
} CanardTransferType;

/**
 * This structure represents a received transfer for the application.
 * An instance of it is passed to the application via callback when
 * the library receives a new transfer.
 */
typedef struct
{
    /**
     * Timestamp at which the first frame of this transfer was received.
     */
    uint64_t timestamp_usec;

    /**
     * Payload is scattered across three storages:
     *  - Head points to CanardRxState.buffer_head (length of which is up to Canard_PAYLOAD_HEAD_SIZE), or to the
     *    payload field (possibly with offset) of the last received CAN frame.
     *  - Middle is located in the linked list of dynamic blocks.
     *  - Tail points to the payload field (possibly with offset) of the last received CAN frame
     *    (only for multi-frame transfers).
     *
     * The tail offset depends on how much data of the last frame was accommodated in the last allocated
     * block.
     *
     * For single-frame transfers, middle and tail will be NULL, and the head will point at first byte
     * of the payload of the CAN frame.
     *
     * In simple cases it should be possible to get data directly from the head and/or tail pointers.
     * Otherwise it is advised to use canardDecodeScalar().
     */
    const uint8_t*           payload_head;   ///< Always valid, i.e. not NULL.
    const CanardBufferBlock* payload_middle; ///< May be NULL if the buffer was not needed. Always NULL for single-frame transfers.
    const uint8_t*           payload_tail;   ///< Last bytes of multi-frame transfers. Always NULL for single-frame transfers.
    uint16_t payload_len;

    /**
     * These fields identify the transfer for the application logic.
     */
    uint16_t data_type_id;                  ///< 0 to 255 for services, 0 to 65535 for messages
    uint8_t transfer_type;                  ///< See @ref CanardTransferType
    uint8_t transfer_id;                    ///< 0 to 31
    uint8_t priority;                       ///< 0 to 31
    uint8_t source_node_id;                 ///< 1 to 127, or 0 if the source is anonymous
} CanardRxTransfer;

/**
 * Definition is not provided in this draft.
 * This structure should include at least the following:
 * - Two callback function pointers
 * - Local node ID
 * - Memory pool and the associated state variables
 * - List of RX state objects
 * - An optional user-provided untyped pointer to user-specific data
 */
struct CanardInstance;

/**
 * Initializes the library state.
 * Local node ID will be set to zero, i.e. the node will be anonymous.
 */
void canardInit(CanardInstance* out_ins,
                void* mem_arena,
                size_t mem_arena_size,
                CanardOnTransferReception on_reception,
                CanardShouldAcceptTransfer should_accept,
                void* user_reference);

/**
 * Assigns a new node ID value to the current node.
 */
void canardSetLocalNodeID(CanardInstance* ins, uint8_t self_node_id);

/**
 * Returns node ID of the local node.
 * Returns zero if the node ID has not been set.
 */
uint8_t canardGetLocalNodeID(const CanardInstance* ins);

/**
 * Sends a broadcast transfer.
 * If the node is in passive mode, only single-frame transfers will be allowed.
 */
int canardBroadcast(CanardInstance* ins,
                    uint64_t data_type_signature,
                    uint16_t data_type_id,
                    uint8_t* inout_transfer_id,
                    uint8_t priority,
                    const void* payload,
                    uint16_t payload_len);

/**
 * Sends a request or a response transfer.
 * Fails if the node is in passive mode.
 */
int canardRequestOrRespond(CanardInstance* ins,
                           uint8_t destination_node_id,
                           uint64_t data_type_signature,
                           uint16_t data_type_id,
                           uint8_t* inout_transfer_id,
                           uint8_t priority,
                           CanardRequestResponse kind,
                           const void* payload,
                           uint16_t payload_len);

/**
 * Returns a pointer to the top priority frame in the TX queue.
 * Returns NULL if the TX queue is empty.
 */
const CanardCANFrame* canardPeekTxQueue(const CanardInstance* ins);

/**
 * Removes the top priority frame from the TX queue.
 */
void canardPopTxQueue(CanardInstance* ins);

/**
 * Processes a received CAN frame with a timestamp.
 */
void canardHandleRxFrame(CanardInstance* ins,
                         const CanardCANFrame* frame,
                         uint64_t timestamp_usec);

/**
 * Traverses the list of transfers and removes those that were last updated more than
 * timeout_usec microseconds ago.
 */
void canardCleanupStaleTransfers(CanardInstance* ins,
                                 uint64_t timeout_usec,
                                 uint64_t current_time_usec);

/**
 * The library calls this function when it receives first frame of a transfer to determine whether
 * the transfer should be received.
 * If the application returns true, the pointer out_data_type_signature must be written with the data type signature.
 */
typedef bool (*CanardShouldAcceptTransfer)(const CanardInstance* ins,
                                           uint64_t* out_data_type_signature,
                                           uint16_t data_type_id,
                                           CanardTransferType transfer_type,
                                           uint8_t source_node_id);

/**
 * This function will be invoked by the library every time a transfer is successfully received.
 * If the application needs to send another transfer from this callback, it is recommended
 * to call canardReleaseRxTransferPayload() first, so that the memory that was used for the block
 * buffer can be released and re-used by the TX queue.
 */
typedef void (*CanardOnTransferReception)(CanardInstance* ins,
                                          const CanardRxTransfer* transfer);

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
int canardDecodeScalar(const CanardRxTransfer* transfer,    ///< The RX transfer where the data will be copied from
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
 * This function can be invoked by the application to release pool blocks that are used to store
 * the payload of this transfer.
 */
void canardReleaseRxTransferPayload(CanardInstance* ins,
                                    CanardRxTransfer* transfer);
```

### Code generation

As noted above, this shall remain an optional part of the library, as some applications may opt to implement serialization and deserialization manually.
The DSDL compiler should generate a tiny standalone single-header library for every data type the user needs to use in deserialized form.
The generated library should include the following entities:

* A C structure that contains the fields of the target type (see below).
* A deserialization function that accepts a pointer to an CanardRxTransfer object and a pointer to the C structure above.
* A serialization function that accepts a pointer to a raw storage space and a pointer to the C structure above.
* References to other types this type depends on.

The primitive DSDL types will be mapped to C types as follows:

| DSDL type | C type |
|-----------|--------|
| `intX` | `int8_t, int16_t, int32_t, int64_t` |
| `uintX` | `uint8_t, uint16_t, uint32_t, uint64_t` |
| `bool` | `bool (from stdbool.h)` |
| `float16` | `float` |
| `float32` | `float` |
| `float64` | `double` |

A dynamic DSDL array that contains values of type T and has maximum length N will be converted into two fields: one of them will be of type T mapped to the corresponding native C type as defined above; the second field will be of integer type uint16_t.
The first field will be given the same name as the original array, the second field’s name will be appended with “_len”.
An example is provided below.


| DSDL definition | C definition |
|-----------------|--------------|
| `Foo[<5] bars`  | `Foo bars[4]; uint16_t bars_len;`

# Implementation notes

## Packaging

The suggested approach is to keep all of the library’s functions in just one C file, and expose the entire API via just one header file.

## Coding style

Please refer to [the Zubax coding conventions](https://kb.zubax.com/x/84Ah).

## Testing

The library should be equipped with a testing suite (like libuavcan).
The testing suite should be based on the Google Test library (in which case it can be written in C++), or it can be just a dedicated application with a custom testing environment (in which case it is recommended to stick to C99).

The testing suite does not have to be portable - it is quite acceptable to make it require an x86 or AMD64 machine running OS X or Linux.
Since 64-bit systems are not supported by the proposed design, on AMD64 systems the library should be compiled in 32-bit mode (on GCC use flag -m32).

A continuous integration environment like Travis CI should be set up early in the project to run the test suite on each commit / pull request.

## Name
It has been agreed to name the library “libcanard”.

## License
The library must be released under the MIT open source license.
A short summary can be found [on tl;dr legal](http://www.tldrlegal.com/l/mit).
The license statement must declare that the code was developed by the UAVCAN project team.
