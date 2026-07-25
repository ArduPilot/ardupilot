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
 */

#include <gtest/gtest.h>
#include <canard.h>

//Independent implementation of ID layout from https://uavcan.org/Specification/4._CAN_bus_transport_layer/
#define CONSTRUCT_PRIO(prio)        (((prio) & 0x1F) << 24)
#define CONSTRUCT_MTID(mtid)        (((mtid) & 0xFFFF) << 8)
#define CONSTRUCT_SID(sid)          (((sid)  & 0x7F))
#define CONSTRUCT_DISC(disc)        (((disc) & 0x3FFF) << 10)
#define CONSTRUCT_DISC_MTID(mtid)   (((mtid) & 0x3) << 8)
#define CONSTRUCT_STID(stid)        (((stid) & 0xFF) << 16)
#define CONSTRUCT_DID(did)          (((did)  & 0x7F) << 8)
#define CONSTRUCT_REQUEST(request)  (((request) & 0x1) << 15)

#define CONSTRUCT_SNM_BIT (1 << 7)

#define CONSTRUCT_ANON_MSG_ID(prio, disc, mtid, sid)    (CONSTRUCT_PRIO(prio) | \
                                                         CONSTRUCT_DISC_MTID(mtid) | \
                                                         CONSTRUCT_DISC(disc) | \
                                                         CONSTRUCT_SID(sid) \
                                                         CANARD_CAN_FRAME_EFF)

#define CONSTRUCT_MSG_ID(prio, mtid, sid)   (CONSTRUCT_PRIO(prio) | \
                                             CONSTRUCT_MTID(mtid) | \
                                             CONSTRUCT_SID(sid) | \
                                             CANARD_CAN_FRAME_EFF)

#define CONSTRUCT_SVC_ID(prio, stid, request, did, sid) (CONSTRUCT_PRIO(prio) | \
                                                         CONSTRUCT_STID(stid) | \
                                                         CONSTRUCT_REQUEST(request) | \
                                                         CONSTRUCT_DID(did) | \
                                                         CONSTRUCT_SID(sid) | \
                                                         CONSTRUCT_SNM_BIT | \
                                                         CANARD_CAN_FRAME_EFF)

#define CONSTRUCT_START(start) (((start) & 0x1) << 7)
#define CONSTRUCT_END(end) (((end) & 0x1) << 6)
#define CONSTRUCT_TOGGLE(toggle) (((toggle) & 0x1) << 5)
#define CONSTRUCT_TID(tid) (((tid) & 0x1F))

#define CONSTRUCT_TAIL_BYTE(start, end, toggle, tid)    (CONSTRUCT_START(start) | \
                                                         CONSTRUCT_END(end) | \
                                                         CONSTRUCT_TOGGLE(toggle) | \
                                                         CONSTRUCT_TID(tid))

static bool g_should_accept = true;

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
    (void)ins;
    (void)transfer;
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)ins;
    (void)out_data_type_signature;
    (void)data_type_id;
    (void)transfer_type;
    (void)source_node_id;
    return g_should_accept;
}

TEST(canardHandleRxFrame, incompatiblePacketHandlingCorrectness)
{
    uint8_t canard_memory_pool[1024];
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    //Setup frame data to be single frame transfer
    frame.data[0] = CONSTRUCT_TAIL_BYTE(1, 1, 0, 0);

    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    g_should_accept = true;

    //Frame with good RTR/ERR/data_len bad EFF
    frame.id = 0;
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_INCOMPATIBLE_PACKET == err);

    //Frame with good EFF/ERR/data_len, bad RTR
    frame.id = 0 | CANARD_CAN_FRAME_RTR | CANARD_CAN_FRAME_EFF;
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_INCOMPATIBLE_PACKET == err);

    //Frame with good EFF/RTR/data_len, bad ERR
    frame.id = 0 | CANARD_CAN_FRAME_ERR | CANARD_CAN_FRAME_EFF;
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_INCOMPATIBLE_PACKET == err);

    //Frame with good EFF/RTR/ERR, bad data_len
    frame.id = 0 | CANARD_CAN_FRAME_EFF;
    frame.data_len = 0;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_INCOMPATIBLE_PACKET == err);

    //Frame with good EFF/RTR/ERR/data_len
    frame.id = 0 | CANARD_CAN_FRAME_EFF;
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);
}

//"canardHandleRxFrame wrong address handling, Correctness"
TEST(canardHandleRxFrame, wrongAddressHandlingCorrectness)
{
    uint8_t canard_memory_pool[1024] {};
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    //Setup frame data to be single frame transfer
    frame.data[0] = CONSTRUCT_TAIL_BYTE(1, 1, 0, 0);

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);
    g_should_accept = true;

    //Send package with ID 24, should not be wanted
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 24, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_WRONG_ADDRESS == err);

    //Send package with ID 20, should be OK
    frame.id = 0 | CANARD_CAN_FRAME_EFF;            //Set EFF
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);
}

// "canardHandleRxFrame shouldAccept handling, Correctness"
TEST(canardHandleRxFrame, shouldAcceptHandlingCorrectness)
{
    uint8_t canard_memory_pool[1024] {};
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    //Setup frame data to be single frame transfer
    frame.data[0] = CONSTRUCT_TAIL_BYTE(1, 1, 0, 0);

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);

    //Send packet, don't accept
    g_should_accept = false;
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_NOT_WANTED == err);

    //Send packet, accept
    g_should_accept = true;
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);
}

// "canardHandleRxFrame no state handling, Correctness"
TEST(canardHandleRxFrame, noStateHandlingCorrectness)
{
    uint8_t canard_memory_pool[1024] {};
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    g_should_accept = true;

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);

    //Not start or end of packet, should fail
    frame.data[0] = CONSTRUCT_TAIL_BYTE(0, 0, 0, 0);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_MISSED_START == err);

    //End of packet, should fail
    frame.data[0] = CONSTRUCT_TAIL_BYTE(0, 1, 0, 0);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_MISSED_START == err);

    //1 Frame packet, should pass
    frame.data[0] = CONSTRUCT_TAIL_BYTE(1, 1, 0, 0);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

    //Send a start packet, should pass
    frame.data[7] = CONSTRUCT_TAIL_BYTE(1, 0, 0, 1);

    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

    //Send a middle packet, from the same ID, but don't toggle
    frame.data[0] = CONSTRUCT_TAIL_BYTE(0, 0, 0, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 1;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_WRONG_TOGGLE == err);

    //Send a middle packet, toggle, but use wrong ID
    frame.data[7] = CONSTRUCT_TAIL_BYTE(0, 0, 1, 2);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_UNEXPECTED_TID == err);

    //Send a middle packet, toggle, and use correct ID
    frame.data[7] = CONSTRUCT_TAIL_BYTE(0, 0, 1, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

}

// TEST("canardHandleRxFrame missed start handling, Correctness")
TEST(canardHandleRxFrame, missedStartHandlingCorrectness)
{
    uint8_t canard_memory_pool[1024] {};
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    g_should_accept = true;

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);

    //Send a start packet, should pass
    frame.data[7] = CONSTRUCT_TAIL_BYTE(1, 0, 0, 1);

    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

    //Send a middle packet, toggle, and use correct ID - but timeout
    uint8_t tid = 1;
    frame.data[7] = CONSTRUCT_TAIL_BYTE(0, 0, 1, tid++);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;
    err = canardHandleRxFrame(&canard, &frame, 4000000);
    ASSERT_TRUE(-CANARD_ERROR_RX_MISSED_START == err);

    //Send a start packet, should pass
    frame.data[7] = CONSTRUCT_TAIL_BYTE(1, 0, 0, tid++);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

    //Send a middle packet, toggle, and use correct ID - but timestamp 0
    frame.data[7] = CONSTRUCT_TAIL_BYTE(0, 0, 1, tid++);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;
    err = canardHandleRxFrame(&canard, &frame, 0);
    ASSERT_TRUE(-CANARD_ERROR_RX_MISSED_START == err);

    //Send a start packet, should pass
    frame.data[7] = CONSTRUCT_TAIL_BYTE(1, 0, 0, tid++);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

    //Send a middle packet, toggle, and use an incorrect TID
    frame.data[7] = CONSTRUCT_TAIL_BYTE(0, 0, 1, tid--);
    frame.data[7] = 0 | 3 | (1<<5);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;
    err = canardHandleRxFrame(&canard, &frame, 0);
    ASSERT_TRUE(-CANARD_ERROR_RX_MISSED_START == err);
}

// TEST("canardHandleRxFrame short frame handling, Correctness")
TEST(canardHandleRxFrame, shortFrameHandlingCorrectness)
{
    uint8_t canard_memory_pool[1024] {};
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    g_should_accept = true;

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);

    //Send a start packet which is short, should fail
    frame.data[1] = CONSTRUCT_TAIL_BYTE(1, 0, 0, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 2;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_SHORT_FRAME == err);

    //Send a start packet which is short, should fail
    frame.data[2] = CONSTRUCT_TAIL_BYTE(1, 0, 0, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 3;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_RX_SHORT_FRAME == err);
}

// TEST("canardHandleRxFrame OOM handling, Correctness")
TEST(canardHandleRxFrame, oomHandlingCorrectness)
{
    uint8_t dummy_buf;
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    g_should_accept = true;

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, &dummy_buf, 0,
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);

    //Send a start packet - we shouldn't have any memory
    frame.data[7] = CONSTRUCT_TAIL_BYTE(1, 0, 0, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(-CANARD_ERROR_OUT_OF_MEMORY == err);
}

// TEST("canardHandleRxFrame unusual single frame, Correctness")
TEST(canardHandleRxFrame, unusualSingleFrameCorrectness)
{
    uint8_t canard_memory_pool[1024] {};
    CanardInstance canard;
    CanardCANFrame frame {};
    int16_t err;

    g_should_accept = true;

    //Open canard to accept all transfers with a node ID of 20
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, &canard);
    canardSetLocalNodeID(&canard, 20);

    frame.data[7] = CONSTRUCT_TAIL_BYTE(1, 0, 0, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);


    frame.data[7] = CONSTRUCT_TAIL_BYTE(0, 0, 1, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 8;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);

    frame.data[1] = CONSTRUCT_TAIL_BYTE(1, 1, 1, 1);
    frame.id = CONSTRUCT_SVC_ID(0, 0, 1, 20, 0);
    frame.data_len = 2;                             //Data length MUST be full packet
    err = canardHandleRxFrame(&canard, &frame, 1);
    ASSERT_TRUE(CANARD_OK == err);
}
