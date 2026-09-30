#include <AP_gtest.h>
#include <AP_Networking/AP_Networking_PPP_SoftFlow.h>

#include <string.h>

static uint8_t process_byte(AP_Networking_PPP_SoftFlow &flow, uint8_t byte, uint32_t now_ms)
{
    uint8_t output[AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH];
    return flow.process_byte(byte, now_ms, output);
}

static void process_codeword(AP_Networking_PPP_SoftFlow &flow,
                             AP_Networking_PPP_SoftFlow::Command command,
                             uint32_t now_ms)
{
    const uint8_t *word = AP_Networking_PPP_SoftFlow::codeword(command);
    for (uint8_t i = 0; i < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH; i++) {
        (void)process_byte(flow, word[i], now_ms);
    }
}

TEST(AP_Networking_PPP_SoftFlow, SingleByteCannotStop)
{
    AP_Networking_PPP_SoftFlow flow;

    (void)process_byte(flow, 0x13, 10);
    EXPECT_FALSE(flow.transmit_paused(10));

    (void)process_byte(flow, 0x11, 11);
    EXPECT_FALSE(flow.transmit_paused(11));
}

TEST(AP_Networking_PPP_SoftFlow, SingleCorruptionCannotCreateStop)
{
    const uint8_t *start = AP_Networking_PPP_SoftFlow::codeword(
                               AP_Networking_PPP_SoftFlow::Command::START);

    for (uint8_t corrupt_index = 0;
         corrupt_index < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH;
         corrupt_index++) {
        AP_Networking_PPP_SoftFlow flow;
        for (uint8_t i = 0; i < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH; i++) {
            uint8_t byte = start[i];
            if (i == corrupt_index) {
                byte = byte == 0x11 ? 0x13 : 0x11;
            }
            (void)process_byte(flow, byte, 20);
        }
        EXPECT_FALSE(flow.transmit_paused(20));
    }
}

TEST(AP_Networking_PPP_SoftFlow, CorruptStopIsRejected)
{
    const uint8_t *stop = AP_Networking_PPP_SoftFlow::codeword(
                              AP_Networking_PPP_SoftFlow::Command::STOP);

    for (uint8_t corrupt_index = 0;
         corrupt_index < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH;
         corrupt_index++) {
        AP_Networking_PPP_SoftFlow flow;
        for (uint8_t i = 0; i < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH; i++) {
            uint8_t byte = stop[i];
            if (i == corrupt_index) {
                byte = byte == 0x11 ? 0x13 : 0x11;
            }
            (void)process_byte(flow, byte, 20);
        }
        EXPECT_FALSE(flow.transmit_paused(20));
    }
}

TEST(AP_Networking_PPP_SoftFlow, CompleteRetryAfterPartialCommand)
{
    for (const auto command : {
             AP_Networking_PPP_SoftFlow::Command::START,
             AP_Networking_PPP_SoftFlow::Command::STOP
         }) {
        const uint8_t *word = AP_Networking_PPP_SoftFlow::codeword(command);
        for (uint8_t partial = 1;
             partial < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH;
             partial++) {
            AP_Networking_PPP_SoftFlow flow;
            for (uint8_t i = 0; i < partial; i++) {
                (void)process_byte(flow, word[i], 20);
            }
            process_codeword(flow, command, 20);
            EXPECT_EQ(flow.transmit_paused(20),
                      command == AP_Networking_PPP_SoftFlow::Command::STOP);
        }
    }
}

TEST(AP_Networking_PPP_SoftFlow, OneEditCannotCreateOppositeCommand)
{
    const uint8_t *start = AP_Networking_PPP_SoftFlow::codeword(
                               AP_Networking_PPP_SoftFlow::Command::START);
    static constexpr uint8_t SYMBOLS =
        3 * AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH;

    // Exhaust every deletion and substitution in three repeated commands.
    for (uint8_t edit_index = 0; edit_index < SYMBOLS;
         edit_index++) {
        for (const bool substitute : {
                 false, true
             }) {
            AP_Networking_PPP_SoftFlow flow;
            for (uint8_t i = 0; i < SYMBOLS; i++) {
                if (!substitute && i == edit_index) {
                    continue;
                }
                uint8_t byte = start[i % AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH];
                if (substitute && i == edit_index) {
                    byte = byte == 0x11 ? 0x13 : 0x11;
                }
                (void)process_byte(flow, byte, 20);
                EXPECT_FALSE(flow.transmit_paused(20));
            }
        }
    }

    // Exhaust both possible inserted symbols at every boundary.
    for (uint8_t edit_index = 0; edit_index <= SYMBOLS;
         edit_index++) {
        for (const uint8_t inserted_byte : {
                 0x11, 0x13
             }) {
            AP_Networking_PPP_SoftFlow flow;
            for (uint8_t i = 0; i <= SYMBOLS; i++) {
                if (i == edit_index) {
                    (void)process_byte(flow, inserted_byte, 20);
                    EXPECT_FALSE(flow.transmit_paused(20));
                }
                if (i == SYMBOLS) {
                    break;
                }
                (void)process_byte(flow,
                                   start[i % AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH], 20);
                EXPECT_FALSE(flow.transmit_paused(20));
            }
        }
    }
}

TEST(AP_Networking_PPP_SoftFlow, StopIsALease)
{
    AP_Networking_PPP_SoftFlow flow;

    process_codeword(flow, AP_Networking_PPP_SoftFlow::Command::STOP, 100);
    EXPECT_TRUE(flow.transmit_paused(100));
    EXPECT_TRUE(flow.transmit_paused(
                    100 + AP_Networking_PPP_SoftFlow::STOP_LEASE_MS));
    EXPECT_FALSE(flow.transmit_paused(
                     101 + AP_Networking_PPP_SoftFlow::STOP_LEASE_MS));
}

TEST(AP_Networking_PPP_SoftFlow, StopRefreshAndStart)
{
    AP_Networking_PPP_SoftFlow flow;

    process_codeword(flow, AP_Networking_PPP_SoftFlow::Command::STOP, 100);
    process_codeword(flow, AP_Networking_PPP_SoftFlow::Command::STOP, 180);
    EXPECT_TRUE(flow.transmit_paused(
                    180 + AP_Networking_PPP_SoftFlow::STOP_LEASE_MS));

    process_codeword(flow, AP_Networking_PPP_SoftFlow::Command::START, 200);
    EXPECT_FALSE(flow.transmit_paused(200));
}

TEST(AP_Networking_PPP_SoftFlow, ReceiveWindowUsesHysteresis)
{
    AP_Networking_PPP_SoftFlow flow;

    flow.update_receive_space(1024, 4096, 100);
    EXPECT_EQ(flow.pending_command(100),
              AP_Networking_PPP_SoftFlow::Command::STOP);
    flow.command_sent(AP_Networking_PPP_SoftFlow::Command::STOP, 100);
    flow.command_sent(AP_Networking_PPP_SoftFlow::Command::STOP, 101);
    flow.command_sent(AP_Networking_PPP_SoftFlow::Command::STOP, 102);
    EXPECT_EQ(flow.pending_command(119),
              AP_Networking_PPP_SoftFlow::Command::NONE);
    EXPECT_EQ(flow.pending_command(122),
              AP_Networking_PPP_SoftFlow::Command::STOP);

    flow.update_receive_space(3072, 4096, 123);
    EXPECT_EQ(flow.pending_command(123),
              AP_Networking_PPP_SoftFlow::Command::START);
}

TEST(AP_Networking_PPP_SoftFlow, UnmatchedControlBytesArePreserved)
{
    AP_Networking_PPP_SoftFlow flow;
    const uint8_t input[] { 0x7E, 0x11, 0x13, 0x11, 0x13, 0x11, 0x13, 0x11, 0x55 };
    uint8_t output[sizeof(input)] {};
    uint8_t output_length = 0;

    for (uint8_t i = 0; i < sizeof(input); i++) {
        output_length += flow.process_byte(input[i], 10, &output[output_length]);
    }

    EXPECT_EQ(output_length, sizeof(input));
    EXPECT_EQ(memcmp(output, input, sizeof(input)), 0);
}

TEST(AP_Networking_PPP_SoftFlow, TimedOutCandidatesArePreserved)
{
    AP_Networking_PPP_SoftFlow flow;
    uint8_t output[4] {};
    uint8_t output_length = 0;

    output_length += flow.process_byte(0x11, 10, &output[output_length]);
    output_length += flow.process_byte(0x13, 10, &output[output_length]);
    output_length += flow.process_byte(0x11, 31, &output[output_length]);
    output_length += flow.process_byte(0x7E, 31, &output[output_length]);

    const uint8_t expected[] { 0x11, 0x13, 0x11, 0x7E };
    EXPECT_EQ(output_length, sizeof(expected));
    EXPECT_EQ(memcmp(output, expected, sizeof(expected)), 0);
}

TEST(AP_Networking_PPP_SoftFlow, TimedOutCandidatesFlushWithoutInput)
{
    AP_Networking_PPP_SoftFlow flow;
    uint8_t output[2] {};

    EXPECT_EQ(flow.process_byte(0x11, 10, output), 0);
    EXPECT_EQ(flow.process_byte(0x13, 10, output), 0);
    EXPECT_EQ(flow.flush(30, output), 0);
    EXPECT_EQ(flow.flush(31, output), 2);

    const uint8_t expected[] { 0x11, 0x13 };
    EXPECT_EQ(memcmp(output, expected, sizeof(expected)), 0);
}

TEST(AP_Networking_PPP_SoftFlow, CommandsAreRemovedFromStream)
{
    for (const auto command : {
             AP_Networking_PPP_SoftFlow::Command::START,
             AP_Networking_PPP_SoftFlow::Command::STOP
         }) {
        AP_Networking_PPP_SoftFlow flow;
        const uint8_t *word = AP_Networking_PPP_SoftFlow::codeword(command);
        uint8_t output[AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH];
        uint8_t output_length = 0;
        for (uint8_t i = 0; i < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH; i++) {
            output_length += flow.process_byte(word[i], 10, &output[output_length]);
        }
        EXPECT_EQ(output_length, 0);
    }
}

TEST(AP_Networking_PPP_SoftFlow, TxFrameCapacityIsReservedAtomically)
{
    AP_Networking_PPP_SoftFlow flow;
    constexpr uint32_t max_frame_size = 1024;

    // No bytes are accepted unless the complete worst-case frame can fit.
    EXPECT_FALSE(flow.begin_tx_chunk(max_frame_size - 1, 256, max_frame_size));
    EXPECT_TRUE(flow.begin_tx_chunk(max_frame_size, 256, max_frame_size));
    flow.end_tx_chunk(false);

    // Once reserved, each later chunk only needs its own remaining capacity.
    EXPECT_TRUE(flow.begin_tx_chunk(512, 256, max_frame_size));
    flow.end_tx_chunk(false);
    EXPECT_FALSE(flow.begin_tx_chunk(127, 128, max_frame_size));
    EXPECT_FALSE(flow.begin_tx_chunk(max_frame_size - 1, 128, max_frame_size));
    EXPECT_TRUE(flow.begin_tx_chunk(max_frame_size, 128, max_frame_size));
    flow.end_tx_chunk(true);

    // Completing the frame makes the full reservation mandatory again.
    EXPECT_FALSE(flow.begin_tx_chunk(max_frame_size - 1, 128, max_frame_size));
}

AP_GTEST_MAIN()
