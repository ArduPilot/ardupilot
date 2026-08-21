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

TEST(AP_Networking_PPP_SoftFlow, RepeatRecoversOtherCommandAfterPartial)
{
    for (const auto command : {
             AP_Networking_PPP_SoftFlow::Command::START,
             AP_Networking_PPP_SoftFlow::Command::STOP
         }) {
        const auto other = command == AP_Networking_PPP_SoftFlow::Command::STOP ?
                           AP_Networking_PPP_SoftFlow::Command::START :
                           AP_Networking_PPP_SoftFlow::Command::STOP;
        const uint8_t *word = AP_Networking_PPP_SoftFlow::codeword(command);
        for (uint8_t partial = 1;
             partial < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH;
             partial++) {
            AP_Networking_PPP_SoftFlow flow;
            for (uint8_t i = 0; i < partial; i++) {
                (void)process_byte(flow, word[i], 20);
            }
            // the first copy of the other command can be misread
            process_codeword(flow, other, 20);
            process_codeword(flow, other, 20);
            EXPECT_EQ(flow.transmit_paused(20),
                      other == AP_Networking_PPP_SoftFlow::Command::STOP);
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

TEST(AP_Networking_PPP_SoftFlow, ReceiveWindowThresholds)
{
    AP_Networking_PPP_SoftFlow flow;

    flow.update_receive_space(1025, 4096, 100);
    EXPECT_FALSE(flow.receive_stopped());
    flow.update_receive_space(1024, 4096, 100);
    EXPECT_TRUE(flow.receive_stopped());

    flow.update_receive_space(3071, 4096, 100);
    EXPECT_TRUE(flow.receive_stopped());
    flow.update_receive_space(3072, 4096, 100);
    EXPECT_FALSE(flow.receive_stopped());
}

TEST(AP_Networking_PPP_SoftFlow, IgnoredStopIsDetected)
{
    AP_Networking_PPP_SoftFlow flow;
    const uint32_t settle_ms = AP_Networking_PPP_SoftFlow::STOP_SETTLE_MS;
    const uint32_t limit = 512;

    // nothing to detect unless we have stopped the peer
    EXPECT_FALSE(flow.stop_ignored(100000, limit, 1000));

    flow.update_receive_space(1024, 4096, 1000);

    // bytes already in flight when STOP was sent are not counted
    EXPECT_FALSE(flow.stop_ignored(5000, limit, 1000 + settle_ms - 1));
    EXPECT_FALSE(flow.stop_ignored(9000, limit, 1000 + settle_ms));

    EXPECT_FALSE(flow.stop_ignored(9000 + limit, limit, 2000));
    EXPECT_TRUE(flow.stop_ignored(9001 + limit, limit, 2000));

    // releasing the peer ends the check
    flow.update_receive_space(3072, 4096, 2100);
    EXPECT_FALSE(flow.stop_ignored(50000, limit, 3000));
}

TEST(AP_Networking_PPP_SoftFlow, PeerFlushIsNotAnIgnoredStop)
{
    AP_Networking_PPP_SoftFlow flow;
    const uint32_t settle_ms = AP_Networking_PPP_SoftFlow::STOP_SETTLE_MS;
    const uint32_t limit = 512;

    flow.update_receive_space(1024, 4096, 1000);
    EXPECT_FALSE(flow.stop_ignored(0, limit, 1000 + settle_ms));

    // a peer stopping us flushes its queue first, so bytes that
    // arrive while it holds us stopped are expected
    process_codeword(flow, AP_Networking_PPP_SoftFlow::Command::STOP, 1300);
    EXPECT_FALSE(flow.stop_ignored(4000, limit, 1310));

    // once its lease ends the count restarts from there
    EXPECT_FALSE(flow.transmit_paused(
                     1301 + AP_Networking_PPP_SoftFlow::STOP_LEASE_MS));
    EXPECT_FALSE(flow.stop_ignored(4000 + limit, limit, 1500));
    EXPECT_TRUE(flow.stop_ignored(4001 + limit, limit, 1500));
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

AP_GTEST_MAIN()
