#include <AP_gtest.h>
#include <AP_Networking/AP_Networking_PPP_SoftFlow.h>

static void process_codeword(AP_Networking_PPP_SoftFlow &flow,
                             AP_Networking_PPP_SoftFlow::Command command,
                             uint32_t now_ms)
{
    const uint8_t *word = AP_Networking_PPP_SoftFlow::codeword(command);
    for (uint8_t i = 0; i < AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH; i++) {
        EXPECT_TRUE(flow.process_byte(word[i], now_ms));
    }
}

TEST(AP_Networking_PPP_SoftFlow, SingleByteCannotStop)
{
    AP_Networking_PPP_SoftFlow flow;

    EXPECT_TRUE(flow.process_byte(0x13, 10));
    EXPECT_FALSE(flow.transmit_paused(10));

    EXPECT_TRUE(flow.process_byte(0x11, 11));
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
            EXPECT_TRUE(flow.process_byte(byte, 20));
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
            EXPECT_TRUE(flow.process_byte(byte, 20));
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
                EXPECT_TRUE(flow.process_byte(word[i], 20));
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
                EXPECT_TRUE(flow.process_byte(byte, 20));
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
                    EXPECT_TRUE(flow.process_byte(inserted_byte, 20));
                    EXPECT_FALSE(flow.transmit_paused(20));
                }
                if (i == SYMBOLS) {
                    break;
                }
                EXPECT_TRUE(flow.process_byte(
                                start[i % AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH], 20));
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

AP_GTEST_MAIN()
