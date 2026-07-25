#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/seq_num.c>
}

TEST(SeqNumTest, AddFromNegative)
{
    EXPECT_EQ(0, uxr_seq_num_add(uxrSeqNum(UINT16_MAX), 1));
}

TEST(SeqNumTest, AddFromPositive)
{
    EXPECT_EQ(1, uxr_seq_num_add(0, 1));
}

TEST(SeqNumTest, AddMax)
{
    EXPECT_EQ(0, uxr_seq_num_add(SEQ_NUM_MIDSIZE, SEQ_NUM_MIDSIZE));
}

TEST(SeqNumTest, SubToNegative)
{
    EXPECT_EQ(uxrSeqNum(UINT16_MAX), uxr_seq_num_sub(0, 1));
}

TEST(SeqNumTest, SubToPositive)
{
    EXPECT_EQ(0, uxr_seq_num_sub(1, 1));
}

TEST(SeqNumTest, SubMax)
{
    EXPECT_EQ(0, uxr_seq_num_sub(SEQ_NUM_MIDSIZE, SEQ_NUM_MIDSIZE));
}

TEST(SeqNumTest, CmpEQ)
{
    EXPECT_EQ(0, uxr_seq_num_cmp(1, 1));
}

TEST(SeqNumTest, CmpLT)
{
    EXPECT_EQ(-1, uxr_seq_num_cmp(0, 1));
}

TEST(SeqNumTest, CmpGR)
{
    EXPECT_EQ(1, uxr_seq_num_cmp(1, 0));
}

TEST(SeqNumTest, CmpGTMax)
{
    EXPECT_EQ(1, uxr_seq_num_cmp(SEQ_NUM_MIDSIZE, 0));
}

TEST(SeqNumTest, CmpGTMaxInverted)
{
    EXPECT_EQ(1, uxr_seq_num_cmp(0, SEQ_NUM_MIDSIZE));
}

TEST(SeqNumTest, CmpLTMax)
{
    EXPECT_EQ(-1, uxr_seq_num_cmp(SEQ_NUM_MIDSIZE + 1, 0));
}

TEST(SeqNumTest, CmpLTMaxInverted)
{
    EXPECT_EQ(-1, uxr_seq_num_cmp(0, SEQ_NUM_MIDSIZE - 1));
}

TEST(SeqNumTest, CmpLTNegative)
{
    EXPECT_EQ(-1, uxr_seq_num_cmp(uxrSeqNum(UINT16_MAX), 0));
}

TEST(SeqNumTest, CmpGRNegative)
{
    EXPECT_EQ(1, uxr_seq_num_cmp(0, uxrSeqNum(UINT16_MAX)));
}
