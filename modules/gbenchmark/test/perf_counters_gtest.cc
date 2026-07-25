#include <thread>

#include "../src/perf_counters.h"
#include "gtest/gtest.h"

#ifndef GTEST_SKIP
struct MsgHandler {
  void operator=(std::ostream&) {}
};
#define GTEST_SKIP() return MsgHandler() = std::cout
#endif

using benchmark::internal::PerfCounters;
using benchmark::internal::PerfCountersMeasurement;
using benchmark::internal::PerfCounterValues;

namespace {
const char kGenericPerfEvent1[] = "CYCLES";
const char kGenericPerfEvent2[] = "BRANCHES";
const char kGenericPerfEvent3[] = "INSTRUCTIONS";

TEST(PerfCountersTest, Init) {
  EXPECT_EQ(PerfCounters::Initialize(), PerfCounters::kSupported);
}

TEST(PerfCountersTest, OneCounter) {
  if (!PerfCounters::kSupported) {
    GTEST_SKIP() << "Performance counters not supported.\n";
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  EXPECT_TRUE(PerfCounters::Create({kGenericPerfEvent1}).IsValid());
}

TEST(PerfCountersTest, NegativeTest) {
  if (!PerfCounters::kSupported) {
    EXPECT_FALSE(PerfCounters::Initialize());
    return;
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  EXPECT_FALSE(PerfCounters::Create({}).IsValid());
  EXPECT_FALSE(PerfCounters::Create({""}).IsValid());
  EXPECT_FALSE(PerfCounters::Create({"not a counter name"}).IsValid());
  {
    EXPECT_TRUE(PerfCounters::Create({kGenericPerfEvent1, kGenericPerfEvent2,
                                      kGenericPerfEvent3})
                    .IsValid());
  }
  EXPECT_FALSE(
      PerfCounters::Create({kGenericPerfEvent2, "", kGenericPerfEvent1})
          .IsValid());
  EXPECT_FALSE(PerfCounters::Create({kGenericPerfEvent3, "not a counter name",
                                     kGenericPerfEvent1})
                   .IsValid());
  {
    EXPECT_TRUE(PerfCounters::Create({kGenericPerfEvent1, kGenericPerfEvent2,
                                      kGenericPerfEvent3})
                    .IsValid());
  }
  EXPECT_FALSE(
      PerfCounters::Create({kGenericPerfEvent1, kGenericPerfEvent2,
                            kGenericPerfEvent3, "MISPREDICTED_BRANCH_RETIRED"})
          .IsValid());
}

TEST(PerfCountersTest, Read1Counter) {
  if (!PerfCounters::kSupported) {
    GTEST_SKIP() << "Test skipped because libpfm is not supported.\n";
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  auto counters = PerfCounters::Create({kGenericPerfEvent1});
  EXPECT_TRUE(counters.IsValid());
  PerfCounterValues values1(1);
  EXPECT_TRUE(counters.Snapshot(&values1));
  EXPECT_GT(values1[0], 0);
  PerfCounterValues values2(1);
  EXPECT_TRUE(counters.Snapshot(&values2));
  EXPECT_GT(values2[0], 0);
  EXPECT_GT(values2[0], values1[0]);
}

TEST(PerfCountersTest, Read2Counters) {
  if (!PerfCounters::kSupported) {
    GTEST_SKIP() << "Test skipped because libpfm is not supported.\n";
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  auto counters =
      PerfCounters::Create({kGenericPerfEvent1, kGenericPerfEvent2});
  EXPECT_TRUE(counters.IsValid());
  PerfCounterValues values1(2);
  EXPECT_TRUE(counters.Snapshot(&values1));
  EXPECT_GT(values1[0], 0);
  EXPECT_GT(values1[1], 0);
  PerfCounterValues values2(2);
  EXPECT_TRUE(counters.Snapshot(&values2));
  EXPECT_GT(values2[0], 0);
  EXPECT_GT(values2[1], 0);
}

TEST(PerfCountersTest, ReopenExistingCounters) {
  // The test works (i.e. causes read to fail) for the assumptions
  // about hardware capabilities (i.e. small number (3-4) hardware
  // counters) at this date.
  if (!PerfCounters::kSupported) {
    GTEST_SKIP() << "Test skipped because libpfm is not supported.\n";
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  std::vector<PerfCounters> counters;
  counters.reserve(6);
  for (int i = 0; i < 6; i++)
    counters.push_back(PerfCounters::Create({kGenericPerfEvent1}));
  PerfCounterValues values(1);
  EXPECT_TRUE(counters[0].Snapshot(&values));
  EXPECT_FALSE(counters[4].Snapshot(&values));
  EXPECT_FALSE(counters[5].Snapshot(&values));
}

TEST(PerfCountersTest, CreateExistingMeasurements) {
  // The test works (i.e. causes read to fail) for the assumptions
  // about hardware capabilities (i.e. small number (3-4) hardware
  // counters) at this date,
  // the same as previous test ReopenExistingCounters.
  if (!PerfCounters::kSupported) {
    GTEST_SKIP() << "Test skipped because libpfm is not supported.\n";
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  std::vector<PerfCountersMeasurement> perf_counter_measurements;
  std::vector<std::pair<std::string, double>> measurements;

  perf_counter_measurements.reserve(10);
  for (int i = 0; i < 10; i++)
    perf_counter_measurements.emplace_back(
        std::vector<std::string>{kGenericPerfEvent1});

  perf_counter_measurements[0].Start();
  EXPECT_TRUE(perf_counter_measurements[0].Stop(measurements));

  measurements.clear();
  perf_counter_measurements[8].Start();
  EXPECT_FALSE(perf_counter_measurements[8].Stop(measurements));

  measurements.clear();
  perf_counter_measurements[9].Start();
  EXPECT_FALSE(perf_counter_measurements[9].Stop(measurements));
}

size_t do_work() {
  size_t res = 0;
  for (size_t i = 0; i < 100000000; ++i) res += i * i;
  return res;
}

void measure(size_t threadcount, PerfCounterValues* values1,
             PerfCounterValues* values2) {
  BM_CHECK_NE(values1, nullptr);
  BM_CHECK_NE(values2, nullptr);
  std::vector<std::thread> threads(threadcount);
  auto work = [&]() { BM_CHECK(do_work() > 1000); };

  // We need to first set up the counters, then start the threads, so the
  // threads would inherit the counters. But later, we need to first destroy the
  // thread pool (so all the work finishes), then measure the counters. So the
  // scopes overlap, and we need to explicitly control the scope of the
  // threadpool.
  auto counters =
      PerfCounters::Create({kGenericPerfEvent1, kGenericPerfEvent3});
  for (auto& t : threads) t = std::thread(work);
  counters.Snapshot(values1);
  for (auto& t : threads) t.join();
  counters.Snapshot(values2);
}

TEST(PerfCountersTest, MultiThreaded) {
  if (!PerfCounters::kSupported) {
    GTEST_SKIP() << "Test skipped because libpfm is not supported.";
  }
  EXPECT_TRUE(PerfCounters::Initialize());
  PerfCounterValues values1(2);
  PerfCounterValues values2(2);

  measure(2, &values1, &values2);
  std::vector<double> D1{static_cast<double>(values2[0] - values1[0]),
                         static_cast<double>(values2[1] - values1[1])};

  measure(4, &values1, &values2);
  std::vector<double> D2{static_cast<double>(values2[0] - values1[0]),
                         static_cast<double>(values2[1] - values1[1])};

  // Some extra work will happen on the main thread - like joining the threads
  // - so the ratio won't be quite 2.0, but very close.
  EXPECT_GE(D2[0], 1.9 * D1[0]);
  EXPECT_GE(D2[1], 1.9 * D1[1]);
}
}  // namespace
