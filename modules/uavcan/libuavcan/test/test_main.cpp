/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/build_config.hpp>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

static void sigsegv_handler(int sig)
{
    const int BacktraceSize = 32;
    void* array[BacktraceSize];
    const int size = backtrace(array, BacktraceSize);

    std::fprintf(stderr, "SIGNAL %d RECEIVED; STACKTRACE:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    std::exit(1);
}

int main(int argc, char **argv)
{
    signal(SIGSEGV, sigsegv_handler);

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif
#if UAVCAN_CPP_VERSION == UAVCAN_CPP11
    std::cout << "C++11" << std::endl;
#elif UAVCAN_CPP_VERSION == UAVCAN_CPP03
    std::cout << "C++03" << std::endl;
#else
# error UAVCAN_CPP_VERSION
#endif

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
