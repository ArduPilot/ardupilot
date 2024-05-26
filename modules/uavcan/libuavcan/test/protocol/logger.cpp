/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/logger.hpp>
#include "helpers.hpp"


struct LogSink : public uavcan::ILogSink
{
    std::queue<uavcan::protocol::debug::LogMessage> msgs;
    LogLevel level;

    LogSink()
        : level(uavcan::protocol::debug::LogLevel::ERROR)
    { }

    LogLevel getLogLevel() const { return level; }

    void log(const uavcan::protocol::debug::LogMessage& message)
    {
        msgs.push(message);
        std::cout << message << std::endl;
    }

    uavcan::protocol::debug::LogMessage pop()
    {
        if (msgs.empty())
        {
            std::cout << "LogSink is empty" << std::endl;
            std::abort();
        }
        const uavcan::protocol::debug::LogMessage m = msgs.front();
        msgs.pop();
        return m;
    }

    bool popMatchByLevelAndText(int level, const std::string& source, const std::string& text)
    {
        if (msgs.empty())
        {
            std::cout << "LogSink is empty" << std::endl;
            return false;
        }
        const uavcan::protocol::debug::LogMessage m = pop();
        return
            level == m.level.value &&
            source == m.source &&
            text == m.text;
    }
};


TEST(Logger, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::Logger logger(nodes.a);

    ASSERT_EQ(uavcan::protocol::debug::LogLevel::ERROR, logger.getLevel());

    LogSink sink;

    // Will fail - types are not registered
    uavcan::GlobalDataTypeRegistry::instance().reset();
    ASSERT_GT(0, logger.logError("foo", "Error (fail - type is not registered)"));
    ASSERT_EQ(0, logger.logDebug("foo", "Debug (ignored - low logging level)"));

    ASSERT_FALSE(logger.getExternalSink());
    logger.setExternalSink(&sink);
    ASSERT_EQ(&sink, logger.getExternalSink());

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::debug::LogMessage> _reg1;

    SubscriberWithCollector<uavcan::protocol::debug::LogMessage> log_sub(nodes.b);
    ASSERT_LE(0, log_sub.start());

    // Sink test
    ASSERT_EQ(0, logger.logDebug("foo", "Debug (ignored due to low logging level)"));
    ASSERT_TRUE(sink.msgs.empty());

    sink.level = uavcan::protocol::debug::LogLevel::DEBUG;
    ASSERT_EQ(0, logger.logDebug("foo", "Debug (sink only)"));
    ASSERT_TRUE(sink.popMatchByLevelAndText(uavcan::protocol::debug::LogLevel::DEBUG, "foo", "Debug (sink only)"));

    ASSERT_LE(0, logger.logError("foo", "Error"));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_TRUE(log_sub.collector.msg.get());
    ASSERT_EQ(log_sub.collector.msg->level.value, uavcan::protocol::debug::LogLevel::ERROR);
    ASSERT_EQ(log_sub.collector.msg->source, "foo");
    ASSERT_EQ(log_sub.collector.msg->text, "Error");

    logger.setLevel(uavcan::protocol::debug::LogLevel::DEBUG);
    ASSERT_LE(0, logger.logWarning("foo", "Warning"));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_EQ(log_sub.collector.msg->level.value, uavcan::protocol::debug::LogLevel::WARNING);
    ASSERT_EQ(log_sub.collector.msg->source, "foo");
    ASSERT_EQ(log_sub.collector.msg->text, "Warning");

    ASSERT_LE(0, logger.logInfo("foo", "Info"));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_EQ(log_sub.collector.msg->level.value, uavcan::protocol::debug::LogLevel::INFO);
    ASSERT_EQ(log_sub.collector.msg->source, "foo");
    ASSERT_EQ(log_sub.collector.msg->text, "Info");

    ASSERT_LE(0, logger.logDebug("foo", "Debug"));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_EQ(log_sub.collector.msg->level.value, uavcan::protocol::debug::LogLevel::DEBUG);
    ASSERT_EQ(log_sub.collector.msg->source, "foo");
    ASSERT_EQ(log_sub.collector.msg->text, "Debug");

    ASSERT_TRUE(sink.popMatchByLevelAndText(uavcan::protocol::debug::LogLevel::ERROR,   "foo", "Error"));
    ASSERT_TRUE(sink.popMatchByLevelAndText(uavcan::protocol::debug::LogLevel::WARNING, "foo", "Warning"));
    ASSERT_TRUE(sink.popMatchByLevelAndText(uavcan::protocol::debug::LogLevel::INFO,    "foo", "Info"));
    ASSERT_TRUE(sink.popMatchByLevelAndText(uavcan::protocol::debug::LogLevel::DEBUG,   "foo", "Debug"));
}

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

TEST(Logger, Cpp11Formatting)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::Logger logger(nodes.a);
    logger.setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    SubscriberWithCollector<uavcan::protocol::debug::LogMessage> log_sub(nodes.b);
    ASSERT_LE(0, log_sub.start());

    ASSERT_LE(0, logger.logWarning("foo", "char='%*', %* is %*", '$', "double", 12.34));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_TRUE(log_sub.collector.msg.get());
    ASSERT_EQ(log_sub.collector.msg->level.value, uavcan::protocol::debug::LogLevel::WARNING);
    ASSERT_EQ(log_sub.collector.msg->source, "foo");
    ASSERT_EQ(log_sub.collector.msg->text, "char='$', double is 12.34");
}

#endif
