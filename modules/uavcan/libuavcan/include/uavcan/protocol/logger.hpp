/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_LOGGER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_LOGGER_HPP_INCLUDED

#include <uavcan/time.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/marshal/char_array_formatter.hpp>
#include <uavcan/node/publisher.hpp>
#include <cstdlib>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

namespace uavcan
{
/**
 * External log sink interface.
 * External log sink allows the application to install a hook on the logger output.
 * This can be used for application-wide logging.
 * Please refer to the @ref Logger class docs.
 */
class UAVCAN_EXPORT ILogSink
{
public:
    typedef typename StorageType<typename protocol::debug::LogLevel::FieldTypes::value>::Type LogLevel;

    virtual ~ILogSink() { }

    /**
     * Logger will not sink messages with a severity level lower than returned by this method.
     * Default level is DEBUG.
     */
    virtual LogLevel getLogLevel() const { return protocol::debug::LogLevel::DEBUG; }

    /**
     * Logger will call this method for every log message which severity level
     * is not less than the current level of this sink.
     */
    virtual void log(const protocol::debug::LogMessage& message) = 0;
};

/**
 * Node logging convenience class.
 *
 * This class is based on the standard UAVCAN message type for logging - uavcan.protocol.debug.LogMessage.
 *
 * Provides logging methods of different severity; implements two sinks for the log messages:
 *  - Broadcast via the UAVCAN bus;
 *  - Sink into the application via @ref ILogSink.
 *
 * For each sink an individual severity threshold filter can be configured.
 */
class UAVCAN_EXPORT Logger
{
public:
    typedef ILogSink::LogLevel LogLevel;

    /**
     * This value is higher than any valid severity value.
     * Use it to completely suppress the output.
     */
    static LogLevel getLogLevelAboveAll() { return (1U << protocol::debug::LogLevel::FieldTypes::value::BitLen) - 1U; }

private:
    enum { DefaultTxTimeoutMs = 2000 };

    Publisher<protocol::debug::LogMessage> logmsg_pub_;
    protocol::debug::LogMessage msg_buf_;
    LogLevel level_;
    ILogSink* external_sink_;

    LogLevel getExternalSinkLevel() const
    {
        return (external_sink_ == UAVCAN_NULLPTR) ? getLogLevelAboveAll() : external_sink_->getLogLevel();
    }

public:
    explicit Logger(INode& node)
        : logmsg_pub_(node)
        , external_sink_(UAVCAN_NULLPTR)
    {
        level_ = protocol::debug::LogLevel::ERROR;
        setTxTimeout(MonotonicDuration::fromMSec(DefaultTxTimeoutMs));
        UAVCAN_ASSERT(getTxTimeout() == MonotonicDuration::fromMSec(DefaultTxTimeoutMs));
    }

    /**
     * Initializes the logger, does not perform any network activity.
     * Must be called once before use.
     * Returns negative error code.
     */
    int init(const TransferPriority priority = TransferPriority::Lowest)
    {
        const int res = logmsg_pub_.init(priority);
        if (res < 0)
        {
            return res;
        }
        return 0;
    }

    /**
     * Logs one message. Please consider using helper methods instead of this one.
     *
     * The message will be broadcasted via the UAVCAN bus if the severity level of the
     * message is >= severity level of the logger.
     *
     * The message will be reported into the external log sink if the external sink is
     * installed and the severity level of the message is >= severity level of the external sink.
     *
     * Returns negative error code.
     */
    int log(const protocol::debug::LogMessage& message)
    {
        int retval = 0;
        if (message.level.value >= getExternalSinkLevel())
        {
            external_sink_->log(message);
        }
        if (message.level.value >= level_)
        {
            retval = logmsg_pub_.broadcast(message);
        }
        return retval;
    }

    /**
     * Severity filter for UAVCAN broadcasting.
     * Log message will be broadcasted via the UAVCAN network only if its severity is >= getLevel().
     * This does not affect the external sink.
     * Default level is ERROR.
     */
    LogLevel getLevel() const { return level_; }
    void setLevel(LogLevel level) { level_ = level; }

    /**
     * External log sink allows the application to install a hook on the logger output.
     * This can be used for application-wide logging.
     * Null pointer means that there's no log sink (can be used to remove it).
     * By default there's no log sink.
     */
    ILogSink* getExternalSink() const { return external_sink_; }
    void setExternalSink(ILogSink* sink) { external_sink_ = sink; }

    /**
     * Log message broadcast transmission timeout.
     * The default value should be acceptable for any use case.
     */
    MonotonicDuration getTxTimeout() const { return logmsg_pub_.getTxTimeout(); }
    void setTxTimeout(MonotonicDuration val) { logmsg_pub_.setTxTimeout(val); }

    /**
     * Helper methods for various severity levels and with formatting support.
     * These methods build a formatted log message and pass it into the method @ref log().
     *
     * Format string usage is a bit unusual: use "%*" for any argument type, use "%%" to print a percent character.
     * No other formating options are supported. Insufficient/extra arguments are ignored.
     *
     * Example format string:
     *      "What do you get if you %* %* by %*? %*. Extra arguments: %* %* %%"
     * ...with the following arguments:
     *      "multiply", 6, 9.0F 4.2e1
     * ...will likely produce this (floating point representation is platform dependent):
     *      "What do you get if you multiply 6 by 9.000000? 42.000000. Extra arguments: %* %* %"
     *
     * Formatting is not supported in C++03 mode.
     * @{
     */
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

    template <typename... Args>
    int log(LogLevel level, const char* source, const char* format, Args... args) UAVCAN_NOEXCEPT;

    template <typename... Args>
    inline int logDebug(const char* source, const char* format, Args... args) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::DEBUG, source, format, args...);
    }

    template <typename... Args>
    inline int logInfo(const char* source, const char* format, Args... args) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::INFO, source, format, args...);
    }

    template <typename... Args>
    inline int logWarning(const char* source, const char* format, Args... args) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::WARNING, source, format, args...);
    }

    template <typename... Args>
    inline int logError(const char* source, const char* format, Args... args) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::ERROR, source, format, args...);
    }

#else

    int log(LogLevel level, const char* source, const char* text) UAVCAN_NOEXCEPT
    {
    #if UAVCAN_EXCEPTIONS
        try
    #endif
        {
            if (level >= level_ || level >= getExternalSinkLevel())
            {
                msg_buf_.level.value = level;
                msg_buf_.source = source;
                msg_buf_.text = text;
                return log(msg_buf_);
            }
            return 0;
        }
    #if UAVCAN_EXCEPTIONS
        catch (...)
        {
            return -ErrFailure;
        }
    #endif
    }

    int logDebug(const char* source, const char* text) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::DEBUG, source, text);
    }

    int logInfo(const char* source, const char* text) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::INFO, source, text);
    }

    int logWarning(const char* source, const char* text) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::WARNING, source, text);
    }

    int logError(const char* source, const char* text) UAVCAN_NOEXCEPT
    {
        return log(protocol::debug::LogLevel::ERROR, source, text);
    }

#endif
    /**
     * @}
     */
};

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

template <typename... Args>
int Logger::log(LogLevel level, const char* source, const char* format, Args... args) UAVCAN_NOEXCEPT
{
#if UAVCAN_EXCEPTIONS
    try
#endif
    {
        if (level >= level_ || level >= getExternalSinkLevel())
        {
            msg_buf_.level.value = level;
            msg_buf_.source = source;
            msg_buf_.text.clear();
            CharArrayFormatter<typename protocol::debug::LogMessage::FieldTypes::text> formatter(msg_buf_.text);
            formatter.write(format, args...);
            return log(msg_buf_);
        }
        return 0;
    }
#if UAVCAN_EXCEPTIONS
    catch (...)
    {
        return -ErrFailure;
    }
#endif
}

#endif

}

#endif // UAVCAN_PROTOCOL_LOGGER_HPP_INCLUDED
