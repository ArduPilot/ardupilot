/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/panic_listener.hpp>
#include <uavcan/protocol/panic_broadcaster.hpp>
#include "helpers.hpp"


struct PanicHandler
{
    uavcan::protocol::Panic msg;

    PanicHandler() : msg() { }

    void handle(const uavcan::protocol::Panic& msg)
    {
        std::cout << msg << std::endl;
        this->msg = msg;
    }

    typedef uavcan::MethodBinder<PanicHandler*, void (PanicHandler::*)(const uavcan::protocol::Panic& msg)> Binder;

    Binder bind() { return Binder(this, &PanicHandler::handle); }
};


TEST(PanicListener, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::Panic> _reg1;

    uavcan::PanicListener<PanicHandler::Binder> pl(nodes.a);
    uavcan::PanicBroadcaster pbr(nodes.b);
    PanicHandler handler;
    ASSERT_LE(0, pl.start(handler.bind()));

    pbr.panic("PANIC!!!");
    ASSERT_TRUE(handler.msg == uavcan::protocol::Panic()); // One message published, panic is not registered yet

    pbr.dontPanic();
    ASSERT_FALSE(pbr.isPanicking());
    std::cout << "Not panicking" << std::endl;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000)); // Will reset
    ASSERT_TRUE(handler.msg == uavcan::protocol::Panic());

    pbr.panic("PANIC2!!!");     // Message text doesn't matter
    ASSERT_TRUE(pbr.isPanicking());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_STREQ("PANIC2!", handler.msg.reason_text.c_str()); // Registered, only 7 chars preserved
}
