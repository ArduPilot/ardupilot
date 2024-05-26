/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <iostream>
#include <string>
#include <list>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <uavcan/time.hpp>
#include "../../clock.hpp"


class EventTracer : public uavcan::dynamic_node_id_server::IEventTracer
{
    struct EventLogEntry
    {
        const uavcan::dynamic_node_id_server::TraceCode code;
        const uavcan::int64_t argument;

        EventLogEntry(uavcan::dynamic_node_id_server::TraceCode arg_code, uavcan::int64_t arg_argument)
            : code(arg_code)
            , argument(arg_argument)
        { }
    };

    const std::string id_;
    const uavcan::MonotonicTime startup_ts_;
    std::list<EventLogEntry> event_log_;

public:
    EventTracer() :
        startup_ts_(SystemClockDriver().getMonotonic())
    { }

    EventTracer(const std::string& id) :
        id_(id),
        startup_ts_(SystemClockDriver().getMonotonic())
    { }

    virtual void onEvent(uavcan::dynamic_node_id_server::TraceCode code, uavcan::int64_t argument)
    {
        const uavcan::MonotonicDuration ts = SystemClockDriver().getMonotonic() - startup_ts_;
        std::cout << "EVENT [" << id_ << "]\t" << ts.toString() << "\t"
                  << int(code) << "\t" << getEventName(code) << "\t" << argument << std::endl;
        event_log_.push_back(EventLogEntry(code, argument));
    }

    unsigned countEvents(const uavcan::dynamic_node_id_server::TraceCode code) const
    {
        unsigned count = 0;
        for (std::list<EventLogEntry>::const_iterator it = event_log_.begin(); it != event_log_.end(); ++it)
        {
            count += (it->code == code) ? 1U : 0U;
        }
        return count;
    }

    uavcan::int64_t getLastEventArgumentOrFail(const uavcan::dynamic_node_id_server::TraceCode code) const
    {
        for (std::list<EventLogEntry>::const_reverse_iterator it = event_log_.rbegin(); it != event_log_.rend(); ++it)
        {
            if (it->code == code)
            {
                return it->argument;
            }
        }

        std::cout << "No such event in the event log, code " << int(code) << ", log length " << event_log_.size()
            << std::endl;

        throw std::runtime_error("EventTracer::getLastEventArgumentOrFail()");
    }

    unsigned getNumEvents() const { return static_cast<unsigned>(event_log_.size()); }
};
