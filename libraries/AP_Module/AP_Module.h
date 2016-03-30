
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Frontend Callback broker for APM
 *  Author: Siddharth B Purohit
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <stdio.h>
#include "List.h"
#define MODULE_DEBUG    0

#define MAX_EVENTS                          64
#define NUM_NEW_SAMPLE_EVENTS               5
#define NUM_NEW_OUTPUT_EVENTS               10
#define NUM_NEW_INPUT_EVENTS                5
#define NUM_NEW_BACKEND_EVENTS              5
#define NUM_FAULT_EVENTS                    5
#define NUM_BACKEND_DISCONNECTED_EVENTS     5
#define NUM_MODULE_DEFINED_EVENTS           20

#define EVENT_ID(event_type, instance)      (event_type + instance)
#define EVENT_BITMASK(event_type, instance)    (1<<EVENT_ID(event_type, instance))

class Callback;

// This Class is not to be used as an independent entity, It should be inherited
// by class for which callback interface needs to be laid out
class AP_Module
{
public:
    // This will be a central event list, will contains all the possible event
    // possibles inside APM for which callbacks need implementing
    // Generate offsets for each event types
    enum event_type {
        EVENT_NEW_SAMPLE = 0,
        EVENT_NEW_OUTPUT = EVENT_NEW_SAMPLE + NUM_NEW_SAMPLE_EVENTS,
        EVENT_NEW_INPUT = EVENT_NEW_OUTPUT + NUM_NEW_OUTPUT_EVENTS,
        EVENT_NEW_BACKEND = EVENT_NEW_INPUT + NUM_NEW_INPUT_EVENTS,
        EVENT_FAULT_EVENTS = EVENT_NEW_BACKEND + NUM_NEW_BACKEND_EVENTS,
        EVENT_BACKEND_DISCONNECTED = EVENT_FAULT_EVENTS + NUM_FAULT_EVENTS,
        EVENT_MODULE_DEFINED = EVENT_BACKEND_DISCONNECTED + NUM_BACKEND_DISCONNECTED_EVENTS
    };

    //Interface for classes to add callbacks
    // Example call:
    // <directed_class_name>::frontend()->add_callback(FUNCTOR_BIND_MEMBER(&<class_name>::<func>, void), EVENT_ID(AP_Module::<event_type>, instance));
    int32_t add_callback(AP_HAL::MemberProc cfunc, uint8_t e);

    //Interface for classes to modify callback trigger
    // Example call:
    // <directed_class_name>::frontend()->modify_callback_trigger(<callback_id>, EVENT_ID(AP_Module::<event_type>, instance));
    void modify_callback_trigger(uint32_t callback_id, uint8_t new_e);

    //Runs all the functors for the child class based on Triggered events
    void invoke_callbacks(uint64_t &event_bitmask);

    uint32_t last_callback_id = 0;
protected:
    calllist_t<Callback *> calllist;
};

class Callback : public node_t<Callback *>{
public:
    AP_HAL::MemberProc callfunc; //callback function
    Callback() {
        static_assert((AP_Module::EVENT_MODULE_DEFINED + NUM_MODULE_DEFINED_EVENTS) < MAX_EVENTS, "Total number of events overflow!");
    }
    Callback(AP_HAL::MemberProc func, uint8_t e, uint32_t callback_id) :
    callfunc(func),
    event(e),
    cb_id(callback_id),
    num_calls(0) {
        static_assert((AP_Module::EVENT_MODULE_DEFINED + NUM_MODULE_DEFINED_EVENTS) < MAX_EVENTS, "Total number of events overflow!");
    }

    void init(AP_HAL::MemberProc func, uint8_t e, uint32_t callback_id) {
        callfunc = func;
        event = e;
        cb_id = callback_id;
        num_calls = 0;
    }

    bool operator==(uint8_t e) { return (this->event == e); }
    bool is_callback_id(uint32_t callback_id) { return (cb_id == callback_id); }
    bool is_event(uint8_t e) { return (event == e); }
    void set_event(uint8_t e) { event = e; }
    // Perf data for debugging purposes
    void callperf(){
        num_calls++;
        if(num_calls ==  100) {
            mspercent = (AP_HAL::millis() - last_time);
            last_time = AP_HAL::millis();
#if MODULE_DEBUG
            printf("CALLBACK ID: %d RATE: %d\n",cb_id, mspercent/100);
#endif
        }
    }
private:
    uint8_t event;          //type of event
    uint32_t cb_id;         //ID number of callback function
    uint32_t num_calls;     //num calls since last batch of 100 calls
    uint32_t mspercent;     //Total time for 100 calls (milliseconds per cent calls)
    uint32_t last_time;     //time since last batch of 100 calls
};
