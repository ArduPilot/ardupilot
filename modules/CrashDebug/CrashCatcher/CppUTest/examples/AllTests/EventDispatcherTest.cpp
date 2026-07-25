/*
 * Copyright (c) 2007, Michael Feathers, James Grenning and Bas Vodde
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE EARLIER MENTIONED AUTHORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "EventDispatcher.h"

class ObserverMock : public EventObserver
{
public:
	virtual void notify(const Event& event, int timeOutInSeconds)
	{
		mock().actualCall("notify").onObject(this).withParameterOfType("Event", "event", (void*) &event).withParameter("timeOutInSeconds", timeOutInSeconds);
	}
	virtual void notifyRegistration(EventObserver* newObserver)
	{
		mock().actualCall("notifyRegistration").onObject(this).withParameter("newObserver", newObserver);
	}
};

class EventComparator : public MockNamedValueComparator
{
public:
	virtual bool isEqual(void* object1, void* object2)
	{
		return ((Event*)object1)->type == ((Event*)object2)->type;
	}
	virtual SimpleString valueToString(void* object)
	{
		return StringFrom(((Event*)object)->type);
	}
};


TEST_GROUP(EventDispatcher)
{
	Event event;
	EventDispatcher* dispatcher;
	ObserverMock observer;
	ObserverMock observer2;
	EventComparator eventComparator;

	void setup()
	{
		dispatcher = new EventDispatcher;
		mock().installComparator("Event", eventComparator);
	}
	void teardown()
	{
		delete dispatcher;
		mock().removeAllComparators();
	}
};


TEST(EventDispatcher, EventWithoutRegistrationsResultsIntoNoCalls)
{
	dispatcher->dispatchEvent(event, 10);
}

TEST(EventDispatcher, EventWithRegistrationForEventResultsIntoCallback)
{
	mock().expectOneCall("notify").onObject(&observer).withParameterOfType("Event", "event", &event).withParameter("timeOutInSeconds", 10);
	event.type = IMPORTANT_EVENT;

	dispatcher->registerObserver(IMPORTANT_EVENT, &observer);
	dispatcher->dispatchEvent(event, 10);
}

TEST(EventDispatcher, DifferentEventWithRegistrationDoesNotResultIntoCallback)
{
	event.type = LESS_IMPORTANT_EVENT;
	dispatcher->registerObserver(IMPORTANT_EVENT, &observer);
	dispatcher->dispatchEvent(event, 10);
}

TEST(EventDispatcher, RegisterTwoObserversResultIntoTwoCallsAndARegistrationNotification)
{
	mock().expectOneCall("notify").onObject(&observer).withParameterOfType("Event", "event", &event).withParameter("timeOutInSeconds", 10);
	mock().expectOneCall("notify").onObject(&observer2).withParameterOfType("Event", "event", &event).withParameter("timeOutInSeconds", 10);
	mock().expectOneCall("notifyRegistration").onObject(&observer).withParameter("newObserver", &observer2);

	event.type = IMPORTANT_EVENT;
	dispatcher->registerObserver(IMPORTANT_EVENT, &observer);
	dispatcher->registerObserver(IMPORTANT_EVENT, &observer2);
	dispatcher->dispatchEvent(event, 10);
}
