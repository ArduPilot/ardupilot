// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _TEST_TRANSPORT_SERIALCOMM_HPP_
#define _TEST_TRANSPORT_SERIALCOMM_HPP_

#include <gtest/gtest.h>
#include <uxr/client/client.h>

class CircularBuffer
{
public:

    CircularBuffer(
            size_t len)
        : maxlen(len)
        , head(0)
        , tail(0)
    {
        buffer = static_cast<uint8_t*>(malloc(maxlen));
    }

    ~CircularBuffer()
    {
        free(buffer);
    }

    int write(
            uint8_t data)
    {
        size_t next;

        next = head + 1;
        if (next >= maxlen)
        {
            next = 0;
        }

        if (next == tail)
        {
            return -1;
        }

        buffer[head] = data;
        head = next;
        return 0;
    }

    int read(
            uint8_t* data)
    {
        size_t next;

        if (head == tail)
        {
            return -1;
        }

        next = tail + 1;
        if (next >= maxlen)
        {
            next = 0;
        }

        *data = buffer[tail];
        tail = next;
        return 0;
    }

private:

    uint8_t* buffer;
    size_t maxlen;
    size_t head;
    size_t tail;
};

class CustomComm : public testing::Test
{
public:

    CustomComm();
    ~CustomComm();

protected:

    uxrCustomTransport master_;
    uxrCustomTransport slave_;

    static bool open(
            uxrCustomTransport* args);
    static bool close(
            uxrCustomTransport* transport);
    static size_t write(
            uxrCustomTransport* transport,
            const uint8_t* buf,
            size_t len,
            uint8_t* error);
    static size_t read(
            uxrCustomTransport* transport,
            uint8_t* buf,
            size_t len,
            int timeout,
            uint8_t* error);

    CircularBuffer buffer;
    size_t max_payload;
};

#endif //_TEST_TRANSPORT_SERIALCOMM_HPP_
