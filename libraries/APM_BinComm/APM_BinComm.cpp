// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-
//
// Copyright (c) 2010 Michael Smith. All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

/// @file       BinComm.cpp
/// @brief      Implementation of the ArduPilot Mega binary communications
///		library.

#include "APM_BinComm.h"
#include "WProgram.h"

/// @name decoder state machine phases
//@{
#define DEC_WAIT_P1      0
#define DEC_WAIT_P2      1
#define DEC_WAIT_HEADER  2
#define DEC_WAIT_MESSAGE 3
#define DEC_WAIT_SUM_A   4
#define DEC_WAIT_SUM_B   5
//@}


/// inter-byte timeout for decode (ms)
#define DEC_MESSAGE_TIMEOUT     100

BinComm::BinComm(const BinComm::MessageHandler *handlerTable,
                 Stream *interface) :
        _handlerTable(handlerTable),
        _interface(interface)
{
};

void
BinComm::_sendMessage(void)
{
        uint8_t bytesToSend;
        uint8_t sumA, sumB;
        const uint8_t *p;

        // send the preamble first
        _interface->write((uint8_t)MSG_PREAMBLE_1);
        _interface->write((uint8_t)MSG_PREAMBLE_2);

        // set up to send the payload
        bytesToSend = _encodeBuf.header.length + sizeof(_encodeBuf.header);
        sumA = sumB = 0;
        p = (const uint8_t *)&_encodeBuf;

        // send message bytes and compute checksum on the fly
        while (bytesToSend--) {
                sumA += *p;
                sumB += sumA;
                _interface->write(*p++);
        }

        // send the checksum
        _interface->write(sumA);
        _interface->write(sumB);
}

void
BinComm::update(void)
{
        uint8_t         count;

        // Ensure that we don't spend too long here by only processing
        // the bytes that were available when we started.
        //
        // XXX we might want to further constrain this count
        count = _interface->available();
        while (count--)
                _decode(_interface->read());
}

void
BinComm::_decode(uint8_t inByte)
{
        uint8_t         tableIndex;

        // handle inter-byte timeouts (resync after link loss, etc.)
        //
        if ((millis() - _lastReceived) > DEC_MESSAGE_TIMEOUT)
                _decodePhase = DEC_WAIT_P1;

        // run the decode state machine
        //
        switch (_decodePhase) {

                // Preamble detection
                //
                // Note the fallthrough from P2 to P1 deals with the case where
                // we see 0x34, 0x34, 0x44 where the first 0x34 is garbage or 
                // a SUM_B byte we never looked at.
                //
        case DEC_WAIT_P2:
                if (MSG_PREAMBLE_2 == inByte) {
                        _decodePhase++;

                        // prepare for the header
                        _bytesIn = 0;
                        _bytesExpected = sizeof(MessageHeader);

                        // intialise the checksum accumulators
                        _sumA = _sumB = 0;

                        break;
                }
                _decodePhase = DEC_WAIT_P1;
                // FALLTHROUGH
        case DEC_WAIT_P1:
                if (MSG_PREAMBLE_1 == inByte) {
                        _decodePhase++;
                }
                break;

                // receiving the header
                //
        case DEC_WAIT_HEADER:
                // do checksum accumulation
                _sumA += inByte;
                _sumB += _sumA;

                // store the byte
                _decodeBuf.bytes[_bytesIn++] = inByte;

                // check for complete header received
                if (_bytesIn == _bytesExpected) {
                        _decodePhase++;

                        // prepare for the payload
                        // variable-length data?
                        _bytesIn = 0;
                        _bytesExpected = _decodeBuf.header.length;
                        _messageID = _decodeBuf.header.messageID;
                        _messageVersion = _decodeBuf.header.messageVersion;

                        // sanity check to avoid buffer overflow - revert back to waiting
                        if (_bytesExpected > sizeof(_decodeBuf))
                                _decodePhase = DEC_WAIT_P1;
                }
                break;

                // receiving payload data
                //
        case DEC_WAIT_MESSAGE:
                // do checksum accumulation
                _sumA += inByte;
                _sumB += _sumA;

                // store the byte
                _decodeBuf.bytes[_bytesIn++] = inByte;

                // check for complete payload received
                if (_bytesIn == _bytesExpected) {
                        _decodePhase++;
                }
                break;

                // waiting for the checksum bytes
                //
        case DEC_WAIT_SUM_A:
                if (inByte != _sumA) {
                        badMessagesReceived++;
                        _decodePhase = DEC_WAIT_P1;
                } else {
                        _decodePhase++;
                }
                break;
        case DEC_WAIT_SUM_B:
                if (inByte == _sumB) {
                        // if we got this far, we have a message
                        messagesReceived++;

                        // call any handler interested in this message
                        for (tableIndex = 0; MSG_NULL != _handlerTable[tableIndex].messageID; tableIndex++)
                                if ((_handlerTable[tableIndex].messageID == _messageID) ||
                                    (_handlerTable[tableIndex].messageID == MSG_ANY))
                                        _handlerTable[tableIndex].handler(_handlerTable[tableIndex].arg, _messageID, _messageVersion, &_decodeBuf);
                } else {
                        badMessagesReceived++;
                }
                _decodePhase = DEC_WAIT_P1;
                break;
        }
}
