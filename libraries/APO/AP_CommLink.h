/*
 * AP_CommLink.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_CommLink_H
#define AP_CommLink_H

#include <inttypes.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Common/AP_Vector.h"
#include "../GCS_MAVLink/GCS_MAVLink.h"

class FastSerial;

namespace apo {

class AP_Controller;
class AP_Navigator;
class AP_Guide;
class AP_Board;

enum {
    SEVERITY_LOW, SEVERITY_MED, SEVERITY_HIGH
};

// forward declarations
//class ArduPilotOne;
//class AP_Controller;

/// CommLink class
class AP_CommLink {
public:

    AP_CommLink(FastSerial * link, AP_Navigator * navigator, AP_Guide * guide,
                AP_Controller * controller, AP_Board * board, const uint16_t heartBeatTimeout);
    virtual void send() = 0;
    virtual void receive() = 0;
    virtual void sendMessage(uint8_t id, uint32_t param = 0) = 0;
    virtual void sendText(uint8_t severity, const char *str) = 0;
    virtual void sendText(uint8_t severity, const prog_char_t *str) = 0;
    virtual void acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2) = 0;
    virtual void sendParameters() = 0;
    virtual void requestCmds() = 0;

    /// check if heartbeat is lost
    bool heartBeatLost() {
        if (_heartBeatTimeout == 0)
            return false;
        else
            return ((micros() - _lastHeartBeat) / 1e6) > _heartBeatTimeout;
    }

protected:
    FastSerial * _link;
    AP_Navigator * _navigator;
    AP_Guide * _guide;
    AP_Controller * _controller;
    AP_Board * _board;
    uint16_t _heartBeatTimeout;              /// vehicle heartbeat timeout, s
    uint32_t _lastHeartBeat;                 /// time of last heartbeat, s
};

class MavlinkComm: public AP_CommLink {
public:
    MavlinkComm(FastSerial * link, AP_Navigator * nav, AP_Guide * guide,
                AP_Controller * controller, AP_Board * board, uint16_t heartBeatTimeout);

    virtual void send();
    void sendMessage(uint8_t id, uint32_t param = 0);
    virtual void receive();
    void sendText(uint8_t severity, const char *str);
    void sendText(uint8_t severity, const prog_char_t *str);
    void acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2);

    /**
     * sends parameters one at a time
     */
    void sendParameters();

    /**
     * request commands one at a time
     */
    void requestCmds();

private:

    // options
    bool _useRelativeAlt;

    // commands
    bool _sendingCmds;
    bool _receivingCmds;
    uint16_t _cmdTimeLastSent;
    uint16_t _cmdTimeLastReceived;
    uint16_t _cmdDestSysId;
    uint16_t _cmdDestCompId;
    uint16_t _cmdRequestIndex;
    uint16_t _cmdNumberRequested;
    uint16_t _cmdMax;
    Vector<mavlink_command_t *> _cmdList;

    // parameters
    static uint8_t _paramNameLengthMax;
    uint16_t _parameterCount;
    AP_Var * _queuedParameter;
    uint16_t _queuedParameterIndex;

    // channel
    mavlink_channel_t _channel;
    uint16_t _packetDrops;
    static uint8_t _nChannels;

    void _handleMessage(mavlink_message_t * msg);

    uint16_t _countParameters();

    AP_Var * _findParameter(uint16_t index);

    // check the target
    uint8_t _checkTarget(uint8_t sysid, uint8_t compid);

};

} // namespace apo

#endif // AP_CommLink_H
// vim:ts=4:sw=4:tw=78:expandtab
