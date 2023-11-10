/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  IQ C++ API is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  IQ C++ API is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Name: client_communication.hpp
  Last update: 4/12/2019 by Matthew Piccoli
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

//Vertiq's clients provide access to various motor parameters and control mechanisms through entries.
//This communication communication object provides a method of commuincating
//with the various clients available on a Vertiq module. Each entry has a type id,
//an object id, and a sub id each allowing this client and the motor to communicate
//with a specified endpoint.

#ifndef CLIENT_COMMUNICATION_H
#define CLIENT_COMMUNICATION_H

#include <string.h> // for memcpy
#include "communication_interface.h"

//Determines what action is performed on a client: getting a value, setting a value, saving a value, or replying to a get request
enum Access {kGet=0, kSet=1, kSave=2, kReply=3};

//This is the top level, abstract, client entry. It provides the template for all entries to follow.
//It has a type, object, and sub id to define what endpoint it belongs to, the module ID, and the location within the endpoint
class Client_Entry_Abstract {
public:

    //Create an instance of a Client_Entry_Abstract with a type id, object id, and sub id
    //type_idn represents the endpoint type for this entry
    //obj_idn represents the target module's module ID
    //sub_idn represents the array index of this entry in the endpoint
    Client_Entry_Abstract(uint8_t type_idn, uint8_t obj_idn, uint8_t sub_idn):
        type_idn_(type_idn),
        obj_idn_(obj_idn),
        sub_idn_(sub_idn) {};

    //Destructor of a Client_Entry_Abstract object
    virtual ~Client_Entry_Abstract() {};

    //Completely virtual reply function. Must be defined by all child classes
    virtual void Reply(const uint8_t* data, uint8_t len) = 0;

    //The endpoint type for this entry
    const uint8_t type_idn_;
    //The target module's module ID
    const uint8_t obj_idn_;
    //The array index of this entry in the endpoint
    const uint8_t sub_idn_;
};

//A typeless extension of the Client_Entry_Abstract class. Defines get, set, and save functions as well as the Reply function
class Client_Entry_Void: public Client_Entry_Abstract {
public:
    //Create an instance of a Client_Entry_Void with a type id, object id, and sub id
    //type_idn represents the endpoint type for this entry
    //obj_idn represents the target module's module ID
    //sub_idn represents the array index of this entry in the endpoint
    Client_Entry_Void(uint8_t type_idn, uint8_t obj_idn, uint8_t sub_idn):
        Client_Entry_Abstract(type_idn, obj_idn, sub_idn),
        _is_fresh_(false)
    {};

    //Use the Communication_Interface to send a get request for this entry from the flight controller to the module
    void get(Communication_Interface &com)
    {
        uint8_t tx_msg[2];
        tx_msg[0] = sub_idn_;
        tx_msg[1] = (obj_idn_<<2) | kGet; // high six | low two
        com.SendPacket(type_idn_, tx_msg, 2);
    };

    //Use the Communication_Interface to send a set command for this entry from the flight controller to the module
    void set(Communication_Interface &com)
    {
        uint8_t tx_msg[2]; // must fit outgoing message
        tx_msg[0] = sub_idn_;
        tx_msg[1] = (obj_idn_<<2) | kSet; // high six | low two
        com.SendPacket(type_idn_, tx_msg, 2);
    }

    //Use the Communication_Interface to send a save request for this entry from the flight controller to the module
    void save(Communication_Interface &com)
    {
        uint8_t tx_msg[2];
        tx_msg[0] = sub_idn_;
        tx_msg[1] = (obj_idn_<<2) | kSave; // high six | low two
        com.SendPacket(type_idn_, tx_msg, 2);
    }

    //Do nothing
    void Reply(const uint8_t* data, uint8_t len) override
    {
        (void)data;
        if (len == 0) {
            _is_fresh_ = true;
        }
    };

    //Return our internal _is_fresh_ bool's state
    bool IsFresh()
    {
        return _is_fresh_;
    };

private:
    bool _is_fresh_;
};

//Represents a client entry that has a known type, for example a float. This Client stores a parameter of type T,
//that can be gotten, set, or saved.
template <typename T>
class ClientEntry: public Client_Entry_Abstract {
public:
    //Create an instance of a ClientEntry with a type id, object id, sub id, and a type T through the template
    //type_idn represents the endpoint type for this entry
    //obj_idn represents the target module's module ID
    //sub_idn represents the array index of this entry in the endpoint
    ClientEntry(uint8_t type_idn, uint8_t obj_idn, uint8_t sub_idn):
        Client_Entry_Abstract(type_idn, obj_idn, sub_idn),
        _is_fresh_(false),
        _value_(),
        _unfulfilled_(0)
    {};

    //Use the Communication_Interface to send a get request for this entry from the flight controller to the module
    void get(Communication_Interface &com)
    {
        uint8_t tx_msg[2];
        tx_msg[0] = sub_idn_;
        tx_msg[1] = (obj_idn_<<2) | kGet; // high six | low two
        com.SendPacket(type_idn_, tx_msg, 2);
        _unfulfilled_++;
    };

    //Use the Communication_Interface to send a set request for this entry from the flight controller to the module
    void set(Communication_Interface &com, T value)
    {
        uint8_t tx_msg[2+sizeof(T)]; // must fit outgoing message
        tx_msg[0] = sub_idn_;
        tx_msg[1] = (obj_idn_<<2) | kSet; // high six | low two
        memcpy(&tx_msg[2], &value, sizeof(T));
        com.SendPacket(type_idn_, tx_msg, 2+sizeof(T));
    }

    //Use the Communication_Interface to send a save request for this entry from the flight controller to the module
    void save(Communication_Interface &com)
    {
        uint8_t tx_msg[2];
        tx_msg[0] = sub_idn_;
        tx_msg[1] = (obj_idn_<<2) | kSave; // high six | low two
        com.SendPacket(type_idn_, tx_msg, 2);
    }

    //Use the Communication_Interface to send a reply for this entry from the flight controller to the module
    void Reply(const uint8_t* data, uint8_t len) override
    {
        if (len == sizeof(T)) {
            memcpy(&_value_, data, sizeof(T));
            _is_fresh_ = true;
            _unfulfilled_--;
        }
    };

    //Return the value stored in this entry, and set _is_fresh_ to false
    T get_reply()
    {
        _is_fresh_ = false;
        return _value_;
    };

    //Return our internal _is_fresh_ bool's state
    bool IsFresh()
    {
        return _is_fresh_;
    };

    //Get the number of uncompleted get requests
    int Unfulfilled()
    {
        return _unfulfilled_;
    };

    //Reset the number of uncompleted get requests
    void ResetUnfulfilled()
    {
        _unfulfilled_ = 0;
    };

private:
    bool _is_fresh_;
    int _unfulfilled_;
    T _value_;
};

//A client holds client entries. All clients have a type and an object id that make them unique to each
//module with a unique module ID
class ClientAbstract {
public:
    //Create a client with a type id and object id
    ClientAbstract(uint8_t type_idn, uint8_t obj_idn):
        type_idn_(type_idn),
        obj_idn_(obj_idn) {};

    //Destructor for a ClientAbstract object
    virtual ~ClientAbstract() {};

    //All clients must know how to read incoming data in order to send requests to its endpoints properly
    virtual void ReadMsg(uint8_t* rx_data, uint8_t rx_length) = 0;

    //The type of this Client
    const uint8_t type_idn_;

    //The object id of the module in use
    const uint8_t obj_idn_;
};

//Definition of a function used to read data from a recveived message, and find who the message is for. Must be defined by each client.
int8_t ParseMsg(uint8_t* rx_data, uint8_t rx_length,
                Client_Entry_Abstract** entry_array, uint8_t entry_length);

//Definition of a function used to read data from a recveived message, and find who the message is for. Must be defined by each client.
int8_t ParseMsg(uint8_t* rx_data, uint8_t rx_length,
                Client_Entry_Abstract& entry);

#endif // CLIENT_COMMUNICATION_H
