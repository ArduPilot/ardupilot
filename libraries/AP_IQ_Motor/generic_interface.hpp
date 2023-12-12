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
  Name: generic_interface.hpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#ifndef GENERIC_INTERFACE_H
#define GENERIC_INTERFACE_H

#include "communication_interface.h"
#include "packet_finder.h"
#include "byte_queue.h"
#include "bipbuffer.h"

#define GENERIC_PF_INDEX_DATA_SIZE 20   // size of index buffer in packet_finder

#ifndef GENERIC_TX_BUFFER_SIZE
#define GENERIC_TX_BUFFER_SIZE 256
#endif

class GenericInterface: public CommunicationInterface
{
private:

public:
    // Member Variables
    // packet_finder instance
    struct PacketFinder pf;

    // needed by pf for storing indices
    struct ByteQueue index_queue;

    // data for index_queue used by pf
    uint8_t pf_index_data[GENERIC_PF_INDEX_DATA_SIZE];

    // bipbuffer for transmissions
    BipBuffer tx_bipbuf;

    // raw buffer for transmissions
    uint8_t tx_buffer[GENERIC_TX_BUFFER_SIZE];

    // Default Constructor
    GenericInterface();

    /*******************************************************************************
     * Receive
     ******************************************************************************/

    /// Poll the hardware for new byte data.
    ///   Returns: 1 packet ready
    ///            0 normal operation
    ///           -1 failure
    ///
    int8_t GetBytes() override;
    int8_t SetRxBytes(uint8_t* data_in, uint16_t length_in);

    /// Peek at the next available incoming packet. If a packet is ready, pointer
    /// 'packet' will point to the first byte of type+data and 'length' will give
    /// the length of packet type+data. Arguments 'packet' and 'length' are ignored
    /// if no packet is ready.  Repeated calls to Peek will return pointers to the
    /// same packet data until Drop is used.
    ///   Returns: 1 packet peek available
    ///            0 no packet peek available
    ///           -1 failure
    ///
    int8_t PeekPacket(uint8_t **packet, uint8_t *length) override;

    /// Drop the next available packet from queue. Usually called after Peek.
    ///   Returns: 1 packet removed
    ///            0 no packet ready to remove
    ///           -1 failure
    ///
    int8_t DropPacket() override;

    /*******************************************************************************
     * Send
     ******************************************************************************/

    /// Add a packet to the outgoing queue with automatically generated header
    /// and CRC. If the buffer fills, the most recent data is lost.
    int8_t SendPacket(uint8_t msg_type, uint8_t *data, uint16_t length) override;

    /// Add bytes to the outgoing queue.
    /// If the buffer fills, the most recent data is lost.
    int8_t SendBytes(uint8_t *bytes, uint16_t length) override;

    /// Does nothing in this interface
    void SendNow() override;

    /// Gets all outbound bytes
    /// The data is copied into the user supplied data_out buffer.
    /// The length of data transferred is copied into length_out.
    /// Returns: 1 for data transferred
    ///          0 for no data transferred (buffer empty)
    int8_t GetTxBytes(uint8_t* data_out, uint8_t& length_out);

    /*******************************************************************************
     * Parsing
     ******************************************************************************/

    /// Read a given message and act appropriately.
    void ReadMsg(CommunicationInterface& com, uint8_t* data, uint8_t length) override;
}; // class GenericInterface

#endif // GENERIC_INTERFACE_H