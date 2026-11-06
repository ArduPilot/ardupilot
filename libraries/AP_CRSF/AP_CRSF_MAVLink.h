/*
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

/*
 * Bidirectional MAVLink transport over CRSF 0xAA envelope frames.
 *
 * Incoming MAVLink bytes arrive chunked in 0xAA frames and are reassembled
 * here before being injected into a virtual UART for the GCS_MAVLINK backend
 * to read normally.  Outgoing MAVLink responses are drained from the virtual
 * UART TX buffer, chunked into 0xAA frames, and returned to the CRSF
 * telemetry scheduler for transmission.
 */
#pragma once

#include <AP_RCTelemetry/AP_RCTelemetry_config.h>

#if AP_CRSF_MAVLINK_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_CRSF_Protocol.h"

class AP_CRSF_MAVLink
{
    friend class TestCRSFMAVLink;
public:
    // register a virtual GCS backend on our internal UART
    bool init();

    // handle an incoming 0xAA frame from the CRSF protocol decoder
    void process_frame(const uint8_t *payload, uint8_t len);

    // produce the next outbound 0xAA frame from pending MAVLink TX data.
    // returns true if a frame was written into *frame
    bool get_telem_frame(uint8_t *payload, uint8_t &len);

    // return true if the virtual UART has TX data waiting to be sent
    bool tx_pending() const;

private:

    // maximum data bytes per CRSF 0xAA chunk
    static constexpr uint8_t MAX_CHUNK_DATA = 58;

    // reassembly timeout in milliseconds
    static constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 500;

    // max MAVLink2 frame size
    static constexpr uint16_t MAVLINK_MAX_FRAME = 280;

    // --- Virtual UART ---
    // minimal UARTDriver subclass backed by two ByteBuffers
    class VirtualUARTDriver : public AP_HAL::UARTDriver
    {
    public:
        VirtualUARTDriver();

        // methods used by CRSF side
        uint32_t inject_rx(const uint8_t *data, uint32_t len);
        uint32_t drain_tx(uint8_t *data, uint32_t len);
        uint32_t peek_tx(uint8_t *data, uint32_t len);
        uint32_t tx_available() const;

        // UARTDriver interface
        bool is_initialized() override
        {
            return _initialized;
        }
        bool tx_pending() override
        {
            return _tx_buf.available() > 0;
        }
        uint32_t txspace() override
        {
            return _tx_buf.space();
        }

    protected:
        void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
        size_t _write(const uint8_t *buffer, size_t size) override;
        ssize_t _read(uint8_t *buffer, uint16_t count) override;
        void _end() override;
        void _flush() override {}
        uint32_t _available() override;
        bool _discard_input() override;

    private:
        static constexpr uint16_t BUF_SIZE = 2048;
        uint8_t _rx_backing[BUF_SIZE];
        uint8_t _tx_backing[BUF_SIZE];
        ByteBuffer _rx_buf;  // CRSF → GCS (incoming MAVLink from remote)
        ByteBuffer _tx_buf;  // GCS → CRSF (outgoing MAVLink responses)
        bool _initialized;
    };

    VirtualUARTDriver _uart;

    // --- RX reassembly state ---
    struct Reassembly {
        uint8_t buf[MAVLINK_MAX_FRAME];
        uint16_t write_offset;
        uint8_t expected_total;
        uint8_t next_chunk;
        uint32_t start_time_ms;
        bool active;
        void reset();
    } _reassembly;

    // --- TX chunking state ---
    struct TxState {
        uint8_t buf[MAVLINK_MAX_FRAME];
        uint16_t msg_len;
        uint8_t total_chunks;
        uint8_t next_chunk;
        bool active;
        void reset();
    } _tx_state;

    bool _initialized;
};

#endif  // AP_CRSF_MAVLINK_ENABLED
