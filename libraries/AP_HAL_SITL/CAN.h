/* Copyright (C) 2017 Eugene Shamaev
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

#pragma once

#include "AP_HAL_SITL.h"

#if HAL_WITH_UAVCAN
#include <AP_HAL/CAN.h>

class HALSITL::HALSITLCAN : public AP_HAL::CAN {
public:
    HALSITLCAN() { }
    ~HALSITLCAN() { }
    int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override;
    int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic, uavcan::UtcTime& out_ts_utc,
            uavcan::CanIOFlags& out_flags) override;
    int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs, uint16_t num_configs) override;
    uint16_t getNumFilters() const override;
    uint64_t getErrorCount() const override;

    bool begin(uint32_t bitrate) override;
    void end() override;
    void reset() override;
    bool is_initialized() override;
    int32_t tx_pending() override;
    int32_t available() override;


private:
    uint32_t _baudrate;
    volatile bool _initialised;
    volatile bool _in_timer;
};

class HALSITL::HALSITLCANDriver : public AP_HAL::CANManager
{
public:
    HALSITLCANDriver() { }
    ~HALSITLCANDriver() { }
    uavcan::ICanIface* getIface(uint8_t iface_index) override;
    const uavcan::ICanIface* getIface(uint8_t iface_index) const override;
    uint8_t getNumIfaces() const override;
    int16_t select(uavcan::CanSelectMasks& inout_masks,
                   const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
                   uavcan::MonotonicTime blocking_deadline) override;
};

#endif
