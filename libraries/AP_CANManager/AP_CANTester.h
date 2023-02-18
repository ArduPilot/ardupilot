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

#pragma once

#include "AP_CANDriver.h"
#include <AP_HAL/Semaphores.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#ifndef HAL_ENABLE_CANTESTER
#define HAL_ENABLE_CANTESTER 0
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 1 && !HAL_MINIMIZE_FEATURES && HAL_CANMANAGER_ENABLED && HAL_ENABLE_CANTESTER

class CANTester : public AP_CANDriver
{
public:
    CANTester()
    {
        // update protected var for parameter interface
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    CLASS_NO_COPY(CANTester);

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;
    bool run_kdecan_enumeration(bool start_stop);

    static CANTester *get_cantester(uint8_t driver_index);

private:

    enum {
        TEST_NONE,
        TEST_LOOPBACK,
        TEST_BUSOFF_RECOVERY,
        TEST_UAVCAN_DNA,
        TEST_KDE_CAN,
        TEST_UAVCAN_ESC,
        TEST_UAVCAN_FD_ESC,
        TEST_END,
    };

    struct loopback_stats_s {
        uint32_t num_tx;
        uint32_t failed_tx;
        uint32_t num_rx;
        uint32_t num_flushed_rx;
        uint32_t bad_rx_data;
        uint32_t bad_rx_seq;
        uint16_t tx_seq;
        uint16_t next_valid_seq;
    } _loopback_stats[HAL_NUM_CAN_IFACES];

    void main_thread();
    bool test_loopback(uint32_t loop_rate);
    void create_loopback_frame(loopback_stats_s &stats, AP_HAL::CANFrame& frame);
    void check_loopback_frame(loopback_stats_s &stats, AP_HAL::CANFrame& frame);


    bool test_busoff_recovery();

    bool test_uavcan_dna();

    bool test_kdecan();

    bool test_uavcan_esc(bool enable_canfd);

    // write frame on CAN bus, returns true on success
    bool write_frame(uint8_t iface, AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(uint8_t iface, AP_HAL::CANFrame &recv_frame, uint64_t timeout, AP_HAL::CANIface::CanIOFlags &flags);

    void reset_frame(AP_HAL::CANFrame& frame);

    bool _initialized;
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_ifaces[HAL_NUM_CAN_IFACES];

    // Classes required for UAVCAN Test
    class RaiiSynchronizer {};
    uavcan::PoolAllocator<UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_BLOCK_SIZE, CANTester::RaiiSynchronizer> _node_allocator;

    HAL_EventHandle _event_handle;
    AP_Int8 _test_driver_index;
    AP_Int8 _test_port;
    AP_Int32 _test_id;
    AP_Int32 _loop_rate;
    uint8_t _num_ifaces;
    bool _kdecan_enumeration;
};
#endif //#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 1 && !HAL_MINIMIZE_FEATURES && HAL_MAX_CAN_PROTOCOL_DRIVERS

