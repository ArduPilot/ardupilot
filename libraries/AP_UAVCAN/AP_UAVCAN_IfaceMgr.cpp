
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
 *
 * Author: Siddharth Bharat Purohit
 */

#include "AP_UAVCAN_IfaceMgr.h"

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_UAVCAN_Clock.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
using namespace uavcan;
extern const AP_HAL::HAL& hal;
#define LOG_TAG "UAVCANIface"

/*****************************************************
 *                                                   *
 *                    CAN Iface                      *
 *                                                   *
 * ***************************************************/

/**
 * Non-blocking transmission.
 *
 * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
 *
 * Note that it is LIKELY that the library will want to send the frames that were passed into the select()
 * method as the next ones to transmit, but it is NOT guaranteed. The library can replace those with new
 * frames between the calls.
 *
 * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
 */
int16_t CanIface::send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags)
{
    if (can_iface_ == UAVCAN_NULLPTR) {
        return -1;
    }
    return can_iface_->send(AP_HAL::CANFrame(frame.id, frame.data, AP_HAL::CANFrame::dlcToDataLength(frame.dlc), frame.isCanFDFrame()), tx_deadline.toUSec(), flags);
}

/**
 * Non-blocking reception.
 *
 * Timestamps should be provided by the CAN driver, ideally by the hardware CAN controller.
 *
 * Monotonic timestamp is required and can be not precise since it is needed only for
 * protocol timing validation (transfer timeouts and inter-transfer intervals).
 *
 * UTC timestamp is optional, if available it will be used for precise time synchronization;
 * must be set to zero if not available.
 *
 * Refer to @ref ISystemClock to learn more about timestamps.
 *
 * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
 * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
 * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
 */
int16_t CanIface::receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                          CanIOFlags& out_flags)
{

    if (can_iface_ == UAVCAN_NULLPTR) {
        return -1;
    }
    AP_HAL::CANFrame frame;
    uint64_t rx_timestamp;
    uint16_t flags;
    out_ts_monotonic = SystemClock::instance().getMonotonic();
    int16_t ret = can_iface_->receive(frame, rx_timestamp, flags);
    if (ret < 0) {
        return ret;
    }
    out_frame = CanFrame(frame.id, (const uint8_t*)frame.data, AP_HAL::CANFrame::dlcToDataLength(frame.dlc), frame.canfd);
    out_flags = flags;
    if (rx_timestamp != 0) {
        out_ts_utc = uavcan::UtcTime::fromUSec(SystemClock::instance().getAdjustUsec() + rx_timestamp);
    } else {
        out_ts_utc = uavcan::UtcTime::fromUSec(0);
    }
    return ret;
}

/**
 * Number of available hardware filters.
 */
uint16_t CanIface::getNumFilters() const
{
    if (can_iface_ == UAVCAN_NULLPTR) {
        return 0;
    }
    return can_iface_->getNumFilters();
}

/**
 * Continuously incrementing counter of hardware errors.
 * Arbitration lost should not be treated as a hardware error.
 */
uint64_t CanIface::getErrorCount() const
{
    if (can_iface_ == UAVCAN_NULLPTR) {
        return 0;
    }
    return can_iface_->getErrorCount();
}

/*****************************************************
 *                                                   *
 *                    CAN Driver                     *
 *                                                   *
 * ***************************************************/

bool CanIfaceMgr::add_interface(AP_HAL::CANIface *can_iface)
{
    if (num_ifaces > HAL_NUM_CAN_IFACES) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "UAVCANIfaceMgr: Num Ifaces Exceeded\n");
        return false;
    }
    if (can_iface == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "UAVCANIfaceMgr: Iface Null\n");
        return false;
    }
    if (ifaces[num_ifaces] != nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "UAVCANIfaceMgr: Iface already added\n");
        return false;
    }
    ifaces[num_ifaces] = new CanIface(can_iface);
    if (ifaces[num_ifaces] == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "UAVCANIfaceMgr: Can't alloc uavcan::iface\n");
        return false;
    }
    if (!ifaces[num_ifaces]->can_iface_->set_event_handle(&_event_handle)) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "UAVCANIfaceMgr: Setting event handle failed\n");
        return false;
    }
    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG, "UAVCANIfaceMgr: Successfully added interface %d\n", int(num_ifaces));
    num_ifaces++;
    return true;
}
/**
 * Returns an interface by index, or null pointer if the index is out of range.
 */
ICanIface* CanIfaceMgr::getIface(uint8_t iface_index)
{
    if (iface_index >= num_ifaces) {
        return UAVCAN_NULLPTR;
    }
    return ifaces[iface_index];
}

/**
 * Total number of available CAN interfaces.
 * This value shall not change after initialization.
 */
uint8_t CanIfaceMgr::getNumIfaces() const
{
    return num_ifaces;
}

CanSelectMasks CanIfaceMgr::makeSelectMasks(const CanSelectMasks in_mask, const CanFrame* (& pending_tx)[MaxCanIfaces]) const
{
    CanSelectMasks msk;
    for (uint8_t i = 0; i < num_ifaces; i++) {
        bool read = in_mask.read & (1 << i);
        bool write = in_mask.write & (1 << i);
        CanIface* iface = ifaces[i];
        if (iface == nullptr) {
            continue;
        }
        if (pending_tx[i] == UAVCAN_NULLPTR) {
            if (iface->can_iface_->select(read, write, nullptr, 0)) {
                msk.read  |= (read ? 1 : 0) << i;
                msk.write |= (write ? 1 : 0) << i;
            }
        } else {
            AP_HAL::CANFrame frame {pending_tx[i]->id, pending_tx[i]->data, AP_HAL::CANFrame::dlcToDataLength(pending_tx[i]->dlc)};
            if (iface->can_iface_->select(read, write, &frame, 0)) {
                msk.read  |= (read ? 1 : 0) << i;
                msk.write |= (write ? 1 : 0) << i;
            }
        }
    }

    return msk;
}

/**
 * Block until the deadline, or one of the specified interfaces becomes available for read or write.
 *
 * Iface masks will be modified by the driver to indicate which exactly interfaces are available for IO.
 *
 * Bit position in the masks defines interface index.
 *
 * Note that it is allowed to return from this method even if no requested events actually happened, or if
 * there are events that were not requested by the library.
 *
 * The pending TX argument contains an array of pointers to CAN frames that the library wants to transmit
 * next, per interface. This is intended to allow the driver to properly prioritize transmissions; many
 * drivers will not need to use it. If a write flag for the given interface is set to one in the select mask
 * structure, then the corresponding pointer is guaranteed to be valid (not UAVCAN_NULLPTR).
 *
 * @param [in,out] inout_masks        Masks indicating which interfaces are needed/available for IO.
 * @param [in]     pending_tx         Array of frames, per interface, that are likely to be transmitted next.
 * @param [in]     blocking_deadline  Zero means non-blocking operation.
 * @return Positive number of ready interfaces or negative error code.
 */
int16_t CanIfaceMgr::select(CanSelectMasks& inout_masks,
                            const CanFrame* (& pending_tx)[MaxCanIfaces],
                            const MonotonicTime blocking_deadline)
{
    const CanSelectMasks in_masks = inout_masks;
    const uint64_t time = SystemClock::instance().getMonotonic().toUSec();

    inout_masks = makeSelectMasks(in_masks, pending_tx);          // Check if we already have some of the requested events
    if ((inout_masks.read  & in_masks.read)  != 0 ||
        (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }
    if (time < blocking_deadline.toUSec()) {
        _event_handle.wait(blocking_deadline.toUSec() - time); // Block until timeout expires or any iface updates
    }

    inout_masks = makeSelectMasks(in_masks, pending_tx);  // Return what we got even if none of the requested events are set
    return 1;                                   // Return value doesn't matter as long as it is non-negative
}
#endif //HAL_ENABLE_LIBUAVCAN_DRIVERSs
