/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * With modifications for Ardupilot CAN driver
 * Copyright (C) 2017 Eugene Shamaev
 */

#pragma once

#include "AP_HAL_PX4.h"
#include <systemlib/perf_counter.h>
#include <AP_HAL/CAN.h>
#include <pthread.h>
#include <semaphore.h>

#include "bxcan.h"
#include "AP_HAL/utility/RingBuffer.h"
                                      
#if defined(GPIO_CAN2_RX) && defined(GPIO_CAN2_TX)
#define CAN_STM32_NUM_IFACES 2
#else
#define CAN_STM32_NUM_IFACES 1
#endif

#define CAN_STM32_RX_QUEUE_SIZE 64

namespace PX4 {
/**
 * Driver error codes.
 * These values can be returned from driver functions negated.
 */
static const int16_t ErrUnknown = 1000; ///< Reserved for future use
static const int16_t ErrNotImplemented = 1001; ///< Feature not implemented
static const int16_t ErrInvalidBitRate = 1002; ///< Bit rate not supported
static const int16_t ErrLogic = 1003; ///< Internal logic error
static const int16_t ErrUnsupportedFrame = 1004; ///< Frame not supported (e.g. RTR, CAN FD, etc)
static const int16_t ErrMsrInakNotSet = 1005; ///< INAK bit of the MSR register is not 1
static const int16_t ErrMsrInakNotCleared = 1006; ///< INAK bit of the MSR register is not 0
static const int16_t ErrBitRateNotDetected = 1007; ///< Auto bit rate detection could not be finished
static const int16_t ErrFilterNumConfigs = 1008; ///< Auto bit rate detection could not be finished

/**
 * RX queue item.
 * The application shall not use this directly.
 */
struct CanRxItem {
    uint64_t utc_usec;
    uavcan::CanFrame frame;
    uavcan::CanIOFlags flags;
    CanRxItem() :
        utc_usec(0), flags(0)
    {
    }
};

struct CriticalSectionLocker {
    const irqstate_t flags_;

    CriticalSectionLocker() :
        flags_(irqsave())
    {
    }

    ~CriticalSectionLocker()
    {
        irqrestore(flags_);
    }
};

namespace clock {
uint64_t getUtcUSecFromCanInterrupt();
uavcan::MonotonicTime getMonotonic();
}

class BusEvent: uavcan::Noncopyable {
public:
    BusEvent(PX4CANManager& can_driver);
    ~BusEvent();

    bool wait(uavcan::MonotonicDuration duration);
    static void signalFromCallOut(BusEvent *sem);

    void signalFromInterrupt();
    sem_t _wait_semaphore;
    volatile uint16_t _signal;
};

class PX4CAN: public AP_HAL::CAN {
    struct Timings {
        uint16_t prescaler;
        uint8_t sjw;
        uint8_t bs1;
        uint8_t bs2;

        Timings() :
            prescaler(0), sjw(0), bs1(0), bs2(0)
        {
        }
    };

    struct TxItem {
        uavcan::MonotonicTime deadline;
        uavcan::CanFrame frame;

        bool pending;

        bool loopback;

        bool abort_on_error;

        TxItem() :
            pending(false), loopback(false), abort_on_error(false)
        {
        }
    };

    enum {
        NumTxMailboxes = 3
    };
    enum {
        NumFilters = 14
    };

    static const uint32_t TSR_ABRQx[NumTxMailboxes];

    ObjectBuffer<CanRxItem> rx_queue_;
    bxcan::CanType* const can_;
    uint64_t error_cnt_;
    uint32_t served_aborts_cnt_;
    BusEvent* update_event_;
    TxItem pending_tx_[NumTxMailboxes];
    uint8_t peak_tx_mailbox_index_;
    const uint8_t self_index_;

    bool had_activity_;

    uint32_t bitrate_;

    int computeTimings(uint32_t target_bitrate, Timings& out_timings);

    virtual int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                         uavcan::CanIOFlags flags) override;

    virtual int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                            uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override;

    virtual int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs, uint16_t num_configs) override;

    virtual uint16_t getNumFilters() const override
    {
        return NumFilters;
    }

    void handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, uint64_t utc_usec);

    bool waitMsrINakBitStateChange(bool target_state);

    bool initialized_;
public:
    enum {
        MaxRxQueueCapacity = 254
    };

    enum OperatingMode {
        NormalMode, SilentMode
    };

    PX4CAN(bxcan::CanType* can, BusEvent* update_event, uint8_t self_index, uint8_t rx_queue_capacity) :
        rx_queue_(rx_queue_capacity), can_(can), error_cnt_(0), served_aborts_cnt_(0), update_event_(
            update_event), peak_tx_mailbox_index_(0), self_index_(self_index), had_activity_(false), bitrate_(
                0), initialized_(false)
    {
        UAVCAN_ASSERT(self_index_ < CAN_STM32_NUM_IFACES);
    }

    /**
     * Initializes the hardware CAN controller.
     * Assumes:
     *   - Iface clock is enabled
     *   - Iface has been resetted via RCC
     *   - Caller will configure NVIC by itself
     */
    int init(const uint32_t bitrate, const OperatingMode mode);

    void set_update_event(BusEvent* update_event)
    {
        update_event_ = update_event;
    }

    void handleTxInterrupt(uint64_t utc_usec);
    void handleRxInterrupt(uint8_t fifo_index, uint64_t utc_usec);

    /**
     * This method is used to count errors and abort transmission on error if necessary.
     * This functionality used to be implemented in the SCE interrupt handler, but that approach was
     * generating too much processing overhead, especially on disconnected interfaces.
     *
     * Should be called from RX ISR, TX ISR, and select(); interrupts must be enabled.
     */
    void pollErrorFlagsFromISR();

    void discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time);

    bool canAcceptNewTxFrame(const uavcan::CanFrame& frame) const;
    bool isRxBufferEmpty() const;

    /**
     * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
     * May increase continuously if the interface is not connected to the bus.
     */
    virtual uint64_t getErrorCount() const override;

    /**
     * Number of times the driver exercised library's requirement to abort transmission on first error.
     * This is an atomic read, it doesn't require a critical section.
     * See @ref uavcan::CanIOFlagAbortOnError.
     */
    uint32_t getVoluntaryTxAbortCount() const
    {
        return served_aborts_cnt_;
    }

    /**
     * Returns the number of frames pending in the RX queue.
     * This is intended for debug use only.
     */
    unsigned getRxQueueLength() const;

    /**
     * Whether this iface had at least one successful IO since the previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    bool hadActivity();

    /**
     * Peak number of TX mailboxes used concurrently since initialization.
     * Range is [1, 3].
     * Value of 3 suggests that priority inversion could be taking place.
     */
    uint8_t getPeakNumTxMailboxesUsed() const
    {
        return uint8_t(peak_tx_mailbox_index_ + 1);
    }

    bool begin(uint32_t bitrate) override;
    void end() override
    {
    }

    void reset() override;

    int32_t tx_pending() override;
    int32_t available() override;

    bool is_initialized() override;
};

class PX4CANManager: public AP_HAL::CANManager, public uavcan::ICanDriver {
    BusEvent update_event_;
    PX4CAN if0_;
    PX4CAN if1_;

    virtual int16_t select(uavcan::CanSelectMasks& inout_masks,
                           const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline) override;

    void initOnce(uint8_t can_number);

    bool initialized_;

    PX4CAN* ifaces[CAN_STM32_NUM_IFACES];
    uint8_t _ifaces_num;
    uint8_t _ifaces_out_to_in[CAN_STM32_NUM_IFACES];

    AP_UAVCAN *p_uavcan;

public:
    PX4CANManager();

    /**
     * This function returns select masks indicating which interfaces are available for read/write.
     */
    uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces]) const;

    /**
     * Whether there's at least one interface where receive() would return a frame.
     */
    bool hasReadableInterfaces() const;

    /**
     * Returns zero if OK.
     * Returns negative value if failed (e.g. invalid bitrate).
     */
    int init(const uint32_t bitrate, const PX4CAN::OperatingMode mode, uint8_t can_number);

    virtual PX4CAN* getIface(uint8_t iface_index) override;
    PX4CAN* getIface_out_to_in(uint8_t iface_index);

    virtual uint8_t getNumIfaces() const override
    {
        return _ifaces_num;
    }

    /**
     * Whether at least one iface had at least one successful IO since previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    bool hadActivity();

    bool begin(uint32_t bitrate, uint8_t can_number) override;

    bool is_initialized() override;
    void initialized(bool val) override;

    AP_UAVCAN *get_UAVCAN(void) override;
    void set_UAVCAN(AP_UAVCAN *uavcan) override;
};
}
