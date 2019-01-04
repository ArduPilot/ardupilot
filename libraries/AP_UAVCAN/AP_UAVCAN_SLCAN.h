
#pragma once

#include <AP_HAL/AP_HAL.h>
#if HAL_WITH_UAVCAN && !HAL_MINIMIZE_FEATURES && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "AP_HAL/utility/RingBuffer.h"


#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64
#define SLCAN_DRIVER_INDEX 2

class SLCANRouter;

namespace SLCAN {
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
class CANManager;
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
class CAN: public AP_HAL::CAN {
    friend class CANManager;
    friend class ::SLCANRouter;
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

    uint32_t bitrate_;

    virtual int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                         uavcan::CanIOFlags flags) override;

    virtual int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                            uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override;

    int16_t reportFrame(const uavcan::CanFrame& frame, bool loopback, uint64_t timestamp_usec);

    virtual int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs, uint16_t num_configs) override
    {
        //TODO: possibly check at the first serial read
        return 0;
    }

    virtual uint16_t getNumFilters() const override
    {
        return NumFilters;
    }

    /**
     * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
     * May increase continuously if the interface is not connected to the bus.
     */
    virtual uint64_t getErrorCount() const override
    {
        return 0;
    }

    const char* processCommand(char* cmd);

    bool push_Frame(uavcan::CanFrame &frame);

    bool handle_FrameRTRStd(const char* cmd);
    bool handle_FrameRTRExt(const char* cmd);
    bool handle_FrameDataStd(const char* cmd);
    bool handle_FrameDataExt(const char* cmd);
    void reader();

    inline void addByte(const uint8_t byte);

    bool initialized_;
    bool _port_initialised;
    char buf_[SLCAN_BUFFER_SIZE + 1];
    int16_t pos_ = 0;
    AP_HAL::UARTDriver *_port = nullptr;

    ObjectBuffer<CanRxItem> rx_queue_;
    uint8_t self_index_;
    HAL_Semaphore rx_sem_;
    unsigned _pending_frame_size = 0;

    const uint32_t _serial_lock_key = 0x53494442;
    bool _close = true;
public:

    CAN(uint8_t self_index, uint8_t rx_queue_capacity):
    self_index_(self_index), rx_queue_(rx_queue_capacity), _port_initialised(false) 
    {
        UAVCAN_ASSERT(self_index_ < CAN_STM32_NUM_IFACES);
    }
    enum {
        MaxRxQueueCapacity = 254
    };

    enum OperatingMode {
        NormalMode, SilentMode
    };

    int init(const uint32_t bitrate, const OperatingMode mode, AP_HAL::UARTDriver* port);

    bool begin(uint32_t bitrate) override
    {
        if (init(bitrate, OperatingMode::NormalMode, nullptr) == 0) {
            bitrate_ = bitrate;
            initialized_ = true;
        } else {
            initialized_ = false;
        }
        return initialized_;
    }

    void end() override
    {
    }

    void reset() override;

    int32_t tx_pending() override {
        return _port->tx_pending() ? 0:-1;
    }

    int32_t available() override {
        return _port->available() ? 0:-1;
    }

    bool is_initialized() override {
        return initialized_;
    }
    bool closed() { return _close; }
    bool pending_frame_sent();

    bool isRxBufferEmpty(void);
    bool canAcceptNewTxFrame() const;
};


class CANManager: public AP_HAL::CANManager, public uavcan::ICanDriver {
    bool initialized_;
    CAN driver_;
    uint8_t _ifaces_num = 1;

    virtual int16_t select(uavcan::CanSelectMasks& inout_masks,
                           const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline) override;

    uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces]);
    thread_t *_irq_handler_ctx = nullptr;
public:
    CANManager()
        :  AP_HAL::CANManager(this), initialized_(false), driver_(SLCAN_DRIVER_INDEX, SLCAN_RX_QUEUE_SIZE)
        { }

    /**
     * Whether at least one iface had at least one successful IO since previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    //bool hadActivity();

    static CANManager *from(AP_HAL::CANManager *can)
    {
        return static_cast<CANManager*>(can);
    }

    bool begin(uint32_t bitrate, uint8_t can_number) override;

    /*
     Test if CAN manager is ready and initialized
     return      false - CAN manager not initialized
     true - CAN manager is initialized
     */
    bool is_initialized() override;
    void initialized(bool val) override;

    virtual CAN* getIface(uint8_t iface_index) override { return &driver_; }

    virtual uint8_t getNumIfaces() const override
    {
        return _ifaces_num;
    }

    void reader_trampoline(void);
};

}
#include <AP_HAL_ChibiOS/CANSerialRouter.h>

#endif //#if HAL_WITH_UAVCAN && !HAL_MINIMIZE_FEATURES
