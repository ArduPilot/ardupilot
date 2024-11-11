

#include "AP_HAL_ESP32.h"

#ifndef HAL_NUM_CAN_IFACES
#error "HAL_NUM_CAN_IFACES must be defined for this driver, set it to 0 to not use CAN"
#endif

#if HAL_NUM_CAN_IFACES > 0

#ifndef HAL_CAN_DEFAULT_NODE_ID
#error "HAL_CAN_DEFAULT_NODE_ID must be defined for this driver, set it to 0 if not using CAN"
#endif

#include <cassert>
#include <cstring>
#include <AP_Math/AP_Math.h>
#include <xtensa/hal.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>

#include "CANIface.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"



#define CAN1_TX_IRQ_Handler      ESP32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQ_Handler     ESP32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQ_Handler     ESP32_CAN1_RX1_HANDLER
#define CAN2_TX_IRQ_Handler      ESP32_CAN2_TX_HANDLER
#define CAN2_RX0_IRQ_Handler     ESP32_CAN2_RX0_HANDLER
#define CAN2_RX1_IRQ_Handler     ESP32_CAN2_RX1_HANDLER

// from canard.h
#define CANARD_CAN_FRAME_EFF                        (1UL << 31U)         ///< Extended frame format
#define CANARD_CAN_FRAME_RTR                        (1UL << 30U)         ///< Remote transmission (not used by UAVCAN)
#define CANARD_CAN_FRAME_ERR                        (1UL << 29U)         ///< Error frame (not used by UAVCAN)


// periph doesn't use HAL_CANMANAGER_ENABLED 
#if HAL_CANMANAGER_ENABLED
#define printf(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANIface", fmt, ##args); } while (0)
#else
// for periph on esp32, we have a console to get prinf's on..
//#define printf(fmt, args...) do { printf(fmt, ##args); } while (0)
#endif

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
#define PERF_STATS(x) (x++)
#else
#define PERF_STATS(x)
#endif

// ref code https://github.com/espressif/esp-idf/blob/release/v4.4/examples/peripherals/twai/twai_network/twai_network_master/main/twai_network_example_master_main.c


extern AP_HAL::HAL& hal;

using namespace ESP32;

//constexpr bxcan::CanType* const CANIface::Can[];

static ESP32::CANIface* can_ifaces[HAL_NUM_CAN_IFACES+1];//interface number zero is the first one.
uint8_t CANIface::next_interface;

// on -S3 devkit-M rgb LED on pin GPIO48
// esp32:
#define TX_GPIO_NUM         (gpio_num_t)47 
//--  GPIO_NUM_47
#define RX_GPIO_NUM         (gpio_num_t)38
//--  GPIO_NUM_38

//static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
// A workaround is to mark the ISR with IRAM_ATTR attribute to place it into RAM - todo 
// default LEVEL1 intr flag bug.?  https://github.com/espressif/arduino-esp32/issues/489  
static const twai_general_config_t g_config =                      {.mode = TWAI_MODE_NORMAL, .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,        \
                                                                    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      \
                                                                    .tx_queue_len = 5, .rx_queue_len = 5,                           \
                                                                    .alerts_enabled = TWAI_ALERT_NONE,  .clkout_divider = 0,        \
                                                                    .intr_flags = ESP_INTR_FLAG_LEVEL2};

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();//TWAI_TIMING_CONFIG_500KBITS
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// https://github.com/espressif/esp-idf/issues/7955
//g_config.intr_flags = ESP_INTR_FLAG_LEVEL2; //ESP_INTR_FLAG_LEVEL1 is the default but conflicts 

// #define ID_MASTER_PING          0x0A2
// static const twai_message_t ping_message = {.identifier = ID_MASTER_PING, .data_length_code = 0,
//                                            .ss = 1, .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};

//ESP-IDF driver provides three main functions which you can call from your app: twai_receive, twai_transmit and twai_read_alerts. When you call twai_receive or twai_read_alerts, the execution of the task is suspended until a message or an alert becomes available. Once the message or an alert arrives, the execution of the task is resumed. Please take a look at the examples in this section of the docs: https://docs.espressif.com/projects/esp ... -reception

static inline void handleTxInterrupt(uint8_t phys_index)
{
    const int8_t iface_index = 0;//can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    uint64_t precise_time = AP_HAL::micros64();
    if (precise_time > 0) {
        precise_time--;
    }
    if (can_ifaces[iface_index] != nullptr) {
        can_ifaces[iface_index]->handleTxInterrupt(precise_time);
    }
}

static inline void handleRxInterrupt(uint8_t phys_index, uint8_t fifo_index)
{
    const int8_t iface_index = 0;//can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    uint64_t precise_time = AP_HAL::micros64();
    if (precise_time > 0) {
        precise_time--;
    }
    if (can_ifaces[iface_index] != nullptr) {
        can_ifaces[iface_index]->handleRxInterrupt(fifo_index, precise_time);
    }
}

const uint32_t CANIface::TSR_ABRQx[CANIface::NumTxMailboxes] = {// unused
};

CANIface::CANIface(uint8_t index) :
    self_index_(index),
    rx_bytebuffer_((uint8_t*)rx_buffer, sizeof(rx_buffer)),
    rx_queue_(&rx_bytebuffer_)
{
    if (index > (uint8_t)HAL_NUM_CAN_IFACES) {
        AP_HAL::panic("Bad CANIface index.");
    } else {
       // can_ = Can[index];

    }
    printf("CANIface");

}

// constructor suitable for array
CANIface::CANIface() :
    CANIface(next_interface++)
{}

bool CANIface::computeTimings(uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return false;
    }


    /*
     * Hardware configuration
     */
    const uint32_t pclk = 100000;

    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    static const int MaxSamplePointLocation = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    const uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false;              // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find the values so that the sample point is as close as possible to the optimal value.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    struct BsPair {
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair() :
            bs1(0),
            bs2(0),
            sample_point_permill(0)
        { }

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1) :
            bs1(arg_bs1),
            bs2(uint8_t(bs1_bs2_sum - bs1)),
            sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {}

        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation) {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid()) {
        return false;
    }

    printf("Timings: quanta/bit: %d, sample point location: %.1f%%",
          int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

    out_timings.prescaler = uint16_t(prescaler - 1U);
    out_timings.sjw = 0;                                        // Which means one
    out_timings.bs1 = uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uint8_t(solution.bs2 - 1);
    return true;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{


    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -1;
    }

    //esp32:
    twai_message_t message;
    message.identifier = frame.id;//buff->id;
    message.extd = frame.isExtended() ? 1 : 0;
    message.data_length_code = frame.dlc;
    //std::
    memcpy(message.data, frame.data, 8);
    //
    esp_err_t sts = twai_transmit(&message, portMAX_DELAY);
    ESP_ERROR_CHECK(sts);
    if (sts == ESP_OK) {
        printf("s"); // s for send 
        return 1;
    }
    printf("CAN send fail\n");
    return -1;

}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{

   #define MAX_RECV_MSGS_PER_SEC 200

   // we don't use a CanRxItem like chibios does, we go from IDF's twai_message_t type to AP_HAL::CANFrame type
 
   //esp32:
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#message-reception
    // https://github.com/espressif/esp-idf/blob/release/v4.4/components/driver/twai.c

    //Wait for message to be received - blocks thread for x 'ticks' - 100ms too much? todo recieve in thread.
    twai_message_t message;
    esp_err_t recverr = twai_receive(&message, pdMS_TO_TICKS(1000/MAX_RECV_MSGS_PER_SEC));
    if ( recverr == ESP_OK) {
        printf(".");
    } else if ( recverr == ESP_ERR_TIMEOUT) {
        // no msg on timout, as we have short timeout and retry a lot.
    } else { 
        //ESP_ERR_INVALID_STATE or ESP_ERR_INVALID_ARG
        //printf("Failed to receive message\n");// will get here if received message contains no data bytes, timed-out, or invalid driver state.
        printf("x");
        return -1;
    }

    // take out of IDF struct, and put into HAL struct.
    memcpy(out_frame.data, message.data, 8);// copy new data
    out_frame.dlc = message.data_length_code;
    out_frame.id = message.identifier;
    out_frame.id = out_frame.id | CANARD_CAN_FRAME_EFF;
    // we don't pas CAN eror frames to libcanard, as it jsut rebuffs them anywway with CANARD_ERROR_RX_INCOMPATIBLE_PACKET*
    if (out_frame.id & AP_HAL::CANFrame::FlagERR) { // same as a message.isErrorFrame() if done later.
        return -1;
    }
    out_timestamp_us = AP_HAL::native_micros64(); // arrival time.

    return 1;// comment this out to see or not, the below print statements and data bytes etc

    //Process received message
    if (message.extd) {
        printf("Message is in Extended Format\n");
    } else {
        printf("Message is in Standard Format\n");
    }
    printf("ID is %ld\n", message.identifier);
    if (!(message.rtr)) {
        for (int i = 0; i < message.data_length_code; i++) {
            printf("Data byte %d = %d\n", i, message.data[i]);
        }
    }
    //esp32
    //printf("receive done\n");
    return 1;
}

#if !defined(HAL_BOOTLOADER_BUILD)
bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
    //todo not impl on esp32
    return false;
}
#endif

bool CANIface::waitMsrINakBitStateChange(bool target_state)// unused
{
    return false;
}

void CANIface::handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, const uint64_t timestamp_us) // unused
{
}

void CANIface::handleTxInterrupt(const uint64_t utc_usec)
{


// #if CH_CFG_USE_EVENTS == TRUE
//     if (event_handle_ != nullptr) {
//         PERF_STATS(stats.num_events);
//         evt_src_.signalI(1 << self_index_);
//     }
// #endif
//     pollErrorFlagsFromISR();
}

void CANIface::handleRxInterrupt(uint8_t fifo_index, uint64_t timestamp_us)
{


//     /*
//      * Read the frame contents
//      */
//     AP_HAL::CANFrame &frame = isr_rx_frame;
//     // const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

//     // if ((rf.RIR & bxcan::RIR_IDE) == 0) {
//     //     frame.id = AP_HAL::CANFrame::MaskStdID & (rf.RIR >> 21);
//     // } else {
//     //     frame.id = AP_HAL::CANFrame::MaskExtID & (rf.RIR >> 3);
//     //     frame.id |= AP_HAL::CANFrame::FlagEFF;
//     // }

//     // if ((rf.RIR & bxcan::RIR_RTR) != 0) {
//     //     frame.id |= AP_HAL::CANFrame::FlagRTR;
//     // }

//     // frame.dlc = rf.RDTR & 15;

//     // frame.data[0] = uint8_t(0xFF & (rf.RDLR >> 0));
//     // frame.data[1] = uint8_t(0xFF & (rf.RDLR >> 8));
//     // frame.data[2] = uint8_t(0xFF & (rf.RDLR >> 16));
//     // frame.data[3] = uint8_t(0xFF & (rf.RDLR >> 24));
//     // frame.data[4] = uint8_t(0xFF & (rf.RDHR >> 0));
//     // frame.data[5] = uint8_t(0xFF & (rf.RDHR >> 8));
//     // frame.data[6] = uint8_t(0xFF & (rf.RDHR >> 16));
//     // frame.data[7] = uint8_t(0xFF & (rf.RDHR >> 24));

//     /*
//      * Store with timeout into the FIFO buffer and signal update event
//      */
//     CanRxItem &rx_item = isr_rx_item;
//     rx_item.frame = frame;
//     rx_item.timestamp_us = timestamp_us;
//     rx_item.flags = 0;
//     if (rx_queue_.push(rx_item)) {
//        // PERF_STATS(stats.rx_received);
//     } else {
//        // PERF_STATS(stats.rx_overflow);
//     }

//     had_activity_ = true;

// #if CH_CFG_USE_EVENTS == TRUE
//     if (event_handle_ != nullptr) {
//         PERF_STATS(stats.num_events);
//         evt_src_.signalI(1 << self_index_);
//     }
// #endif
//     pollErrorFlagsFromISR();
}

void CANIface::pollErrorFlagsFromISR()
{
//     const uint8_t lec = uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
//     if (lec != 0) {
// #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
//         stats.esr = can_->ESR; // Record error status
// #endif
//         can_->ESR = 0;

//         // Serving abort requests
//         for (int i = 0; i < NumTxMailboxes; i++) {
//             CanTxItem& txi = pending_tx_[i];
//             if (txi.aborted && txi.abort_on_error) {
//                 can_->TSR = TSR_ABRQx[i];
//                 txi.aborted = true;
//                // PERF_STATS(stats.tx_abort);
//             }
//         }
//     }
}

void CANIface::discardTimedOutTxMailboxes(uint64_t current_time) // unused
{

}

void CANIface::clear_rx()
{
    CriticalSectionLocker lock;
    rx_queue_.clear();
}

void CANIface::pollErrorFlags()
{
    // CriticalSectionLocker cs_locker;
    // pollErrorFlagsFromISR();
}

bool CANIface::canAcceptNewTxFrame(const AP_HAL::CANFrame& frame) const
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        // static const uint32_t TME = bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2;
        // const uint32_t tme = can_->TSR & TME;

        // if (tme == TME) {   // All TX mailboxes are free (as in freedom).
        //     return true;
        // }

        // if (tme == 0) {     // All TX mailboxes are busy transmitting.
       //     return false;
      //  }
      return true; // till we are smarter, always accept.
    }

    /*
     * The second condition requires a critical section.
     */
    // CriticalSectionLocker lock;

    // for (int mbx = 0; mbx < NumTxMailboxes; mbx++) {
    //     if (!(pending_tx_[mbx].pushed || pending_tx_[mbx].aborted) && !frame.priorityHigherThan(pending_tx_[mbx].frame)) {
    //         return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
    //     }
    // }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
}

bool CANIface::isRxBufferEmpty() const
{
    // CriticalSectionLocker lock;
    // return rx_queue_.available() == 0;

    #include "driver/twai.h"

    // //Reconfigure alerts to detect rx-related stuff only...
    // uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;
    // if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    //     printf("Alerts reconfigured\n");
    // } else {
    //     printf("Failed to reconfigure alerts");
    // }

    //to Block indefinitely until an alert occurs portMAX_DELAY, otherwisse non-blocking use tiny value like 1
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, 1);

  //because we only alert on RX data, and non-zero value meansthere's RX available 
    if ( alerts_triggered > 0) return false;

    return true;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
uint32_t CANIface::getErrorCount() const
{
    CriticalSectionLocker lock;
    return stats.num_busoff_err +
           stats.rx_errors +
           stats.rx_overflow +
           stats.tx_rejected +
           stats.tx_abort +
           stats.tx_timedout;
}


#endif // #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)

// #if CH_CFG_USE_EVENTS == TRUE
// ChibiOS::EventSource CANIface::evt_src_;
// bool CANIface::set_event_handle(AP_HAL::EventHandle* handle)
// {
//     CriticalSectionLocker lock;
//     event_handle_ = handle;
//     event_handle_->set_source(&evt_src_);
//     return event_handle_->register_event(1 << self_index_);
// }

// #endif // #if CH_CFG_USE_EVENTS == TRUE

void CANIface::checkAvailable(bool& read, bool& write, const AP_HAL::CANFrame* pending_tx) const
{
    write = false;
    read = !isRxBufferEmpty();

    //if (pending_tx != nullptr) {
        write = canAcceptNewTxFrame(*pending_tx);
    //}
}

bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame* pending_tx,
                      uint64_t blocking_deadline)
{
    const bool in_read = read;
    const bool in_write= write;
   //uint64_t time = AP_HAL::micros64();

    if (!read && !write) {
        //invalid request
        return false;
    }

    //pollErrorFlags();

    checkAvailable(read, write, pending_tx);          // Check if we already have some of the requested events
    if ((read && in_read) || (write && in_write)) {
        return true;
    }

// #if CH_CFG_USE_EVENTS == TRUE
//     // we don't support blocking select in AP_Periph and bootloader
//     while (time < blocking_deadline) {
//         if (event_handle_ == nullptr) {
//             break;
//         }
//         event_handle_->wait(blocking_deadline - time); // Block until timeout expires or any iface updates
//         checkAvailable(read, write, pending_tx);  // Check what we got
//         if ((read && in_read) || (write && in_write)) {
//             return true;
//         }
//         time = AP_HAL::micros64();
//     }
// #endif // #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    return true;
}

void CANIface::initOnce(bool enable_irq)
{


    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("CAN/TWAI Driver installed\n");
    }
    else
    {
        printf("Failed to install CAN/TWAI driver\n");
        return;
    }

    //Reconfigure alerts to detect rx-related stuff only...
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        printf("CAN/TWAI Alerts reconfigured\n");
    } else {
        printf("Failed to reconfigure CAN/TWAI alerts");
    }
    
     //Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        printf("CAN/TWAI Driver started\n");
    }
    else
    {
        printf("Failed to start CAN/TWAI driver\n");
        return;
    }

    hal.console->printf("Ardu CAN Driver installed\n");



    /*
     * CAN1, CAN2
     */
//     {
//         CriticalSectionLocker lock;
//         switch (can_interfaces[self_index_]) {
//         case 0:
// // #if defined(RCC_APB1ENR1_CAN1EN)
// //             RCC->APB1ENR1 |=  RCC_APB1ENR1_CAN1EN;
// //             RCC->APB1RSTR1 |=  RCC_APB1RSTR1_CAN1RST;
// //             RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN1RST;
// // #else
// //             RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
// //             RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
// //             RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
// // #endif
//             break;
// //#ifdef RCC_APB1ENR_CAN2EN
//         case 1:
//             // RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
//             // RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
//             // RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
//             break;
// //#endif
// //#ifdef RCC_APB1ENR_CAN3EN
//         case 2:
//             // RCC->APB1ENR  |=  RCC_APB1ENR_CAN3EN;
//             // RCC->APB1RSTR |=  RCC_APB1RSTR_CAN3RST;
//             // RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN3RST;
//             break;
// //#endif
//         }
//     }

    /*
     * IRQ
     */
//     if (!irq_init_ && enable_irq) {
//         CriticalSectionLocker lock;
//         switch (can_interfaces[self_index_]) {
//         case 0:
// // #ifdef HAL_CAN_IFACE1_ENABLE
// //             nvicEnableVector(CAN1_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
// //             nvicEnableVector(CAN1_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
// //             nvicEnableVector(CAN1_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
// // #endif
//             break;
//         case 1:
// // #ifdef HAL_CAN_IFACE2_ENABLE
// //             nvicEnableVector(CAN2_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
// //             nvicEnableVector(CAN2_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
// //             nvicEnableVector(CAN2_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
// // #endif
//             break;
//         case 2:
// // #ifdef HAL_CAN_IFACE3_ENABLE
// //             nvicEnableVector(CAN3_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
// //             nvicEnableVector(CAN3_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
// //             nvicEnableVector(CAN3_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
// // #endif
//             break;
//         }

       // irq_init_ = true;
   // }
}

bool CANIface::init(const uint32_t bitrate, const CANIface::OperatingMode mode)
{
    printf("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));
    if (self_index_ > HAL_NUM_CAN_IFACES) {
        printf("CAN drv init failed");
        return false;
    }
    if (can_ifaces[self_index_] == nullptr) {
        can_ifaces[self_index_] = this;
#if !defined(HAL_BOOTLOADER_BUILD)
        hal.can[self_index_] = this;
#endif
    }

    bitrate_ = bitrate;
    mode_ = mode;

    if (can_ifaces[0] == nullptr) {
        can_ifaces[0] = new CANIface(0);
        printf("Failed to allocate CAN iface 0");
        if (can_ifaces[0] == nullptr) {
            return false;
        }
    }
    if (self_index_ == 1 && !can_ifaces[0]->is_initialized()) {
        printf("Iface 0 is not initialized yet but we need it for Iface 1, trying to init it");
        printf("Enabling CAN iface 0");
        can_ifaces[0]->initOnce(false);
        printf("Initing iface 0...");
        if (!can_ifaces[0]->init(bitrate, mode)) {
            printf("Iface 0 init failed");
            return false;
        }

        printf("Enabling CAN iface");
    }
    initOnce(true);
    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        CriticalSectionLocker lock;

        // can_->MCR &= ~bxcan::MCR_SLEEP; // Exit sleep mode
        // can_->MCR |= bxcan::MCR_INRQ;   // Request init

        //can_->IER = 0;                  // Disable interrupts while initialization is in progress
    }


    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.clear();

    // for (uint32_t i=0; i < NumTxMailboxes; i++) {
    //     pending_tx_[i] = CanTxItem();
    // }
    had_activity_ = false;

    /*
     * CAN timings for this bitrate
     */
    Timings timings;
    if (!computeTimings(bitrate, timings)) {
        //can_->MCR = bxcan::MCR_RESET;
        return false;
    }
    printf("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
          unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    /*
     * Hardware initialization (the hardware has already confirmed initialization mode, see above)
     */
    // can_->MCR = bxcan::MCR_ABOM | bxcan::MCR_AWUM | bxcan::MCR_INRQ;  // RM page 648

    // can_->BTR = ((timings.sjw & 3U)  << 24) |
    //             ((timings.bs1 & 15U) << 16) |
    //             ((timings.bs2 & 7U)  << 20) |
    //             (timings.prescaler & 1023U) |
    //             ((mode == SilentMode) ? bxcan::BTR_SILM : 0);

    // can_->IER = bxcan::IER_TMEIE |   // TX mailbox empty
    //             bxcan::IER_FMPIE0 |  // RX FIFO 0 is not empty
    //             bxcan::IER_FMPIE1;   // RX FIFO 1 is not empty

    // can_->MCR &= ~bxcan::MCR_INRQ;   // Leave init mode


    /*
     * Default filter configuration
     */
//     if (self_index_ == 0) {
//         can_->FMR |= bxcan::FMR_FINIT;

//         can_->FMR &= 0xFFFFC0F1;
//         can_->FMR |= static_cast<uint32_t>(NumFilters) << 8;  // Slave (CAN2) gets half of the filters

//         can_->FFA1R = 0;                           // All assigned to FIFO0 by default
//         can_->FM1R = 0;                            // Indentifier Mask mode

// #if HAL_NUM_CAN_IFACES > 1
//         can_->FS1R = 0x7ffffff;                    // Single 32-bit for all
//         can_->FilterRegister[0].FR1 = 0;          // CAN1 accepts everything
//         can_->FilterRegister[0].FR2 = 0;
//         can_->FilterRegister[NumFilters].FR1 = 0; // CAN2 accepts everything
//         can_->FilterRegister[NumFilters].FR2 = 0;
//         can_->FA1R = 1 | (1 << NumFilters);        // One filter per each iface
// #else
//         can_->FS1R = 0x1fff;
//         can_->FilterRegister[0].FR1 = 0;
//         can_->FilterRegister[0].FR2 = 0;
//         can_->FA1R = 1;
// #endif

//         can_->FMR &= ~bxcan::FMR_FINIT;
//     }
    initialised_ = true;

    return true;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
void CANIface::get_stats(ExpandingString &str)
{
    CriticalSectionLocker lock;
    str.printf("tx_requests:    %lu\n"
               "tx_rejected:    %lu\n"
               "tx_success:     %lu\n"
               "tx_timedout:    %lu\n"
               "tx_abort:       %lu\n"
               "rx_received:    %lu\n"
               "rx_overflow:    %lu\n"
               "rx_errors:      %lu\n"
               "num_busoff_err: %lu\n"
               "num_events:     %lu\n"
               "ESR:            %lx\n",
               (long unsigned int)stats.tx_requests,
               (long unsigned int)stats.tx_rejected,
               (long unsigned int)stats.tx_success,
               (long unsigned int)stats.tx_timedout,
               (long unsigned int)stats.tx_abort,
               (long unsigned int)stats.rx_received,
               (long unsigned int)stats.rx_overflow,
               (long unsigned int)stats.rx_errors,
               (long unsigned int)stats.num_busoff_err,
               (long unsigned int)stats.num_events,
               (long unsigned int)stats.esr);
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{
#ifdef HAL_CAN_IFACE1_ENABLE
    // CAN1
    // CH_IRQ_HANDLER(CAN1_TX_IRQ_Handler);
    // CH_IRQ_HANDLER(CAN1_TX_IRQ_Handler)
    // {
    //     CH_IRQ_PROLOGUE();
    //     handleTxInterrupt(0);
    //     CH_IRQ_EPILOGUE();
    // }

    // CH_IRQ_HANDLER(CAN1_RX0_IRQ_Handler);
    // CH_IRQ_HANDLER(CAN1_RX0_IRQ_Handler)
    // {
    //     CH_IRQ_PROLOGUE();
    //     handleRxInterrupt(0, 0);
    //     CH_IRQ_EPILOGUE();
    // }

    // CH_IRQ_HANDLER(CAN1_RX1_IRQ_Handler);
    // CH_IRQ_HANDLER(CAN1_RX1_IRQ_Handler)
    // {
    //     CH_IRQ_PROLOGUE();
    //     handleRxInterrupt(0, 1);
    //     CH_IRQ_EPILOGUE();
    // }
#endif

// #ifdef HAL_CAN_IFACE2_ENABLE
//     // CAN2
//     CH_IRQ_HANDLER(CAN2_TX_IRQ_Handler);
//     CH_IRQ_HANDLER(CAN2_TX_IRQ_Handler)
//     {
//         CH_IRQ_PROLOGUE();
//         handleTxInterrupt(1);
//         CH_IRQ_EPILOGUE();
//     }

//     CH_IRQ_HANDLER(CAN2_RX0_IRQ_Handler);
//     CH_IRQ_HANDLER(CAN2_RX0_IRQ_Handler)
//     {
//         CH_IRQ_PROLOGUE();
//         handleRxInterrupt(1, 0);
//         CH_IRQ_EPILOGUE();
//     }

//     CH_IRQ_HANDLER(CAN2_RX1_IRQ_Handler);
//     CH_IRQ_HANDLER(CAN2_RX1_IRQ_Handler)
//     {
//         CH_IRQ_PROLOGUE();
//         handleRxInterrupt(1, 1);
//         CH_IRQ_EPILOGUE();
//     }
// #endif

// #ifdef HAL_CAN_IFACE3_ENABLE
//     // CAN3
//     CH_IRQ_HANDLER(CAN3_TX_IRQ_Handler);
//     CH_IRQ_HANDLER(CAN3_TX_IRQ_Handler)
//     {
//         CH_IRQ_PROLOGUE();
//         handleTxInterrupt(2);
//         CH_IRQ_EPILOGUE();
//     }

//     CH_IRQ_HANDLER(CAN3_RX0_IRQ_Handler);
//     CH_IRQ_HANDLER(CAN3_RX0_IRQ_Handler)
//     {
//         CH_IRQ_PROLOGUE();
//         handleRxInterrupt(2, 0);
//         CH_IRQ_EPILOGUE();
//     }

//     CH_IRQ_HANDLER(CAN3_RX1_IRQ_Handler);
//     CH_IRQ_HANDLER(CAN3_RX1_IRQ_Handler)
//     {
//         CH_IRQ_PROLOGUE();
//         handleRxInterrupt(2, 1);
//         CH_IRQ_EPILOGUE();
//     }
// #endif
    
} // extern "C"


#endif //HAL_NUM_CAN_IFACES
