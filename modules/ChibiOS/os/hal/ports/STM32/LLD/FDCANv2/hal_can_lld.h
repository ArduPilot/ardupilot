/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    FDCANv2/hal_can_lld.h
 * @brief   STM32 CAN subsystem low level driver header.
 *
 * @addtogroup CAN
 * @{
 */

#ifndef HAL_CAN_LLD_H
#define HAL_CAN_LLD_H

#if HAL_USE_CAN || defined(__DOXYGEN__)

#include "stm32_fdcan.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Maximum number of bytes in data of CAN packets.
 */
#define CAN_MAX_DLC_BYTES           64

/**
 * @brief   Number of transmit mailboxes.
 */
#define CAN_TX_MAILBOXES            1

/**
 * @brief   Number of receive mailboxes.
 */
#define CAN_RX_MAILBOXES            2

/**
 * @brief   bytes to DLC (Data length code).
 */
#define FDCAN_0BYTES_TO_DLC         0U
#define FDCAN_1BYTES_TO_DLC         1U
#define FDCAN_2BYTES_TO_DLC         2U
#define FDCAN_3BYTES_TO_DLC         3U
#define FDCAN_4BYTES_TO_DLC         4U
#define FDCAN_5BYTES_TO_DLC         5U
#define FDCAN_6BYTES_TO_DLC         6U
#define FDCAN_7BYTES_TO_DLC         7U
#define FDCAN_8BYTES_TO_DLC         8U
#define FDCAN_12BYTES_TO_DLC        9U
#define FDCAN_16BYTES_TO_DLC        10U
#define FDCAN_20BYTES_TO_DLC        11U
#define FDCAN_24BYTES_TO_DLC        12U
#define FDCAN_32BYTES_TO_DLC        13U
#define FDCAN_48BYTES_TO_DLC        14U
#define FDCAN_64BYTES_TO_DLC        15U

/**
 * @brief   DLC (Data length code) to bytes.
 */
#define FDCAN_0DLC_TO_BYTES         0U
#define FDCAN_1DLC_TO_BYTES         1U
#define FDCAN_2DLC_TO_BYTES         2U
#define FDCAN_3DLC_TO_BYTES         3U
#define FDCAN_4DLC_TO_BYTES         4U
#define FDCAN_5DLC_TO_BYTES         5U
#define FDCAN_6DLC_TO_BYTES         6U
#define FDCAN_7DLC_TO_BYTES         7U
#define FDCAN_8DLC_TO_BYTES         8U
#define FDCAN_9DLC_TO_BYTES         12U
#define FDCAN_10DLC_TO_BYTES        16U
#define FDCAN_11DLC_TO_BYTES        20U
#define FDCAN_12DLC_TO_BYTES        24U
#define FDCAN_13DLC_TO_BYTES        32U
#define FDCAN_14DLC_TO_BYTES        48U
#define FDCAN_15DLC_TO_BYTES        64U

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   CAN1 driver enable switch.
 * @details If set to @p TRUE the support for FDCAN1 is included.
 */
#if !defined(STM32_CAN_USE_FDCAN1) || defined(__DOXYGEN__)
#define STM32_CAN_USE_FDCAN1                FALSE
#endif

/**
 * @brief   CAN2 driver enable switch.
 * @details If set to @p TRUE the support for FDCAN2 is included.
 */
#if !defined(STM32_CAN_USE_FDCAN2) || defined(__DOXYGEN__)
#define STM32_CAN_USE_FDCAN2                FALSE
#endif

/**
 * @brief   CAN3 driver enable switch.
 * @details If set to @p TRUE the support for FDCAN3 is included.
 */
#if !defined(STM32_CAN_USE_FDCAN3) || defined(__DOXYGEN__)
#define STM32_CAN_USE_FDCAN3                FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_FDCAN1)
#error "STM32_HAS_FDCAN1 not defined in registry"
#endif

#if !defined(STM32_HAS_FDCAN2)
#error "STM32_HAS_FDCAN2 not defined in registry"
#endif

#if !defined(STM32_HAS_FDCAN3)
#error "STM32_HAS_FDCAN3 not defined in registry"
#endif

#if STM32_CAN_USE_FDCAN1 && !STM32_HAS_FDCAN1
#error "FDCAN1 not present in the selected device"
#endif

#if STM32_CAN_USE_FDCAN2 && !STM32_HAS_FDCAN2
#error "FDCAN2 not present in the selected device"
#endif

#if STM32_CAN_USE_FDCAN3 && !STM32_HAS_FDCAN3
#error "FDCAN3 not present in the selected device"
#endif

#if !STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN3
#error "CAN driver activated but no FDCAN peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Supported modes for the peripheral FDCAN.
 */
typedef enum {
  OPMODE_FDCAN = 1,
  OPMODE_CAN = 2
} fdcanopmode_t;

/**
 * @brief   Convert DLC in BYTES.
 */
static const uint8_t dlc_to_bytes[] = {
  FDCAN_0DLC_TO_BYTES,
  FDCAN_1DLC_TO_BYTES,
  FDCAN_2DLC_TO_BYTES,
  FDCAN_3DLC_TO_BYTES,
  FDCAN_4DLC_TO_BYTES,
  FDCAN_5DLC_TO_BYTES,
  FDCAN_6DLC_TO_BYTES,
  FDCAN_7DLC_TO_BYTES,
  FDCAN_8DLC_TO_BYTES,
  FDCAN_9DLC_TO_BYTES,
  FDCAN_10DLC_TO_BYTES,
  FDCAN_11DLC_TO_BYTES,
  FDCAN_12DLC_TO_BYTES,
  FDCAN_13DLC_TO_BYTES,
  FDCAN_14DLC_TO_BYTES,
  FDCAN_15DLC_TO_BYTES
};

/**
 * @brief   Type of a CAN driver.
 */
typedef struct hal_can_driver CANDriver;

/**
 * @brief   Type of a transmission mailbox index.
 */
typedef uint32_t canmbx_t;

/**
 * @brief   Type of a word size.
 */
typedef uint8_t wordsize_t;

#if (CAN_ENFORCE_USE_CALLBACKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a CAN notification callback.
 *
 * @param[in] canp      pointer to the @p CANDriver object triggering the
 *                      callback
 * @param[in] flags     flags associated to the mailbox callback
 */
typedef void (*can_callback_t)(CANDriver *canp, uint32_t flags);
#endif

/**
 * @brief   CAN transmission frame.
 * @note    Accessing the frame data as word16 or word32 is not portable
 *          because machine data endianness, it can be still useful for a
 *          quick filling.
 */
typedef struct {
  /**
   * @brief   Frame header.
   */
  union {
    struct {
      union {
        struct {
          uint32_t          EID:29;     /**< @brief Extended identifier.    */
        } ext;
        struct {
          uint32_t          _R1:18;     /**< @brief Reserved for offset.    */
          uint32_t          SID:11;     /**< @brief Standard identifier.    */
        } std;
        struct {
          uint32_t          _R1:29;     /**< @brief Reserved for offset.    */
          uint32_t          RTR:1;      /**< @brief Remote transmit request.*/
          uint32_t          XTD:1;      /**< @brief Extended identifier.    */
          uint32_t          ESI:1;      /**< @brief Error state indicator.  */
        } common;
      };
      uint32_t              _R2:16;
      uint32_t              DLC:4;      /**< @brief Data length code.       */
      uint32_t              BPS:1;      /**< @brief Accepted non-matching
                                                    frame.                  */
      uint32_t              FDF:1;      /**< @brief FDCAN frame format.     */
      uint32_t              _R3:1;
      uint32_t              EFC:1;      /**< @brief Event FIFO control.     */
      uint32_t              MM:8;       /**< @brief Message event marker.   */
    };
    uint32_t                header32[2];
  };
  /**
   * @brief   Frame data.
   */
  union {
    uint8_t                 data8[CAN_MAX_DLC_BYTES];
    uint16_t                data16[CAN_MAX_DLC_BYTES / 2];
    uint32_t                data32[CAN_MAX_DLC_BYTES / 4];
  };
} CANTxFrame;

/**
 * @brief   CAN received frame.
 * @note    Accessing the frame data as word16 or word32 is not portable
 *          because machine data endianness, it can be still useful for a
 *          quick filling.
 */
typedef struct {
  /**
   * @brief   Frame header.
   */
  union {
    struct {
      union {
        struct {
          uint32_t          EID:29;     /**< @brief Extended identifier.    */
        } ext;
        struct {
          uint32_t          _R1:18;
          uint32_t          SID:11;     /**< @brief Standard identifier.    */
        } std;
        struct {
          uint32_t          _R1:29;     /**< @brief Reserved for offset.    */
          uint32_t          RTR:1;      /**< @brief Remote transmit request.*/
          uint32_t          XTD:1;      /**< @brief Extended identifier.    */
          uint32_t          ESI:1;      /**< @brief Error state indicator.  */
        } common;
      };
      uint32_t              RXTS:16;    /**< @brief TX time stamp.          */
      uint32_t              DLC:4;      /**< @brief Data length code.       */
      uint32_t              BRS:1;      /**< @brief Bit rate switch.        */
      uint32_t              FDF:1;      /**< @brief FDCAN frame format.     */
      uint32_t              _R2:2;
      uint32_t              FIDX:7;     /**< @brief Filter index.           */
      uint32_t              ANMF:1;     /**< @brief Accepted non-matching
                                                    frame.                  */
    };
    uint32_t                header32[2];
  };
  /**
   * @brief   Frame data.
   */
  union {
    uint8_t                 data8[CAN_MAX_DLC_BYTES];
    uint16_t                data16[CAN_MAX_DLC_BYTES / 2];
    uint32_t                data32[CAN_MAX_DLC_BYTES / 4];
  };
} CANRxFrame;

/**
 * @brief   CAN standard filter.
 * @note    Accessing the frame data as word16 or word32 is not portable
 *          because machine data endianness, it can be still useful for a
 *          quick filling.
 */
typedef struct {
  union {
    struct {
      uint16_t              SFID2:11;
      uint8_t               _R1:5;
      uint16_t              SFID1:11;
      uint8_t               SFEC:3;
      uint8_t               SFT:2;
    };
    union {
      uint32_t              data32;
      uint16_t              data16[2];
      uint8_t               data8[4];
    };
  };
} CANRxStandardFilter;

/**
 * @brief   CAN extended filter.
 * @note    Accessing the frame data as word16 or word32 is not portable
 *          because machine data endianness, it can be still useful for a
 *          quick filling.
*/
typedef struct {
  union {
    struct {
      uint32_t              EFID1:29;
      uint8_t               EFEC:3;
      uint32_t              EFID2:29;
      uint8_t               _R1:1;
      uint8_t               EFT:2;
    };
    union {
      uint32_t              data32[2];
      uint16_t              data16[4];
      uint8_t               data8[8];
    };
  };
} CANRxExtendedFilter;

/**
 * @brief   Filter type element.
 */
typedef enum {
  CAN_FILTER_TYPE_STD = 0x00,         /**< Standard filter */
  CAN_FILTER_TYPE_EXT = 0x01          /**< Extended filter */
} filter_type_t;

/**
 * @brief   Filter mode.
 */
typedef enum {
  CAN_FILTER_MODE_RANGE = 0x00,       /**< Range filter from SFID1 to SFID2 */
  CAN_FILTER_MODE_DUAL = 0x01,        /**< Dual ID filter for SFID1 or SFID2 */
  CAN_FILTER_MODE_CLASSIC = 0x02      /**< Classic filter: SFID1 = filter, SFID2 = mask */
} filter_mode_t;

/**
 * @brief   Filter configuration.
 */
typedef enum {
  CAN_FILTER_CFG_FIFO_0 = 0x01,       /**< Store in Rx FIFO 0 if filter matches */
  CAN_FILTER_CFG_FIFO_1 = 0x02,       /**< Store in Rx FIFO 1 if filter matches */
  CAN_FILTER_CFG_REJECT = 0x03        /**< Reject ID if filter matches in range FID1-FID2 */
} filter_cfg_t;

/**
 * @brief   CAN filter configuration.
 * @note    Refer to the STM32 reference manual for info about filters.
 */
typedef struct {
  /**
   * @brief   Filter type;
   * @note    Standard filter
   *          Extended filter
   */
  filter_type_t             filter_type;

  /**
   * @brief   Filter mode;
   * @note    Field SFT / EFT;
   * @note    Range filter from SFID1 to SFID2
   *          Dual ID filter for SFID1 or SFID2
   *          Classic filter: SFID1 = filter, SFID2 = mask
   */
  filter_mode_t             filter_mode;

  /**
   * @brief   Filter configuration;
   * @note    Field SFEC / EFEC;
   * @note    Store in Rx FIFO 0 if filter matches
   *          Store in Rx FIFO 1 if filter matches
   *          Reject ID if filter matches in range FID1-FID2
   */
  filter_cfg_t              filter_cfg;

  /**
   * @brief   Message address to filter.
   * @note    29-bit Extended identifier.
   *          11-bit Standard identifier.
   */
  uint32_t                  identifier1;

  /**
   * @brief   Message address to filter.
   * @note    29-bit Extended identifier.
   *          11-bit Standard identifier.
   */
  uint32_t                  identifier2;

} CANFilter;

/**
 * @brief   Type of a CAN configuration structure.
 */
typedef struct hal_can_config {
  /**
   * @brief Specifies the FDCAN operation mode.
   */
  fdcanopmode_t             op_mode;
  /**
   * @brief   Nominal bit timing and prescaler register.
   */
  uint32_t                  NBTP;
  /**
   * @brief   Data bit timing and prescaler register.
   */
  uint32_t                  DBTP;
  /**
   * @brief   Data delay compensation register.
   */
  uint32_t                  TDCR;
 /**
   * @brief   CC control register.
   */
  uint32_t                  CCCR;
  /**
   * @brief   Test configuration register.
   */
  uint32_t                  TEST;
  /**
   * @brief   Global filter configuration register.
   */
  uint32_t                  RXGFC;
} CANConfig;

/**
 * @brief   Structure representing an CAN driver.
 */
struct hal_can_driver {
  /**
   * @brief   Driver state.
   */
  canstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const CANConfig           *config;
  /**
   * @brief   Transmission threads queue.
   */
  threads_queue_t           txqueue;
  /**
   * @brief   Receive threads queue.
   */
  threads_queue_t           rxqueue;
#if (CAN_ENFORCE_USE_CALLBACKS == FALSE) || defined(__DOXYGEN__)
  /**
   * @brief   One or more frames become available.
   * @note    After broadcasting this event it will not be broadcasted again
   *          until the received frames queue has been completely emptied. It
   *          is <b>not</b> broadcasted for each received frame. It is
   *          responsibility of the application to empty the queue by
   *          repeatedly invoking @p canReceive() when listening to this event.
   *          This behavior minimizes the interrupt served by the system
   *          because CAN traffic.
   * @note    The flags associated to the listeners will indicate which
   *          receive mailboxes become non-empty.
   */
  event_source_t            rxfull_event;
  /**
   * @brief   One or more transmission mailbox become available.
   * @note    The flags associated to the listeners will indicate which
   *          transmit mailboxes become empty.
   * @note    The upper 16 bits are transmission error flags associated
   *          to the transmit mailboxes.
   */
  event_source_t            txempty_event;
  /**
   * @brief   A CAN bus error happened.
   * @note    The flags associated to the listeners will indicate that
   *          receive error(s) have occurred.
   * @note    In this implementation the upper 16 bits are filled with the
   *          unprocessed content of the ESR register.
   */
  event_source_t            error_event;
#if CAN_USE_SLEEP_MODE || defined (__DOXYGEN__)
  /**
   * @brief   Entering sleep state event.
   */
  event_source_t            sleep_event;
  /**
   * @brief   Exiting sleep state event.
   */
  event_source_t            wakeup_event;
#endif /* CAN_USE_SLEEP_MODE */
#else /* CAN_ENFORCE_USE_CALLBACKS == TRUE */
  /**
   * @brief   One or more frames become available.
   * @note    After calling this function it will not be called again
   *          until the received frames queue has been completely emptied. It
   *          is <b>not</b> called for each received frame. It is
   *          responsibility of the application to empty the queue by
   *          repeatedly invoking @p chTryReceiveI().
   *          This behavior minimizes the interrupt served by the system
   *          because CAN traffic.
   */
  can_callback_t            rxfull_cb;
  /**
   * @brief   One or more transmission mailbox become available.
   * @note    The flags associated to the callback will indicate which
   *          transmit mailboxes become empty.
   */
  can_callback_t            txempty_cb;
  /**
   * @brief   A CAN bus error happened.
   */
  can_callback_t            error_cb;
#if (CAN_USE_SLEEP_MODE == TRUE) || defined (__DOXYGEN__)
  /**
   * @brief   Exiting sleep state.
   */
  can_callback_t            wakeup_cb;
#endif
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Element size (RAM words).
   */
  wordsize_t                word_size;
  /**
   * @brief   Pointer to the CAN registers.
   */
  FDCAN_GlobalTypeDef       *fdcan;
  /**
   * @brief   Pointer to FDCAN RAM base.
   */
  uint32_t                  *ram_base;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_CAN_USE_FDCAN1 && !defined(__DOXYGEN__)
extern CANDriver CAND1;
#endif

#if STM32_CAN_USE_FDCAN2 && !defined(__DOXYGEN__)
extern CANDriver CAND2;
#endif

#if STM32_CAN_USE_FDCAN3 && !defined(__DOXYGEN__)
extern CANDriver CAND3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void can_lld_init(void);
  bool can_lld_start(CANDriver *canp);
  void can_lld_stop(CANDriver *canp);
  bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox);
  void can_lld_transmit(CANDriver *canp,
                        canmbx_t mailbox,
                        const CANTxFrame *crfp);
  bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox);
  void can_lld_receive(CANDriver *canp,
                       canmbx_t mailbox,
                       CANRxFrame *ctfp);
  void can_lld_abort(CANDriver *canp,
                     canmbx_t mailbox);
#if CAN_USE_SLEEP_MODE
  void can_lld_sleep(CANDriver *canp);
  void can_lld_wakeup(CANDriver *canp);
#endif /* CAN_USE_SLEEP_MODE */
  void can_lld_serve_interrupt(CANDriver *canp);
  void canSTM32SetFilters(CANDriver *canp, uint8_t num, const CANFilter *cfp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CAN */

#endif /* HAL_CAN_LLD_H */

/** @} */
