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
 * @file    CANv1/hal_can_lld.h
 * @brief   STM32 CAN subsystem low level driver header.
 *
 * @addtogroup CAN
 * @{
 */

#ifndef HAL_CAN_LLD_H
#define HAL_CAN_LLD_H

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * The following macros from the ST header file are replaced with better
 * equivalents.
 */
#undef CAN_BTR_BRP
#undef CAN_BTR_TS1
#undef CAN_BTR_TS2
#undef CAN_BTR_SJW

/**
 * @brief   This switch defines whether the driver implementation supports
 *          a low power switch mode with automatic an wakeup feature.
 */
#define CAN_SUPPORTS_SLEEP          TRUE

/**
 * @brief   This implementation supports three transmit mailboxes.
 */
#define CAN_TX_MAILBOXES            3

/**
 * @brief   This implementation supports two receive mailboxes.
 */
#define CAN_RX_MAILBOXES            2

/**
 * @name    CAN registers helper macros
 * @{
 */
#define CAN_BTR_BRP(n)              (n)         /**< @brief BRP field macro.*/
#define CAN_BTR_TS1(n)              ((n) << 16) /**< @brief TS1 field macro.*/
#define CAN_BTR_TS2(n)              ((n) << 20) /**< @brief TS2 field macro.*/
#define CAN_BTR_SJW(n)              ((n) << 24) /**< @brief SJW field macro.*/

#define CAN_IDE_STD                 0           /**< @brief Standard id.    */
#define CAN_IDE_EXT                 1           /**< @brief Extended id.    */

#define CAN_RTR_DATA                0           /**< @brief Data frame.     */
#define CAN_RTR_REMOTE              1           /**< @brief Remote frame.   */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   CAN pedantic errors report.
 * @details Use of this option is IRQ-intensive.
 */
#if !defined(STM32_CAN_REPORT_ALL_ERRORS) || defined(__DOXYGEN__)
#define STM32_CAN_REPORT_ALL_ERRORS         FALSE
#endif

/**
 * @brief   CAN1 driver enable switch.
 * @details If set to @p TRUE the support for CAN1 is included.
 */
#if !defined(STM32_CAN_USE_CAN1) || defined(__DOXYGEN__)
#define STM32_CAN_USE_CAN1                  FALSE
#endif

/**
 * @brief   CAN2 driver enable switch.
 * @details If set to @p TRUE the support for CAN2 is included.
 */
#if !defined(STM32_CAN_USE_CAN2) || defined(__DOXYGEN__)
#define STM32_CAN_USE_CAN2                  FALSE
#endif

/**
 * @brief   CAN3 driver enable switch.
 * @details If set to @p TRUE the support for CAN3 is included.
 */
#if !defined(STM32_CAN_USE_CAN3) || defined(__DOXYGEN__)
#define STM32_CAN_USE_CAN3                  FALSE
#endif

/**
 * @brief   CAN1 interrupt priority level setting.
 */
#if !defined(STM32_CAN_CAN1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_CAN_CAN1_IRQ_PRIORITY         11
#endif

/**
 * @brief   CAN2 interrupt priority level setting.
 */
#if !defined(STM32_CAN_CAN2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_CAN_CAN2_IRQ_PRIORITY         11
#endif

/**
 * @brief   CAN3 interrupt priority level setting.
 */
#if !defined(STM32_CAN_CAN3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_CAN_CAN3_IRQ_PRIORITY         11
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_CAN1)
#error "STM32_HAS_CAN1 not defined in registry"
#endif

#if !defined(STM32_HAS_CAN2)
#error "STM32_HAS_CAN2 not defined in registry"
#endif

#if !defined(STM32_HAS_CAN3)
#error "STM32_HAS_CAN3 not defined in registry"
#endif

#if (STM32_HAS_CAN1 | STM32_HAS_CAN2) && !defined(STM32_CAN_MAX_FILTERS)
#error "STM32_CAN_MAX_FILTERS not defined in registry"
#endif

#if STM32_HAS_CAN3 && !defined(STM32_CAN3_MAX_FILTERS)
#error "STM32_CAN3_MAX_FILTERS not defined in registry"
#endif

#if STM32_CAN_USE_CAN1 && !STM32_HAS_CAN1
#error "CAN1 not present in the selected device"
#endif

#if STM32_CAN_USE_CAN2 && !STM32_HAS_CAN2
#error "CAN2 not present in the selected device"
#endif

#if STM32_CAN_USE_CAN3 && !STM32_HAS_CAN3
#error "CAN2 not present in the selected device"
#endif

#if !STM32_CAN_USE_CAN1 && !STM32_CAN_USE_CAN2 && !STM32_CAN_USE_CAN3
#error "CAN driver activated but no CAN peripheral assigned"
#endif

#if !STM32_CAN_USE_CAN1 && STM32_CAN_USE_CAN2
#error "CAN2 requires CAN1, it cannot operate independently"
#endif

#if CAN_USE_SLEEP_MODE && !CAN_SUPPORTS_SLEEP
#error "CAN sleep mode not supported in this architecture"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a CAN driver.
 */
typedef struct hal_can_driver CANDriver;

/**
 * @brief   Type of a transmission mailbox index.
 */
typedef uint32_t canmbx_t;

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
 * @note    Accessing the frame data as word16 or word32 is not portable because
 *          machine data endianness, it can be still useful for a quick filling.
 */
typedef struct {
  struct {
    uint8_t                 DLC:4;          /**< @brief Data length.        */
    uint8_t                 RTR:1;          /**< @brief Frame type.         */
    uint8_t                 IDE:1;          /**< @brief Identifier type.    */
  };
  union {
    struct {
      uint32_t              SID:11;         /**< @brief Standard identifier.*/
    };
    struct {
      uint32_t              EID:29;         /**< @brief Extended identifier.*/
    };
  };
  union {
    uint8_t                 data8[8];       /**< @brief Frame data.         */
    uint16_t                data16[4];      /**< @brief Frame data.         */
    uint32_t                data32[2];      /**< @brief Frame data.         */
    uint64_t                data64[1];      /**< @brief Frame data.         */
  };
} CANTxFrame;

/**
 * @brief   CAN received frame.
 * @note    Accessing the frame data as word16 or word32 is not portable because
 *          machine data endianness, it can be still useful for a quick filling.
 */
typedef struct {
  struct {
    uint8_t                 FMI;            /**< @brief Filter id.          */
    uint16_t                TIME;           /**< @brief Time stamp.         */
  };
  struct {
    uint8_t                 DLC:4;          /**< @brief Data length.        */
    uint8_t                 RTR:1;          /**< @brief Frame type.         */
    uint8_t                 IDE:1;          /**< @brief Identifier type.    */
  };
  union {
    struct {
      uint32_t              SID:11;         /**< @brief Standard identifier.*/
    };
    struct {
      uint32_t              EID:29;         /**< @brief Extended identifier.*/
    };
  };
  union {
    uint8_t                 data8[8];       /**< @brief Frame data.         */
    uint16_t                data16[4];      /**< @brief Frame data.         */
    uint32_t                data32[2];      /**< @brief Frame data.         */
    uint64_t                data64[1];      /**< @brief Frame data.         */
  };
} CANRxFrame;

/**
 * @brief   CAN filter.
 * @note    Refer to the STM32 reference manual for info about filters.
 */
typedef struct {
  /**
   * @brief   Number of the filter bank to be programmed.
   */
  uint32_t                  filter:16;
  /**
   * @brief   Filter mode.
   * @note    This bit represent the CAN_FM1R register bit associated to this
   *          filter (0=mask mode, 1=list mode).
   */
  uint32_t                  mode:1;
  /**
   * @brief   Filter scale.
   * @note    This bit represent the CAN_FS1R register bit associated to this
   *          filter (0=16 bits mode, 1=32 bits mode).
   */
  uint32_t                  scale:1;
  /**
   * @brief   Filter mode.
   * @note    This bit represent the CAN_FFA1R register bit associated to this
   *          filter, must be set to zero in this version of the driver.
   */
  uint32_t                  assignment:1;
  /**
   * @brief   Filter register 1 (identifier).
   */
  uint32_t                  register1;
  /**
   * @brief   Filter register 2 (mask/identifier depending on mode=0/1).
   */
  uint32_t                  register2;
} CANFilter;

/**
 * @brief   Type of a CAN configuration structure.
 */
typedef struct hal_can_config {
  /**
   * @brief   CAN MCR register initialization data.
   * @note    Some bits in this register are enforced by the driver regardless
   *          their status in this field.
   */
  uint32_t                  mcr;
  /**
   * @brief   CAN BTR register initialization data.
   * @note    Some bits in this register are enforced by the driver regardless
   *          their status in this field.
   */
  uint32_t                  btr;
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
   * @brief   Pointer to the CAN registers.
   */
  CAN_TypeDef               *can;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_CAN_USE_CAN1 && !defined(__DOXYGEN__)
extern CANDriver CAND1;
#endif

#if STM32_CAN_USE_CAN2 && !defined(__DOXYGEN__)
extern CANDriver CAND2;
#endif

#if STM32_CAN_USE_CAN3 && !defined(__DOXYGEN__)
extern CANDriver CAND3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void can_lld_init(void);
  void can_lld_start(CANDriver *canp);
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
  void canSTM32SetFilters(CANDriver *canp, uint32_t can2sb,
                          uint32_t num, const CANFilter *cfp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CAN */

#endif /* HAL_CAN_LLD_H */

/** @} */
