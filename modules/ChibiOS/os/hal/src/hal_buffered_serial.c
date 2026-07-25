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
 * @file    hal_buffered_serial.c
 * @brief   Buffered Serial Driver code.
 *
 * @addtogroup HAL_BUFFERED_SERIAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Handles incoming data.
 * @details This function must be called from the input interrupt service
 *          routine in order to enqueue incoming data and generate the
 *          related events.
 * @note    The incoming data event is only generated when the input queue
 *          becomes non-empty.
 *
 * @param[in] bsp       pointer to a @p BufferedSerial structure
 * @param[in] b         the byte to be written in the driver's Input Queue
 *
 * @iclass
 */
void bsIncomingDataI(BufferedSerial *bsp, uint8_t b) {

  osalDbgCheckClassI();
  osalDbgCheck(bsp != NULL);

  if (iqIsEmptyI(&bsp->iqueue)) {
    chnAddFlagsI(bsp, CHN_INPUT_AVAILABLE);
  }

  if (iqPutI(&bsp->iqueue, b) < MSG_OK) {
    chnAddFlagsI(bsp, CHN_BUFFER_FULL_ERROR);
  }
}

/**
 * @brief   Handles outgoing data.
 * @details Must be called from the output interrupt service routine in order
 *          to get the next byte to be transmitted.
 * @note    In order to gain some performance it is suggested to not use
 *          this function directly but copy this code directly into the
 *          interrupt service routine.
 *
 * @param[in] bsp       pointer to a @p BufferedSerial structure
 * @return              The byte value read from the driver's output queue.
 * @retval MSG_TIMEOUT  if the queue is empty.
 *
 * @iclass
 */
msg_t bsRequestDataI(BufferedSerial *bsp) {
  msg_t  b;

  osalDbgCheckClassI();
  osalDbgCheck(bsp != NULL);

  b = oqGetI(&bsp->oqueue);
  if (b < MSG_OK) {
    chnAddFlagsI(bsp, CHN_OUTPUT_EMPTY);
  }

  return b;
}

/** @} */
