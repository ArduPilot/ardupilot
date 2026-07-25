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
 * @file    hal_serial_lld.c
 * @brief   Win32 simulator low level serial driver code.
 *
 * @addtogroup WIN32_SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief Serial driver 1 identifier.*/
#if USE_WIN32_SERIAL1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif
/** @brief Serial driver 2 identifier.*/
#if USE_WIN32_SERIAL2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
};

static u_long nb = 1;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void init(SerialDriver *sdp, uint16_t port) {
  struct sockaddr_in sad;
  struct protoent *prtp;

  if ((prtp = getprotobyname("tcp")) == NULL) {
    printf("%s: Error mapping protocol name to protocol number\n", sdp->com_name);
    goto abort;
  }

  sdp->com_listen = socket(PF_INET, SOCK_STREAM, prtp->p_proto);
  if (sdp->com_listen == INVALID_SOCKET) {
    printf("%s: Error creating simulator socket\n", sdp->com_name);
    goto abort;
  }

  if (ioctlsocket(sdp->com_listen, FIONBIO, &nb) != 0) {
    printf("%s: Unable to setup non blocking mode on socket\n", sdp->com_name);
    goto abort;
  }

  memset(&sad, 0, sizeof(sad));
  sad.sin_family = AF_INET;
  sad.sin_addr.s_addr = INADDR_ANY;
  sad.sin_port = htons(port);
  if (bind(sdp->com_listen, (struct sockaddr *)&sad, sizeof(sad))) {
    printf("%s: Error binding socket\n", sdp->com_name);
    goto abort;
  }

  if (listen(sdp->com_listen, 1) != 0) {
    printf("%s: Error listening socket\n", sdp->com_name);
    goto abort;
  }
  printf("Full Duplex Channel %s listening on port %d\n", sdp->com_name, port);
  return;

abort:
  if (sdp->com_listen != INVALID_SOCKET)
    closesocket(sdp->com_listen);
  WSACleanup();
  exit(1);
}

static bool connint(SerialDriver *sdp) {

  if (sdp->com_data == INVALID_SOCKET) {
    struct sockaddr addr;
    int addrlen = sizeof(addr);

    if ((sdp->com_data = accept(sdp->com_listen, &addr, &addrlen)) == INVALID_SOCKET)
      return false;

    if (ioctlsocket(sdp->com_data, FIONBIO, &nb) != 0) {
      printf("%s: Unable to setup non blocking mode on data socket\n", sdp->com_name);
      goto abort;
    }
    chSysLockFromISR();
    chnAddFlagsI(sdp, CHN_CONNECTED);
    chSysUnlockFromISR();
    return true;
  }
  return false;
abort:
  if (sdp->com_listen != INVALID_SOCKET)
    closesocket(sdp->com_listen);
  if (sdp->com_data != INVALID_SOCKET)
    closesocket(sdp->com_data);
  WSACleanup();
  exit(1);
}

static bool inint(SerialDriver *sdp) {

  if (sdp->com_data != INVALID_SOCKET) {
    int i;
    uint8_t data[32];
    size_t nfree;

    /*
     * Input.
     */
    chSysLockFromISR();
    nfree = iqGetEmptyI(&sdp->iqueue);
    chSysUnlockFromISR();
    if (nfree == 0U) {
      return false;
    }
    if (nfree > sizeof(data)) {
      nfree = sizeof(data);
    }

    int n = recv(sdp->com_data, (char *)data, nfree, 0);
    switch (n) {
    case 0:
      closesocket(sdp->com_data);
      sdp->com_data = INVALID_SOCKET;
      chSysLockFromISR();
      chnAddFlagsI(sdp, CHN_DISCONNECTED);
      chSysUnlockFromISR();
      return false;
    case SOCKET_ERROR:
      if (WSAGetLastError() == WSAEWOULDBLOCK)
        return false;
      closesocket(sdp->com_data);
      sdp->com_data = INVALID_SOCKET;
      return false;
    }
    for (i = 0; i < n; i++) {
      chSysLockFromISR();
      sdIncomingDataI(sdp, data[i]);
      chSysUnlockFromISR();
    }
    return true;
  }
  return false;
}

static bool outint(SerialDriver *sdp) {

  if (sdp->com_data != INVALID_SOCKET) {
    int n;
    uint8_t data[1];

    /*
     * Input.
     */
    chSysLockFromISR();
    n = sdRequestDataI(sdp);
    chSysUnlockFromISR();
    if (n < 0)
      return false;
    data[0] = (uint8_t)n;
    n = send(sdp->com_data, (char *)data, sizeof(data), 0);
    switch (n) {
    case 0:
      closesocket(sdp->com_data);
      sdp->com_data = INVALID_SOCKET;
      chSysLockFromISR();
      chnAddFlagsI(sdp, CHN_DISCONNECTED);
      chSysUnlockFromISR();
      return false;
    case SOCKET_ERROR:
      if (WSAGetLastError() == WSAEWOULDBLOCK)
        return false;
      closesocket(sdp->com_data);
      sdp->com_data = INVALID_SOCKET;
      return false;
    }
    return true;
  }
  return false;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * Low level serial driver initialization.
 */
void sd_lld_init(void) {

#if USE_WIN32_SERIAL1
  sdObjectInit(&SD1, NULL, NULL);
  SD1.com_listen = INVALID_SOCKET;
  SD1.com_data = INVALID_SOCKET;
  SD1.com_name = "SD1";
#endif

#if USE_WIN32_SERIAL2
  sdObjectInit(&SD2, NULL, NULL);
  SD2.com_listen = INVALID_SOCKET;
  SD2.com_data = INVALID_SOCKET;
  SD2.com_name = "SD2";
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

#if USE_WIN32_SERIAL1
  if (sdp == &SD1)
    init(&SD1, SD1_PORT);
#endif

#if USE_WIN32_SERIAL2
  if (sdp == &SD2)
    init(&SD2, SD2_PORT);
#endif
}

/**
 * @brief Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp pointer to a @p SerialDriver object
 */
void sd_lld_stop(SerialDriver *sdp) {

  (void)sdp;
}

bool sd_lld_interrupt_pending(void) {
  bool b = false;

  CH_IRQ_PROLOGUE();

#if USE_WIN32_SERIAL1
  b |= connint(&SD1) || inint(&SD1) || outint(&SD1);
#endif

#if USE_WIN32_SERIAL2
  b |= connint(&SD2) || inint(&SD2) || outint(&SD2);
#endif

  CH_IRQ_EPILOGUE();

  return b;
}

#endif /* HAL_USE_SERIAL */

/** @} */
