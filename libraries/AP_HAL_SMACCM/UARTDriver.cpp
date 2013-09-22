/*
 * UARTDriver.cpp --- AP_HAL_SMACCM UART driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include "UARTDriver.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include <stdio.h>              // for vsnprintf

using namespace SMACCM;

// XXX the AVR driver enables itself in the constructor.  This seems
// like a very bad idea, since it will run somewhere in the startup
// code before our clocks are all set up and such.
SMACCMUARTDriver::SMACCMUARTDriver(struct usart *dev)
  : m_dev(dev), m_initialized(false), m_blocking(true)
{
}

void SMACCMUARTDriver::begin(uint32_t baud)
{
  if (m_dev != NULL) {
    usart_init(m_dev, baud);
    usart_enable(m_dev);
  }

  m_initialized = true;
}

// XXX buffer sizes ignored
void SMACCMUARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
  begin(baud);
}

// XXX hwf4 doesn't support de-initializing a USART
void SMACCMUARTDriver::end()
{
}

// XXX hwf4 doesn't support flushing, could be tricky to get the
// synchronization right.  Would we just force the TX/RX queues to
// empty?
void SMACCMUARTDriver::flush()
{
}

bool SMACCMUARTDriver::is_initialized()
{
  return m_initialized;
}

void SMACCMUARTDriver::set_blocking_writes(bool blocking)
{
  m_blocking = blocking;
}

bool SMACCMUARTDriver::tx_pending()
{
  if (m_dev != NULL) {
    return usart_is_tx_pending(m_dev);
  }

  return false;
}

/* SMACCM implementations of Stream virtual methods */
int16_t SMACCMUARTDriver::available()
{
  if (m_dev != NULL)
    return (int16_t)usart_available(m_dev);

  return 0;
}

int16_t SMACCMUARTDriver::txspace()
{
  if (m_dev != NULL)
    return (int16_t)usart_txspace(m_dev);

  return 0;
}

// It looks like this should always be a non-blocking read, so return
// -1 if there is nothing to receive immediately.
int16_t SMACCMUARTDriver::read()
{
  uint8_t c;

  if (m_dev == NULL)
    return -1;

  if (usart_read_timeout(m_dev, 0, &c, 1) == 0)
    return -1;

  return (int16_t)c;
}

/* SMACCM implementations of Print virtual methods */
size_t SMACCMUARTDriver::write(uint8_t c)
{
  if (m_dev == NULL)
    return 1;

  portTickType delay = m_blocking ? portMAX_DELAY : 0;
  return usart_write_timeout(m_dev, delay, &c, 1);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
