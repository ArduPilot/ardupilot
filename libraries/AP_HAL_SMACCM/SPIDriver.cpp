/*
 * SPIDriver.cpp --- AP_HAL_SMACCM SPI driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include "SPIDriver.h"
#include <FreeRTOS.h>
#include <hwf4/spi.h>

using namespace SMACCM;

//////////////////////////////////////////////////////////////////////
// SPI Device Driver

SMACCMSPIDeviceDriver::SMACCMSPIDeviceDriver(spi_bus *bus, spi_device *device)
  : _bus(bus), _device(device)
{
}

void SMACCMSPIDeviceDriver::init()
{
  _semaphore.init();
}

AP_HAL::Semaphore* SMACCMSPIDeviceDriver::get_semaphore()
{
  return &_semaphore;
}

void SMACCMSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  spi_transfer(_bus, _device, portMAX_DELAY, tx, rx, len);
}

// XXX these methods are not implemented
void SMACCMSPIDeviceDriver::cs_assert()
{
}

void SMACCMSPIDeviceDriver::cs_release()
{
}

uint8_t SMACCMSPIDeviceDriver::transfer (uint8_t data)
{
  return 0;
}

//////////////////////////////////////////////////////////////////////
// SPI Device Instances

// SPIDevice_Dataflash (I2C device in PX4FMU)

// SPIDevice_ADS7844 (not present in PX4FMU)

// SPIDevice_MS5611 (I2C device in PX4FMU)

// SPIDevice_MPU6000 (on SPI1)
static spi_device g_mpu6000_spi_dev = {
  pin_b0,                       // chip_select
  false,                        // chip_select_active
  SPI_BAUD_DIV_128,             // baud XXX check frequency
  SPI_CLOCK_POLARITY_LOW,       // clock_polarity
  SPI_CLOCK_PHASE_1,            // clock_phase
  SPI_BIT_ORDER_MSB_FIRST       // bit_order
};

static SMACCMSPIDeviceDriver g_mpu6000_dev(spi1, &g_mpu6000_spi_dev);

// SPIDevice_ADNS3080_SPI0 (not present in PX4FMU)

// SPIDevice_ADNS3080_SPI3 (not present in PX4FMU)

//////////////////////////////////////////////////////////////////////
// SPI Device Manager

SMACCMSPIDeviceManager::SMACCMSPIDeviceManager()
{
}

// Initialize all SPI busses and devices.
void SMACCMSPIDeviceManager::init(void *)
{
  spi_init(spi1);

  spi_device_init(&g_mpu6000_spi_dev);
  g_mpu6000_dev.init();
}

AP_HAL::SPIDeviceDriver* SMACCMSPIDeviceManager::device(AP_HAL::SPIDevice dev)
{
  switch (dev) {
    case AP_HAL::SPIDevice_MPU6000:
      return &g_mpu6000_dev;

    default:
      return NULL;
  }
}
