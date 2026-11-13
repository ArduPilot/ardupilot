/**
 ******************************************************************************
 * @file    vl53l3x_class.cpp
 * @author  IMG
 * @version V0.0.1
 * @date    14-December-2018
 * @brief   Implementation file for the VL53LX driver class
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/

/* Includes */
#include <stdlib.h>
#include "vl53lx_class.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wshadow"
#if !defined(__clang__)
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/* Write and read functions from I2C */

VL53LX_Error VL53LX::VL53LX_WriteMulti(VL53LX_DEV dev_handle, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int  status;

  status = VL53LX_I2CWrite(dev_handle->I2cDevAddr, index, pdata, (uint16_t)count);
  return status;
}

VL53LX_Error VL53LX::VL53LX_ReadMulti(VL53LX_DEV dev_handle, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int status;

  status = VL53LX_I2CRead(dev_handle->I2cDevAddr, index, pdata, (uint16_t)count);

  return status;
}


VL53LX_Error VL53LX::VL53LX_WrByte(VL53LX_DEV dev_handle, uint16_t index, uint8_t data)
{
  int  status;

  status = VL53LX_I2CWrite(dev_handle->I2cDevAddr, index, &data, 1);
  return status;
}

VL53LX_Error VL53LX::VL53LX_WrWord(VL53LX_DEV dev_handle, uint16_t index, uint16_t data)
{
  int  status;
  uint8_t buffer[2];

  buffer[0] = data >> 8;
  buffer[1] = data & 0x00FF;
  status = VL53LX_I2CWrite(dev_handle->I2cDevAddr, index, (uint8_t *)buffer, 2);
  return status;
}

VL53LX_Error VL53LX::VL53LX_WrDWord(VL53LX_DEV dev_handle, uint16_t index, uint32_t data)
{
  int  status;
  uint8_t buffer[4];

  buffer[0] = (data >> 24) & 0xFF;
  buffer[1] = (data >> 16) & 0xFF;
  buffer[2] = (data >>  8) & 0xFF;
  buffer[3] = (data >>  0) & 0xFF;
  status = VL53LX_I2CWrite(dev_handle->I2cDevAddr, index, (uint8_t *)buffer, 4);
  return status;
}


VL53LX_Error VL53LX::VL53LX_RdByte(VL53LX_DEV dev_handle, uint16_t index, uint8_t *data)
{
  int  status;

  status = VL53LX_I2CRead(dev_handle->I2cDevAddr, index, data, 1);

  if (status) {
    return -1;
  }

  return 0;
}

VL53LX_Error VL53LX::VL53LX_RdWord(VL53LX_DEV dev_handle, uint16_t index, uint16_t *data)
{
  int  status;
  uint8_t buffer[2] = {0, 0};

  status = VL53LX_I2CRead(dev_handle->I2cDevAddr, index, buffer, 2);
  if (!status) {
    *data = (buffer[0] << 8) + buffer[1];
  }
  return status;

}

VL53LX_Error VL53LX::VL53LX_RdDWord(VL53LX_DEV dev_handle, uint16_t index, uint32_t *data)
{
  int status;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = VL53LX_I2CRead(dev_handle->I2cDevAddr, index, buffer, 4);
  if (!status) {
    *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
  }
  return status;

}

VL53LX_Error VL53LX::VL53LX_UpdateByte(VL53LX_DEV dev_handle, uint16_t index, uint8_t AndData, uint8_t OrData)
{
  int  status;
  uint8_t buffer = 0;

  /* read data direct onto buffer */
  status = VL53LX_I2CRead(dev_handle->I2cDevAddr, index, &buffer, 1);
  if (!status) {
    buffer = (buffer & AndData) | OrData;
    status = VL53LX_I2CWrite(dev_handle->I2cDevAddr, index, &buffer, (uint16_t)1);
  }
  return status;
}

VL53LX_Error VL53LX::VL53LX_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
  if (dev_i2c == nullptr || NumByteToWrite > VL53LX_MAX_I2C_XFER_SIZE) {
    return VL53LX_ERROR_CONTROL_INTERFACE;
  }

  dev_i2c->set_address((DeviceAddr >> 1) & 0x7F);

  uint8_t buffer[VL53LX_MAX_I2C_XFER_SIZE + 2];
  buffer[0] = (uint8_t)(RegisterAddr >> 8);
  buffer[1] = (uint8_t)(RegisterAddr & 0xFF);
  if (NumByteToWrite > 0) {
    memcpy(&buffer[2], pBuffer, NumByteToWrite);
  }

  return dev_i2c->transfer(buffer, NumByteToWrite + 2, nullptr, 0) ?
         VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX::VL53LX_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToRead)
{
  if (dev_i2c == nullptr || NumByteToRead > VL53LX_MAX_I2C_XFER_SIZE) {
    return VL53LX_ERROR_CONTROL_INTERFACE;
  }

  dev_i2c->set_address((DeviceAddr >> 1) & 0x7F);

  const uint8_t buffer[2] {
    (uint8_t)(RegisterAddr >> 8),
    (uint8_t)(RegisterAddr & 0xFF)
  };

  return dev_i2c->transfer(buffer, sizeof(buffer), pBuffer, NumByteToRead) ?
         VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}


VL53LX_Error VL53LX::VL53LX_GetTickCount(
  uint32_t *ptick_count_ms)
{

  /* Returns current tick count in [ms] */

  VL53LX_Error status  = VL53LX_ERROR_NONE;

  *ptick_count_ms = AP_HAL::millis();

  return status;
}



VL53LX_Error VL53LX::VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us)
{
  (void)pdev;
  if (wait_us > 0) {
    hal.scheduler->delay_microseconds(wait_us);
  }
  return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX::VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms)
{
  (void)pdev;
  if (wait_ms > 0) {
    hal.scheduler->delay(wait_ms);
  }
  return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX::VL53LX_WaitValueMaskEx(
  VL53LX_Dev_t *pdev,
  uint32_t      timeout_ms,
  uint16_t      index,
  uint8_t       value,
  uint8_t       mask,
  uint32_t      poll_delay_ms)
{

  /*
   * Platform implementation of WaitValueMaskEx V2WReg script command
   *
   * WaitValueMaskEx(
   *          duration_ms,
   *          index,
   *          value,
   *          mask,
   *          poll_delay_ms);
   */

  VL53LX_Error status         = VL53LX_ERROR_NONE;
  uint32_t     start_time_ms = 0;
  uint32_t     current_time_ms = 0;
  uint32_t     polling_time_ms = 0;
  uint8_t      byte_value      = 0;
  uint8_t      found           = 0;



  /* calculate time limit in absolute time */

  VL53LX_GetTickCount(&start_time_ms);

  /* remember current trace functions and temporarily disable
   * function logging
   */


  /* wait until value is found, timeout reached on error occurred */

  while ((status == VL53LX_ERROR_NONE) &&
         (polling_time_ms < timeout_ms) &&
         (found == 0)) {

    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_RdByte(
                 pdev,
                 index,
                 &byte_value);

    if ((byte_value & mask) == value) {
      found = 1;
    }

    if (status == VL53LX_ERROR_NONE  &&
        found == 0 &&
        poll_delay_ms > 0)
      status = VL53LX_WaitMs(
                 pdev,
                 poll_delay_ms);

    /* Update polling time (Compare difference rather than absolute to
    negate 32bit wrap around issue) */
    VL53LX_GetTickCount(&current_time_ms);
    polling_time_ms = current_time_ms - start_time_ms;

  }


  if (found == 0 && status == VL53LX_ERROR_NONE) {
    status = VL53LX_ERROR_TIME_OUT;
  }

  return status;
}



/* vl53lx_api_core.c */
VL53LX_Error VL53LX::select_offset_per_vcsel(VL53LX_LLDriverData_t *pdev, int16_t *poffset)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  int16_t tA, tB;
  uint8_t isc;

  switch (pdev->preset_mode) {
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:
      tA = pdev->per_vcsel_cal_data.short_a_offset_mm;
      tB = pdev->per_vcsel_cal_data.short_b_offset_mm;
      break;
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
      tA = pdev->per_vcsel_cal_data.medium_a_offset_mm;
      tB = pdev->per_vcsel_cal_data.medium_b_offset_mm;
      break;
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:
      tA = pdev->per_vcsel_cal_data.long_a_offset_mm;
      tB = pdev->per_vcsel_cal_data.long_b_offset_mm;
      break;
    default:
      tA = pdev->per_vcsel_cal_data.long_a_offset_mm;
      tB = pdev->per_vcsel_cal_data.long_b_offset_mm;
      status = VL53LX_ERROR_INVALID_PARAMS;
      *poffset = 0;
      break;
  }

  isc = pdev->ll_state.cfg_internal_stream_count;
  if (status == VL53LX_ERROR_NONE) {
    *poffset = (isc & 0x01) ? tA : tB;
  }

  return status;
}


void VL53LX::vl53lx_diff_histo_stddev(VL53LX_LLDriverData_t *pdev, VL53LX_histogram_bin_data_t *pdata, uint8_t timing, uint8_t HighIndex, uint8_t prev_pos, int32_t *pdiff_histo_stddev)
{
  uint16_t   bin                      = 0;
  int32_t    total_rate_pre = 0;
  int32_t    total_rate_cur = 0;
  int32_t    PrevBin, CurrBin;

  total_rate_pre = 0;
  total_rate_cur = 0;


  for (bin = timing * 4; bin < HighIndex; bin++) {
    total_rate_pre +=
      pdev->multi_bins_rec[prev_pos][timing][bin];
    total_rate_cur += pdata->bin_data[bin];
  }

  if ((total_rate_pre != 0) && (total_rate_cur != 0))
    for (bin = timing * 4; bin < HighIndex; bin++) {
      PrevBin = pdev->multi_bins_rec[prev_pos][timing][bin];
      PrevBin = (PrevBin * 1000) / total_rate_pre;
      CurrBin = pdata->bin_data[bin] * 1000 / total_rate_cur;
      *pdiff_histo_stddev += (PrevBin - CurrBin) *
                             (PrevBin - CurrBin);
    }
}


void VL53LX::vl53lx_histo_merge(VL53LX_histogram_bin_data_t *pdata)
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  uint16_t   bin                      = 0;
  uint8_t    i                        = 0;
  int32_t    TuningBinRecSize       = 0;
  uint8_t    recom_been_reset     = 0;
  uint8_t    timing         = 0;
  int32_t    rmt  = 0;
  int32_t    diff_histo_stddev    = 0;
  uint8_t    HighIndex, prev_pos;
  uint8_t    BuffSize = VL53LX_HISTOGRAM_BUFFER_SIZE;
  uint8_t    pos;

  VL53LX_get_tuning_parm(VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
                         &TuningBinRecSize);

  VL53LX_get_tuning_parm(VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD,
                         &rmt);


  if (pdev->pos_before_next_recom == 0) {

    timing = 1 - pdata->result__stream_count % 2;

    diff_histo_stddev = 0;
    HighIndex = BuffSize - timing * 4;
    if (pdev->bin_rec_pos > 0) {
      prev_pos = pdev->bin_rec_pos - 1;
    } else {
      prev_pos = (TuningBinRecSize - 1);
    }

    if (pdev->multi_bins_rec[prev_pos][timing][4] > 0)
      vl53lx_diff_histo_stddev(pdev, pdata,
                               timing, HighIndex, prev_pos,
                               &diff_histo_stddev);

    if (diff_histo_stddev >= rmt) {
      memset(pdev->multi_bins_rec, 0,
             sizeof(pdev->multi_bins_rec));
      pdev->bin_rec_pos = 0;

      recom_been_reset = 1;

      if (timing == 0)
        pdev->pos_before_next_recom =
          VL53LX_FRAME_WAIT_EVENT;
      else
        pdev->pos_before_next_recom =
          VL53LX_FRAME_WAIT_EVENT + 1;
    } else {

      pos = pdev->bin_rec_pos;
      for (i = 0; i < BuffSize; i++)
        pdev->multi_bins_rec[pos][timing][i] =
          pdata->bin_data[i];
    }

    if (pdev->bin_rec_pos == (TuningBinRecSize - 1) && timing == 1) {
      pdev->bin_rec_pos = 0;
    } else if (timing == 1) {
      pdev->bin_rec_pos++;
    }

    if (!((recom_been_reset == 1) && (timing == 0)) &&
        (pdev->pos_before_next_recom == 0)) {

      for (bin = 0; bin < BuffSize; bin++) {
        pdata->bin_data[bin] = 0;
      }

      for (bin = 0; bin < BuffSize; bin++)
        for (i = 0; i < TuningBinRecSize; i++)
          pdata->bin_data[bin] +=
            (pdev->multi_bins_rec[i][timing][bin]);
    }
  } else {

    pdev->pos_before_next_recom--;
    if (pdev->pos_before_next_recom == 255) {
      pdev->pos_before_next_recom = 0;
    }
  }
}

VL53LX_Error VL53LX::VL53LX_load_patch()
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  int32_t patch_tuning = 0;
  uint8_t comms_buffer[256];
  uint32_t patch_power;

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(Dev,
                           VL53LX_FIRMWARE__ENABLE, 0x00);

  if (status == VL53LX_ERROR_NONE) {
    VL53LX_enable_powerforce();
  }

  VL53LX_get_tuning_parm(VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER,
                         &patch_tuning);

  switch (patch_tuning) {
    case 0:
      patch_power = 0x00;
      break;
    case 1:
      patch_power = 0x10;
      break;
    case 2:
      patch_power = 0x20;
      break;
    case 3:
      patch_power = 0x40;
      break;
    default:
      patch_power = 0x00;
  }

  if (status == VL53LX_ERROR_NONE) {

    comms_buffer[0] = 0x29;
    comms_buffer[1] = 0xC9;
    comms_buffer[2] = 0x0E;
    comms_buffer[3] = 0x40;
    comms_buffer[4] = 0x28;
    comms_buffer[5] = patch_power;

    status = VL53LX_WriteMulti(Dev,
                               VL53LX_PATCH__OFFSET_0, comms_buffer, 6);
  }

  if (status == VL53LX_ERROR_NONE) {
    comms_buffer[0] = 0x03;
    comms_buffer[1] = 0x6D;
    comms_buffer[2] = 0x03;
    comms_buffer[3] = 0x6F;
    comms_buffer[4] = 0x07;
    comms_buffer[5] = 0x29;
    status = VL53LX_WriteMulti(Dev,
                               VL53LX_PATCH__ADDRESS_0, comms_buffer, 6);
  }

  if (status == VL53LX_ERROR_NONE) {
    comms_buffer[0] = 0x00;
    comms_buffer[1] = 0x07;
    status = VL53LX_WriteMulti(Dev, VL53LX_PATCH__JMP_ENABLES, comms_buffer, 2);
  }

  if (status == VL53LX_ERROR_NONE) {
    comms_buffer[0] = 0x00;
    comms_buffer[1] = 0x07;
    status = VL53LX_WriteMulti(Dev,
                               VL53LX_PATCH__DATA_ENABLES, comms_buffer, 2);
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(Dev,
                           VL53LX_PATCH__CTRL, 0x01);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(Dev,
                           VL53LX_FIRMWARE__ENABLE, 0x01);


  return status;
}

VL53LX_Error VL53LX:: VL53LX_unload_patch()
{
  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_WrByte(Dev, VL53LX_FIRMWARE__ENABLE, 0x00);
  }

  if (status == VL53LX_ERROR_NONE) {
    VL53LX_disable_powerforce();
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_WrByte(Dev, VL53LX_PATCH__CTRL, 0x00);
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_WrByte(Dev, VL53LX_FIRMWARE__ENABLE, 0x01);
  }



  return status;
}

VL53LX_Error VL53LX::VL53LX_get_version(VL53LX_ll_version_t *pdata)
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_init_version();

  memcpy(pdata, &(pdev->version), sizeof(VL53LX_ll_version_t));

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX::VL53LX_get_device_firmware_version(uint16_t         *pfw_version)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_RdWord(
               Dev,
               VL53LX_MCU_GENERAL_PURPOSE__GP_0,
               pfw_version);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}



VL53LX_Error VL53LX::VL53LX_data_init(uint8_t           read_p2p_data)
{
  VL53LX_Error status       = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t    *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);



  VL53LX_zone_objects_t    *pobjects;

  uint8_t  i = 0;

  VL53LX_init_ll_driver_state(VL53LX_DEVICESTATE_UNKNOWN);

  pres->range_results.max_results    = VL53LX_MAX_RANGE_RESULTS;
  pres->range_results.active_results = 0;
  pres->zone_results.max_zones       = VL53LX_MAX_USER_ZONES;
  pres->zone_results.active_zones    = 0;

  for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
    pobjects = &(pres->zone_results.VL53LX_p_003[i]);
    pobjects->xmonitor.VL53LX_p_016 = 0;
    pobjects->xmonitor.VL53LX_p_017  = 0;
    pobjects->xmonitor.VL53LX_p_011          = 0;
    pobjects->xmonitor.range_status =
      VL53LX_DEVICEERROR_NOUPDATE;
  }



  pres->zone_hists.max_zones         = VL53LX_MAX_USER_ZONES;
  pres->zone_hists.active_zones      = 0;



  pres->zone_cal.max_zones           = VL53LX_MAX_USER_ZONES;
  pres->zone_cal.active_zones        = 0;
  for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
    pres->zone_cal.VL53LX_p_003[i].no_of_samples   = 0;
    pres->zone_cal.VL53LX_p_003[i].effective_spads = 0;
    pres->zone_cal.VL53LX_p_003[i].peak_rate_mcps  = 0;
    pres->zone_cal.VL53LX_p_003[i].median_range_mm = 0;
    pres->zone_cal.VL53LX_p_003[i].range_mm_offset = 0;
  }

  pdev->wait_method             = VL53LX_WAIT_METHOD_BLOCKING;
  pdev->preset_mode   = VL53LX_DEVICEPRESETMODE_STANDARD_RANGING;
  pdev->zone_preset             = VL53LX_DEVICEZONEPRESET_NONE;
  pdev->measurement_mode        = VL53LX_DEVICEMEASUREMENTMODE_STOP;

  pdev->offset_calibration_mode =
    VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD;
  pdev->offset_correction_mode  =
    VL53LX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
  pdev->dmax_mode  =
    VL53LX_DEVICEDMAXMODE__FMT_CAL_DATA;

  pdev->phasecal_config_timeout_us  =  1000;
  pdev->mm_config_timeout_us        =  2000;
  pdev->range_config_timeout_us     = 13000;
  pdev->inter_measurement_period_ms =   100;
  pdev->dss_config__target_total_rate_mcps = 0x0A00;
  pdev->debug_mode                  =  0x00;

  pdev->offset_results.max_results    = VL53LX_MAX_OFFSET_RANGE_RESULTS;
  pdev->offset_results.active_results = 0;



  pdev->gain_cal.standard_ranging_gain_factor =
    VL53LX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT;
  pdev->gain_cal.histogram_ranging_gain_factor =
    VL53LX_TUNINGPARM_HIST_GAIN_FACTOR_DEFAULT;


  VL53LX_init_version();


  memset(pdev->multi_bins_rec, 0, sizeof(pdev->multi_bins_rec));
  pdev->bin_rec_pos = 0;
  pdev->pos_before_next_recom = 0;



  if (read_p2p_data > 0 && status == VL53LX_ERROR_NONE) {
    status = VL53LX_read_p2p_data();
  }


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_refspadchar_config_struct(
               &(pdev->refspadchar));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_ssc_config_struct(
               &(pdev->ssc_cfg));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_xtalk_config_struct(
               &(pdev->customer),
               &(pdev->xtalk_cfg));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_xtalk_extract_config_struct(
               &(pdev->xtalk_extract_cfg));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_offset_cal_config_struct(
               &(pdev->offsetcal_cfg));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_zone_cal_config_struct(
               &(pdev->zonecal_cfg));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_hist_post_process_config_struct(
               pdev->xtalk_cfg.global_crosstalk_compensation_enable,
               &(pdev->histpostprocess));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_hist_gen3_dmax_config_struct(
               &(pdev->dmax_cfg));


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_tuning_parm_storage_struct(
               &(pdev->tuning_parms));



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_set_preset_mode(
               pdev->preset_mode,
               pdev->dss_config__target_total_rate_mcps,
               pdev->phasecal_config_timeout_us,
               pdev->mm_config_timeout_us,
               pdev->range_config_timeout_us,
               pdev->inter_measurement_period_ms);


  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(pdev->hist_data));

  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(pdev->hist_xtalk));


  VL53LX_init_xtalk_bin_data_struct(
    0,
    VL53LX_XTALK_HISTO_BINS,
    &(pdev->xtalk_shapes.xtalk_shape));



  VL53LX_xtalk_cal_data_init();



  VL53LX_dynamic_xtalk_correction_data_init();



  VL53LX_low_power_auto_data_init();
  /*
  #ifdef VL53LX_LOG_ENABLE



    VL53LX_print_static_nvm_managed(
      &(pdev->stat_nvm),
      "data_init():pdev->lldata.stat_nvm.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_customer_nvm_managed(
      &(pdev->customer),
      "data_init():pdev->lldata.customer.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_nvm_copy_data(
      &(pdev->nvm_copy_data),
      "data_init():pdev->lldata.nvm_copy_data.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_dmax_calibration_data(
      &(pdev->fmt_dmax_cal),
      "data_init():pdev->lldata.fmt_dmax_cal.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_dmax_calibration_data(
      &(pdev->cust_dmax_cal),
      "data_init():pdev->lldata.cust_dmax_cal.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_additional_offset_cal_data(
      &(pdev->add_off_cal_data),
      "data_init():pdev->lldata.add_off_cal_data.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_user_zone(
      &(pdev->mm_roi),
      "data_init():pdev->lldata.mm_roi.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_optical_centre(
      &(pdev->optical_centre),
      "data_init():pdev->lldata.optical_centre.",
      VL53LX_TRACE_MODULE_DATA_INIT);

    VL53LX_print_cal_peak_rate_map(
      &(pdev->cal_peak_rate_map),
      "data_init():pdev->lldata.cal_peak_rate_map.",
      VL53LX_TRACE_MODULE_DATA_INIT);

  #endif

    LOG_FUNCTION_END(status);
  */
  return status;
}

VL53LX_Error VL53LX::VL53LX_read_p2p_data()
{
  VL53LX_Error status       = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);
  VL53LX_additional_offset_cal_data_t *pCD = &(pdev->add_off_cal_data);

  VL53LX_decoded_nvm_fmt_range_data_t fmt_rrd;

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_get_static_nvm_managed(&(pdev->stat_nvm));
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_get_customer_nvm_managed(&(pdev->customer));
  }

  if (status == VL53LX_ERROR_NONE) {

    status = VL53LX_get_nvm_copy_data(&(pdev->nvm_copy_data));


    if (status == VL53LX_ERROR_NONE)
      VL53LX_copy_rtn_good_spads_to_buffer(
        &(pdev->nvm_copy_data),
        &(pdev->rtn_good_spads[0]));
  }



  if (status == VL53LX_ERROR_NONE) {
    pHP->algo__crosstalk_compensation_plane_offset_kcps =
      pN->algo__crosstalk_compensation_plane_offset_kcps;
    pHP->algo__crosstalk_compensation_x_plane_gradient_kcps =
      pN->algo__crosstalk_compensation_x_plane_gradient_kcps;
    pHP->algo__crosstalk_compensation_y_plane_gradient_kcps =
      pN->algo__crosstalk_compensation_y_plane_gradient_kcps;
  }


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_read_nvm_optical_centre(&(pdev->optical_centre));



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_read_nvm_cal_peak_rate_map(&(pdev->cal_peak_rate_map));



  if (status == VL53LX_ERROR_NONE) {

    status =
      VL53LX_read_nvm_additional_offset_cal_data(&(pdev->add_off_cal_data));



    if (pCD->result__mm_inner_peak_signal_count_rtn_mcps == 0 &&
        pCD->result__mm_outer_peak_signal_count_rtn_mcps == 0) {

      pCD->result__mm_inner_peak_signal_count_rtn_mcps
        = 0x0080;
      pCD->result__mm_outer_peak_signal_count_rtn_mcps
        = 0x0180;



      VL53LX_calc_mm_effective_spads(
        pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
        pdev->nvm_copy_data.roi_config__mode_roi_xy_size,
        0xC7,
        0xFF,
        &(pdev->rtn_good_spads[0]),
        VL53LX_RTN_SPAD_APERTURE_TRANSMISSION,
        &(pCD->result__mm_inner_actual_effective_spads),
        &(pCD->result__mm_outer_actual_effective_spads));
    }
  }


  if (status == VL53LX_ERROR_NONE) {

    status =
      VL53LX_read_nvm_fmt_range_results_data(VL53LX_NVM__FMT__RANGE_RESULTS__140MM_DARK,
                                             &fmt_rrd);

    if (status == VL53LX_ERROR_NONE) {
      pdev->fmt_dmax_cal.ref__actual_effective_spads =
        fmt_rrd.result__actual_effective_rtn_spads;
      pdev->fmt_dmax_cal.ref__peak_signal_count_rate_mcps =
        fmt_rrd.result__peak_signal_count_rate_rtn_mcps;
      pdev->fmt_dmax_cal.ref__distance_mm =
        fmt_rrd.measured_distance_mm;


      if (pdev->cal_peak_rate_map.cal_reflectance_pc != 0) {
        pdev->fmt_dmax_cal.ref_reflectance_pc =
          pdev->cal_peak_rate_map.cal_reflectance_pc;
      } else {
        pdev->fmt_dmax_cal.ref_reflectance_pc = 0x0014;
      }


      pdev->fmt_dmax_cal.coverglass_transmission = 0x0100;
    }
  }


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_RdWord(
        Dev,
        VL53LX_RESULT__OSC_CALIBRATE_VAL,
        &(pdev->dbg_results.result__osc_calibrate_val));



  if (pdev->stat_nvm.osc_measured__fast_osc__frequency < 0x1000) {

    pdev->stat_nvm.osc_measured__fast_osc__frequency = 0xBCCC;
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_get_mode_mitigation_roi(&(pdev->mm_roi));



  if (pdev->optical_centre.x_centre == 0 &&
      pdev->optical_centre.y_centre == 0) {
    pdev->optical_centre.x_centre =
      pdev->mm_roi.x_centre << 4;
    pdev->optical_centre.y_centre =
      pdev->mm_roi.y_centre << 4;
  }


  return status;
}



VL53LX_Error VL53LX::VL53LX_software_reset()
{
  VL53LX_Error status       = VL53LX_ERROR_NONE;

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_SOFT_RESET,
               0x00);


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WaitUs(
        Dev,
        VL53LX_SOFTWARE_RESET_DURATION_US);


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_SOFT_RESET,
               0x01);


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_wait_for_boot_completion();
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_part_to_part_data(VL53LX_calibration_data_t            *pcal_data)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

  uint32_t tempu32;


  if (pcal_data->struct_version !=
      VL53LX_LL_CALIBRATION_DATA_STRUCT_VERSION) {
    status = VL53LX_ERROR_INVALID_PARAMS;
  }

  if (status == VL53LX_ERROR_NONE) {


    memcpy(
      &(pdev->customer),
      &(pcal_data->customer),
      sizeof(VL53LX_customer_nvm_managed_t));


    memcpy(
      &(pdev->add_off_cal_data),
      &(pcal_data->add_off_cal_data),
      sizeof(VL53LX_additional_offset_cal_data_t));


    memcpy(
      &(pdev->fmt_dmax_cal),
      &(pcal_data->fmt_dmax_cal),
      sizeof(VL53LX_dmax_calibration_data_t));


    memcpy(
      &(pdev->cust_dmax_cal),
      &(pcal_data->cust_dmax_cal),
      sizeof(VL53LX_dmax_calibration_data_t));


    memcpy(
      &(pdev->xtalk_shapes),
      &(pcal_data->xtalkhisto),
      sizeof(VL53LX_xtalk_histogram_data_t));


    memcpy(
      &(pdev->gain_cal),
      &(pcal_data->gain_cal),
      sizeof(VL53LX_gain_calibration_data_t));


    memcpy(
      &(pdev->cal_peak_rate_map),
      &(pcal_data->cal_peak_rate_map),
      sizeof(VL53LX_cal_peak_rate_map_t));


    memcpy(
      &(pdev->per_vcsel_cal_data),
      &(pcal_data->per_vcsel_cal_data),
      sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));



    pC->algo__crosstalk_compensation_plane_offset_kcps =
      pN->algo__crosstalk_compensation_plane_offset_kcps;
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
      pN->algo__crosstalk_compensation_x_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
      pN->algo__crosstalk_compensation_y_plane_gradient_kcps;

    pHP->algo__crosstalk_compensation_plane_offset_kcps =
      VL53LX_calc_crosstalk_plane_offset_with_margin(
        pC->algo__crosstalk_compensation_plane_offset_kcps,
        pC->histogram_mode_crosstalk_margin_kcps);

    pHP->algo__crosstalk_compensation_x_plane_gradient_kcps =
      pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
    pHP->algo__crosstalk_compensation_y_plane_gradient_kcps =
      pC->algo__crosstalk_compensation_y_plane_gradient_kcps;



    if (pC->global_crosstalk_compensation_enable == 0x00) {
      pN->algo__crosstalk_compensation_plane_offset_kcps =
        0x00;
      pN->algo__crosstalk_compensation_x_plane_gradient_kcps =
        0x00;
      pN->algo__crosstalk_compensation_y_plane_gradient_kcps =
        0x00;
    } else {
      tempu32 =
        VL53LX_calc_crosstalk_plane_offset_with_margin(
          pC->algo__crosstalk_compensation_plane_offset_kcps,
          pC->lite_mode_crosstalk_margin_kcps);


      if (tempu32 > 0xFFFF) {
        tempu32 = 0xFFFF;
      }

      pN->algo__crosstalk_compensation_plane_offset_kcps =
        (uint16_t)tempu32;
    }
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_part_to_part_data(VL53LX_calibration_data_t      *pcal_data)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
  VL53LX_customer_nvm_managed_t *pCN = &(pcal_data->customer);

  pcal_data->struct_version =
    VL53LX_LL_CALIBRATION_DATA_STRUCT_VERSION;


  memcpy(
    &(pcal_data->customer),
    &(pdev->customer),
    sizeof(VL53LX_customer_nvm_managed_t));




  if (pC->algo__crosstalk_compensation_plane_offset_kcps > 0xFFFF) {
    pCN->algo__crosstalk_compensation_plane_offset_kcps =
      0xFFFF;
  } else {
    pCN->algo__crosstalk_compensation_plane_offset_kcps =
      (uint16_t)pC->algo__crosstalk_compensation_plane_offset_kcps;
  }
  pCN->algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pCN->algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps;


  memcpy(
    &(pcal_data->fmt_dmax_cal),
    &(pdev->fmt_dmax_cal),
    sizeof(VL53LX_dmax_calibration_data_t));


  memcpy(
    &(pcal_data->cust_dmax_cal),
    &(pdev->cust_dmax_cal),
    sizeof(VL53LX_dmax_calibration_data_t));


  memcpy(
    &(pcal_data->add_off_cal_data),
    &(pdev->add_off_cal_data),
    sizeof(VL53LX_additional_offset_cal_data_t));


  memcpy(
    &(pcal_data->optical_centre),
    &(pdev->optical_centre),
    sizeof(VL53LX_optical_centre_t));


  memcpy(
    &(pcal_data->xtalkhisto),
    &(pdev->xtalk_shapes),
    sizeof(VL53LX_xtalk_histogram_data_t));


  memcpy(
    &(pcal_data->gain_cal),
    &(pdev->gain_cal),
    sizeof(VL53LX_gain_calibration_data_t));


  memcpy(
    &(pcal_data->cal_peak_rate_map),
    &(pdev->cal_peak_rate_map),
    sizeof(VL53LX_cal_peak_rate_map_t));


  memcpy(
    &(pcal_data->per_vcsel_cal_data),
    &(pdev->per_vcsel_cal_data),
    sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_inter_measurement_period_ms(
  uint32_t                inter_measurement_period_ms)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  if (pdev->dbg_results.result__osc_calibrate_val == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  }

  if (status == VL53LX_ERROR_NONE) {
    pdev->inter_measurement_period_ms = inter_measurement_period_ms;
    pdev->tim_cfg.system__intermeasurement_period =
      inter_measurement_period_ms *
      (uint32_t)pdev->dbg_results.result__osc_calibrate_val;
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_inter_measurement_period_ms(uint32_t               *pinter_measurement_period_ms)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  if (pdev->dbg_results.result__osc_calibrate_val == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  }

  if (status == VL53LX_ERROR_NONE)
    *pinter_measurement_period_ms =
      pdev->tim_cfg.system__intermeasurement_period /
      (uint32_t)pdev->dbg_results.result__osc_calibrate_val;

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_timeouts_us(
  uint32_t            phasecal_config_timeout_us,
  uint32_t            mm_config_timeout_us,
  uint32_t            range_config_timeout_us)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);


  if (pdev->stat_nvm.osc_measured__fast_osc__frequency == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  }

  if (status == VL53LX_ERROR_NONE) {

    pdev->phasecal_config_timeout_us = phasecal_config_timeout_us;
    pdev->mm_config_timeout_us       = mm_config_timeout_us;
    pdev->range_config_timeout_us    = range_config_timeout_us;

    status =
      VL53LX_calc_timeout_register_values(
        phasecal_config_timeout_us,
        mm_config_timeout_us,
        range_config_timeout_us,
        pdev->stat_nvm.osc_measured__fast_osc__frequency,
        &(pdev->gen_cfg),
        &(pdev->tim_cfg));
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_timeouts_us(
  uint32_t            *pphasecal_config_timeout_us,
  uint32_t            *pmm_config_timeout_us,
  uint32_t      *prange_config_timeout_us)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  uint32_t  macro_period_us = 0;
  uint16_t  timeout_encoded = 0;

  if (pdev->stat_nvm.osc_measured__fast_osc__frequency == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  }

  if (status == VL53LX_ERROR_NONE) {


    macro_period_us =
      VL53LX_calc_macro_period_us(
        pdev->stat_nvm.osc_measured__fast_osc__frequency,
        pdev->tim_cfg.range_config__vcsel_period_a);



    *pphasecal_config_timeout_us =
      VL53LX_calc_timeout_us(
        (uint32_t)pdev->gen_cfg.phasecal_config__timeout_macrop,
        macro_period_us);



    timeout_encoded =
      (uint16_t)pdev->tim_cfg.mm_config__timeout_macrop_a_hi;
    timeout_encoded = (timeout_encoded << 8) +
                      (uint16_t)pdev->tim_cfg.mm_config__timeout_macrop_a_lo;

    *pmm_config_timeout_us =
      VL53LX_calc_decoded_timeout_us(
        timeout_encoded,
        macro_period_us);



    timeout_encoded =
      (uint16_t)pdev->tim_cfg.range_config__timeout_macrop_a_hi;
    timeout_encoded = (timeout_encoded << 8) +
                      (uint16_t)pdev->tim_cfg.range_config__timeout_macrop_a_lo;

    *prange_config_timeout_us =
      VL53LX_calc_decoded_timeout_us(
        timeout_encoded,
        macro_period_us);

    pdev->phasecal_config_timeout_us = *pphasecal_config_timeout_us;
    pdev->mm_config_timeout_us       = *pmm_config_timeout_us;
    pdev->range_config_timeout_us    = *prange_config_timeout_us;

  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_calibration_repeat_period(
  uint16_t            cal_config__repeat_period)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->gen_cfg.cal_config__repeat_rate = cal_config__repeat_period;

  return status;

}

VL53LX_Error VL53LX::VL53LX_get_calibration_repeat_period(
  uint16_t           *pcal_config__repeat_period)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  *pcal_config__repeat_period = pdev->gen_cfg.cal_config__repeat_rate;

  return status;

}

VL53LX_Error VL53LX::VL53LX_set_sequence_config_bit(
  VL53LX_DeviceSequenceConfig   bit_id,
  uint8_t                       value)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t  bit_mask        = 0x01;
  uint8_t  clr_mask        = 0xFF  - bit_mask;
  uint8_t  bit_value       = value & bit_mask;

  if (bit_id <= VL53LX_DEVICESEQUENCECONFIG_RANGE) {

    if (bit_id > 0) {
      bit_mask  = 0x01 << bit_id;
      bit_value = bit_value << bit_id;
      clr_mask  = 0xFF  - bit_mask;
    }

    pdev->dyn_cfg.system__sequence_config =
      (pdev->dyn_cfg.system__sequence_config & clr_mask) |
      bit_value;

  } else {
    status = VL53LX_ERROR_INVALID_PARAMS;
  }

  return status;

}


VL53LX_Error VL53LX::VL53LX_get_sequence_config_bit(
  VL53LX_DeviceSequenceConfig   bit_id,
  uint8_t                      *pvalue)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t  bit_mask        = 0x01;

  if (bit_id <= VL53LX_DEVICESEQUENCECONFIG_RANGE) {

    if (bit_id > 0) {
      bit_mask  = 0x01 << bit_id;
    }

    *pvalue =
      pdev->dyn_cfg.system__sequence_config & bit_mask;

    if (bit_id > 0) {
      *pvalue  = *pvalue >> bit_id;
    }

  } else {
    status = VL53LX_ERROR_INVALID_PARAMS;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_interrupt_polarity(
  VL53LX_DeviceInterruptPolarity  interrupt_polarity)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->stat_cfg.gpio_hv_mux__ctrl =
    (pdev->stat_cfg.gpio_hv_mux__ctrl &
     VL53LX_DEVICEINTERRUPTPOLARITY_CLEAR_MASK) |
    (interrupt_polarity &
     VL53LX_DEVICEINTERRUPTPOLARITY_BIT_MASK);

  return status;

}

VL53LX_Error VL53LX::VL53LX_set_refspadchar_config_struct(
  VL53LX_refspadchar_config_t   *pdata)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->refspadchar.device_test_mode = pdata->device_test_mode;
  pdev->refspadchar.VL53LX_p_005     = pdata->VL53LX_p_005;
  pdev->refspadchar.timeout_us       = pdata->timeout_us;
  pdev->refspadchar.target_count_rate_mcps    =
    pdata->target_count_rate_mcps;
  pdev->refspadchar.min_count_rate_limit_mcps =
    pdata->min_count_rate_limit_mcps;
  pdev->refspadchar.max_count_rate_limit_mcps =
    pdata->max_count_rate_limit_mcps;


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_refspadchar_config_struct(
  VL53LX_refspadchar_config_t   *pdata)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  pdata->device_test_mode       = pdev->refspadchar.device_test_mode;
  pdata->VL53LX_p_005           = pdev->refspadchar.VL53LX_p_005;
  pdata->timeout_us             = pdev->refspadchar.timeout_us;
  pdata->target_count_rate_mcps =
    pdev->refspadchar.target_count_rate_mcps;
  pdata->min_count_rate_limit_mcps =
    pdev->refspadchar.min_count_rate_limit_mcps;
  pdata->max_count_rate_limit_mcps =
    pdev->refspadchar.max_count_rate_limit_mcps;


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_range_ignore_threshold(
  uint8_t                 range_ignore_thresh_mult,
  uint16_t                range_ignore_threshold_mcps)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
    range_ignore_threshold_mcps;

  pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult =
    range_ignore_thresh_mult;

  return status;

}


VL53LX_Error VL53LX::VL53LX_get_range_ignore_threshold(
  uint8_t                *prange_ignore_thresh_mult,
  uint16_t               *prange_ignore_threshold_mcps_internal,
  uint16_t               *prange_ignore_threshold_mcps_current)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  *prange_ignore_thresh_mult =
    pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult;

  *prange_ignore_threshold_mcps_current =
    pdev->stat_cfg.algo__range_ignore_threshold_mcps;

  *prange_ignore_threshold_mcps_internal =
    pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps;

  return status;

}

VL53LX_Error VL53LX::VL53LX_get_interrupt_polarity(
  VL53LX_DeviceInterruptPolarity  *pinterrupt_polarity)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  *pinterrupt_polarity =
    pdev->stat_cfg.gpio_hv_mux__ctrl &
    VL53LX_DEVICEINTERRUPTPOLARITY_BIT_MASK;

  return status;

}

VL53LX_Error VL53LX::VL53LX_set_user_zone(
  VL53LX_user_zone_t     *puser_zone)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_encode_row_col(
    puser_zone->y_centre,
    puser_zone->x_centre,
    &(pdev->dyn_cfg.roi_config__user_roi_centre_spad));


  VL53LX_encode_zone_size(
    puser_zone->width,
    puser_zone->height,
    &(pdev->dyn_cfg.roi_config__user_roi_requested_global_xy_size));


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_user_zone(
  VL53LX_user_zone_t     *puser_zone)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_decode_row_col(
    pdev->dyn_cfg.roi_config__user_roi_centre_spad,
    &(puser_zone->y_centre),
    &(puser_zone->x_centre));


  VL53LX_decode_zone_size(
    pdev->dyn_cfg.roi_config__user_roi_requested_global_xy_size,
    &(puser_zone->width),
    &(puser_zone->height));


  return status;
}
VL53LX_Error VL53LX::VL53LX_get_mode_mitigation_roi(
  VL53LX_user_zone_t     *pmm_roi)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t  x       = 0;
  uint8_t  y       = 0;
  uint8_t  xy_size = 0;


  VL53LX_decode_row_col(
    pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
    &y,
    &x);

  pmm_roi->x_centre = x;
  pmm_roi->y_centre = y;


  xy_size = pdev->nvm_copy_data.roi_config__mode_roi_xy_size;

  pmm_roi->height = xy_size >> 4;
  pmm_roi->width  = xy_size & 0x0F;


  return status;
}
VL53LX_Error VL53LX::VL53LX_set_zone_config(
  VL53LX_zone_config_t      *pzone_cfg)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  memcpy(&(pdev->zone_cfg.user_zones), &(pzone_cfg->user_zones),
         sizeof(pdev->zone_cfg.user_zones));


  pdev->zone_cfg.max_zones    = pzone_cfg->max_zones;
  pdev->zone_cfg.active_zones = pzone_cfg->active_zones;

  status = VL53LX_init_zone_config_histogram_bins(&pdev->zone_cfg);



  if (pzone_cfg->active_zones == 0) {
    pdev->gen_cfg.global_config__stream_divider = 0;
  } else if (pzone_cfg->active_zones < VL53LX_MAX_USER_ZONES)
    pdev->gen_cfg.global_config__stream_divider =
      pzone_cfg->active_zones + 1;
  else
    pdev->gen_cfg.global_config__stream_divider =
      VL53LX_MAX_USER_ZONES + 1;

  return status;

}



VL53LX_Error VL53LX::VL53LX_get_zone_config(
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  memcpy(pzone_cfg, &(pdev->zone_cfg), sizeof(VL53LX_zone_config_t));

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_preset_mode_timing_cfg(
  VL53LX_DevicePresetModes     device_preset_mode,
  uint16_t                    *pdss_config__target_total_rate_mcps,
  uint32_t                    *pphasecal_config_timeout_us,
  uint32_t                    *pmm_config_timeout_us,
  uint32_t                    *prange_config_timeout_us)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  switch (device_preset_mode) {

    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING:
    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
    case VL53LX_DEVICEPRESETMODE_OLT:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_lite_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_lite_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_lite_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_lite_us;
      break;

    case VL53LX_DEVICEPRESETMODE_TIMED_RANGING:
    case VL53LX_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
    case VL53LX_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
    case VL53LX_DEVICEPRESETMODE_SINGLESHOT_RANGING:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_timed_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_timed_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_timed_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_timed_us;
      break;

    case VL53LX_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
    case VL53LX_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
    case VL53LX_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_timed_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_timed_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_lpa_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_lpa_us;
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_WITH_MM1:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_WITH_MM2:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_MM1_CAL:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_MM2_CAL:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_REF_ARRAY:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE_MM1:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE_MM2:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_CHARACTERISATION:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_hist_long_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_histo_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_histo_us;

      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mz_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_mz_med_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_mz_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_mz_us;
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_SHORT_RANGE:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mz_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_mz_short_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_mz_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_mz_us;
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_LONG_RANGE:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mz_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_mz_long_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_mz_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_mz_us;
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_SHORT_TIMING:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_hist_short_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_histo_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_histo_us;
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE_MM1:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE_MM2:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_hist_med_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_histo_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_histo_us;
      break;


    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE_MM1:
    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE_MM2:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_histo_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_hist_short_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_histo_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_histo_us;
      break;

    case VL53LX_DEVICEPRESETMODE_SPECIAL_HISTOGRAM_SHORT_RANGE:
      *pdss_config__target_total_rate_mcps =
        pdev->tuning_parms.tp_dss_target_very_short_mcps;
      *pphasecal_config_timeout_us =
        pdev->tuning_parms.tp_phasecal_timeout_hist_short_us;
      *pmm_config_timeout_us =
        pdev->tuning_parms.tp_mm_timeout_histo_us;
      *prange_config_timeout_us =
        pdev->tuning_parms.tp_range_timeout_histo_us;
      break;

    default:
      status = VL53LX_ERROR_INVALID_PARAMS;
      break;

  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_preset_mode(
  VL53LX_DevicePresetModes     device_preset_mode,
  uint16_t                     dss_config__target_total_rate_mcps,
  uint32_t                     phasecal_config_timeout_us,
  uint32_t                     mm_config_timeout_us,
  uint32_t                     range_config_timeout_us,
  uint32_t                     inter_measurement_period_ms)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_hist_post_process_config_t *phistpostprocess =
    &(pdev->histpostprocess);

  VL53LX_static_config_t        *pstatic       = &(pdev->stat_cfg);
  VL53LX_histogram_config_t     *phistogram    = &(pdev->hist_cfg);
  VL53LX_general_config_t       *pgeneral      = &(pdev->gen_cfg);
  VL53LX_timing_config_t        *ptiming       = &(pdev->tim_cfg);
  VL53LX_dynamic_config_t       *pdynamic      = &(pdev->dyn_cfg);
  VL53LX_system_control_t       *psystem       = &(pdev->sys_ctrl);
  VL53LX_zone_config_t          *pzone_cfg     = &(pdev->zone_cfg);
  VL53LX_tuning_parm_storage_t  *ptuning_parms = &(pdev->tuning_parms);
  VL53LX_low_power_auto_data_t  *plpadata      =
    &(pdev->low_power_auto_data);


  pdev->preset_mode                 = device_preset_mode;
  pdev->mm_config_timeout_us        = mm_config_timeout_us;
  pdev->range_config_timeout_us     = range_config_timeout_us;
  pdev->inter_measurement_period_ms = inter_measurement_period_ms;



  VL53LX_init_ll_driver_state(VL53LX_DEVICESTATE_SW_STANDBY);



  switch (device_preset_mode) {

    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING:
      status = VL53LX_preset_mode_standard_ranging(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
      status = VL53LX_preset_mode_standard_ranging_short_range(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
      status = VL53LX_preset_mode_standard_ranging_long_range(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
      status = VL53LX_preset_mode_standard_ranging_mm1_cal(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
      status = VL53LX_preset_mode_standard_ranging_mm2_cal(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_TIMED_RANGING:
      status = VL53LX_preset_mode_timed_ranging(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
      status = VL53LX_preset_mode_timed_ranging_short_range(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
      status = VL53LX_preset_mode_timed_ranging_long_range(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING:
      status = VL53LX_preset_mode_histogram_ranging(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_WITH_MM1:
      status = VL53LX_preset_mode_histogram_ranging_with_mm1(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_WITH_MM2:
      status = VL53LX_preset_mode_histogram_ranging_with_mm2(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_MM1_CAL:
      status = VL53LX_preset_mode_histogram_ranging_mm1_cal(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_MM2_CAL:
      status = VL53LX_preset_mode_histogram_ranging_mm2_cal(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE:
      status = VL53LX_preset_mode_histogram_multizone(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_SHORT_RANGE:
      status = VL53LX_preset_mode_histogram_multizone_short_range(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_LONG_RANGE:
      status = VL53LX_preset_mode_histogram_multizone_long_range(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_REF_ARRAY:
      status = VL53LX_preset_mode_histogram_ranging_ref(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_SHORT_TIMING:
      status = VL53LX_preset_mode_histogram_ranging_short_timing(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:
      status = VL53LX_preset_mode_histogram_long_range(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE_MM1:
      status = VL53LX_preset_mode_histogram_long_range_mm1(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE_MM2:
      status = VL53LX_preset_mode_histogram_long_range_mm2(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
      status = VL53LX_preset_mode_histogram_medium_range(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE_MM1:
      status = VL53LX_preset_mode_histogram_medium_range_mm1(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE_MM2:
      status = VL53LX_preset_mode_histogram_medium_range_mm2(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:
      status = VL53LX_preset_mode_histogram_short_range(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE_MM1:
      status = VL53LX_preset_mode_histogram_short_range_mm1(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE_MM2:
      status = VL53LX_preset_mode_histogram_short_range_mm2(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_CHARACTERISATION:
      status = VL53LX_preset_mode_histogram_characterisation(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_XTALK_PLANAR:
      status = VL53LX_preset_mode_histogram_xtalk_planar(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_XTALK_MM1:
      status = VL53LX_preset_mode_histogram_xtalk_mm1(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_HISTOGRAM_XTALK_MM2:
      status = VL53LX_preset_mode_histogram_xtalk_mm2(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_OLT:
      status = VL53LX_preset_mode_olt(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_SINGLESHOT_RANGING:
      status = VL53LX_preset_mode_singleshot_ranging(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    case VL53LX_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
      status = VL53LX_preset_mode_low_power_auto_short_ranging(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg,
                 plpadata);
      break;

    case VL53LX_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
      status = VL53LX_preset_mode_low_power_auto_ranging(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg,
                 plpadata);
      break;

    case VL53LX_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
      status = VL53LX_preset_mode_low_power_auto_long_ranging(
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg,
                 plpadata);
      break;


    case VL53LX_DEVICEPRESETMODE_SPECIAL_HISTOGRAM_SHORT_RANGE:
      status = VL53LX_preset_mode_special_histogram_short_range(
                 phistpostprocess,
                 pstatic,
                 phistogram,
                 pgeneral,
                 ptiming,
                 pdynamic,
                 psystem,
                 ptuning_parms,
                 pzone_cfg);
      break;

    default:
      status = VL53LX_ERROR_INVALID_PARAMS;
      break;

  }



  if (status == VL53LX_ERROR_NONE) {

    pstatic->dss_config__target_total_rate_mcps =
      dss_config__target_total_rate_mcps;
    pdev->dss_config__target_total_rate_mcps    =
      dss_config__target_total_rate_mcps;

  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_timeouts_us(
        phasecal_config_timeout_us,
        mm_config_timeout_us,
        range_config_timeout_us);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_inter_measurement_period_ms(inter_measurement_period_ms);



  V53L1_init_zone_results_structure(
    pdev->zone_cfg.active_zones + 1,
    &(pres->zone_results));

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_zone_preset(
  VL53LX_DeviceZonePreset    zone_preset)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_general_config_t       *pgeneral      = &(pdev->gen_cfg);
  VL53LX_zone_config_t          *pzone_cfg     = &(pdev->zone_cfg);

  pdev->zone_preset        = zone_preset;



  switch (zone_preset) {

    case VL53LX_DEVICEZONEPRESET_XTALK_PLANAR:
      status =
        VL53LX_zone_preset_xtalk_planar(
          pgeneral,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_1X1_SIZE_16X16:
      status =
        VL53LX_init_zone_config_structure(
          8, 1, 1,
          8, 1, 1,
          15, 15,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_1X2_SIZE_16X8:
      status =
        VL53LX_init_zone_config_structure(
          8, 1, 1,
          4, 8, 2,
          15, 7,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_2X1_SIZE_8X16:
      status =
        VL53LX_init_zone_config_structure(
          4, 8, 2,
          8, 1, 1,
          7, 15,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_2X2_SIZE_8X8:
      status =
        VL53LX_init_zone_config_structure(
          4, 8, 2,
          4, 8, 2,
          7, 7,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_3X3_SIZE_5X5:
      status =
        VL53LX_init_zone_config_structure(
          2, 5, 3,
          2, 5, 3,
          4, 4,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_4X4_SIZE_4X4:
      status =
        VL53LX_init_zone_config_structure(
          2, 4, 4,
          2, 4, 4,
          3, 3,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_5X5_SIZE_4X4:
      status =
        VL53LX_init_zone_config_structure(
          2, 3, 5,
          2, 3, 5,
          3, 3,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_11X11_SIZE_5X5:
      status =
        VL53LX_init_zone_config_structure(
          3, 1, 11,
          3, 1, 11,
          4, 4,
          pzone_cfg);
      break;

    case VL53LX_DEVICEZONEPRESET_13X13_SIZE_4X4:
      status =
        VL53LX_init_zone_config_structure(
          2, 1, 13,
          2, 1, 13,
          3, 3,
          pzone_cfg);

      break;

    case VL53LX_DEVICEZONEPRESET_1X1_SIZE_4X4_POS_8X8:
      status =
        VL53LX_init_zone_config_structure(
          8, 1, 1,
          8, 1, 1,
          3, 3,
          pzone_cfg);
      break;

  }



  if (pzone_cfg->active_zones == 0) {
    pdev->gen_cfg.global_config__stream_divider = 0;
  } else if (pzone_cfg->active_zones < VL53LX_MAX_USER_ZONES)
    pdev->gen_cfg.global_config__stream_divider =
      pzone_cfg->active_zones + 1;
  else
    pdev->gen_cfg.global_config__stream_divider =
      VL53LX_MAX_USER_ZONES + 1;

  return status;
}

VL53LX_Error  VL53LX::VL53LX_enable_xtalk_compensation()
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint32_t tempu32;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

  tempu32 = VL53LX_calc_crosstalk_plane_offset_with_margin(
              pC->algo__crosstalk_compensation_plane_offset_kcps,
              pC->lite_mode_crosstalk_margin_kcps);
  if (tempu32 > 0xFFFF) {
    tempu32 = 0xFFFF;
  }

  pN->algo__crosstalk_compensation_plane_offset_kcps =
    (uint16_t)tempu32;

  pN->algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps;

  pN->algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps;


  pHP->algo__crosstalk_compensation_plane_offset_kcps =
    VL53LX_calc_crosstalk_plane_offset_with_margin(
      pC->algo__crosstalk_compensation_plane_offset_kcps,
      pC->histogram_mode_crosstalk_margin_kcps);

  pHP->algo__crosstalk_compensation_x_plane_gradient_kcps
    = pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pHP->algo__crosstalk_compensation_y_plane_gradient_kcps
    = pC->algo__crosstalk_compensation_y_plane_gradient_kcps;



  pC->global_crosstalk_compensation_enable = 0x01;

  pHP->algo__crosstalk_compensation_enable =
    pC->global_crosstalk_compensation_enable;




  if (status == VL53LX_ERROR_NONE) {
    pC->crosstalk_range_ignore_threshold_rate_mcps =
      VL53LX_calc_range_ignore_threshold(
        pC->algo__crosstalk_compensation_plane_offset_kcps,
        pC->algo__crosstalk_compensation_x_plane_gradient_kcps,
        pC->algo__crosstalk_compensation_y_plane_gradient_kcps,
        pC->crosstalk_range_ignore_threshold_mult);
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_customer_nvm_managed(&(pdev->customer));

  return status;

}

void VL53LX::VL53LX_get_xtalk_compensation_enable(uint8_t       *pcrosstalk_compensation_enable)
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  *pcrosstalk_compensation_enable =
    pdev->xtalk_cfg.global_crosstalk_compensation_enable;

}


VL53LX_Error VL53LX::VL53LX_get_lite_xtalk_margin_kcps(
  int16_t                           *pxtalk_margin)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  *pxtalk_margin = pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps;

  return status;

}
VL53LX_Error VL53LX::VL53LX_set_lite_xtalk_margin_kcps(
  int16_t                        xtalk_margin)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps = xtalk_margin;

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_histogram_xtalk_margin_kcps(
  int16_t                           *pxtalk_margin)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *pxtalk_margin = pdev->xtalk_cfg.histogram_mode_crosstalk_margin_kcps;

  return status;

}
VL53LX_Error VL53LX::VL53LX_set_histogram_xtalk_margin_kcps(
  int16_t                        xtalk_margin)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->xtalk_cfg.histogram_mode_crosstalk_margin_kcps = xtalk_margin;

  return status;
}

VL53LX_Error VL53LX::VL53LX_restore_xtalk_nvm_default()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);

  pC->algo__crosstalk_compensation_plane_offset_kcps =
    pC->nvm_default__crosstalk_compensation_plane_offset_kcps;
  pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC->nvm_default__crosstalk_compensation_x_plane_gradient_kcps;
  pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC->nvm_default__crosstalk_compensation_y_plane_gradient_kcps;


  return status;
}

VL53LX_Error  VL53LX::VL53LX_disable_xtalk_compensation()
{
  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

  pN->algo__crosstalk_compensation_plane_offset_kcps =
    0x00;

  pN->algo__crosstalk_compensation_x_plane_gradient_kcps =
    0x00;

  pN->algo__crosstalk_compensation_y_plane_gradient_kcps =
    0x00;



  pdev->xtalk_cfg.global_crosstalk_compensation_enable = 0x00;

  pHP->algo__crosstalk_compensation_enable =
    pdev->xtalk_cfg.global_crosstalk_compensation_enable;



  if (status == VL53LX_ERROR_NONE) {
    pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
      0x0000;
  }



  if (status == VL53LX_ERROR_NONE) {
    status =
      VL53LX_set_customer_nvm_managed(&(pdev->customer));
  }


  return status;

}
VL53LX_Error VL53LX::VL53LX_get_histogram_phase_consistency(
  uint8_t                            *pphase_consistency)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);

  *pphase_consistency =
    pHP->algo__consistency_check__phase_tolerance;

  return status;

}

VL53LX_Error VL53LX::VL53LX_set_histogram_phase_consistency(
  uint8_t                             phase_consistency)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->histpostprocess.algo__consistency_check__phase_tolerance =
    phase_consistency;

  return status;

}
VL53LX_Error VL53LX::VL53LX_get_histogram_event_consistency(
  uint8_t                            *pevent_consistency)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *pevent_consistency =
    pdev->histpostprocess.algo__consistency_check__event_sigma;

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_histogram_event_consistency(
  uint8_t                             event_consistency)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->histpostprocess.algo__consistency_check__event_sigma =
    event_consistency;

  return status;

}
VL53LX_Error VL53LX::VL53LX_get_histogram_ambient_threshold_sigma(
  uint8_t                            *pamb_thresh_sigma)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *pamb_thresh_sigma =
    pdev->histpostprocess.ambient_thresh_sigma1;

  return status;

}

VL53LX_Error VL53LX::VL53LX_set_histogram_ambient_threshold_sigma(
  uint8_t                             amb_thresh_sigma)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->histpostprocess.ambient_thresh_sigma1 =
    amb_thresh_sigma;


  return status;

}

VL53LX_Error VL53LX::VL53LX_get_lite_sigma_threshold(
  uint16_t                           *plite_sigma)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *plite_sigma =
    pdev->tim_cfg.range_config__sigma_thresh;


  return status;

}

VL53LX_Error VL53LX::VL53LX_set_lite_sigma_threshold(
  uint16_t                           lite_sigma)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->tim_cfg.range_config__sigma_thresh = lite_sigma;

  return status;

}

VL53LX_Error VL53LX::VL53LX_get_lite_min_count_rate(
  uint16_t                           *plite_mincountrate)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  *plite_mincountrate =
    pdev->tim_cfg.range_config__min_count_rate_rtn_limit_mcps;

  return status;

}


VL53LX_Error VL53LX::VL53LX_set_lite_min_count_rate(
  uint16_t                            lite_mincountrate)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->tim_cfg.range_config__min_count_rate_rtn_limit_mcps =
    lite_mincountrate;

  return status;

}
VL53LX_Error VL53LX::VL53LX_get_xtalk_detect_config(
  int16_t                            *pmax_valid_range_mm,
  int16_t                            *pmin_valid_range_mm,
  uint16_t                           *pmax_valid_rate_kcps,
  uint16_t                           *pmax_sigma_mm)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *pmax_valid_range_mm =
    pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm;
  *pmin_valid_range_mm =
    pdev->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm;
  *pmax_valid_rate_kcps =
    pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps;
  *pmax_sigma_mm =
    pdev->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm;

  return status;

}



VL53LX_Error VL53LX::VL53LX_set_xtalk_detect_config(
  int16_t                             max_valid_range_mm,
  int16_t                             min_valid_range_mm,
  uint16_t                            max_valid_rate_kcps,
  uint16_t                            max_sigma_mm)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm =
    max_valid_range_mm;
  pdev->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm =
    min_valid_range_mm;
  pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps =
    max_valid_rate_kcps;
  pdev->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm =
    max_sigma_mm;

  return status;

}

VL53LX_Error VL53LX::VL53LX_get_target_order_mode(
  VL53LX_HistTargetOrder             *phist_target_order)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *phist_target_order =
    pdev->histpostprocess.hist_target_order;

  return status;

}
VL53LX_Error VL53LX::VL53LX_set_target_order_mode(
  VL53LX_HistTargetOrder              hist_target_order)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  pdev->histpostprocess.hist_target_order = hist_target_order;


  return status;

}

VL53LX_Error VL53LX::VL53LX_get_dmax_reflectance_values(
  VL53LX_dmax_reflectance_array_t    *pdmax_reflectances)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint8_t i = 0;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  for (i = 0; i < VL53LX_MAX_AMBIENT_DMAX_VALUES; i++) {
    pdmax_reflectances->target_reflectance_for_dmax[i] =
      pdev->dmax_cfg.target_reflectance_for_dmax_calc[i];
  }

  return status;

}
VL53LX_Error VL53LX::VL53LX_set_dmax_reflectance_values(
  VL53LX_dmax_reflectance_array_t    *pdmax_reflectances)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint8_t i = 0;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  for (i = 0; i < VL53LX_MAX_AMBIENT_DMAX_VALUES; i++) {
    pdev->dmax_cfg.target_reflectance_for_dmax_calc[i] =
      pdmax_reflectances->target_reflectance_for_dmax[i];
  }


  return status;

}

VL53LX_Error VL53LX::VL53LX_get_vhv_loopbound(
  uint8_t                     *pvhv_loopbound)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *pvhv_loopbound =
    pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound / 4;

  return status;

}

VL53LX_Error VL53LX::VL53LX_set_vhv_loopbound(
  uint8_t                      vhv_loopbound)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
    (pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03) +
    (vhv_loopbound * 4);

  return status;

}
VL53LX_Error VL53LX::VL53LX_get_vhv_config(
  uint8_t                     *pvhv_init_en,
  uint8_t                     *pvhv_init_value)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  *pvhv_init_en    = (pdev->stat_nvm.vhv_config__init & 0x80) >> 7;
  *pvhv_init_value =
    (pdev->stat_nvm.vhv_config__init & 0x7F);

  return status;

}


VL53LX_Error VL53LX::VL53LX_set_vhv_config(
  uint8_t                      vhv_init_en,
  uint8_t                      vhv_init_value)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->stat_nvm.vhv_config__init =
    ((vhv_init_en   & 0x01) << 7) +
    (vhv_init_value & 0x7F);


  return status;

}

VL53LX_Error VL53LX::VL53LX_init_and_start_range(
  uint8_t                        measurement_mode,
  VL53LX_DeviceConfigLevel       device_config_level)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t  *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  uint8_t buffer[VL53LX_MAX_I2C_XFER_SIZE];

  VL53LX_static_nvm_managed_t   *pstatic_nvm   = &(pdev->stat_nvm);
  VL53LX_customer_nvm_managed_t *pcustomer_nvm = &(pdev->customer);
  VL53LX_static_config_t        *pstatic       = &(pdev->stat_cfg);
  VL53LX_general_config_t       *pgeneral      = &(pdev->gen_cfg);
  VL53LX_timing_config_t        *ptiming       = &(pdev->tim_cfg);
  VL53LX_dynamic_config_t       *pdynamic      = &(pdev->dyn_cfg);
  VL53LX_system_control_t       *psystem       = &(pdev->sys_ctrl);

  VL53LX_ll_driver_state_t  *pstate   = &(pdev->ll_state);
  VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

  uint8_t  *pbuffer                   = &buffer[0];
  uint16_t i                          = 0;
  uint16_t i2c_index                  = 0;
  uint16_t i2c_buffer_offset_bytes    = 0;
  uint16_t i2c_buffer_size_bytes      = 0;


  pdev->measurement_mode = measurement_mode;



  psystem->system__mode_start =
    (psystem->system__mode_start &
     VL53LX_DEVICEMEASUREMENTMODE_STOP_MASK) |
    measurement_mode;



  status =
    VL53LX_set_user_zone(
      &(pdev->zone_cfg.user_zones[pdev->ll_state.cfg_zone_id]));


  if (pdev->zone_cfg.active_zones > 0) {
    status =
      VL53LX_set_zone_dss_config(
        &(pres->zone_dyn_cfgs.VL53LX_p_003[pdev->ll_state.cfg_zone_id])
      );
  }




  if (((pdev->sys_ctrl.system__mode_start &
        VL53LX_DEVICESCHEDULERMODE_HISTOGRAM) == 0x00) &&
      (pdev->xtalk_cfg.global_crosstalk_compensation_enable
       == 0x01)) {
    pdev->stat_cfg.algo__range_ignore_threshold_mcps =
      pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps;
  }





  if (pdev->low_power_auto_data.low_power_auto_range_count == 0xFF) {
    pdev->low_power_auto_data.low_power_auto_range_count = 0x0;
  }


  if ((pdev->low_power_auto_data.is_low_power_auto_mode == 1) &&
      (pdev->low_power_auto_data.low_power_auto_range_count == 0)) {

    pdev->low_power_auto_data.saved_interrupt_config =
      pdev->gen_cfg.system__interrupt_config_gpio;

    pdev->gen_cfg.system__interrupt_config_gpio = 1 << 5;

    if ((pdev->dyn_cfg.system__sequence_config & (
           VL53LX_SEQUENCE_MM1_EN | VL53LX_SEQUENCE_MM2_EN)) ==
        0x0) {
      pN->algo__part_to_part_range_offset_mm =
        (pN->mm_config__outer_offset_mm << 2);
    } else {
      pN->algo__part_to_part_range_offset_mm = 0x0;
    }


    if (device_config_level <
        VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS) {
      device_config_level =
        VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS;
    }
  }

  if ((pdev->low_power_auto_data.is_low_power_auto_mode == 1) &&
      (pdev->low_power_auto_data.low_power_auto_range_count == 1)) {

    pdev->gen_cfg.system__interrupt_config_gpio =
      pdev->low_power_auto_data.saved_interrupt_config;


    device_config_level = VL53LX_DEVICECONFIGLEVEL_FULL;
  }





  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_save_cfg_data();
  }



  switch (device_config_level) {
    case VL53LX_DEVICECONFIGLEVEL_FULL:
      i2c_index = VL53LX_STATIC_NVM_MANAGED_I2C_INDEX;
      break;
    case VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS:
      i2c_index = VL53LX_CUSTOMER_NVM_MANAGED_I2C_INDEX;
      break;
    case VL53LX_DEVICECONFIGLEVEL_STATIC_ONWARDS:
      i2c_index = VL53LX_STATIC_CONFIG_I2C_INDEX;
      break;
    case VL53LX_DEVICECONFIGLEVEL_GENERAL_ONWARDS:
      i2c_index = VL53LX_GENERAL_CONFIG_I2C_INDEX;
      break;
    case VL53LX_DEVICECONFIGLEVEL_TIMING_ONWARDS:
      i2c_index = VL53LX_TIMING_CONFIG_I2C_INDEX;
      break;
    case VL53LX_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS:
      i2c_index = VL53LX_DYNAMIC_CONFIG_I2C_INDEX;
      break;
    default:
      i2c_index = VL53LX_SYSTEM_CONTROL_I2C_INDEX;
      break;
  }



  i2c_buffer_size_bytes =
    (VL53LX_SYSTEM_CONTROL_I2C_INDEX +
     VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES) -
    i2c_index;



  pbuffer = &buffer[0];
  for (i = 0; i < i2c_buffer_size_bytes; i++) {
    *pbuffer++ = 0;
  }



  if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_FULL &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_STATIC_NVM_MANAGED_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_encode_static_nvm_managed(
        pstatic_nvm,
        VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }

  if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_CUSTOMER_NVM_MANAGED_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_encode_customer_nvm_managed(
        pcustomer_nvm,
        VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }

  if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_STATIC_ONWARDS &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_STATIC_CONFIG_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_encode_static_config(
        pstatic,
        VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }

  if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_GENERAL_ONWARDS &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_GENERAL_CONFIG_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_encode_general_config(
        pgeneral,
        VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }

  if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_TIMING_ONWARDS &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_TIMING_CONFIG_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_encode_timing_config(
        ptiming,
        VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }

  if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_DYNAMIC_CONFIG_I2C_INDEX - i2c_index;


    if ((psystem->system__mode_start &
         VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK) ==
        VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK) {
      pdynamic->system__grouped_parameter_hold_0 =
        pstate->cfg_gph_id | 0x01;
      pdynamic->system__grouped_parameter_hold_1 =
        pstate->cfg_gph_id | 0x01;
      pdynamic->system__grouped_parameter_hold   =
        pstate->cfg_gph_id;
    }
    status =
      VL53LX_i2c_encode_dynamic_config(
        pdynamic,
        VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }

  if (status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_SYSTEM_CONTROL_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_encode_system_control(
        psystem,
        VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes]);
  }



  if (status == VL53LX_ERROR_NONE) {
    status =
      VL53LX_WriteMulti(
        Dev,
        i2c_index,
        buffer,
        (uint32_t)i2c_buffer_size_bytes);
  }


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_update_ll_driver_rd_state();
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_update_ll_driver_cfg_state();
  }


  return status;
}
VL53LX_Error VL53LX::VL53LX_stop_range()
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);



  pdev->sys_ctrl.system__mode_start =
    (pdev->sys_ctrl.system__mode_start &
     VL53LX_DEVICEMEASUREMENTMODE_STOP_MASK) |
    VL53LX_DEVICEMEASUREMENTMODE_ABORT;

  status = VL53LX_set_system_control(
             &pdev->sys_ctrl);


  pdev->sys_ctrl.system__mode_start =
    (pdev->sys_ctrl.system__mode_start &
     VL53LX_DEVICEMEASUREMENTMODE_STOP_MASK);


  VL53LX_init_ll_driver_state(
    VL53LX_DEVICESTATE_SW_STANDBY);


  V53L1_init_zone_results_structure(
    pdev->zone_cfg.active_zones + 1,
    &(pres->zone_results));


  V53L1_init_zone_dss_configs();


  if (pdev->low_power_auto_data.is_low_power_auto_mode == 1) {
    VL53LX_low_power_auto_data_stop_range();
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_measurement_results(
  VL53LX_DeviceResultsLevel      device_results_level)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t buffer[VL53LX_MAX_I2C_XFER_SIZE];

  VL53LX_system_results_t   *psystem_results = &(pdev->sys_results);
  VL53LX_core_results_t     *pcore_results   = &(pdev->core_results);
  VL53LX_debug_results_t    *pdebug_results  = &(pdev->dbg_results);

  uint16_t i2c_index               = VL53LX_SYSTEM_RESULTS_I2C_INDEX;
  uint16_t i2c_buffer_offset_bytes = 0;
  uint16_t i2c_buffer_size_bytes   = 0;

  switch (device_results_level) {
    case VL53LX_DEVICERESULTSLEVEL_FULL:
      i2c_buffer_size_bytes =
        (VL53LX_DEBUG_RESULTS_I2C_INDEX +
         VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES) -
        i2c_index;
      break;
    case VL53LX_DEVICERESULTSLEVEL_UPTO_CORE:
      i2c_buffer_size_bytes =
        (VL53LX_CORE_RESULTS_I2C_INDEX +
         VL53LX_CORE_RESULTS_I2C_SIZE_BYTES) -
        i2c_index;
      break;
    default:
      i2c_buffer_size_bytes =
        VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES;
      break;
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_ReadMulti(
        Dev,
        i2c_index,
        buffer,
        (uint32_t)i2c_buffer_size_bytes);



  if (device_results_level >= VL53LX_DEVICERESULTSLEVEL_FULL &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_DEBUG_RESULTS_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_decode_debug_results(
        VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes],
        pdebug_results);
  }

  if (device_results_level >= VL53LX_DEVICERESULTSLEVEL_UPTO_CORE &&
      status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes =
      VL53LX_CORE_RESULTS_I2C_INDEX - i2c_index;

    status =
      VL53LX_i2c_decode_core_results(
        VL53LX_CORE_RESULTS_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes],
        pcore_results);
  }

  if (status == VL53LX_ERROR_NONE) {

    i2c_buffer_offset_bytes = 0;
    status =
      VL53LX_i2c_decode_system_results(
        VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES,
        &buffer[i2c_buffer_offset_bytes],
        psystem_results);
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_device_results(
  VL53LX_DeviceResultsLevel     device_results_level,
  VL53LX_range_results_t       *prange_results)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_range_results_t   *presults =
    &(pres->range_results);
  VL53LX_zone_objects_t    *pobjects =
    &(pres->zone_results.VL53LX_p_003[0]);
  VL53LX_ll_driver_state_t *pstate   =
    &(pdev->ll_state);
  VL53LX_zone_config_t     *pzone_cfg =
    &(pdev->zone_cfg);
  VL53LX_zone_hist_info_t  *phist_info =
    &(pres->zone_hists.VL53LX_p_003[0]);

  VL53LX_dmax_calibration_data_t   dmax_cal;
  VL53LX_dmax_calibration_data_t *pdmax_cal = &dmax_cal;
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
  VL53LX_low_power_auto_data_t *pL = &(pdev->low_power_auto_data);
  VL53LX_histogram_bin_data_t *pHD = &(pdev->hist_data);
  VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);
  VL53LX_zone_histograms_t *pZH = &(pres->zone_hists);
  VL53LX_xtalk_calibration_results_t *pXCR = &(pdev->xtalk_cal);
  uint8_t tmp8;
  uint8_t zid;
  uint8_t i;
  uint8_t histo_merge_nb, idx;
  VL53LX_range_data_t *pdata;


  if ((pdev->sys_ctrl.system__mode_start &
       VL53LX_DEVICESCHEDULERMODE_HISTOGRAM)
      == VL53LX_DEVICESCHEDULERMODE_HISTOGRAM) {



    status = VL53LX_get_histogram_bin_data(&(pdev->hist_data));




    if (status == VL53LX_ERROR_NONE &&
        pHD->number_of_ambient_bins == 0) {
      zid = pdev->ll_state.rd_zone_id;
      status = VL53LX_hist_copy_and_scale_ambient_info(
                 &(pZH->VL53LX_p_003[zid]),
                 &(pdev->hist_data));
    }


    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }

    VL53LX_compute_histo_merge_nb(&histo_merge_nb);
    if (histo_merge_nb == 0) {
      histo_merge_nb = 1;
    }
    idx = histo_merge_nb - 1;
    if (pdev->tuning_parms.tp_hist_merge == 1)
      pC->algo__crosstalk_compensation_plane_offset_kcps =
        pXCR->algo__xtalk_cpo_HistoMerge_kcps[idx];

    pHP->gain_factor =
      pdev->gain_cal.histogram_ranging_gain_factor;

    pHP->algo__crosstalk_compensation_plane_offset_kcps =
      VL53LX_calc_crosstalk_plane_offset_with_margin(
        pC->algo__crosstalk_compensation_plane_offset_kcps,
        pC->histogram_mode_crosstalk_margin_kcps);

    pHP->algo__crosstalk_compensation_x_plane_gradient_kcps =
      pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
    pHP->algo__crosstalk_compensation_y_plane_gradient_kcps =
      pC->algo__crosstalk_compensation_y_plane_gradient_kcps;

    pdev->dmax_cfg.ambient_thresh_sigma =
      pHP->ambient_thresh_sigma1;
    pdev->dmax_cfg.min_ambient_thresh_events =
      pHP->min_ambient_thresh_events;
    pdev->dmax_cfg.signal_total_events_limit =
      pHP->signal_total_events_limit;
    pdev->dmax_cfg.dss_config__target_total_rate_mcps =
      pdev->stat_cfg.dss_config__target_total_rate_mcps;
    pdev->dmax_cfg.dss_config__aperture_attenuation =
      pdev->gen_cfg.dss_config__aperture_attenuation;

    pHP->algo__crosstalk_detect_max_valid_range_mm =
      pC->algo__crosstalk_detect_max_valid_range_mm;
    pHP->algo__crosstalk_detect_min_valid_range_mm =
      pC->algo__crosstalk_detect_min_valid_range_mm;
    pHP->algo__crosstalk_detect_max_valid_rate_kcps =
      pC->algo__crosstalk_detect_max_valid_rate_kcps;
    pHP->algo__crosstalk_detect_max_sigma_mm =
      pC->algo__crosstalk_detect_max_sigma_mm;



    VL53LX_copy_rtn_good_spads_to_buffer(
      &(pdev->nvm_copy_data),
      &(pdev->rtn_good_spads[0]));



    switch (pdev->offset_correction_mode) {

      case VL53LX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS:
        tmp8 = pdev->gen_cfg.dss_config__aperture_attenuation;

        VL53LX_hist_combine_mm1_mm2_offsets(
          pN->mm_config__inner_offset_mm,
          pN->mm_config__outer_offset_mm,
          pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
          pdev->nvm_copy_data.roi_config__mode_roi_xy_size,
          pHD->roi_config__user_roi_centre_spad,
          pHD->roi_config__user_roi_requested_global_xy_size,
          &(pdev->add_off_cal_data),
          &(pdev->rtn_good_spads[0]),
          (uint16_t)tmp8,
          &(pHP->range_offset_mm));
        break;
      case VL53LX_OFFSETCORRECTIONMODE__PER_VCSEL_OFFSETS:
        select_offset_per_vcsel(
          pdev,
          &(pHP->range_offset_mm));
        pHP->range_offset_mm *= 4;
        break;
      default:
        pHP->range_offset_mm = 0;
        break;

    }



    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }


    VL53LX_calc_max_effective_spads(
      pHD->roi_config__user_roi_centre_spad,
      pHD->roi_config__user_roi_requested_global_xy_size,
      &(pdev->rtn_good_spads[0]),
      (uint16_t)pdev->gen_cfg.dss_config__aperture_attenuation,
      &(pdev->dmax_cfg.max_effective_spads));

    status =
      VL53LX_get_dmax_calibration_data(
        pdev->dmax_mode,
        pdmax_cal);


    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }

    status = VL53LX_ipp_hist_process_data(
               pdmax_cal,
               &(pdev->dmax_cfg),
               &(pdev->histpostprocess),
               &(pdev->hist_data),
               &(pdev->xtalk_shapes),
               pdev->wArea1,
               pdev->wArea2,
               &histo_merge_nb,
               presults);

    if ((pdev->tuning_parms.tp_hist_merge == 1) &&
        (histo_merge_nb > 1))
      for (i = 0; i < VL53LX_MAX_RANGE_RESULTS; i++) {
        pdata = &(presults->VL53LX_p_003[i]);
        pdata->VL53LX_p_016 /= histo_merge_nb;
        pdata->VL53LX_p_017 /= histo_merge_nb;
        pdata->VL53LX_p_010 /= histo_merge_nb;
        pdata->peak_signal_count_rate_mcps /= histo_merge_nb;
        pdata->avg_signal_count_rate_mcps /= histo_merge_nb;
        pdata->ambient_count_rate_mcps /= histo_merge_nb;
        pdata->VL53LX_p_009 /= histo_merge_nb;
      }


    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }

    status = VL53LX_hist_wrap_dmax(
               &(pdev->histpostprocess),
               &(pdev->hist_data),
               &(presults->wrap_dmax_mm));


    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }

    zid = pdev->ll_state.rd_zone_id;
    status = VL53LX_hist_phase_consistency_check(
               &(pZH->VL53LX_p_003[zid]),
               &(pres->zone_results.VL53LX_p_003[zid]),
               presults);


    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }

    zid = pdev->ll_state.rd_zone_id;
    status = VL53LX_hist_xmonitor_consistency_check(
               &(pZH->VL53LX_p_003[zid]),
               &(pres->zone_results.VL53LX_p_003[zid]),
               &(presults->xmonitor));


    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }


    zid = pdev->ll_state.rd_zone_id;
    pZH->max_zones    = VL53LX_MAX_USER_ZONES;
    pZH->active_zones =
      pdev->zone_cfg.active_zones + 1;
    pHD->zone_id       = zid;

    if (zid <
        pres->zone_results.max_zones) {

      phist_info =
        &(pZH->VL53LX_p_003[zid]);

      phist_info->rd_device_state =
        pHD->rd_device_state;

      phist_info->number_of_ambient_bins =
        pHD->number_of_ambient_bins;

      phist_info->result__dss_actual_effective_spads =
        pHD->result__dss_actual_effective_spads;

      phist_info->VL53LX_p_005 =
        pHD->VL53LX_p_005;

      phist_info->total_periods_elapsed =
        pHD->total_periods_elapsed;

      phist_info->ambient_events_sum =
        pHD->ambient_events_sum;
    }



    if (status != VL53LX_ERROR_NONE) {
      goto UPDATE_DYNAMIC_CONFIG;
    }

    VL53LX_hist_copy_results_to_sys_and_core(
      &(pdev->hist_data),
      presults,
      &(pdev->sys_results),
      &(pdev->core_results));


UPDATE_DYNAMIC_CONFIG:
    if (pzone_cfg->active_zones > 0) {
      if (pstate->rd_device_state !=
          VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {
        if (status == VL53LX_ERROR_NONE) {
          status = VL53LX_dynamic_zone_update(presults);
        }
      }


      for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
        pzone_cfg->bin_config[i] =
          ((pdev->ll_state.cfg_internal_stream_count)
           & 0x01) ?
          VL53LX_ZONECONFIG_BINCONFIG__HIGHAMB :
          VL53LX_ZONECONFIG_BINCONFIG__LOWAMB;
      }

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_multizone_hist_bins_update();
      }

    }



    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_dynamic_xtalk_correction_corrector();
    }
    /*
    #ifdef VL53LX_LOG_ENABLE
        if (status == VL53LX_ERROR_NONE)
          VL53LX_print_histogram_bin_data(
            &(pdev->hist_data),
            "get_device_results():pdev->lldata.hist_data.",
            VL53LX_TRACE_MODULE_HISTOGRAM_DATA);
    #endif
    */
  } else {

    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_get_measurement_results(
                 device_results_level);

    if (status == VL53LX_ERROR_NONE)
      VL53LX_copy_sys_and_core_results_to_range_results(
        (int32_t)pdev->gain_cal.standard_ranging_gain_factor,
        &(pdev->sys_results),
        &(pdev->core_results),
        presults);



    if (pL->is_low_power_auto_mode == 1) {

      if ((status == VL53LX_ERROR_NONE) &&
          (pL->low_power_auto_range_count == 0)) {

        status =
          VL53LX_low_power_auto_setup_manual_calibration();
        pL->low_power_auto_range_count = 1;
      } else if ((status == VL53LX_ERROR_NONE) &&
                 (pL->low_power_auto_range_count == 1)) {
        pL->low_power_auto_range_count = 2;
      }


      if ((pL->low_power_auto_range_count != 0xFF) &&
          (status == VL53LX_ERROR_NONE)) {
        status = VL53LX_low_power_auto_update_DSS();
      }
    }

  }


  presults->cfg_device_state = pdev->ll_state.cfg_device_state;
  presults->rd_device_state  = pdev->ll_state.rd_device_state;
  presults->zone_id          = pdev->ll_state.rd_zone_id;

  if (status == VL53LX_ERROR_NONE) {


    pres->zone_results.max_zones    = VL53LX_MAX_USER_ZONES;
    pres->zone_results.active_zones = pdev->zone_cfg.active_zones + 1;
    zid = pdev->ll_state.rd_zone_id;

    if (zid < pres->zone_results.max_zones) {

      pobjects =
        &(pres->zone_results.VL53LX_p_003[zid]);

      pobjects->cfg_device_state  =
        presults->cfg_device_state;
      pobjects->rd_device_state   = presults->rd_device_state;
      pobjects->zone_id           = presults->zone_id;
      pobjects->stream_count      = presults->stream_count;



      pobjects->xmonitor.VL53LX_p_016 =
        presults->xmonitor.VL53LX_p_016;
      pobjects->xmonitor.VL53LX_p_017 =
        presults->xmonitor.VL53LX_p_017;
      pobjects->xmonitor.VL53LX_p_011 =
        presults->xmonitor.VL53LX_p_011;
      pobjects->xmonitor.range_status =
        presults->xmonitor.range_status;

      pobjects->max_objects      = presults->max_results;
      pobjects->active_objects   = presults->active_results;

      for (i = 0; i < presults->active_results; i++) {
        pobjects->VL53LX_p_003[i].VL53LX_p_016 =
          presults->VL53LX_p_003[i].VL53LX_p_016;
        pobjects->VL53LX_p_003[i].VL53LX_p_017 =
          presults->VL53LX_p_003[i].VL53LX_p_017;
        pobjects->VL53LX_p_003[i].VL53LX_p_011 =
          presults->VL53LX_p_003[i].VL53LX_p_011;
        pobjects->VL53LX_p_003[i].range_status =
          presults->VL53LX_p_003[i].range_status;
      }


    }
  }



  memcpy(
    prange_results,
    presults,
    sizeof(VL53LX_range_results_t));



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_check_ll_driver_rd_state();
  }
  /*
  #ifdef VL53LX_LOG_ENABLE
    if (status == VL53LX_ERROR_NONE)
      VL53LX_print_range_results(
        presults,
        "get_device_results():pdev->llresults.range_results.",
        VL53LX_TRACE_MODULE_RANGE_RESULTS_DATA);
  #endif
  */

  return status;
}

VL53LX_Error VL53LX::VL53LX_clear_interrupt_and_enable_next_range(
  uint8_t           measurement_mode)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_and_start_range(
               measurement_mode,
               VL53LX_DEVICECONFIGLEVEL_GENERAL_ONWARDS);


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_histogram_bin_data(
  VL53LX_histogram_bin_data_t *pdata)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_zone_private_dyn_cfg_t *pzone_dyn_cfg;

  VL53LX_static_nvm_managed_t   *pstat_nvm = &(pdev->stat_nvm);
  VL53LX_static_config_t        *pstat_cfg = &(pdev->stat_cfg);
  VL53LX_general_config_t       *pgen_cfg  = &(pdev->gen_cfg);
  VL53LX_timing_config_t        *ptim_cfg  = &(pdev->tim_cfg);
  VL53LX_range_results_t        *presults  = &(pres->range_results);

  uint8_t    buffer[VL53LX_MAX_I2C_XFER_SIZE];
  uint8_t   *pbuffer = &buffer[0];
  uint8_t    bin_23_0 = 0x00;
  uint16_t   bin                      = 0;
  uint16_t   i2c_buffer_offset_bytes  = 0;
  uint16_t   encoded_timeout          = 0;

  uint32_t   pll_period_us            = 0;
  uint32_t   periods_elapsed_tmp      = 0;

  uint8_t    i                        = 0;

  int32_t    hist_merge       = 0;



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX,
               pbuffer,
               VL53LX_HISTOGRAM_BIN_DATA_I2C_SIZE_BYTES);



  pdata->result__interrupt_status               = *(pbuffer +   0);
  pdata->result__range_status                   = *(pbuffer +   1);
  pdata->result__report_status                  = *(pbuffer +   2);
  pdata->result__stream_count                   = *(pbuffer +   3);
  pdata->result__dss_actual_effective_spads =
    VL53LX_i2c_decode_uint16_t(2, pbuffer +   4);



  i2c_buffer_offset_bytes =
    VL53LX_PHASECAL_RESULT__REFERENCE_PHASE -
    VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

  pbuffer = &buffer[i2c_buffer_offset_bytes];

  pdata->phasecal_result__reference_phase =
    VL53LX_i2c_decode_uint16_t(2, pbuffer);

  i2c_buffer_offset_bytes =
    VL53LX_PHASECAL_RESULT__VCSEL_START -
    VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

  pdata->phasecal_result__vcsel_start = buffer[i2c_buffer_offset_bytes];



  pdev->dbg_results.phasecal_result__reference_phase =
    pdata->phasecal_result__reference_phase;
  pdev->dbg_results.phasecal_result__vcsel_start =
    pdata->phasecal_result__vcsel_start;



  i2c_buffer_offset_bytes =
    VL53LX_RESULT__HISTOGRAM_BIN_23_0_MSB -
    VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

  bin_23_0 = buffer[i2c_buffer_offset_bytes] << 2;

  i2c_buffer_offset_bytes =
    VL53LX_RESULT__HISTOGRAM_BIN_23_0_LSB -
    VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

  bin_23_0 += buffer[i2c_buffer_offset_bytes];

  i2c_buffer_offset_bytes =
    VL53LX_RESULT__HISTOGRAM_BIN_23_0 -
    VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

  buffer[i2c_buffer_offset_bytes] = bin_23_0;



  i2c_buffer_offset_bytes =
    VL53LX_RESULT__HISTOGRAM_BIN_0_2 -
    VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

  pbuffer = &buffer[i2c_buffer_offset_bytes];
  for (bin = 0; bin < VL53LX_HISTOGRAM_BUFFER_SIZE; bin++) {
    pdata->bin_data[bin] =
      (int32_t)VL53LX_i2c_decode_uint32_t(3, pbuffer);
    pbuffer += 3;
  }




  VL53LX_get_tuning_parm(VL53LX_TUNINGPARM_HIST_MERGE, &hist_merge);

  if (pdata->result__stream_count == 0) {

    memset(pdev->multi_bins_rec, 0, sizeof(pdev->multi_bins_rec));
    pdev->bin_rec_pos = 0;
    pdev->pos_before_next_recom = 0;
  }

  if (hist_merge == 1) {
    vl53lx_histo_merge(pdata);
  }


  pdata->zone_id                 = pdev->ll_state.rd_zone_id;
  pdata->VL53LX_p_019               = 0;
  pdata->VL53LX_p_020             = VL53LX_HISTOGRAM_BUFFER_SIZE;
  pdata->VL53LX_p_021          = VL53LX_HISTOGRAM_BUFFER_SIZE;

  pdata->cal_config__vcsel_start = pgen_cfg->cal_config__vcsel_start;



  pdata->vcsel_width =
    ((uint16_t)pgen_cfg->global_config__vcsel_width) << 4;
  pdata->vcsel_width +=
    (uint16_t)pstat_cfg->ana_config__vcsel_pulse_width_offset;


  pdata->VL53LX_p_015 =
    pstat_nvm->osc_measured__fast_osc__frequency;



  VL53LX_hist_get_bin_sequence_config(pdata);



  if (pdev->ll_state.rd_timing_status == 0) {

    encoded_timeout =
      (ptim_cfg->range_config__timeout_macrop_a_hi << 8)
      + ptim_cfg->range_config__timeout_macrop_a_lo;
    pdata->VL53LX_p_005 =  ptim_cfg->range_config__vcsel_period_a;
  } else {

    encoded_timeout =
      (ptim_cfg->range_config__timeout_macrop_b_hi << 8)
      + ptim_cfg->range_config__timeout_macrop_b_lo;
    pdata->VL53LX_p_005 = ptim_cfg->range_config__vcsel_period_b;
  }



  pdata->number_of_ambient_bins  = 0;

  for (i = 0; i < 6; i++) {
    if ((pdata->bin_seq[i] & 0x07) == 0x07)
      pdata->number_of_ambient_bins  =
        pdata->number_of_ambient_bins + 0x04;
  }

  pdata->total_periods_elapsed =
    VL53LX_decode_timeout(encoded_timeout);




  pll_period_us =
    VL53LX_calc_pll_period_us(pdata->VL53LX_p_015);



  periods_elapsed_tmp = pdata->total_periods_elapsed + 1;



  pdata->peak_duration_us =
    VL53LX_duration_maths(
      pll_period_us,
      (uint32_t)pdata->vcsel_width,
      VL53LX_RANGING_WINDOW_VCSEL_PERIODS,
      periods_elapsed_tmp);

  pdata->woi_duration_us     = 0;



  VL53LX_hist_calc_zero_distance_phase(pdata);



  VL53LX_hist_estimate_ambient_from_ambient_bins(pdata);



  pdata->cfg_device_state = pdev->ll_state.cfg_device_state;
  pdata->rd_device_state  = pdev->ll_state.rd_device_state;



  pzone_dyn_cfg = &(pres->zone_dyn_cfgs.VL53LX_p_003[pdata->zone_id]);

  pdata->roi_config__user_roi_centre_spad =
    pzone_dyn_cfg->roi_config__user_roi_centre_spad;
  pdata->roi_config__user_roi_requested_global_xy_size =
    pzone_dyn_cfg->roi_config__user_roi_requested_global_xy_size;



  presults->device_status = VL53LX_DEVICEERROR_NOUPDATE;



  switch (pdata->result__range_status &
          VL53LX_RANGE_STATUS__RANGE_STATUS_MASK) {

    case VL53LX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53LX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53LX_DEVICEERROR_NOVHVVALUEFOUND:
    case VL53LX_DEVICEERROR_USERROICLIP:
    case VL53LX_DEVICEERROR_MULTCLIPFAIL:

      presults->device_status = (pdata->result__range_status &
                                 VL53LX_RANGE_STATUS__RANGE_STATUS_MASK);

      status = VL53LX_ERROR_RANGE_ERROR;

      break;

  }


  return status;
}

void VL53LX::VL53LX_copy_sys_and_core_results_to_range_results(
  int32_t                           gain_factor,
  VL53LX_system_results_t          *psys,
  VL53LX_core_results_t            *pcore,
  VL53LX_range_results_t           *presults)
{
  uint8_t  i = 0;

  VL53LX_range_data_t *pdata;
  int32_t range_mm = 0;
  uint32_t tmpu32 = 0;
  uint16_t rpscr_crosstalk_corrected_mcps_sd0;
  uint16_t rmmo_effective_spads_sd0;
  uint16_t rmmi_effective_spads_sd0;




  presults->zone_id         = 0;
  presults->stream_count    = psys->result__stream_count;
  presults->wrap_dmax_mm    = 0;
  presults->max_results     = VL53LX_MAX_RANGE_RESULTS;
  presults->active_results  = 1;
  rpscr_crosstalk_corrected_mcps_sd0 =
    psys->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
  rmmo_effective_spads_sd0 =
    psys->result__mm_outer_actual_effective_spads_sd0;
  rmmi_effective_spads_sd0 =
    psys->result__mm_inner_actual_effective_spads_sd0;


  for (i = 0; i < VL53LX_MAX_AMBIENT_DMAX_VALUES; i++) {
    presults->VL53LX_p_022[i] = 0;
  }

  pdata = &(presults->VL53LX_p_003[0]);

  for (i = 0; i < 2; i++) {

    pdata->range_id     = i;
    pdata->time_stamp   = 0;

    if ((psys->result__stream_count == 0) &&
        ((psys->result__range_status &
          VL53LX_RANGE_STATUS__RANGE_STATUS_MASK) ==
         VL53LX_DEVICEERROR_RANGECOMPLETE)) {
      pdata->range_status =
        VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK;
    } else {
      pdata->range_status =
        psys->result__range_status &
        VL53LX_RANGE_STATUS__RANGE_STATUS_MASK;
    }

    pdata->VL53LX_p_012 = 0;
    pdata->VL53LX_p_019    = 0;
    pdata->VL53LX_p_023   = 0;
    pdata->VL53LX_p_024     = 0;
    pdata->VL53LX_p_013   = 0;
    pdata->VL53LX_p_025    = 0;

    switch (i) {

      case 0:
        if (psys->result__report_status ==
            VL53LX_DEVICEREPORTSTATUS_MM1)
          pdata->VL53LX_p_004 =
            rmmi_effective_spads_sd0;
        else if (psys->result__report_status ==
                 VL53LX_DEVICEREPORTSTATUS_MM2)
          pdata->VL53LX_p_004 =
            rmmo_effective_spads_sd0;
        else
          pdata->VL53LX_p_004 =
            psys->result__dss_actual_effective_spads_sd0;

        pdata->peak_signal_count_rate_mcps =
          rpscr_crosstalk_corrected_mcps_sd0;
        pdata->avg_signal_count_rate_mcps =
          psys->result__avg_signal_count_rate_mcps_sd0;
        pdata->ambient_count_rate_mcps =
          psys->result__ambient_count_rate_mcps_sd0;




        tmpu32 = ((uint32_t)psys->result__sigma_sd0 << 5);
        if (tmpu32 > 0xFFFF) {
          tmpu32 = 0xFFFF;
        }

        pdata->VL53LX_p_002 = (uint16_t)tmpu32;



        pdata->VL53LX_p_011 =
          psys->result__phase_sd0;

        range_mm = (int32_t)(
                     psys->result__final_crosstalk_corrected_range_mm_sd0);


        range_mm *= gain_factor;
        range_mm += 0x0400;
        range_mm /= 0x0800;

        pdata->median_range_mm = (int16_t)range_mm;

        pdata->VL53LX_p_017 =
          pcore->result_core__ranging_total_events_sd0;
        pdata->VL53LX_p_010 =
          pcore->result_core__signal_total_events_sd0;
        pdata->total_periods_elapsed =
          pcore->result_core__total_periods_elapsed_sd0;
        pdata->VL53LX_p_016 =
          pcore->result_core__ambient_window_events_sd0;

        break;
      case 1:

        pdata->VL53LX_p_004 =
          psys->result__dss_actual_effective_spads_sd1;
        pdata->peak_signal_count_rate_mcps =
          psys->result__peak_signal_count_rate_mcps_sd1;
        pdata->avg_signal_count_rate_mcps =
          0xFFFF;
        pdata->ambient_count_rate_mcps =
          psys->result__ambient_count_rate_mcps_sd1;




        tmpu32 = ((uint32_t)psys->result__sigma_sd1 << 5);
        if (tmpu32 > 0xFFFF) {
          tmpu32 = 0xFFFF;
        }

        pdata->VL53LX_p_002 = (uint16_t)tmpu32;



        pdata->VL53LX_p_011 =
          psys->result__phase_sd1;

        range_mm = (int32_t)(
                     psys->result__final_crosstalk_corrected_range_mm_sd1);


        range_mm *= gain_factor;
        range_mm += 0x0400;
        range_mm /= 0x0800;

        pdata->median_range_mm = (int16_t)range_mm;

        pdata->VL53LX_p_017 =
          pcore->result_core__ranging_total_events_sd1;
        pdata->VL53LX_p_010 =
          pcore->result_core__signal_total_events_sd1;
        pdata->total_periods_elapsed  =
          pcore->result_core__total_periods_elapsed_sd1;
        pdata->VL53LX_p_016 =
          pcore->result_core__ambient_window_events_sd1;

        break;
    }


    pdata->VL53LX_p_026    = pdata->VL53LX_p_011;
    pdata->VL53LX_p_027    = pdata->VL53LX_p_011;
    pdata->min_range_mm = pdata->median_range_mm;
    pdata->max_range_mm = pdata->median_range_mm;

    pdata++;
  }



  presults->device_status = VL53LX_DEVICEERROR_NOUPDATE;



  switch (psys->result__range_status &
          VL53LX_RANGE_STATUS__RANGE_STATUS_MASK) {

    case VL53LX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53LX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53LX_DEVICEERROR_NOVHVVALUEFOUND:
    case VL53LX_DEVICEERROR_USERROICLIP:
    case VL53LX_DEVICEERROR_MULTCLIPFAIL:

      presults->device_status = (psys->result__range_status &
                                 VL53LX_RANGE_STATUS__RANGE_STATUS_MASK);

      presults->VL53LX_p_003[0].range_status =
        VL53LX_DEVICEERROR_NOUPDATE;
      break;

  }

}

VL53LX_Error VL53LX::VL53LX_set_zone_dss_config(
  VL53LX_zone_private_dyn_cfg_t  *pzone_dyn_cfg)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);

  if (pstate->cfg_device_state ==
      VL53LX_DEVICESTATE_RANGING_DSS_MANUAL) {
    pdev->gen_cfg.dss_config__roi_mode_control =
      VL53LX_DSS_CONTROL__MODE_EFFSPADS;
    pdev->gen_cfg.dss_config__manual_effective_spads_select =
      pzone_dyn_cfg->dss_requested_effective_spad_count;
  } else {
    pdev->gen_cfg.dss_config__roi_mode_control =
      VL53LX_DSS_CONTROL__MODE_TARGET_RATE;
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_calc_ambient_dmax(
  uint16_t        target_reflectance,
  int16_t         *pambient_dmax_mm)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_dmax_calibration_data_t   dmax_cal;
  VL53LX_dmax_calibration_data_t *pdmax_cal = &dmax_cal;

  status =
    VL53LX_get_dmax_calibration_data(
      pdev->debug_mode,
      pdmax_cal);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_ipp_hist_ambient_dmax(
        target_reflectance,
        &(pdev->fmt_dmax_cal),
        &(pdev->dmax_cfg),
        &(pdev->hist_data),
        pambient_dmax_mm);

  return status;
}


VL53LX_Error VL53LX::VL53LX_set_GPIO_interrupt_config(
  VL53LX_GPIO_Interrupt_Mode  intr_mode_distance,
  VL53LX_GPIO_Interrupt_Mode  intr_mode_rate,
  uint8_t       intr_new_measure_ready,
  uint8_t       intr_no_target,
  uint8_t       intr_combined_mode,
  uint16_t      thresh_distance_high,
  uint16_t      thresh_distance_low,
  uint16_t      thresh_rate_high,
  uint16_t      thresh_rate_low
)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_GPIO_interrupt_config_t *pintconf =
    &(pdev->gpio_interrupt_config);

  pintconf->intr_mode_distance = intr_mode_distance;
  pintconf->intr_mode_rate = intr_mode_rate;
  pintconf->intr_new_measure_ready = intr_new_measure_ready;
  pintconf->intr_no_target = intr_no_target;
  pintconf->intr_combined_mode = intr_combined_mode;
  pintconf->threshold_distance_high = thresh_distance_high;
  pintconf->threshold_distance_low = thresh_distance_low;
  pintconf->threshold_rate_high = thresh_rate_high;
  pintconf->threshold_rate_low = thresh_rate_low;


  pdev->gen_cfg.system__interrupt_config_gpio =
    VL53LX_encode_GPIO_interrupt_config(pintconf);



  status = VL53LX_set_GPIO_thresholds_from_struct(
             pintconf);

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_GPIO_interrupt_config_struct(
  VL53LX_GPIO_interrupt_config_t  intconf)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_GPIO_interrupt_config_t *pintconf =
    &(pdev->gpio_interrupt_config);
  memcpy(pintconf, &(intconf), sizeof(VL53LX_GPIO_interrupt_config_t));


  pdev->gen_cfg.system__interrupt_config_gpio =
    VL53LX_encode_GPIO_interrupt_config(pintconf);


  status = VL53LX_set_GPIO_thresholds_from_struct(
             pintconf);

  return status;
}


VL53LX_Error VL53LX::VL53LX_get_GPIO_interrupt_config(
  VL53LX_GPIO_interrupt_config_t  *pintconf)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->gpio_interrupt_config = VL53LX_decode_GPIO_interrupt_config(
                                  pdev->gen_cfg.system__interrupt_config_gpio);


  pdev->gpio_interrupt_config.threshold_distance_high =
    pdev->dyn_cfg.system__thresh_high;
  pdev->gpio_interrupt_config.threshold_distance_low =
    pdev->dyn_cfg.system__thresh_low;

  pdev->gpio_interrupt_config.threshold_rate_high =
    pdev->gen_cfg.system__thresh_rate_high;
  pdev->gpio_interrupt_config.threshold_rate_low =
    pdev->gen_cfg.system__thresh_rate_low;

  if (pintconf == &(pdev->gpio_interrupt_config)) {

  } else {


    memcpy(pintconf, &(pdev->gpio_interrupt_config),
           sizeof(VL53LX_GPIO_interrupt_config_t));
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_set_dmax_mode(
  VL53LX_DeviceDmaxMode    dmax_mode)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->dmax_mode = dmax_mode;

  return status;
}
VL53LX_Error VL53LX::VL53LX_get_dmax_mode(
  VL53LX_DeviceDmaxMode   *pdmax_mode)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *pdmax_mode = pdev->dmax_mode;


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_dmax_calibration_data(
  VL53LX_DeviceDmaxMode           dmax_mode,
  VL53LX_dmax_calibration_data_t *pdmax_cal)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t    *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);


  switch (dmax_mode) {

    case VL53LX_DEVICEDMAXMODE__CUST_CAL_DATA:
      memcpy(
        pdmax_cal,
        &(pdev->cust_dmax_cal),
        sizeof(VL53LX_dmax_calibration_data_t));
      break;

    case VL53LX_DEVICEDMAXMODE__FMT_CAL_DATA:
      memcpy(
        pdmax_cal,
        &(pdev->fmt_dmax_cal),
        sizeof(VL53LX_dmax_calibration_data_t));
      break;

    default:
      status = VL53LX_ERROR_INVALID_PARAMS;
      break;

  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_hist_dmax_config(
  VL53LX_hist_gen3_dmax_config_t *pdmax_cfg)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  memcpy(
    &(pdev->dmax_cfg),
    pdmax_cfg,
    sizeof(VL53LX_hist_gen3_dmax_config_t));


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_hist_dmax_config(
  VL53LX_hist_gen3_dmax_config_t *pdmax_cfg)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  memcpy(
    pdmax_cfg,
    &(pdev->dmax_cfg),
    sizeof(VL53LX_hist_gen3_dmax_config_t));


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_offset_calibration_mode(
  VL53LX_OffsetCalibrationMode   offset_cal_mode)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->offset_calibration_mode = offset_cal_mode;

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_offset_calibration_mode(
  VL53LX_OffsetCalibrationMode  *poffset_cal_mode)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  *poffset_cal_mode = pdev->offset_calibration_mode;

  return status;
}


VL53LX_Error VL53LX::VL53LX_set_offset_correction_mode(
  VL53LX_OffsetCorrectionMode    offset_cor_mode)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->offset_correction_mode = offset_cor_mode;


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_offset_correction_mode(
  VL53LX_OffsetCorrectionMode   *poffset_cor_mode)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  *poffset_cor_mode = pdev->offset_correction_mode;


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_zone_calibration_data(
  VL53LX_zone_calibration_results_t  *pzone_cal)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);


  if (pzone_cal->struct_version !=
      VL53LX_LL_ZONE_CALIBRATION_DATA_STRUCT_VERSION) {
    status = VL53LX_ERROR_INVALID_PARAMS;
  }


  if (status == VL53LX_ERROR_NONE)

    memcpy(
      &(pres->zone_cal),
      pzone_cal,
      sizeof(VL53LX_zone_calibration_results_t));


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_zone_calibration_data(
  VL53LX_zone_calibration_results_t  *pzone_cal)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);

  memcpy(
    pzone_cal,
    &(pres->zone_cal),
    sizeof(VL53LX_zone_calibration_results_t));

  pzone_cal->struct_version =
    VL53LX_LL_ZONE_CALIBRATION_DATA_STRUCT_VERSION;


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_tuning_debug_data(
  VL53LX_tuning_parameters_t           *ptun_data)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_xtalkextract_config_t *pXC = &(pdev->xtalk_extract_cfg);


  ptun_data->vl53lx_tuningparm_version =
    pdev->tuning_parms.tp_tuning_parm_version;

  ptun_data->vl53lx_tuningparm_key_table_version =
    pdev->tuning_parms.tp_tuning_parm_key_table_version;


  ptun_data->vl53lx_tuningparm_lld_version =
    pdev->tuning_parms.tp_tuning_parm_lld_version;

  ptun_data->vl53lx_tuningparm_hist_algo_select =
    pHP->hist_algo_select;

  ptun_data->vl53lx_tuningparm_hist_target_order =
    pHP->hist_target_order;

  ptun_data->vl53lx_tuningparm_hist_filter_woi_0 =
    pHP->filter_woi0;

  ptun_data->vl53lx_tuningparm_hist_filter_woi_1 =
    pHP->filter_woi1;

  ptun_data->vl53lx_tuningparm_hist_amb_est_method =
    pHP->hist_amb_est_method;

  ptun_data->vl53lx_tuningparm_hist_amb_thresh_sigma_0 =
    pHP->ambient_thresh_sigma0;

  ptun_data->vl53lx_tuningparm_hist_amb_thresh_sigma_1 =
    pHP->ambient_thresh_sigma1;

  ptun_data->vl53lx_tuningparm_hist_min_amb_thresh_events =
    pHP->min_ambient_thresh_events;

  ptun_data->vl53lx_tuningparm_hist_amb_events_scaler =
    pHP->ambient_thresh_events_scaler;

  ptun_data->vl53lx_tuningparm_hist_noise_threshold =
    pHP->noise_threshold;

  ptun_data->vl53lx_tuningparm_hist_signal_total_events_limit =
    pHP->signal_total_events_limit;

  ptun_data->vl53lx_tuningparm_hist_sigma_est_ref_mm =
    pHP->sigma_estimator__sigma_ref_mm;

  ptun_data->vl53lx_tuningparm_hist_sigma_thresh_mm =
    pHP->sigma_thresh;

  ptun_data->vl53lx_tuningparm_hist_gain_factor =
    pdev->gain_cal.histogram_ranging_gain_factor;

  ptun_data->vl53lx_tuningparm_consistency_hist_phase_tolerance =
    pHP->algo__consistency_check__phase_tolerance;

  ptun_data->vl53lx_tuningparm_consistency_hist_min_max_tolerance_mm =
    pHP->algo__consistency_check__min_max_tolerance;

  ptun_data->vl53lx_tuningparm_consistency_hist_event_sigma =
    pHP->algo__consistency_check__event_sigma;

  ptun_data->vl53lx_tuningparm_consistency_hist_event_sigma_min_spad_limit
    = pHP->algo__consistency_check__event_min_spad_count;

  ptun_data->vl53lx_tuningparm_initial_phase_rtn_histo_long_range =
    pdev->tuning_parms.tp_init_phase_rtn_hist_long;

  ptun_data->vl53lx_tuningparm_initial_phase_rtn_histo_med_range =
    pdev->tuning_parms.tp_init_phase_rtn_hist_med;

  ptun_data->vl53lx_tuningparm_initial_phase_rtn_histo_short_range =
    pdev->tuning_parms.tp_init_phase_rtn_hist_short;

  ptun_data->vl53lx_tuningparm_initial_phase_ref_histo_long_range =
    pdev->tuning_parms.tp_init_phase_ref_hist_long;

  ptun_data->vl53lx_tuningparm_initial_phase_ref_histo_med_range =
    pdev->tuning_parms.tp_init_phase_ref_hist_med;

  ptun_data->vl53lx_tuningparm_initial_phase_ref_histo_short_range =
    pdev->tuning_parms.tp_init_phase_ref_hist_short;

  ptun_data->vl53lx_tuningparm_xtalk_detect_min_valid_range_mm =
    pdev->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm;

  ptun_data->vl53lx_tuningparm_xtalk_detect_max_valid_range_mm =
    pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm;

  ptun_data->vl53lx_tuningparm_xtalk_detect_max_sigma_mm =
    pdev->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm;

  ptun_data->vl53lx_tuningparm_xtalk_detect_min_max_tolerance =
    pHP->algo__crosstalk_detect_min_max_tolerance;

  ptun_data->vl53lx_tuningparm_xtalk_detect_max_valid_rate_kcps =
    pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps;

  ptun_data->vl53lx_tuningparm_xtalk_detect_event_sigma =
    pHP->algo__crosstalk_detect_event_sigma;

  ptun_data->vl53lx_tuningparm_hist_xtalk_margin_kcps =
    pdev->xtalk_cfg.histogram_mode_crosstalk_margin_kcps;

  ptun_data->vl53lx_tuningparm_consistency_lite_phase_tolerance =
    pdev->tuning_parms.tp_consistency_lite_phase_tolerance;

  ptun_data->vl53lx_tuningparm_phasecal_target =
    pdev->tuning_parms.tp_phasecal_target;

  ptun_data->vl53lx_tuningparm_lite_cal_repeat_rate =
    pdev->tuning_parms.tp_cal_repeat_rate;

  ptun_data->vl53lx_tuningparm_lite_ranging_gain_factor =
    pdev->gain_cal.standard_ranging_gain_factor;

  ptun_data->vl53lx_tuningparm_lite_min_clip_mm =
    pdev->tuning_parms.tp_lite_min_clip;

  ptun_data->vl53lx_tuningparm_lite_long_sigma_thresh_mm =
    pdev->tuning_parms.tp_lite_long_sigma_thresh_mm;

  ptun_data->vl53lx_tuningparm_lite_med_sigma_thresh_mm =
    pdev->tuning_parms.tp_lite_med_sigma_thresh_mm;

  ptun_data->vl53lx_tuningparm_lite_short_sigma_thresh_mm =
    pdev->tuning_parms.tp_lite_short_sigma_thresh_mm;

  ptun_data->vl53lx_tuningparm_lite_long_min_count_rate_rtn_mcps =
    pdev->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps;

  ptun_data->vl53lx_tuningparm_lite_med_min_count_rate_rtn_mcps =
    pdev->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps;

  ptun_data->vl53lx_tuningparm_lite_short_min_count_rate_rtn_mcps =
    pdev->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps;

  ptun_data->vl53lx_tuningparm_lite_sigma_est_pulse_width =
    pdev->tuning_parms.tp_lite_sigma_est_pulse_width_ns;

  ptun_data->vl53lx_tuningparm_lite_sigma_est_amb_width_ns =
    pdev->tuning_parms.tp_lite_sigma_est_amb_width_ns;

  ptun_data->vl53lx_tuningparm_lite_sigma_ref_mm =
    pdev->tuning_parms.tp_lite_sigma_ref_mm;

  ptun_data->vl53lx_tuningparm_lite_rit_mult =
    pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult;

  ptun_data->vl53lx_tuningparm_lite_seed_config =
    pdev->tuning_parms.tp_lite_seed_cfg;

  ptun_data->vl53lx_tuningparm_lite_quantifier =
    pdev->tuning_parms.tp_lite_quantifier;

  ptun_data->vl53lx_tuningparm_lite_first_order_select =
    pdev->tuning_parms.tp_lite_first_order_select;

  ptun_data->vl53lx_tuningparm_lite_xtalk_margin_kcps =
    pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps;

  ptun_data->vl53lx_tuningparm_initial_phase_rtn_lite_long_range =
    pdev->tuning_parms.tp_init_phase_rtn_lite_long;

  ptun_data->vl53lx_tuningparm_initial_phase_rtn_lite_med_range =
    pdev->tuning_parms.tp_init_phase_rtn_lite_med;

  ptun_data->vl53lx_tuningparm_initial_phase_rtn_lite_short_range =
    pdev->tuning_parms.tp_init_phase_rtn_lite_short;

  ptun_data->vl53lx_tuningparm_initial_phase_ref_lite_long_range =
    pdev->tuning_parms.tp_init_phase_ref_lite_long;

  ptun_data->vl53lx_tuningparm_initial_phase_ref_lite_med_range =
    pdev->tuning_parms.tp_init_phase_ref_lite_med;

  ptun_data->vl53lx_tuningparm_initial_phase_ref_lite_short_range =
    pdev->tuning_parms.tp_init_phase_ref_lite_short;

  ptun_data->vl53lx_tuningparm_timed_seed_config =
    pdev->tuning_parms.tp_timed_seed_cfg;

  ptun_data->vl53lx_tuningparm_dmax_cfg_signal_thresh_sigma =
    pdev->dmax_cfg.signal_thresh_sigma;

  ptun_data->vl53lx_tuningparm_dmax_cfg_reflectance_array_0 =
    pdev->dmax_cfg.target_reflectance_for_dmax_calc[0];

  ptun_data->vl53lx_tuningparm_dmax_cfg_reflectance_array_1 =
    pdev->dmax_cfg.target_reflectance_for_dmax_calc[1];

  ptun_data->vl53lx_tuningparm_dmax_cfg_reflectance_array_2 =
    pdev->dmax_cfg.target_reflectance_for_dmax_calc[2];

  ptun_data->vl53lx_tuningparm_dmax_cfg_reflectance_array_3 =
    pdev->dmax_cfg.target_reflectance_for_dmax_calc[3];

  ptun_data->vl53lx_tuningparm_dmax_cfg_reflectance_array_4 =
    pdev->dmax_cfg.target_reflectance_for_dmax_calc[4];

  ptun_data->vl53lx_tuningparm_vhv_loopbound =
    pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;

  ptun_data->vl53lx_tuningparm_refspadchar_device_test_mode =
    pdev->refspadchar.device_test_mode;

  ptun_data->vl53lx_tuningparm_refspadchar_vcsel_period =
    pdev->refspadchar.VL53LX_p_005;

  ptun_data->vl53lx_tuningparm_refspadchar_phasecal_timeout_us =
    pdev->refspadchar.timeout_us;

  ptun_data->vl53lx_tuningparm_refspadchar_target_count_rate_mcps =
    pdev->refspadchar.target_count_rate_mcps;

  ptun_data->vl53lx_tuningparm_refspadchar_min_countrate_limit_mcps =
    pdev->refspadchar.min_count_rate_limit_mcps;

  ptun_data->vl53lx_tuningparm_refspadchar_max_countrate_limit_mcps =
    pdev->refspadchar.max_count_rate_limit_mcps;

  ptun_data->vl53lx_tuningparm_xtalk_extract_num_of_samples =
    pXC->num_of_samples;

  ptun_data->vl53lx_tuningparm_xtalk_extract_min_filter_thresh_mm =
    pXC->algo__crosstalk_extract_min_valid_range_mm;

  ptun_data->vl53lx_tuningparm_xtalk_extract_max_filter_thresh_mm =
    pXC->algo__crosstalk_extract_max_valid_range_mm;

  ptun_data->vl53lx_tuningparm_xtalk_extract_dss_rate_mcps =
    pXC->dss_config__target_total_rate_mcps;

  ptun_data->vl53lx_tuningparm_xtalk_extract_phasecal_timeout_us =
    pXC->phasecal_config_timeout_us;

  ptun_data->vl53lx_tuningparm_xtalk_extract_max_valid_rate_kcps =
    pXC->algo__crosstalk_extract_max_valid_rate_kcps;

  ptun_data->vl53lx_tuningparm_xtalk_extract_sigma_threshold_mm =
    pXC->algo__crosstalk_extract_max_sigma_mm;

  ptun_data->vl53lx_tuningparm_xtalk_extract_dss_timeout_us =
    pXC->mm_config_timeout_us;

  ptun_data->vl53lx_tuningparm_xtalk_extract_bin_timeout_us =
    pXC->range_config_timeout_us;

  ptun_data->vl53lx_tuningparm_offset_cal_dss_rate_mcps =
    pdev->offsetcal_cfg.dss_config__target_total_rate_mcps;

  ptun_data->vl53lx_tuningparm_offset_cal_phasecal_timeout_us =
    pdev->offsetcal_cfg.phasecal_config_timeout_us;

  ptun_data->vl53lx_tuningparm_offset_cal_mm_timeout_us =
    pdev->offsetcal_cfg.mm_config_timeout_us;

  ptun_data->vl53lx_tuningparm_offset_cal_range_timeout_us =
    pdev->offsetcal_cfg.range_config_timeout_us;

  ptun_data->vl53lx_tuningparm_offset_cal_pre_samples =
    pdev->offsetcal_cfg.pre_num_of_samples;

  ptun_data->vl53lx_tuningparm_offset_cal_mm1_samples =
    pdev->offsetcal_cfg.mm1_num_of_samples;

  ptun_data->vl53lx_tuningparm_offset_cal_mm2_samples =
    pdev->offsetcal_cfg.mm2_num_of_samples;

  ptun_data->vl53lx_tuningparm_zone_cal_dss_rate_mcps =
    pdev->zonecal_cfg.dss_config__target_total_rate_mcps;

  ptun_data->vl53lx_tuningparm_zone_cal_phasecal_timeout_us =
    pdev->zonecal_cfg.phasecal_config_timeout_us;

  ptun_data->vl53lx_tuningparm_zone_cal_dss_timeout_us =
    pdev->zonecal_cfg.mm_config_timeout_us;

  ptun_data->vl53lx_tuningparm_zone_cal_phasecal_num_samples =
    pdev->zonecal_cfg.phasecal_num_of_samples;

  ptun_data->vl53lx_tuningparm_zone_cal_range_timeout_us =
    pdev->zonecal_cfg.range_config_timeout_us;

  ptun_data->vl53lx_tuningparm_zone_cal_zone_num_samples =
    pdev->zonecal_cfg.zone_num_of_samples;

  ptun_data->vl53lx_tuningparm_spadmap_vcsel_period =
    pdev->ssc_cfg.VL53LX_p_005;

  ptun_data->vl53lx_tuningparm_spadmap_vcsel_start =
    pdev->ssc_cfg.vcsel_start;

  ptun_data->vl53lx_tuningparm_spadmap_rate_limit_mcps =
    pdev->ssc_cfg.rate_limit_mcps;

  ptun_data->vl53lx_tuningparm_lite_dss_config_target_total_rate_mcps =
    pdev->tuning_parms.tp_dss_target_lite_mcps;

  ptun_data->vl53lx_tuningparm_ranging_dss_config_target_total_rate_mcps =
    pdev->tuning_parms.tp_dss_target_histo_mcps;

  ptun_data->vl53lx_tuningparm_mz_dss_config_target_total_rate_mcps =
    pdev->tuning_parms.tp_dss_target_histo_mz_mcps;

  ptun_data->vl53lx_tuningparm_timed_dss_config_target_total_rate_mcps =
    pdev->tuning_parms.tp_dss_target_timed_mcps;

  ptun_data->vl53lx_tuningparm_lite_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_lite_us;

  ptun_data->vl53lx_tuningparm_ranging_long_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_hist_long_us;

  ptun_data->vl53lx_tuningparm_ranging_med_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_hist_med_us;

  ptun_data->vl53lx_tuningparm_ranging_short_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_hist_short_us;

  ptun_data->vl53lx_tuningparm_mz_long_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_mz_long_us;

  ptun_data->vl53lx_tuningparm_mz_med_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_mz_med_us;

  ptun_data->vl53lx_tuningparm_mz_short_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_mz_short_us;

  ptun_data->vl53lx_tuningparm_timed_phasecal_config_timeout_us =
    pdev->tuning_parms.tp_phasecal_timeout_timed_us;

  ptun_data->vl53lx_tuningparm_lite_mm_config_timeout_us =
    pdev->tuning_parms.tp_mm_timeout_lite_us;

  ptun_data->vl53lx_tuningparm_ranging_mm_config_timeout_us =
    pdev->tuning_parms.tp_mm_timeout_histo_us;

  ptun_data->vl53lx_tuningparm_mz_mm_config_timeout_us =
    pdev->tuning_parms.tp_mm_timeout_mz_us;

  ptun_data->vl53lx_tuningparm_timed_mm_config_timeout_us =
    pdev->tuning_parms.tp_mm_timeout_timed_us;

  ptun_data->vl53lx_tuningparm_lite_range_config_timeout_us =
    pdev->tuning_parms.tp_range_timeout_lite_us;

  ptun_data->vl53lx_tuningparm_ranging_range_config_timeout_us =
    pdev->tuning_parms.tp_range_timeout_histo_us;

  ptun_data->vl53lx_tuningparm_mz_range_config_timeout_us =
    pdev->tuning_parms.tp_range_timeout_mz_us;

  ptun_data->vl53lx_tuningparm_timed_range_config_timeout_us =
    pdev->tuning_parms.tp_range_timeout_timed_us;

  ptun_data->vl53lx_tuningparm_dynxtalk_smudge_margin =
    pdev->smudge_correct_config.smudge_margin;

  ptun_data->vl53lx_tuningparm_dynxtalk_noise_margin =
    pdev->smudge_correct_config.noise_margin;

  ptun_data->vl53lx_tuningparm_dynxtalk_xtalk_offset_limit =
    pdev->smudge_correct_config.user_xtalk_offset_limit;

  ptun_data->vl53lx_tuningparm_dynxtalk_xtalk_offset_limit_hi =
    pdev->smudge_correct_config.user_xtalk_offset_limit_hi;

  ptun_data->vl53lx_tuningparm_dynxtalk_sample_limit =
    pdev->smudge_correct_config.sample_limit;

  ptun_data->vl53lx_tuningparm_dynxtalk_single_xtalk_delta =
    pdev->smudge_correct_config.single_xtalk_delta;

  ptun_data->vl53lx_tuningparm_dynxtalk_averaged_xtalk_delta =
    pdev->smudge_correct_config.averaged_xtalk_delta;

  ptun_data->vl53lx_tuningparm_dynxtalk_clip_limit =
    pdev->smudge_correct_config.smudge_corr_clip_limit;

  ptun_data->vl53lx_tuningparm_dynxtalk_scaler_calc_method =
    pdev->smudge_correct_config.scaler_calc_method;

  ptun_data->vl53lx_tuningparm_dynxtalk_xgradient_scaler =
    pdev->smudge_correct_config.x_gradient_scaler;

  ptun_data->vl53lx_tuningparm_dynxtalk_ygradient_scaler =
    pdev->smudge_correct_config.y_gradient_scaler;

  ptun_data->vl53lx_tuningparm_dynxtalk_user_scaler_set =
    pdev->smudge_correct_config.user_scaler_set;

  ptun_data->vl53lx_tuningparm_dynxtalk_smudge_cor_single_apply =
    pdev->smudge_correct_config.smudge_corr_single_apply;

  ptun_data->vl53lx_tuningparm_dynxtalk_xtalk_amb_threshold =
    pdev->smudge_correct_config.smudge_corr_ambient_threshold;

  ptun_data->vl53lx_tuningparm_dynxtalk_nodetect_amb_threshold_kcps =
    pdev->smudge_correct_config.nodetect_ambient_threshold;

  ptun_data->vl53lx_tuningparm_dynxtalk_nodetect_sample_limit =
    pdev->smudge_correct_config.nodetect_sample_limit;

  ptun_data->vl53lx_tuningparm_dynxtalk_nodetect_xtalk_offset_kcps =
    pdev->smudge_correct_config.nodetect_xtalk_offset;

  ptun_data->vl53lx_tuningparm_dynxtalk_nodetect_min_range_mm =
    pdev->smudge_correct_config.nodetect_min_range_mm;

  ptun_data->vl53lx_tuningparm_lowpowerauto_vhv_loop_bound =
    pdev->low_power_auto_data.vhv_loop_bound;

  ptun_data->vl53lx_tuningparm_lowpowerauto_mm_config_timeout_us =
    pdev->tuning_parms.tp_mm_timeout_lpa_us;

  ptun_data->vl53lx_tuningparm_lowpowerauto_range_config_timeout_us =
    pdev->tuning_parms.tp_range_timeout_lpa_us;

  ptun_data->vl53lx_tuningparm_very_short_dss_rate_mcps =
    pdev->tuning_parms.tp_dss_target_very_short_mcps;

  ptun_data->vl53lx_tuningparm_phasecal_patch_power =
    pdev->tuning_parms.tp_phasecal_patch_power;


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_tuning_parm(
  VL53LX_TuningParms             tuning_parm_key,
  int32_t                       *ptuning_parm_value)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_xtalkextract_config_t *pXC = &(pdev->xtalk_extract_cfg);

  switch (tuning_parm_key) {

    case VL53LX_TUNINGPARM_VERSION:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_tuning_parm_version;
      break;
    case VL53LX_TUNINGPARM_KEY_TABLE_VERSION:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_tuning_parm_key_table_version;
      break;
    case VL53LX_TUNINGPARM_LLD_VERSION:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_tuning_parm_lld_version;
      break;
    case VL53LX_TUNINGPARM_HIST_ALGO_SELECT:
      *ptuning_parm_value =
        (int32_t)pHP->hist_algo_select;
      break;
    case VL53LX_TUNINGPARM_HIST_TARGET_ORDER:
      *ptuning_parm_value =
        (int32_t)pHP->hist_target_order;
      break;
    case VL53LX_TUNINGPARM_HIST_FILTER_WOI_0:
      *ptuning_parm_value =
        (int32_t)pHP->filter_woi0;
      break;
    case VL53LX_TUNINGPARM_HIST_FILTER_WOI_1:
      *ptuning_parm_value =
        (int32_t)pHP->filter_woi1;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_EST_METHOD:
      *ptuning_parm_value =
        (int32_t)pHP->hist_amb_est_method;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0:
      *ptuning_parm_value =
        (int32_t)pHP->ambient_thresh_sigma0;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1:
      *ptuning_parm_value =
        (int32_t)pHP->ambient_thresh_sigma1;
      break;
    case VL53LX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS:
      *ptuning_parm_value =
        (int32_t)pHP->min_ambient_thresh_events;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_EVENTS_SCALER:
      *ptuning_parm_value =
        (int32_t)pHP->ambient_thresh_events_scaler;
      break;
    case VL53LX_TUNINGPARM_HIST_NOISE_THRESHOLD:
      *ptuning_parm_value =
        (int32_t)pHP->noise_threshold;
      break;
    case VL53LX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT:
      *ptuning_parm_value =
        (int32_t)pHP->signal_total_events_limit;
      break;
    case VL53LX_TUNINGPARM_HIST_SIGMA_EST_REF_MM:
      *ptuning_parm_value =
        (int32_t)pHP->sigma_estimator__sigma_ref_mm;
      break;
    case VL53LX_TUNINGPARM_HIST_SIGMA_THRESH_MM:
      *ptuning_parm_value =
        (int32_t)pHP->sigma_thresh;
      break;
    case VL53LX_TUNINGPARM_HIST_GAIN_FACTOR:
      *ptuning_parm_value =
        (int32_t)pdev->gain_cal.histogram_ranging_gain_factor;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE:
      *ptuning_parm_value =
        (int32_t)pHP->algo__consistency_check__phase_tolerance;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM:
      *ptuning_parm_value =
        (int32_t)pHP->algo__consistency_check__min_max_tolerance;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA:
      *ptuning_parm_value =
        (int32_t)pHP->algo__consistency_check__event_sigma;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT:
      *ptuning_parm_value =
        (int32_t)pHP->algo__consistency_check__event_min_spad_count;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_rtn_hist_long;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_rtn_hist_med;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_rtn_hist_short;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_ref_hist_long;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_ref_hist_med;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_ref_hist_short;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM:
      *ptuning_parm_value = (int32_t)(
                              pdev->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm);
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM:
      *ptuning_parm_value = (int32_t)(
                              pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm);
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM:
      *ptuning_parm_value =
        (int32_t)pdev->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE:
      *ptuning_parm_value =
        (int32_t)pHP->algo__crosstalk_detect_min_max_tolerance;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS:
      *ptuning_parm_value = (int32_t)(
                              pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps);
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA:
      *ptuning_parm_value =
        (int32_t)pHP->algo__crosstalk_detect_event_sigma;
      break;
    case VL53LX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS:
      *ptuning_parm_value =
        (int32_t)pdev->xtalk_cfg.histogram_mode_crosstalk_margin_kcps;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_consistency_lite_phase_tolerance;
      break;
    case VL53LX_TUNINGPARM_PHASECAL_TARGET:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_target;
      break;
    case VL53LX_TUNINGPARM_LITE_CAL_REPEAT_RATE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_cal_repeat_rate;
      break;
    case VL53LX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR:
      *ptuning_parm_value =
        (int32_t)pdev->gain_cal.standard_ranging_gain_factor;
      break;
    case VL53LX_TUNINGPARM_LITE_MIN_CLIP_MM:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_min_clip;
      break;
    case VL53LX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_long_sigma_thresh_mm;
      break;
    case VL53LX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_med_sigma_thresh_mm;
      break;
    case VL53LX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_short_sigma_thresh_mm;
      break;
    case VL53LX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS:
      *ptuning_parm_value = (int32_t)(
                              pdev->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps);
      break;
    case VL53LX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps;
      break;
    case VL53LX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS:
      *ptuning_parm_value = (int32_t)(
                              pdev->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps);
      break;
    case VL53LX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_sigma_est_pulse_width_ns;
      break;
    case VL53LX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_sigma_est_amb_width_ns;
      break;
    case VL53LX_TUNINGPARM_LITE_SIGMA_REF_MM:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_sigma_ref_mm;
      break;
    case VL53LX_TUNINGPARM_LITE_RIT_MULT:
      *ptuning_parm_value =
        (int32_t)pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult;
      break;
    case VL53LX_TUNINGPARM_LITE_SEED_CONFIG:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_seed_cfg;
      break;
    case VL53LX_TUNINGPARM_LITE_QUANTIFIER:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_quantifier;
      break;
    case VL53LX_TUNINGPARM_LITE_FIRST_ORDER_SELECT:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_lite_first_order_select;
      break;
    case VL53LX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS:
      *ptuning_parm_value =
        (int32_t)pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_rtn_lite_long;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_rtn_lite_med;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_rtn_lite_short;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_ref_lite_long;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_ref_lite_med;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_init_phase_ref_lite_short;
      break;
    case VL53LX_TUNINGPARM_TIMED_SEED_CONFIG:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_timed_seed_cfg;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA:
      *ptuning_parm_value =
        (int32_t)pdev->dmax_cfg.signal_thresh_sigma;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0:
      *ptuning_parm_value =
        (int32_t)pdev->dmax_cfg.target_reflectance_for_dmax_calc[0];
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1:
      *ptuning_parm_value =
        (int32_t)pdev->dmax_cfg.target_reflectance_for_dmax_calc[1];
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2:
      *ptuning_parm_value =
        (int32_t)pdev->dmax_cfg.target_reflectance_for_dmax_calc[2];
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3:
      *ptuning_parm_value =
        (int32_t)pdev->dmax_cfg.target_reflectance_for_dmax_calc[3];
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4:
      *ptuning_parm_value =
        (int32_t)pdev->dmax_cfg.target_reflectance_for_dmax_calc[4];
      break;
    case VL53LX_TUNINGPARM_VHV_LOOPBOUND:
      *ptuning_parm_value =
        (int32_t)pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE:
      *ptuning_parm_value =
        (int32_t)pdev->refspadchar.device_test_mode;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD:
      *ptuning_parm_value =
        (int32_t)pdev->refspadchar.VL53LX_p_005;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->refspadchar.timeout_us;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->refspadchar.target_count_rate_mcps;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->refspadchar.min_count_rate_limit_mcps;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->refspadchar.max_count_rate_limit_mcps;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES:
      *ptuning_parm_value =
        (int32_t)pXC->num_of_samples;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM:
      *ptuning_parm_value =
        (int32_t)pXC->algo__crosstalk_extract_min_valid_range_mm;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM:
      *ptuning_parm_value =
        (int32_t)pXC->algo__crosstalk_extract_max_valid_range_mm;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pXC->dss_config__target_total_rate_mcps;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pXC->phasecal_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS:
      *ptuning_parm_value =
        (int32_t)pXC->algo__crosstalk_extract_max_valid_rate_kcps;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM:
      *ptuning_parm_value =
        (int32_t)pXC->algo__crosstalk_extract_max_sigma_mm;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pXC->mm_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pXC->range_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.dss_config__target_total_rate_mcps;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.phasecal_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.mm_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.range_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.pre_num_of_samples;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.mm1_num_of_samples;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES:
      *ptuning_parm_value =
        (int32_t)pdev->offsetcal_cfg.mm2_num_of_samples;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->zonecal_cfg.dss_config__target_total_rate_mcps;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->zonecal_cfg.phasecal_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->zonecal_cfg.mm_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES:
      *ptuning_parm_value =
        (int32_t)pdev->zonecal_cfg.phasecal_num_of_samples;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->zonecal_cfg.range_config_timeout_us;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES:
      *ptuning_parm_value =
        (int32_t)pdev->zonecal_cfg.zone_num_of_samples;
      break;
    case VL53LX_TUNINGPARM_SPADMAP_VCSEL_PERIOD:
      *ptuning_parm_value =
        (int32_t)pdev->ssc_cfg.VL53LX_p_005;
      break;
    case VL53LX_TUNINGPARM_SPADMAP_VCSEL_START:
      *ptuning_parm_value =
        (int32_t)pdev->ssc_cfg.vcsel_start;
      break;
    case VL53LX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->ssc_cfg.rate_limit_mcps;
      break;
    case VL53LX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_dss_target_lite_mcps;
      break;
    case VL53LX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_dss_target_histo_mcps;
      break;
    case VL53LX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_dss_target_histo_mz_mcps;
      break;
    case VL53LX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_dss_target_timed_mcps;
      break;
    case VL53LX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_lite_us;
      break;
    case VL53LX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_hist_long_us;
      break;
    case VL53LX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_hist_med_us;
      break;
    case VL53LX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_hist_short_us;
      break;
    case VL53LX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_mz_long_us;
      break;
    case VL53LX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_mz_med_us;
      break;
    case VL53LX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_mz_short_us;
      break;
    case VL53LX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_phasecal_timeout_timed_us;
      break;
    case VL53LX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_mm_timeout_lite_us;
      break;
    case VL53LX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_mm_timeout_histo_us;
      break;
    case VL53LX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_mm_timeout_mz_us;
      break;
    case VL53LX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_mm_timeout_timed_us;
      break;
    case VL53LX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_range_timeout_lite_us;
      break;
    case VL53LX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_range_timeout_histo_us;
      break;
    case VL53LX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_range_timeout_mz_us;
      break;
    case VL53LX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_range_timeout_timed_us;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.smudge_margin;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NOISE_MARGIN:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.noise_margin;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.user_xtalk_offset_limit;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.user_xtalk_offset_limit_hi;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.sample_limit;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.single_xtalk_delta;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.averaged_xtalk_delta;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_CLIP_LIMIT:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.smudge_corr_clip_limit;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SCALER_CALC_METHOD:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.scaler_calc_method;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.x_gradient_scaler;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.y_gradient_scaler;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_USER_SCALER_SET:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.user_scaler_set;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.smudge_corr_single_apply;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD:
      *ptuning_parm_value = (int32_t)(
                              pdev->smudge_correct_config.smudge_corr_ambient_threshold);
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.nodetect_ambient_threshold;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.nodetect_sample_limit;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.nodetect_xtalk_offset;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM:
      *ptuning_parm_value =
        (int32_t)pdev->smudge_correct_config.nodetect_min_range_mm;
      break;
    case VL53LX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND:
      *ptuning_parm_value =
        (int32_t)pdev->low_power_auto_data.vhv_loop_bound;
      break;
    case VL53LX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_mm_timeout_lpa_us;
      break;
    case VL53LX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_range_timeout_lpa_us;
      break;
    case VL53LX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS:
      *ptuning_parm_value =
        (int32_t)pdev->tuning_parms.tp_dss_target_very_short_mcps;
      break;
    case VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER:
      *ptuning_parm_value =
        (int32_t) pdev->tuning_parms.tp_phasecal_patch_power;
      break;
    case VL53LX_TUNINGPARM_HIST_MERGE:
      *ptuning_parm_value =
        (int32_t) pdev->tuning_parms.tp_hist_merge;
      break;
    case VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD:
      *ptuning_parm_value =
        (int32_t) pdev->tuning_parms.tp_reset_merge_threshold;
      break;
    case VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE:
      *ptuning_parm_value =
        (int32_t) pdev->tuning_parms.tp_hist_merge_max_size;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR:
      *ptuning_parm_value =
        pdev->smudge_correct_config.max_smudge_factor;
      break;

    default:
      *ptuning_parm_value = 0x7FFFFFFF;
      status = VL53LX_ERROR_INVALID_PARAMS;
      break;

  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_tuning_parm(
  VL53LX_TuningParms    tuning_parm_key,
  int32_t               tuning_parm_value)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
  VL53LX_xtalkextract_config_t *pXC = &(pdev->xtalk_extract_cfg);

  switch (tuning_parm_key) {

    case VL53LX_TUNINGPARM_VERSION:
      pdev->tuning_parms.tp_tuning_parm_version =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_KEY_TABLE_VERSION:
      pdev->tuning_parms.tp_tuning_parm_key_table_version =
        (uint16_t)tuning_parm_value;



      if ((uint16_t)tuning_parm_value
          != VL53LX_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT) {
        status = VL53LX_ERROR_TUNING_PARM_KEY_MISMATCH;
      }

      break;
    case VL53LX_TUNINGPARM_LLD_VERSION:
      pdev->tuning_parms.tp_tuning_parm_lld_version =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_ALGO_SELECT:
      pHP->hist_algo_select =
        (VL53LX_HistAlgoSelect)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_TARGET_ORDER:
      pHP->hist_target_order =
        (VL53LX_HistTargetOrder)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_FILTER_WOI_0:
      pHP->filter_woi0 =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_FILTER_WOI_1:
      pHP->filter_woi1 =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_EST_METHOD:
      pHP->hist_amb_est_method =
        (VL53LX_HistAmbEstMethod)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0:
      pHP->ambient_thresh_sigma0 =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1:
      pHP->ambient_thresh_sigma1 =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS:
      pHP->min_ambient_thresh_events =
        (int32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_AMB_EVENTS_SCALER:
      pHP->ambient_thresh_events_scaler =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_NOISE_THRESHOLD:
      pHP->noise_threshold =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT:
      pHP->signal_total_events_limit =
        (int32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_SIGMA_EST_REF_MM:
      pHP->sigma_estimator__sigma_ref_mm =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_SIGMA_THRESH_MM:
      pHP->sigma_thresh =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_GAIN_FACTOR:
      pdev->gain_cal.histogram_ranging_gain_factor =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE:
      pHP->algo__consistency_check__phase_tolerance =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM:
      pHP->algo__consistency_check__min_max_tolerance =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA:
      pHP->algo__consistency_check__event_sigma =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT:
      pHP->algo__consistency_check__event_min_spad_count =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE:
      pdev->tuning_parms.tp_init_phase_rtn_hist_long =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE:
      pdev->tuning_parms.tp_init_phase_rtn_hist_med =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE:
      pdev->tuning_parms.tp_init_phase_rtn_hist_short =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE:
      pdev->tuning_parms.tp_init_phase_ref_hist_long =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE:
      pdev->tuning_parms.tp_init_phase_ref_hist_med =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE:
      pdev->tuning_parms.tp_init_phase_ref_hist_short =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM:
      pdev->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM:
      pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM:
      pdev->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE:
      pHP->algo__crosstalk_detect_min_max_tolerance =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS:
      pdev->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA:
      pHP->algo__crosstalk_detect_event_sigma =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS:
      pdev->xtalk_cfg.histogram_mode_crosstalk_margin_kcps =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE:
      pdev->tuning_parms.tp_consistency_lite_phase_tolerance =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_PHASECAL_TARGET:
      pdev->tuning_parms.tp_phasecal_target =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_CAL_REPEAT_RATE:
      pdev->tuning_parms.tp_cal_repeat_rate =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR:
      pdev->gain_cal.standard_ranging_gain_factor =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_MIN_CLIP_MM:
      pdev->tuning_parms.tp_lite_min_clip =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM:
      pdev->tuning_parms.tp_lite_long_sigma_thresh_mm =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM:
      pdev->tuning_parms.tp_lite_med_sigma_thresh_mm =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM:
      pdev->tuning_parms.tp_lite_short_sigma_thresh_mm =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS:
      pdev->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS:
      pdev->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS:
      pdev->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH:
      pdev->tuning_parms.tp_lite_sigma_est_pulse_width_ns =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS:
      pdev->tuning_parms.tp_lite_sigma_est_amb_width_ns =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_SIGMA_REF_MM:
      pdev->tuning_parms.tp_lite_sigma_ref_mm =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_RIT_MULT:
      pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_SEED_CONFIG:
      pdev->tuning_parms.tp_lite_seed_cfg =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_QUANTIFIER:
      pdev->tuning_parms.tp_lite_quantifier =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_FIRST_ORDER_SELECT:
      pdev->tuning_parms.tp_lite_first_order_select =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS:
      pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE:
      pdev->tuning_parms.tp_init_phase_rtn_lite_long =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE:
      pdev->tuning_parms.tp_init_phase_rtn_lite_med =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE:
      pdev->tuning_parms.tp_init_phase_rtn_lite_short =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE:
      pdev->tuning_parms.tp_init_phase_ref_lite_long =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE:
      pdev->tuning_parms.tp_init_phase_ref_lite_med =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE:
      pdev->tuning_parms.tp_init_phase_ref_lite_short =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_TIMED_SEED_CONFIG:
      pdev->tuning_parms.tp_timed_seed_cfg =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA:
      pdev->dmax_cfg.signal_thresh_sigma =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0:
      pdev->dmax_cfg.target_reflectance_for_dmax_calc[0] =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1:
      pdev->dmax_cfg.target_reflectance_for_dmax_calc[1] =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2:
      pdev->dmax_cfg.target_reflectance_for_dmax_calc[2] =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3:
      pdev->dmax_cfg.target_reflectance_for_dmax_calc[3] =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4:
      pdev->dmax_cfg.target_reflectance_for_dmax_calc[4] =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_VHV_LOOPBOUND:
      pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE:
      pdev->refspadchar.device_test_mode =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD:
      pdev->refspadchar.VL53LX_p_005 =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US:
      pdev->refspadchar.timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS:
      pdev->refspadchar.target_count_rate_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS:
      pdev->refspadchar.min_count_rate_limit_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS:
      pdev->refspadchar.max_count_rate_limit_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES:
      pXC->num_of_samples =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM:
      pXC->algo__crosstalk_extract_min_valid_range_mm =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM:
      pXC->algo__crosstalk_extract_max_valid_range_mm =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS:
      pXC->dss_config__target_total_rate_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US:
      pXC->phasecal_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS:
      pXC->algo__crosstalk_extract_max_valid_rate_kcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM:
      pXC->algo__crosstalk_extract_max_sigma_mm =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US:
      pXC->mm_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US:
      pXC->range_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS:
      pdev->offsetcal_cfg.dss_config__target_total_rate_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US:
      pdev->offsetcal_cfg.phasecal_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US:
      pdev->offsetcal_cfg.mm_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US:
      pdev->offsetcal_cfg.range_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES:
      pdev->offsetcal_cfg.pre_num_of_samples =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES:
      pdev->offsetcal_cfg.mm1_num_of_samples =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES:
      pdev->offsetcal_cfg.mm2_num_of_samples =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS:
      pdev->zonecal_cfg.dss_config__target_total_rate_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US:
      pdev->zonecal_cfg.phasecal_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US:
      pdev->zonecal_cfg.mm_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES:
      pdev->zonecal_cfg.phasecal_num_of_samples =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US:
      pdev->zonecal_cfg.range_config_timeout_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES:
      pdev->zonecal_cfg.zone_num_of_samples =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_SPADMAP_VCSEL_PERIOD:
      pdev->ssc_cfg.VL53LX_p_005 =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_SPADMAP_VCSEL_START:
      pdev->ssc_cfg.vcsel_start =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS:
      pdev->ssc_cfg.rate_limit_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      pdev->tuning_parms.tp_dss_target_lite_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      pdev->tuning_parms.tp_dss_target_histo_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      pdev->tuning_parms.tp_dss_target_histo_mz_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
      pdev->tuning_parms.tp_dss_target_timed_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_lite_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_hist_long_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_hist_med_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_hist_short_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_mz_long_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_mz_med_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_mz_short_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_phasecal_timeout_timed_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_mm_timeout_lite_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_mm_timeout_histo_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_mm_timeout_mz_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_mm_timeout_timed_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_range_timeout_lite_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_range_timeout_histo_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_range_timeout_mz_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_range_timeout_timed_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN:
      pdev->smudge_correct_config.smudge_margin =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NOISE_MARGIN:
      pdev->smudge_correct_config.noise_margin =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT:
      pdev->smudge_correct_config.user_xtalk_offset_limit =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI:
      pdev->smudge_correct_config.user_xtalk_offset_limit_hi =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT:
      pdev->smudge_correct_config.sample_limit =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA:
      pdev->smudge_correct_config.single_xtalk_delta =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA:
      pdev->smudge_correct_config.averaged_xtalk_delta =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_CLIP_LIMIT:
      pdev->smudge_correct_config.smudge_corr_clip_limit =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_SCALER_CALC_METHOD:
      pdev->smudge_correct_config.scaler_calc_method =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER:
      pdev->smudge_correct_config.x_gradient_scaler =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER:
      pdev->smudge_correct_config.y_gradient_scaler =
        (int16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_USER_SCALER_SET:
      pdev->smudge_correct_config.user_scaler_set =
        (uint8_t)tuning_parm_value;
      break;

    case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY:
      pdev->smudge_correct_config.smudge_corr_single_apply =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD:
      pdev->smudge_correct_config.smudge_corr_ambient_threshold =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS:
      pdev->smudge_correct_config.nodetect_ambient_threshold =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT:
      pdev->smudge_correct_config.nodetect_sample_limit =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS:
      pdev->smudge_correct_config.nodetect_xtalk_offset =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM:
      pdev->smudge_correct_config.nodetect_min_range_mm =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND:
      pdev->low_power_auto_data.vhv_loop_bound =
        (uint8_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_mm_timeout_lpa_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US:
      pdev->tuning_parms.tp_range_timeout_lpa_us =
        (uint32_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS:
      pdev->tuning_parms.tp_dss_target_very_short_mcps =
        (uint16_t)tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER:
      pdev->tuning_parms.tp_phasecal_patch_power =
        (uint16_t) tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_MERGE:
      pdev->tuning_parms.tp_hist_merge =
        (uint16_t) tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD:
      pdev->tuning_parms.tp_reset_merge_threshold =
        (uint16_t) tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE:
      pdev->tuning_parms.tp_hist_merge_max_size =
        (uint16_t) tuning_parm_value;
      break;
    case VL53LX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR:
      pdev->smudge_correct_config.max_smudge_factor =
        (uint32_t)tuning_parm_value;
      break;

    default:
      status = VL53LX_ERROR_INVALID_PARAMS;
      break;

  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_enable()
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->smudge_correct_config.smudge_corr_enabled = 1;

  return status;
}

VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_disable()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->smudge_correct_config.smudge_corr_enabled = 0;

  return status;
}
VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_apply_enable()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->smudge_correct_config.smudge_corr_apply_enabled = 1;

  return status;
}
VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_apply_disable()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->smudge_correct_config.smudge_corr_apply_enabled = 0;


  return status;
}


VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_single_apply_enable()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->smudge_correct_config.smudge_corr_single_apply = 1;


  return status;
}

VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_single_apply_disable()
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  pdev->smudge_correct_config.smudge_corr_single_apply = 0;


  return status;
}

VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_set_scalers(
  int16_t   x_scaler_in,
  int16_t   y_scaler_in,
  uint8_t   user_scaler_set_in
)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  pdev->smudge_correct_config.x_gradient_scaler = x_scaler_in;
  pdev->smudge_correct_config.y_gradient_scaler = y_scaler_in;
  pdev->smudge_correct_config.user_scaler_set = user_scaler_set_in;

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_current_xtalk_settings(
  VL53LX_xtalk_calibration_results_t *pxtalk
)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  uint8_t i;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pxtalk->algo__crosstalk_compensation_plane_offset_kcps =
    pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps;
  pxtalk->algo__crosstalk_compensation_x_plane_gradient_kcps =
    pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps;
  pxtalk->algo__crosstalk_compensation_y_plane_gradient_kcps =
    pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps;
  for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
    pxtalk->algo__xtalk_cpo_HistoMerge_kcps[i] =
      pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[i];


  return status;

}
VL53LX_Error VL53LX::VL53LX_set_current_xtalk_settings(
  VL53LX_xtalk_calibration_results_t *pxtalk
)
{

  uint8_t i;
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps =
    pxtalk->algo__crosstalk_compensation_plane_offset_kcps;
  pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps =
    pxtalk->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps =
    pxtalk->algo__crosstalk_compensation_y_plane_gradient_kcps;
  for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
    pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[i] =
      pxtalk->algo__xtalk_cpo_HistoMerge_kcps[i];


  return status;

}

/* vl53lx_register_funcs.c */

VL53LX_Error VL53LX::VL53LX_i2c_encode_static_nvm_managed(
  VL53LX_static_nvm_managed_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->i2c_slave__device_address & 0x7F;
  *(pbuffer +   1) =
    pdata->ana_config__vhv_ref_sel_vddpix & 0xF;
  *(pbuffer +   2) =
    pdata->ana_config__vhv_ref_sel_vquench & 0x7F;
  *(pbuffer +   3) =
    pdata->ana_config__reg_avdd1v2_sel & 0x3;
  *(pbuffer +   4) =
    pdata->ana_config__fast_osc__trim & 0x7F;
  VL53LX_i2c_encode_uint16_t(
    pdata->osc_measured__fast_osc__frequency,
    2,
    pbuffer +   5);
  *(pbuffer +   7) =
    pdata->vhv_config__timeout_macrop_loop_bound;
  *(pbuffer +   8) =
    pdata->vhv_config__count_thresh;
  *(pbuffer +   9) =
    pdata->vhv_config__offset & 0x3F;
  *(pbuffer +  10) =
    pdata->vhv_config__init;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_static_nvm_managed(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_static_nvm_managed_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->i2c_slave__device_address =
    (*(pbuffer +   0)) & 0x7F;
  pdata->ana_config__vhv_ref_sel_vddpix =
    (*(pbuffer +   1)) & 0xF;
  pdata->ana_config__vhv_ref_sel_vquench =
    (*(pbuffer +   2)) & 0x7F;
  pdata->ana_config__reg_avdd1v2_sel =
    (*(pbuffer +   3)) & 0x3;
  pdata->ana_config__fast_osc__trim =
    (*(pbuffer +   4)) & 0x7F;
  pdata->osc_measured__fast_osc__frequency =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   5));
  pdata->vhv_config__timeout_macrop_loop_bound =
    (*(pbuffer +   7));
  pdata->vhv_config__count_thresh =
    (*(pbuffer +   8));
  pdata->vhv_config__offset =
    (*(pbuffer +   9)) & 0x3F;
  pdata->vhv_config__init =
    (*(pbuffer +  10));

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_static_nvm_managed(
  VL53LX_static_nvm_managed_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_static_nvm_managed(
               pdata,
               VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_I2C_SLAVE__DEVICE_ADDRESS,
               comms_buffer,
               VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES);


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_static_nvm_managed(
  VL53LX_static_nvm_managed_t  *pdata)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_I2C_SLAVE__DEVICE_ADDRESS,
               comms_buffer,
               VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_static_nvm_managed(
               VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_encode_customer_nvm_managed(
  VL53LX_customer_nvm_managed_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->global_config__spad_enables_ref_0;
  *(pbuffer +   1) =
    pdata->global_config__spad_enables_ref_1;
  *(pbuffer +   2) =
    pdata->global_config__spad_enables_ref_2;
  *(pbuffer +   3) =
    pdata->global_config__spad_enables_ref_3;
  *(pbuffer +   4) =
    pdata->global_config__spad_enables_ref_4;
  *(pbuffer +   5) =
    pdata->global_config__spad_enables_ref_5 & 0xF;
  *(pbuffer +   6) =
    pdata->global_config__ref_en_start_select;
  *(pbuffer +   7) =
    pdata->ref_spad_man__num_requested_ref_spads & 0x3F;
  *(pbuffer +   8) =
    pdata->ref_spad_man__ref_location & 0x3;
  VL53LX_i2c_encode_uint16_t(
    pdata->algo__crosstalk_compensation_plane_offset_kcps,
    2,
    pbuffer +   9);
  VL53LX_i2c_encode_int16_t(
    pdata->algo__crosstalk_compensation_x_plane_gradient_kcps,
    2,
    pbuffer +  11);
  VL53LX_i2c_encode_int16_t(
    pdata->algo__crosstalk_compensation_y_plane_gradient_kcps,
    2,
    pbuffer +  13);
  VL53LX_i2c_encode_uint16_t(
    pdata->ref_spad_char__total_rate_target_mcps,
    2,
    pbuffer +  15);
  VL53LX_i2c_encode_int16_t(
    pdata->algo__part_to_part_range_offset_mm & 0x1FFF,
    2,
    pbuffer +  17);
  VL53LX_i2c_encode_int16_t(
    pdata->mm_config__inner_offset_mm,
    2,
    pbuffer +  19);
  VL53LX_i2c_encode_int16_t(
    pdata->mm_config__outer_offset_mm,
    2,
    pbuffer +  21);

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_customer_nvm_managed(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_customer_nvm_managed_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->global_config__spad_enables_ref_0 =
    (*(pbuffer +   0));
  pdata->global_config__spad_enables_ref_1 =
    (*(pbuffer +   1));
  pdata->global_config__spad_enables_ref_2 =
    (*(pbuffer +   2));
  pdata->global_config__spad_enables_ref_3 =
    (*(pbuffer +   3));
  pdata->global_config__spad_enables_ref_4 =
    (*(pbuffer +   4));
  pdata->global_config__spad_enables_ref_5 =
    (*(pbuffer +   5)) & 0xF;
  pdata->global_config__ref_en_start_select =
    (*(pbuffer +   6));
  pdata->ref_spad_man__num_requested_ref_spads =
    (*(pbuffer +   7)) & 0x3F;
  pdata->ref_spad_man__ref_location =
    (*(pbuffer +   8)) & 0x3;
  pdata->algo__crosstalk_compensation_plane_offset_kcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   9));
  pdata->algo__crosstalk_compensation_x_plane_gradient_kcps =
    (VL53LX_i2c_decode_int16_t(2, pbuffer +  11));
  pdata->algo__crosstalk_compensation_y_plane_gradient_kcps =
    (VL53LX_i2c_decode_int16_t(2, pbuffer +  13));
  pdata->ref_spad_char__total_rate_target_mcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  15));
  pdata->algo__part_to_part_range_offset_mm =
    (VL53LX_i2c_decode_int16_t(2, pbuffer +  17)) & 0x1FFF;
  pdata->mm_config__inner_offset_mm =
    (VL53LX_i2c_decode_int16_t(2, pbuffer +  19));
  pdata->mm_config__outer_offset_mm =
    (VL53LX_i2c_decode_int16_t(2, pbuffer +  21));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_customer_nvm_managed(
  VL53LX_customer_nvm_managed_t  *pdata)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_customer_nvm_managed(
               pdata,
               VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
               comms_buffer,
               VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES);

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_customer_nvm_managed(
  VL53LX_customer_nvm_managed_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
               comms_buffer,
               VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_customer_nvm_managed(
               VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_encode_static_config(
  VL53LX_static_config_t   *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  VL53LX_i2c_encode_uint16_t(
    pdata->dss_config__target_total_rate_mcps,
    2,
    pbuffer +   0);
  *(pbuffer +   2) =
    pdata->debug__ctrl & 0x1;
  *(pbuffer +   3) =
    pdata->test_mode__ctrl & 0xF;
  *(pbuffer +   4) =
    pdata->clk_gating__ctrl & 0xF;
  *(pbuffer +   5) =
    pdata->nvm_bist__ctrl & 0x1F;
  *(pbuffer +   6) =
    pdata->nvm_bist__num_nvm_words & 0x7F;
  *(pbuffer +   7) =
    pdata->nvm_bist__start_address & 0x7F;
  *(pbuffer +   8) =
    pdata->host_if__status & 0x1;
  *(pbuffer +   9) =
    pdata->pad_i2c_hv__config;
  *(pbuffer +  10) =
    pdata->pad_i2c_hv__extsup_config & 0x1;
  *(pbuffer +  11) =
    pdata->gpio_hv_pad__ctrl & 0x3;
  *(pbuffer +  12) =
    pdata->gpio_hv_mux__ctrl & 0x1F;
  *(pbuffer +  13) =
    pdata->gpio__tio_hv_status & 0x3;
  *(pbuffer +  14) =
    pdata->gpio__fio_hv_status & 0x3;
  *(pbuffer +  15) =
    pdata->ana_config__spad_sel_pswidth & 0x7;
  *(pbuffer +  16) =
    pdata->ana_config__vcsel_pulse_width_offset & 0x1F;
  *(pbuffer +  17) =
    pdata->ana_config__fast_osc__config_ctrl & 0x1;
  *(pbuffer +  18) =
    pdata->sigma_estimator__effective_pulse_width_ns;
  *(pbuffer +  19) =
    pdata->sigma_estimator__effective_ambient_width_ns;
  *(pbuffer +  20) =
    pdata->sigma_estimator__sigma_ref_mm;
  *(pbuffer +  21) =
    pdata->algo__crosstalk_compensation_valid_height_mm;
  *(pbuffer +  22) =
    pdata->spare_host_config__static_config_spare_0;
  *(pbuffer +  23) =
    pdata->spare_host_config__static_config_spare_1;
  VL53LX_i2c_encode_uint16_t(
    pdata->algo__range_ignore_threshold_mcps,
    2,
    pbuffer +  24);
  *(pbuffer +  26) =
    pdata->algo__range_ignore_valid_height_mm;
  *(pbuffer +  27) =
    pdata->algo__range_min_clip;
  *(pbuffer +  28) =
    pdata->algo__consistency_check__tolerance & 0xF;
  *(pbuffer +  29) =
    pdata->spare_host_config__static_config_spare_2;
  *(pbuffer +  30) =
    pdata->sd_config__reset_stages_msb & 0xF;
  *(pbuffer +  31) =
    pdata->sd_config__reset_stages_lsb;


  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_static_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_static_config_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->dss_config__target_total_rate_mcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   0));
  pdata->debug__ctrl =
    (*(pbuffer +   2)) & 0x1;
  pdata->test_mode__ctrl =
    (*(pbuffer +   3)) & 0xF;
  pdata->clk_gating__ctrl =
    (*(pbuffer +   4)) & 0xF;
  pdata->nvm_bist__ctrl =
    (*(pbuffer +   5)) & 0x1F;
  pdata->nvm_bist__num_nvm_words =
    (*(pbuffer +   6)) & 0x7F;
  pdata->nvm_bist__start_address =
    (*(pbuffer +   7)) & 0x7F;
  pdata->host_if__status =
    (*(pbuffer +   8)) & 0x1;
  pdata->pad_i2c_hv__config =
    (*(pbuffer +   9));
  pdata->pad_i2c_hv__extsup_config =
    (*(pbuffer +  10)) & 0x1;
  pdata->gpio_hv_pad__ctrl =
    (*(pbuffer +  11)) & 0x3;
  pdata->gpio_hv_mux__ctrl =
    (*(pbuffer +  12)) & 0x1F;
  pdata->gpio__tio_hv_status =
    (*(pbuffer +  13)) & 0x3;
  pdata->gpio__fio_hv_status =
    (*(pbuffer +  14)) & 0x3;
  pdata->ana_config__spad_sel_pswidth =
    (*(pbuffer +  15)) & 0x7;
  pdata->ana_config__vcsel_pulse_width_offset =
    (*(pbuffer +  16)) & 0x1F;
  pdata->ana_config__fast_osc__config_ctrl =
    (*(pbuffer +  17)) & 0x1;
  pdata->sigma_estimator__effective_pulse_width_ns =
    (*(pbuffer +  18));
  pdata->sigma_estimator__effective_ambient_width_ns =
    (*(pbuffer +  19));
  pdata->sigma_estimator__sigma_ref_mm =
    (*(pbuffer +  20));
  pdata->algo__crosstalk_compensation_valid_height_mm =
    (*(pbuffer +  21));
  pdata->spare_host_config__static_config_spare_0 =
    (*(pbuffer +  22));
  pdata->spare_host_config__static_config_spare_1 =
    (*(pbuffer +  23));
  pdata->algo__range_ignore_threshold_mcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  24));
  pdata->algo__range_ignore_valid_height_mm =
    (*(pbuffer +  26));
  pdata->algo__range_min_clip =
    (*(pbuffer +  27));
  pdata->algo__consistency_check__tolerance =
    (*(pbuffer +  28)) & 0xF;
  pdata->spare_host_config__static_config_spare_2 =
    (*(pbuffer +  29));
  pdata->sd_config__reset_stages_msb =
    (*(pbuffer +  30)) & 0xF;
  pdata->sd_config__reset_stages_lsb =
    (*(pbuffer +  31));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_static_config(
  VL53LX_static_config_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_static_config(
               pdata,
               VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS,
               comms_buffer,
               VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES);


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_static_config(
  VL53LX_static_config_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS,
               comms_buffer,
               VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_static_config(
               VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_encode_general_config(
  VL53LX_general_config_t  *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->gph_config__stream_count_update_value;
  *(pbuffer +   1) =
    pdata->global_config__stream_divider;
  *(pbuffer +   2) =
    pdata->system__interrupt_config_gpio;
  *(pbuffer +   3) =
    pdata->cal_config__vcsel_start & 0x7F;
  VL53LX_i2c_encode_uint16_t(
    pdata->cal_config__repeat_rate & 0xFFF,
    2,
    pbuffer +   4);
  *(pbuffer +   6) =
    pdata->global_config__vcsel_width & 0x7F;
  *(pbuffer +   7) =
    pdata->phasecal_config__timeout_macrop;
  *(pbuffer +   8) =
    pdata->phasecal_config__target;
  *(pbuffer +   9) =
    pdata->phasecal_config__override & 0x1;
  *(pbuffer +  11) =
    pdata->dss_config__roi_mode_control & 0x7;
  VL53LX_i2c_encode_uint16_t(
    pdata->system__thresh_rate_high,
    2,
    pbuffer +  12);
  VL53LX_i2c_encode_uint16_t(
    pdata->system__thresh_rate_low,
    2,
    pbuffer +  14);
  VL53LX_i2c_encode_uint16_t(
    pdata->dss_config__manual_effective_spads_select,
    2,
    pbuffer +  16);
  *(pbuffer +  18) =
    pdata->dss_config__manual_block_select;
  *(pbuffer +  19) =
    pdata->dss_config__aperture_attenuation;
  *(pbuffer +  20) =
    pdata->dss_config__max_spads_limit;
  *(pbuffer +  21) =
    pdata->dss_config__min_spads_limit;

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_decode_general_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_general_config_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->gph_config__stream_count_update_value =
    (*(pbuffer +   0));
  pdata->global_config__stream_divider =
    (*(pbuffer +   1));
  pdata->system__interrupt_config_gpio =
    (*(pbuffer +   2));
  pdata->cal_config__vcsel_start =
    (*(pbuffer +   3)) & 0x7F;
  pdata->cal_config__repeat_rate =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   4)) & 0xFFF;
  pdata->global_config__vcsel_width =
    (*(pbuffer +   6)) & 0x7F;
  pdata->phasecal_config__timeout_macrop =
    (*(pbuffer +   7));
  pdata->phasecal_config__target =
    (*(pbuffer +   8));
  pdata->phasecal_config__override =
    (*(pbuffer +   9)) & 0x1;
  pdata->dss_config__roi_mode_control =
    (*(pbuffer +  11)) & 0x7;
  pdata->system__thresh_rate_high =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  12));
  pdata->system__thresh_rate_low =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  14));
  pdata->dss_config__manual_effective_spads_select =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  16));
  pdata->dss_config__manual_block_select =
    (*(pbuffer +  18));
  pdata->dss_config__aperture_attenuation =
    (*(pbuffer +  19));
  pdata->dss_config__max_spads_limit =
    (*(pbuffer +  20));
  pdata->dss_config__min_spads_limit =
    (*(pbuffer +  21));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_general_config(
  VL53LX_general_config_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_general_config(
               pdata,
               VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE,
               comms_buffer,
               VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES);

  return status;
}


VL53LX_Error VL53LX::VL53LX_get_general_config(
  VL53LX_general_config_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES];
  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE,
               comms_buffer,
               VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_general_config(
               VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_encode_timing_config(
  VL53LX_timing_config_t   *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->mm_config__timeout_macrop_a_hi & 0xF;
  *(pbuffer +   1) =
    pdata->mm_config__timeout_macrop_a_lo;
  *(pbuffer +   2) =
    pdata->mm_config__timeout_macrop_b_hi & 0xF;
  *(pbuffer +   3) =
    pdata->mm_config__timeout_macrop_b_lo;
  *(pbuffer +   4) =
    pdata->range_config__timeout_macrop_a_hi & 0xF;
  *(pbuffer +   5) =
    pdata->range_config__timeout_macrop_a_lo;
  *(pbuffer +   6) =
    pdata->range_config__vcsel_period_a & 0x3F;
  *(pbuffer +   7) =
    pdata->range_config__timeout_macrop_b_hi & 0xF;
  *(pbuffer +   8) =
    pdata->range_config__timeout_macrop_b_lo;
  *(pbuffer +   9) =
    pdata->range_config__vcsel_period_b & 0x3F;
  VL53LX_i2c_encode_uint16_t(
    pdata->range_config__sigma_thresh,
    2,
    pbuffer +  10);
  VL53LX_i2c_encode_uint16_t(
    pdata->range_config__min_count_rate_rtn_limit_mcps,
    2,
    pbuffer +  12);
  *(pbuffer +  14) =
    pdata->range_config__valid_phase_low;
  *(pbuffer +  15) =
    pdata->range_config__valid_phase_high;
  VL53LX_i2c_encode_uint32_t(
    pdata->system__intermeasurement_period,
    4,
    pbuffer +  18);
  *(pbuffer +  22) =
    pdata->system__fractional_enable & 0x1;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_timing_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_timing_config_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->mm_config__timeout_macrop_a_hi =
    (*(pbuffer +   0)) & 0xF;
  pdata->mm_config__timeout_macrop_a_lo =
    (*(pbuffer +   1));
  pdata->mm_config__timeout_macrop_b_hi =
    (*(pbuffer +   2)) & 0xF;
  pdata->mm_config__timeout_macrop_b_lo =
    (*(pbuffer +   3));
  pdata->range_config__timeout_macrop_a_hi =
    (*(pbuffer +   4)) & 0xF;
  pdata->range_config__timeout_macrop_a_lo =
    (*(pbuffer +   5));
  pdata->range_config__vcsel_period_a =
    (*(pbuffer +   6)) & 0x3F;
  pdata->range_config__timeout_macrop_b_hi =
    (*(pbuffer +   7)) & 0xF;
  pdata->range_config__timeout_macrop_b_lo =
    (*(pbuffer +   8));
  pdata->range_config__vcsel_period_b =
    (*(pbuffer +   9)) & 0x3F;
  pdata->range_config__sigma_thresh =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  10));
  pdata->range_config__min_count_rate_rtn_limit_mcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  12));
  pdata->range_config__valid_phase_low =
    (*(pbuffer +  14));
  pdata->range_config__valid_phase_high =
    (*(pbuffer +  15));
  pdata->system__intermeasurement_period =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  18));
  pdata->system__fractional_enable =
    (*(pbuffer +  22)) & 0x1;


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_timing_config(
  VL53LX_timing_config_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_timing_config(
               pdata,
               VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_MM_CONFIG__TIMEOUT_MACROP_A_HI,
               comms_buffer,
               VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES);

  return status;
}

VL53LX_Error VL53LX::VL53LX_get_timing_config(
  VL53LX_timing_config_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_MM_CONFIG__TIMEOUT_MACROP_A_HI,
               comms_buffer,
               VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_timing_config(
               VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_encode_dynamic_config(
  VL53LX_dynamic_config_t  *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->system__grouped_parameter_hold_0 & 0x3;
  VL53LX_i2c_encode_uint16_t(
    pdata->system__thresh_high,
    2,
    pbuffer +   1);
  VL53LX_i2c_encode_uint16_t(
    pdata->system__thresh_low,
    2,
    pbuffer +   3);
  *(pbuffer +   5) =
    pdata->system__enable_xtalk_per_quadrant & 0x1;
  *(pbuffer +   6) =
    pdata->system__seed_config & 0x7;
  *(pbuffer +   7) =
    pdata->sd_config__woi_sd0;
  *(pbuffer +   8) =
    pdata->sd_config__woi_sd1;
  *(pbuffer +   9) =
    pdata->sd_config__initial_phase_sd0 & 0x7F;
  *(pbuffer +  10) =
    pdata->sd_config__initial_phase_sd1 & 0x7F;
  *(pbuffer +  11) =
    pdata->system__grouped_parameter_hold_1 & 0x3;
  *(pbuffer +  12) =
    pdata->sd_config__first_order_select & 0x3;
  *(pbuffer +  13) =
    pdata->sd_config__quantifier & 0xF;
  *(pbuffer +  14) =
    pdata->roi_config__user_roi_centre_spad;
  *(pbuffer +  15) =
    pdata->roi_config__user_roi_requested_global_xy_size;
  *(pbuffer +  16) =
    pdata->system__sequence_config;
  *(pbuffer +  17) =
    pdata->system__grouped_parameter_hold & 0x3;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_dynamic_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_dynamic_config_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->system__grouped_parameter_hold_0 =
    (*(pbuffer +   0)) & 0x3;
  pdata->system__thresh_high =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   1));
  pdata->system__thresh_low =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   3));
  pdata->system__enable_xtalk_per_quadrant =
    (*(pbuffer +   5)) & 0x1;
  pdata->system__seed_config =
    (*(pbuffer +   6)) & 0x7;
  pdata->sd_config__woi_sd0 =
    (*(pbuffer +   7));
  pdata->sd_config__woi_sd1 =
    (*(pbuffer +   8));
  pdata->sd_config__initial_phase_sd0 =
    (*(pbuffer +   9)) & 0x7F;
  pdata->sd_config__initial_phase_sd1 =
    (*(pbuffer +  10)) & 0x7F;
  pdata->system__grouped_parameter_hold_1 =
    (*(pbuffer +  11)) & 0x3;
  pdata->sd_config__first_order_select =
    (*(pbuffer +  12)) & 0x3;
  pdata->sd_config__quantifier =
    (*(pbuffer +  13)) & 0xF;
  pdata->roi_config__user_roi_centre_spad =
    (*(pbuffer +  14));
  pdata->roi_config__user_roi_requested_global_xy_size =
    (*(pbuffer +  15));
  pdata->system__sequence_config =
    (*(pbuffer +  16));
  pdata->system__grouped_parameter_hold =
    (*(pbuffer +  17)) & 0x3;

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_dynamic_config(
  VL53LX_dynamic_config_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_dynamic_config(
               pdata,
               VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_SYSTEM__GROUPED_PARAMETER_HOLD_0,
               comms_buffer,
               VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES);

  return status;
}


VL53LX_Error VL53LX::VL53LX_get_dynamic_config(
  VL53LX_dynamic_config_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_SYSTEM__GROUPED_PARAMETER_HOLD_0,
               comms_buffer,
               VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_dynamic_config(
               VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_encode_system_control(
  VL53LX_system_control_t  *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->power_management__go1_power_force & 0x1;
  *(pbuffer +   1) =
    pdata->system__stream_count_ctrl & 0x1;
  *(pbuffer +   2) =
    pdata->firmware__enable & 0x1;
  *(pbuffer +   3) =
    pdata->system__interrupt_clear & 0x3;
  *(pbuffer +   4) =
    pdata->system__mode_start;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_system_control(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_system_control_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->power_management__go1_power_force =
    (*(pbuffer +   0)) & 0x1;
  pdata->system__stream_count_ctrl =
    (*(pbuffer +   1)) & 0x1;
  pdata->firmware__enable =
    (*(pbuffer +   2)) & 0x1;
  pdata->system__interrupt_clear =
    (*(pbuffer +   3)) & 0x3;
  pdata->system__mode_start =
    (*(pbuffer +   4));


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_system_control(
  VL53LX_system_control_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_system_control(
               pdata,
               VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_POWER_MANAGEMENT__GO1_POWER_FORCE,
               comms_buffer,
               VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES);

  return status;
}


VL53LX_Error VL53LX::VL53LX_get_system_control(
  VL53LX_system_control_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_POWER_MANAGEMENT__GO1_POWER_FORCE,
               comms_buffer,
               VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_system_control(
               VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_encode_system_results(
  VL53LX_system_results_t  *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->result__interrupt_status & 0x3F;
  *(pbuffer +   1) =
    pdata->result__range_status;
  *(pbuffer +   2) =
    pdata->result__report_status & 0xF;
  *(pbuffer +   3) =
    pdata->result__stream_count;
  VL53LX_i2c_encode_uint16_t(
    pdata->result__dss_actual_effective_spads_sd0,
    2,
    pbuffer +   4);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__peak_signal_count_rate_mcps_sd0,
    2,
    pbuffer +   6);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__ambient_count_rate_mcps_sd0,
    2,
    pbuffer +   8);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__sigma_sd0,
    2,
    pbuffer +  10);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__phase_sd0,
    2,
    pbuffer +  12);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__final_crosstalk_corrected_range_mm_sd0,
    2,
    pbuffer +  14);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0,
    2,
    pbuffer +  16);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__mm_inner_actual_effective_spads_sd0,
    2,
    pbuffer +  18);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__mm_outer_actual_effective_spads_sd0,
    2,
    pbuffer +  20);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__avg_signal_count_rate_mcps_sd0,
    2,
    pbuffer +  22);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__dss_actual_effective_spads_sd1,
    2,
    pbuffer +  24);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__peak_signal_count_rate_mcps_sd1,
    2,
    pbuffer +  26);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__ambient_count_rate_mcps_sd1,
    2,
    pbuffer +  28);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__sigma_sd1,
    2,
    pbuffer +  30);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__phase_sd1,
    2,
    pbuffer +  32);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__final_crosstalk_corrected_range_mm_sd1,
    2,
    pbuffer +  34);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__spare_0_sd1,
    2,
    pbuffer +  36);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__spare_1_sd1,
    2,
    pbuffer +  38);
  VL53LX_i2c_encode_uint16_t(
    pdata->result__spare_2_sd1,
    2,
    pbuffer +  40);
  *(pbuffer +  42) =
    pdata->result__spare_3_sd1;
  *(pbuffer +  43) =
    pdata->result__thresh_info;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_system_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_system_results_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->result__interrupt_status =
    (*(pbuffer +   0)) & 0x3F;
  pdata->result__range_status =
    (*(pbuffer +   1));
  pdata->result__report_status =
    (*(pbuffer +   2)) & 0xF;
  pdata->result__stream_count =
    (*(pbuffer +   3));
  pdata->result__dss_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   4));
  pdata->result__peak_signal_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   6));
  pdata->result__ambient_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   8));
  pdata->result__sigma_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  10));
  pdata->result__phase_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  12));
  pdata->result__final_crosstalk_corrected_range_mm_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  14));
  pdata->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  16));
  pdata->result__mm_inner_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  18));
  pdata->result__mm_outer_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  20));
  pdata->result__avg_signal_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  22));
  pdata->result__dss_actual_effective_spads_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  24));
  pdata->result__peak_signal_count_rate_mcps_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  26));
  pdata->result__ambient_count_rate_mcps_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  28));
  pdata->result__sigma_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  30));
  pdata->result__phase_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  32));
  pdata->result__final_crosstalk_corrected_range_mm_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  34));
  pdata->result__spare_0_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  36));
  pdata->result__spare_1_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  38));
  pdata->result__spare_2_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  40));
  pdata->result__spare_3_sd1 =
    (*(pbuffer +  42));
  pdata->result__thresh_info =
    (*(pbuffer +  43));

  return status;
}
VL53LX_Error VL53LX::VL53LX_set_system_results(
  VL53LX_system_results_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_system_results(
               pdata,
               VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_RESULT__INTERRUPT_STATUS,
               comms_buffer,
               VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES);


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_system_results(
  VL53LX_system_results_t   *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_RESULT__INTERRUPT_STATUS,
               comms_buffer,
               VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_system_results(
               VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_core_results(
  VL53LX_core_results_t    *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_CORE_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  VL53LX_i2c_encode_uint32_t(
    pdata->result_core__ambient_window_events_sd0,
    4,
    pbuffer +   0);
  VL53LX_i2c_encode_uint32_t(
    pdata->result_core__ranging_total_events_sd0,
    4,
    pbuffer +   4);
  VL53LX_i2c_encode_int32_t(
    pdata->result_core__signal_total_events_sd0,
    4,
    pbuffer +   8);
  VL53LX_i2c_encode_uint32_t(
    pdata->result_core__total_periods_elapsed_sd0,
    4,
    pbuffer +  12);
  VL53LX_i2c_encode_uint32_t(
    pdata->result_core__ambient_window_events_sd1,
    4,
    pbuffer +  16);
  VL53LX_i2c_encode_uint32_t(
    pdata->result_core__ranging_total_events_sd1,
    4,
    pbuffer +  20);
  VL53LX_i2c_encode_int32_t(
    pdata->result_core__signal_total_events_sd1,
    4,
    pbuffer +  24);
  VL53LX_i2c_encode_uint32_t(
    pdata->result_core__total_periods_elapsed_sd1,
    4,
    pbuffer +  28);
  *(pbuffer +  32) =
    pdata->result_core__spare_0;

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_decode_core_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_core_results_t     *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_CORE_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->result_core__ambient_window_events_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +   0));
  pdata->result_core__ranging_total_events_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +   4));
  pdata->result_core__signal_total_events_sd0 =
    (VL53LX_i2c_decode_int32_t(4, pbuffer +   8));
  pdata->result_core__total_periods_elapsed_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  12));
  pdata->result_core__ambient_window_events_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  16));
  pdata->result_core__ranging_total_events_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  20));
  pdata->result_core__signal_total_events_sd1 =
    (VL53LX_i2c_decode_int32_t(4, pbuffer +  24));
  pdata->result_core__total_periods_elapsed_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  28));
  pdata->result_core__spare_0 =
    (*(pbuffer +  32));
  return status;
}

VL53LX_Error VL53LX::VL53LX_set_core_results(
  VL53LX_core_results_t     *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_CORE_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_core_results(
               pdata,
               VL53LX_CORE_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0,
               comms_buffer,
               VL53LX_CORE_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_get_core_results(
  VL53LX_core_results_t     *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_CORE_RESULTS_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0,
               comms_buffer,
               VL53LX_CORE_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_core_results(
               VL53LX_CORE_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_debug_results(
  VL53LX_debug_results_t   *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  VL53LX_i2c_encode_uint16_t(
    pdata->phasecal_result__reference_phase,
    2,
    pbuffer +   0);
  *(pbuffer +   2) =
    pdata->phasecal_result__vcsel_start & 0x7F;
  *(pbuffer +   3) =
    pdata->ref_spad_char_result__num_actual_ref_spads & 0x3F;
  *(pbuffer +   4) =
    pdata->ref_spad_char_result__ref_location & 0x3;
  *(pbuffer +   5) =
    pdata->vhv_result__coldboot_status & 0x1;
  *(pbuffer +   6) =
    pdata->vhv_result__search_result & 0x3F;
  *(pbuffer +   7) =
    pdata->vhv_result__latest_setting & 0x3F;
  VL53LX_i2c_encode_uint16_t(
    pdata->result__osc_calibrate_val & 0x3FF,
    2,
    pbuffer +   8);
  *(pbuffer +  10) =
    pdata->ana_config__powerdown_go1 & 0x3;
  *(pbuffer +  11) =
    pdata->ana_config__ref_bg_ctrl & 0x3;
  *(pbuffer +  12) =
    pdata->ana_config__regdvdd1v2_ctrl & 0xF;
  *(pbuffer +  13) =
    pdata->ana_config__osc_slow_ctrl & 0x7;
  *(pbuffer +  14) =
    pdata->test_mode__status & 0x1;
  *(pbuffer +  15) =
    pdata->firmware__system_status & 0x3;
  *(pbuffer +  16) =
    pdata->firmware__mode_status;
  *(pbuffer +  17) =
    pdata->firmware__secondary_mode_status;
  VL53LX_i2c_encode_uint16_t(
    pdata->firmware__cal_repeat_rate_counter & 0xFFF,
    2,
    pbuffer +  18);
  VL53LX_i2c_encode_uint16_t(
    pdata->gph__system__thresh_high,
    2,
    pbuffer +  22);
  VL53LX_i2c_encode_uint16_t(
    pdata->gph__system__thresh_low,
    2,
    pbuffer +  24);
  *(pbuffer +  26) =
    pdata->gph__system__enable_xtalk_per_quadrant & 0x1;
  *(pbuffer +  27) =
    pdata->gph__spare_0 & 0x7;
  *(pbuffer +  28) =
    pdata->gph__sd_config__woi_sd0;
  *(pbuffer +  29) =
    pdata->gph__sd_config__woi_sd1;
  *(pbuffer +  30) =
    pdata->gph__sd_config__initial_phase_sd0 & 0x7F;
  *(pbuffer +  31) =
    pdata->gph__sd_config__initial_phase_sd1 & 0x7F;
  *(pbuffer +  32) =
    pdata->gph__sd_config__first_order_select & 0x3;
  *(pbuffer +  33) =
    pdata->gph__sd_config__quantifier & 0xF;
  *(pbuffer +  34) =
    pdata->gph__roi_config__user_roi_centre_spad;
  *(pbuffer +  35) =
    pdata->gph__roi_config__user_roi_requested_global_xy_size;
  *(pbuffer +  36) =
    pdata->gph__system__sequence_config;
  *(pbuffer +  37) =
    pdata->gph__gph_id & 0x1;
  *(pbuffer +  38) =
    pdata->system__interrupt_set & 0x3;
  *(pbuffer +  39) =
    pdata->interrupt_manager__enables & 0x1F;
  *(pbuffer +  40) =
    pdata->interrupt_manager__clear & 0x1F;
  *(pbuffer +  41) =
    pdata->interrupt_manager__status & 0x1F;
  *(pbuffer +  42) =
    pdata->mcu_to_host_bank__wr_access_en & 0x1;
  *(pbuffer +  43) =
    pdata->power_management__go1_reset_status & 0x1;
  *(pbuffer +  44) =
    pdata->pad_startup_mode__value_ro & 0x3;
  *(pbuffer +  45) =
    pdata->pad_startup_mode__value_ctrl & 0x3F;
  VL53LX_i2c_encode_uint32_t(
    pdata->pll_period_us & 0x3FFFF,
    4,
    pbuffer +  46);
  VL53LX_i2c_encode_uint32_t(
    pdata->interrupt_scheduler__data_out,
    4,
    pbuffer +  50);
  *(pbuffer +  54) =
    pdata->nvm_bist__complete & 0x1;
  *(pbuffer +  55) =
    pdata->nvm_bist__status & 0x1;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_debug_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_debug_results_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->phasecal_result__reference_phase =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   0));
  pdata->phasecal_result__vcsel_start =
    (*(pbuffer +   2)) & 0x7F;
  pdata->ref_spad_char_result__num_actual_ref_spads =
    (*(pbuffer +   3)) & 0x3F;
  pdata->ref_spad_char_result__ref_location =
    (*(pbuffer +   4)) & 0x3;
  pdata->vhv_result__coldboot_status =
    (*(pbuffer +   5)) & 0x1;
  pdata->vhv_result__search_result =
    (*(pbuffer +   6)) & 0x3F;
  pdata->vhv_result__latest_setting =
    (*(pbuffer +   7)) & 0x3F;
  pdata->result__osc_calibrate_val =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   8)) & 0x3FF;
  pdata->ana_config__powerdown_go1 =
    (*(pbuffer +  10)) & 0x3;
  pdata->ana_config__ref_bg_ctrl =
    (*(pbuffer +  11)) & 0x3;
  pdata->ana_config__regdvdd1v2_ctrl =
    (*(pbuffer +  12)) & 0xF;
  pdata->ana_config__osc_slow_ctrl =
    (*(pbuffer +  13)) & 0x7;
  pdata->test_mode__status =
    (*(pbuffer +  14)) & 0x1;
  pdata->firmware__system_status =
    (*(pbuffer +  15)) & 0x3;
  pdata->firmware__mode_status =
    (*(pbuffer +  16));
  pdata->firmware__secondary_mode_status =
    (*(pbuffer +  17));
  pdata->firmware__cal_repeat_rate_counter =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  18)) & 0xFFF;
  pdata->gph__system__thresh_high =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  22));
  pdata->gph__system__thresh_low =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  24));
  pdata->gph__system__enable_xtalk_per_quadrant =
    (*(pbuffer +  26)) & 0x1;
  pdata->gph__spare_0 =
    (*(pbuffer +  27)) & 0x7;
  pdata->gph__sd_config__woi_sd0 =
    (*(pbuffer +  28));
  pdata->gph__sd_config__woi_sd1 =
    (*(pbuffer +  29));
  pdata->gph__sd_config__initial_phase_sd0 =
    (*(pbuffer +  30)) & 0x7F;
  pdata->gph__sd_config__initial_phase_sd1 =
    (*(pbuffer +  31)) & 0x7F;
  pdata->gph__sd_config__first_order_select =
    (*(pbuffer +  32)) & 0x3;
  pdata->gph__sd_config__quantifier =
    (*(pbuffer +  33)) & 0xF;
  pdata->gph__roi_config__user_roi_centre_spad =
    (*(pbuffer +  34));
  pdata->gph__roi_config__user_roi_requested_global_xy_size =
    (*(pbuffer +  35));
  pdata->gph__system__sequence_config =
    (*(pbuffer +  36));
  pdata->gph__gph_id =
    (*(pbuffer +  37)) & 0x1;
  pdata->system__interrupt_set =
    (*(pbuffer +  38)) & 0x3;
  pdata->interrupt_manager__enables =
    (*(pbuffer +  39)) & 0x1F;
  pdata->interrupt_manager__clear =
    (*(pbuffer +  40)) & 0x1F;
  pdata->interrupt_manager__status =
    (*(pbuffer +  41)) & 0x1F;
  pdata->mcu_to_host_bank__wr_access_en =
    (*(pbuffer +  42)) & 0x1;
  pdata->power_management__go1_reset_status =
    (*(pbuffer +  43)) & 0x1;
  pdata->pad_startup_mode__value_ro =
    (*(pbuffer +  44)) & 0x3;
  pdata->pad_startup_mode__value_ctrl =
    (*(pbuffer +  45)) & 0x3F;
  pdata->pll_period_us =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  46)) & 0x3FFFF;
  pdata->interrupt_scheduler__data_out =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  50));
  pdata->nvm_bist__complete =
    (*(pbuffer +  54)) & 0x1;
  pdata->nvm_bist__status =
    (*(pbuffer +  55)) & 0x1;


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_debug_results(
  VL53LX_debug_results_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_debug_results(
               pdata,
               VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_PHASECAL_RESULT__REFERENCE_PHASE,
               comms_buffer,
               VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_debug_results(
  VL53LX_debug_results_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_PHASECAL_RESULT__REFERENCE_PHASE,
               comms_buffer,
               VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_debug_results(
               VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_nvm_copy_data(
  VL53LX_nvm_copy_data_t   *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->identification__model_id;
  *(pbuffer +   1) =
    pdata->identification__module_type;
  *(pbuffer +   2) =
    pdata->identification__revision_id;
  VL53LX_i2c_encode_uint16_t(
    pdata->identification__module_id,
    2,
    pbuffer +   3);
  *(pbuffer +   5) =
    pdata->ana_config__fast_osc__trim_max & 0x7F;
  *(pbuffer +   6) =
    pdata->ana_config__fast_osc__freq_set & 0x7;
  *(pbuffer +   7) =
    pdata->ana_config__vcsel_trim & 0x7;
  *(pbuffer +   8) =
    pdata->ana_config__vcsel_selion & 0x3F;
  *(pbuffer +   9) =
    pdata->ana_config__vcsel_selion_max & 0x3F;
  *(pbuffer +  10) =
    pdata->protected_laser_safety__lock_bit & 0x1;
  *(pbuffer +  11) =
    pdata->laser_safety__key & 0x7F;
  *(pbuffer +  12) =
    pdata->laser_safety__key_ro & 0x1;
  *(pbuffer +  13) =
    pdata->laser_safety__clip & 0x3F;
  *(pbuffer +  14) =
    pdata->laser_safety__mult & 0x3F;
  *(pbuffer +  15) =
    pdata->global_config__spad_enables_rtn_0;
  *(pbuffer +  16) =
    pdata->global_config__spad_enables_rtn_1;
  *(pbuffer +  17) =
    pdata->global_config__spad_enables_rtn_2;
  *(pbuffer +  18) =
    pdata->global_config__spad_enables_rtn_3;
  *(pbuffer +  19) =
    pdata->global_config__spad_enables_rtn_4;
  *(pbuffer +  20) =
    pdata->global_config__spad_enables_rtn_5;
  *(pbuffer +  21) =
    pdata->global_config__spad_enables_rtn_6;
  *(pbuffer +  22) =
    pdata->global_config__spad_enables_rtn_7;
  *(pbuffer +  23) =
    pdata->global_config__spad_enables_rtn_8;
  *(pbuffer +  24) =
    pdata->global_config__spad_enables_rtn_9;
  *(pbuffer +  25) =
    pdata->global_config__spad_enables_rtn_10;
  *(pbuffer +  26) =
    pdata->global_config__spad_enables_rtn_11;
  *(pbuffer +  27) =
    pdata->global_config__spad_enables_rtn_12;
  *(pbuffer +  28) =
    pdata->global_config__spad_enables_rtn_13;
  *(pbuffer +  29) =
    pdata->global_config__spad_enables_rtn_14;
  *(pbuffer +  30) =
    pdata->global_config__spad_enables_rtn_15;
  *(pbuffer +  31) =
    pdata->global_config__spad_enables_rtn_16;
  *(pbuffer +  32) =
    pdata->global_config__spad_enables_rtn_17;
  *(pbuffer +  33) =
    pdata->global_config__spad_enables_rtn_18;
  *(pbuffer +  34) =
    pdata->global_config__spad_enables_rtn_19;
  *(pbuffer +  35) =
    pdata->global_config__spad_enables_rtn_20;
  *(pbuffer +  36) =
    pdata->global_config__spad_enables_rtn_21;
  *(pbuffer +  37) =
    pdata->global_config__spad_enables_rtn_22;
  *(pbuffer +  38) =
    pdata->global_config__spad_enables_rtn_23;
  *(pbuffer +  39) =
    pdata->global_config__spad_enables_rtn_24;
  *(pbuffer +  40) =
    pdata->global_config__spad_enables_rtn_25;
  *(pbuffer +  41) =
    pdata->global_config__spad_enables_rtn_26;
  *(pbuffer +  42) =
    pdata->global_config__spad_enables_rtn_27;
  *(pbuffer +  43) =
    pdata->global_config__spad_enables_rtn_28;
  *(pbuffer +  44) =
    pdata->global_config__spad_enables_rtn_29;
  *(pbuffer +  45) =
    pdata->global_config__spad_enables_rtn_30;
  *(pbuffer +  46) =
    pdata->global_config__spad_enables_rtn_31;
  *(pbuffer +  47) =
    pdata->roi_config__mode_roi_centre_spad;
  *(pbuffer +  48) =
    pdata->roi_config__mode_roi_xy_size;


  return status;
}



VL53LX_Error VL53LX::VL53LX_i2c_decode_nvm_copy_data(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_nvm_copy_data_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->identification__model_id =
    (*(pbuffer +   0));
  pdata->identification__module_type =
    (*(pbuffer +   1));
  pdata->identification__revision_id =
    (*(pbuffer +   2));
  pdata->identification__module_id =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   3));
  pdata->ana_config__fast_osc__trim_max =
    (*(pbuffer +   5)) & 0x7F;
  pdata->ana_config__fast_osc__freq_set =
    (*(pbuffer +   6)) & 0x7;
  pdata->ana_config__vcsel_trim =
    (*(pbuffer +   7)) & 0x7;
  pdata->ana_config__vcsel_selion =
    (*(pbuffer +   8)) & 0x3F;
  pdata->ana_config__vcsel_selion_max =
    (*(pbuffer +   9)) & 0x3F;
  pdata->protected_laser_safety__lock_bit =
    (*(pbuffer +  10)) & 0x1;
  pdata->laser_safety__key =
    (*(pbuffer +  11)) & 0x7F;
  pdata->laser_safety__key_ro =
    (*(pbuffer +  12)) & 0x1;
  pdata->laser_safety__clip =
    (*(pbuffer +  13)) & 0x3F;
  pdata->laser_safety__mult =
    (*(pbuffer +  14)) & 0x3F;
  pdata->global_config__spad_enables_rtn_0 =
    (*(pbuffer +  15));
  pdata->global_config__spad_enables_rtn_1 =
    (*(pbuffer +  16));
  pdata->global_config__spad_enables_rtn_2 =
    (*(pbuffer +  17));
  pdata->global_config__spad_enables_rtn_3 =
    (*(pbuffer +  18));
  pdata->global_config__spad_enables_rtn_4 =
    (*(pbuffer +  19));
  pdata->global_config__spad_enables_rtn_5 =
    (*(pbuffer +  20));
  pdata->global_config__spad_enables_rtn_6 =
    (*(pbuffer +  21));
  pdata->global_config__spad_enables_rtn_7 =
    (*(pbuffer +  22));
  pdata->global_config__spad_enables_rtn_8 =
    (*(pbuffer +  23));
  pdata->global_config__spad_enables_rtn_9 =
    (*(pbuffer +  24));
  pdata->global_config__spad_enables_rtn_10 =
    (*(pbuffer +  25));
  pdata->global_config__spad_enables_rtn_11 =
    (*(pbuffer +  26));
  pdata->global_config__spad_enables_rtn_12 =
    (*(pbuffer +  27));
  pdata->global_config__spad_enables_rtn_13 =
    (*(pbuffer +  28));
  pdata->global_config__spad_enables_rtn_14 =
    (*(pbuffer +  29));
  pdata->global_config__spad_enables_rtn_15 =
    (*(pbuffer +  30));
  pdata->global_config__spad_enables_rtn_16 =
    (*(pbuffer +  31));
  pdata->global_config__spad_enables_rtn_17 =
    (*(pbuffer +  32));
  pdata->global_config__spad_enables_rtn_18 =
    (*(pbuffer +  33));
  pdata->global_config__spad_enables_rtn_19 =
    (*(pbuffer +  34));
  pdata->global_config__spad_enables_rtn_20 =
    (*(pbuffer +  35));
  pdata->global_config__spad_enables_rtn_21 =
    (*(pbuffer +  36));
  pdata->global_config__spad_enables_rtn_22 =
    (*(pbuffer +  37));
  pdata->global_config__spad_enables_rtn_23 =
    (*(pbuffer +  38));
  pdata->global_config__spad_enables_rtn_24 =
    (*(pbuffer +  39));
  pdata->global_config__spad_enables_rtn_25 =
    (*(pbuffer +  40));
  pdata->global_config__spad_enables_rtn_26 =
    (*(pbuffer +  41));
  pdata->global_config__spad_enables_rtn_27 =
    (*(pbuffer +  42));
  pdata->global_config__spad_enables_rtn_28 =
    (*(pbuffer +  43));
  pdata->global_config__spad_enables_rtn_29 =
    (*(pbuffer +  44));
  pdata->global_config__spad_enables_rtn_30 =
    (*(pbuffer +  45));
  pdata->global_config__spad_enables_rtn_31 =
    (*(pbuffer +  46));
  pdata->roi_config__mode_roi_centre_spad =
    (*(pbuffer +  47));
  pdata->roi_config__mode_roi_xy_size =
    (*(pbuffer +  48));


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_nvm_copy_data(
  VL53LX_nvm_copy_data_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_nvm_copy_data(
               pdata,
               VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_IDENTIFICATION__MODEL_ID,
               comms_buffer,
               VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_nvm_copy_data(
  VL53LX_nvm_copy_data_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_IDENTIFICATION__MODEL_ID,
               comms_buffer,
               VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_nvm_copy_data(
               VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_encode_prev_shadow_system_results(
  VL53LX_prev_shadow_system_results_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->prev_shadow_result__interrupt_status & 0x3F;
  *(pbuffer +   1) =
    pdata->prev_shadow_result__range_status;
  *(pbuffer +   2) =
    pdata->prev_shadow_result__report_status & 0xF;
  *(pbuffer +   3) =
    pdata->prev_shadow_result__stream_count;
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__dss_actual_effective_spads_sd0,
    2,
    pbuffer +   4);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__peak_signal_count_rate_mcps_sd0,
    2,
    pbuffer +   6);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__ambient_count_rate_mcps_sd0,
    2,
    pbuffer +   8);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__sigma_sd0,
    2,
    pbuffer +  10);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__phase_sd0,
    2,
    pbuffer +  12);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__final_crosstalk_corrected_range_mm_sd0,
    2,
    pbuffer +  14);
  VL53LX_i2c_encode_uint16_t(
    pdata->psr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0,
    2,
    pbuffer +  16);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__mm_inner_actual_effective_spads_sd0,
    2,
    pbuffer +  18);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__mm_outer_actual_effective_spads_sd0,
    2,
    pbuffer +  20);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__avg_signal_count_rate_mcps_sd0,
    2,
    pbuffer +  22);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__dss_actual_effective_spads_sd1,
    2,
    pbuffer +  24);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__peak_signal_count_rate_mcps_sd1,
    2,
    pbuffer +  26);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__ambient_count_rate_mcps_sd1,
    2,
    pbuffer +  28);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__sigma_sd1,
    2,
    pbuffer +  30);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__phase_sd1,
    2,
    pbuffer +  32);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__final_crosstalk_corrected_range_mm_sd1,
    2,
    pbuffer +  34);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__spare_0_sd1,
    2,
    pbuffer +  36);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__spare_1_sd1,
    2,
    pbuffer +  38);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__spare_2_sd1,
    2,
    pbuffer +  40);
  VL53LX_i2c_encode_uint16_t(
    pdata->prev_shadow_result__spare_3_sd1,
    2,
    pbuffer +  42);


  return status;
}



VL53LX_Error VL53LX::VL53LX_i2c_decode_prev_shadow_system_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_prev_shadow_system_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->prev_shadow_result__interrupt_status =
    (*(pbuffer +   0)) & 0x3F;
  pdata->prev_shadow_result__range_status =
    (*(pbuffer +   1));
  pdata->prev_shadow_result__report_status =
    (*(pbuffer +   2)) & 0xF;
  pdata->prev_shadow_result__stream_count =
    (*(pbuffer +   3));
  pdata->prev_shadow_result__dss_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   4));
  pdata->prev_shadow_result__peak_signal_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   6));
  pdata->prev_shadow_result__ambient_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   8));
  pdata->prev_shadow_result__sigma_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  10));
  pdata->prev_shadow_result__phase_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  12));
  pdata->prev_shadow_result__final_crosstalk_corrected_range_mm_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  14));
  pdata->psr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  16));
  pdata->prev_shadow_result__mm_inner_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  18));
  pdata->prev_shadow_result__mm_outer_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  20));
  pdata->prev_shadow_result__avg_signal_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  22));
  pdata->prev_shadow_result__dss_actual_effective_spads_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  24));
  pdata->prev_shadow_result__peak_signal_count_rate_mcps_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  26));
  pdata->prev_shadow_result__ambient_count_rate_mcps_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  28));
  pdata->prev_shadow_result__sigma_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  30));
  pdata->prev_shadow_result__phase_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  32));
  pdata->prev_shadow_result__final_crosstalk_corrected_range_mm_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  34));
  pdata->prev_shadow_result__spare_0_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  36));
  pdata->prev_shadow_result__spare_1_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  38));
  pdata->prev_shadow_result__spare_2_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  40));
  pdata->prev_shadow_result__spare_3_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  42));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_prev_shadow_system_results(
  VL53LX_prev_shadow_system_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_prev_shadow_system_results(
               pdata,
               VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_PREV_SHADOW_RESULT__INTERRUPT_STATUS,
               comms_buffer,
               VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}
VL53LX_Error VL53LX::VL53LX_get_prev_shadow_system_results(
  VL53LX_prev_shadow_system_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_PREV_SHADOW_RESULT__INTERRUPT_STATUS,
               comms_buffer,
               VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_prev_shadow_system_results(
               VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_prev_shadow_core_results(
  VL53LX_prev_shadow_core_results_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  VL53LX_i2c_encode_uint32_t(
    pdata->prev_shadow_result_core__ambient_window_events_sd0,
    4,
    pbuffer +   0);
  VL53LX_i2c_encode_uint32_t(
    pdata->prev_shadow_result_core__ranging_total_events_sd0,
    4,
    pbuffer +   4);
  VL53LX_i2c_encode_int32_t(
    pdata->prev_shadow_result_core__signal_total_events_sd0,
    4,
    pbuffer +   8);
  VL53LX_i2c_encode_uint32_t(
    pdata->prev_shadow_result_core__total_periods_elapsed_sd0,
    4,
    pbuffer +  12);
  VL53LX_i2c_encode_uint32_t(
    pdata->prev_shadow_result_core__ambient_window_events_sd1,
    4,
    pbuffer +  16);
  VL53LX_i2c_encode_uint32_t(
    pdata->prev_shadow_result_core__ranging_total_events_sd1,
    4,
    pbuffer +  20);
  VL53LX_i2c_encode_int32_t(
    pdata->prev_shadow_result_core__signal_total_events_sd1,
    4,
    pbuffer +  24);
  VL53LX_i2c_encode_uint32_t(
    pdata->prev_shadow_result_core__total_periods_elapsed_sd1,
    4,
    pbuffer +  28);
  *(pbuffer +  32) =
    pdata->prev_shadow_result_core__spare_0;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_prev_shadow_core_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_prev_shadow_core_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  ;

  if (buf_size < VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->prev_shadow_result_core__ambient_window_events_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +   0));
  pdata->prev_shadow_result_core__ranging_total_events_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +   4));
  pdata->prev_shadow_result_core__signal_total_events_sd0 =
    (VL53LX_i2c_decode_int32_t(4, pbuffer +   8));
  pdata->prev_shadow_result_core__total_periods_elapsed_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  12));
  pdata->prev_shadow_result_core__ambient_window_events_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  16));
  pdata->prev_shadow_result_core__ranging_total_events_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  20));
  pdata->prev_shadow_result_core__signal_total_events_sd1 =
    (VL53LX_i2c_decode_int32_t(4, pbuffer +  24));
  pdata->prev_shadow_result_core__total_periods_elapsed_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  28));
  pdata->prev_shadow_result_core__spare_0 =
    (*(pbuffer +  32));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_prev_shadow_core_results(
  VL53LX_prev_shadow_core_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_prev_shadow_core_results(
               pdata,
               VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0,
               comms_buffer,
               VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}
VL53LX_Error VL53LX::VL53LX_get_prev_shadow_core_results(
  VL53LX_prev_shadow_core_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0,
               comms_buffer,
               VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_prev_shadow_core_results(
               VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_patch_debug(
  VL53LX_patch_debug_t     *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->result__debug_status;
  *(pbuffer +   1) =
    pdata->result__debug_stage;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_patch_debug(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_patch_debug_t      *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->result__debug_status =
    (*(pbuffer +   0));
  pdata->result__debug_stage =
    (*(pbuffer +   1));

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_patch_debug(
  VL53LX_patch_debug_t      *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_patch_debug(
               pdata,
               VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_RESULT__DEBUG_STATUS,
               comms_buffer,
               VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}
VL53LX_Error VL53LX::VL53LX_get_patch_debug(
  VL53LX_patch_debug_t      *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_RESULT__DEBUG_STATUS,
               comms_buffer,
               VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_patch_debug(
               VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_gph_general_config(
  VL53LX_gph_general_config_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  VL53LX_i2c_encode_uint16_t(
    pdata->gph__system__thresh_rate_high,
    2,
    pbuffer +   0);
  VL53LX_i2c_encode_uint16_t(
    pdata->gph__system__thresh_rate_low,
    2,
    pbuffer +   2);
  *(pbuffer +   4) =
    pdata->gph__system__interrupt_config_gpio;

  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_decode_gph_general_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_gph_general_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->gph__system__thresh_rate_high =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   0));
  pdata->gph__system__thresh_rate_low =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   2));
  pdata->gph__system__interrupt_config_gpio =
    (*(pbuffer +   4));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_gph_general_config(
  VL53LX_gph_general_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_gph_general_config(
               pdata,
               VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_GPH__SYSTEM__THRESH_RATE_HIGH,
               comms_buffer,
               VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_get_gph_general_config(
  VL53LX_gph_general_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_GPH__SYSTEM__THRESH_RATE_HIGH,
               comms_buffer,
               VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_gph_general_config(
               VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_encode_gph_static_config(
  VL53LX_gph_static_config_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->gph__dss_config__roi_mode_control & 0x7;
  VL53LX_i2c_encode_uint16_t(
    pdata->gph__dss_config__manual_effective_spads_select,
    2,
    pbuffer +   1);
  *(pbuffer +   3) =
    pdata->gph__dss_config__manual_block_select;
  *(pbuffer +   4) =
    pdata->gph__dss_config__max_spads_limit;
  *(pbuffer +   5) =
    pdata->gph__dss_config__min_spads_limit;

  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_decode_gph_static_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_gph_static_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->gph__dss_config__roi_mode_control =
    (*(pbuffer +   0)) & 0x7;
  pdata->gph__dss_config__manual_effective_spads_select =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   1));
  pdata->gph__dss_config__manual_block_select =
    (*(pbuffer +   3));
  pdata->gph__dss_config__max_spads_limit =
    (*(pbuffer +   4));
  pdata->gph__dss_config__min_spads_limit =
    (*(pbuffer +   5));


  return status;
}
VL53LX_Error VL53LX::VL53LX_set_gph_static_config(
  VL53LX_gph_static_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_gph_static_config(
               pdata,
               VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_GPH__DSS_CONFIG__ROI_MODE_CONTROL,
               comms_buffer,
               VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_get_gph_static_config(
  VL53LX_gph_static_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_GPH__DSS_CONFIG__ROI_MODE_CONTROL,
               comms_buffer,
               VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_gph_static_config(
               VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);

  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_gph_timing_config(
  VL53LX_gph_timing_config_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->gph__mm_config__timeout_macrop_a_hi & 0xF;
  *(pbuffer +   1) =
    pdata->gph__mm_config__timeout_macrop_a_lo;
  *(pbuffer +   2) =
    pdata->gph__mm_config__timeout_macrop_b_hi & 0xF;
  *(pbuffer +   3) =
    pdata->gph__mm_config__timeout_macrop_b_lo;
  *(pbuffer +   4) =
    pdata->gph__range_config__timeout_macrop_a_hi & 0xF;
  *(pbuffer +   5) =
    pdata->gph__range_config__timeout_macrop_a_lo;
  *(pbuffer +   6) =
    pdata->gph__range_config__vcsel_period_a & 0x3F;
  *(pbuffer +   7) =
    pdata->gph__range_config__vcsel_period_b & 0x3F;
  *(pbuffer +   8) =
    pdata->gph__range_config__timeout_macrop_b_hi & 0xF;
  *(pbuffer +   9) =
    pdata->gph__range_config__timeout_macrop_b_lo;
  VL53LX_i2c_encode_uint16_t(
    pdata->gph__range_config__sigma_thresh,
    2,
    pbuffer +  10);
  VL53LX_i2c_encode_uint16_t(
    pdata->gph__range_config__min_count_rate_rtn_limit_mcps,
    2,
    pbuffer +  12);
  *(pbuffer +  14) =
    pdata->gph__range_config__valid_phase_low;
  *(pbuffer +  15) =
    pdata->gph__range_config__valid_phase_high;

  return status;
}


VL53LX_Error VL53LX::VL53LX_i2c_decode_gph_timing_config(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_gph_timing_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->gph__mm_config__timeout_macrop_a_hi =
    (*(pbuffer +   0)) & 0xF;
  pdata->gph__mm_config__timeout_macrop_a_lo =
    (*(pbuffer +   1));
  pdata->gph__mm_config__timeout_macrop_b_hi =
    (*(pbuffer +   2)) & 0xF;
  pdata->gph__mm_config__timeout_macrop_b_lo =
    (*(pbuffer +   3));
  pdata->gph__range_config__timeout_macrop_a_hi =
    (*(pbuffer +   4)) & 0xF;
  pdata->gph__range_config__timeout_macrop_a_lo =
    (*(pbuffer +   5));
  pdata->gph__range_config__vcsel_period_a =
    (*(pbuffer +   6)) & 0x3F;
  pdata->gph__range_config__vcsel_period_b =
    (*(pbuffer +   7)) & 0x3F;
  pdata->gph__range_config__timeout_macrop_b_hi =
    (*(pbuffer +   8)) & 0xF;
  pdata->gph__range_config__timeout_macrop_b_lo =
    (*(pbuffer +   9));
  pdata->gph__range_config__sigma_thresh =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  10));
  pdata->gph__range_config__min_count_rate_rtn_limit_mcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  12));
  pdata->gph__range_config__valid_phase_low =
    (*(pbuffer +  14));
  pdata->gph__range_config__valid_phase_high =
    (*(pbuffer +  15));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_gph_timing_config(
  VL53LX_gph_timing_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_gph_timing_config(
               pdata,
               VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI,
               comms_buffer,
               VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_get_gph_timing_config(
  VL53LX_gph_timing_config_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI,
               comms_buffer,
               VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_gph_timing_config(
               VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_fw_internal(
  VL53LX_fw_internal_t     *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_FW_INTERNAL_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->firmware__internal_stream_count_div;
  *(pbuffer +   1) =
    pdata->firmware__internal_stream_counter_val;

  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_decode_fw_internal(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_fw_internal_t      *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_FW_INTERNAL_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->firmware__internal_stream_count_div =
    (*(pbuffer +   0));
  pdata->firmware__internal_stream_counter_val =
    (*(pbuffer +   1));

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_fw_internal(
  VL53LX_fw_internal_t      *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_FW_INTERNAL_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_fw_internal(
               pdata,
               VL53LX_FW_INTERNAL_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_FIRMWARE__INTERNAL_STREAM_COUNT_DIV,
               comms_buffer,
               VL53LX_FW_INTERNAL_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}
VL53LX_Error VL53LX::VL53LX_get_fw_internal(
  VL53LX_fw_internal_t      *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_FW_INTERNAL_I2C_SIZE_BYTES];

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_FIRMWARE__INTERNAL_STREAM_COUNT_DIV,
               comms_buffer,
               VL53LX_FW_INTERNAL_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_fw_internal(
               VL53LX_FW_INTERNAL_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_encode_patch_results(
  VL53LX_patch_results_t   *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->dss_calc__roi_ctrl & 0x3;
  *(pbuffer +   1) =
    pdata->dss_calc__spare_1;
  *(pbuffer +   2) =
    pdata->dss_calc__spare_2;
  *(pbuffer +   3) =
    pdata->dss_calc__spare_3;
  *(pbuffer +   4) =
    pdata->dss_calc__spare_4;
  *(pbuffer +   5) =
    pdata->dss_calc__spare_5;
  *(pbuffer +   6) =
    pdata->dss_calc__spare_6;
  *(pbuffer +   7) =
    pdata->dss_calc__spare_7;
  *(pbuffer +   8) =
    pdata->dss_calc__user_roi_spad_en_0;
  *(pbuffer +   9) =
    pdata->dss_calc__user_roi_spad_en_1;
  *(pbuffer +  10) =
    pdata->dss_calc__user_roi_spad_en_2;
  *(pbuffer +  11) =
    pdata->dss_calc__user_roi_spad_en_3;
  *(pbuffer +  12) =
    pdata->dss_calc__user_roi_spad_en_4;
  *(pbuffer +  13) =
    pdata->dss_calc__user_roi_spad_en_5;
  *(pbuffer +  14) =
    pdata->dss_calc__user_roi_spad_en_6;
  *(pbuffer +  15) =
    pdata->dss_calc__user_roi_spad_en_7;
  *(pbuffer +  16) =
    pdata->dss_calc__user_roi_spad_en_8;
  *(pbuffer +  17) =
    pdata->dss_calc__user_roi_spad_en_9;
  *(pbuffer +  18) =
    pdata->dss_calc__user_roi_spad_en_10;
  *(pbuffer +  19) =
    pdata->dss_calc__user_roi_spad_en_11;
  *(pbuffer +  20) =
    pdata->dss_calc__user_roi_spad_en_12;
  *(pbuffer +  21) =
    pdata->dss_calc__user_roi_spad_en_13;
  *(pbuffer +  22) =
    pdata->dss_calc__user_roi_spad_en_14;
  *(pbuffer +  23) =
    pdata->dss_calc__user_roi_spad_en_15;
  *(pbuffer +  24) =
    pdata->dss_calc__user_roi_spad_en_16;
  *(pbuffer +  25) =
    pdata->dss_calc__user_roi_spad_en_17;
  *(pbuffer +  26) =
    pdata->dss_calc__user_roi_spad_en_18;
  *(pbuffer +  27) =
    pdata->dss_calc__user_roi_spad_en_19;
  *(pbuffer +  28) =
    pdata->dss_calc__user_roi_spad_en_20;
  *(pbuffer +  29) =
    pdata->dss_calc__user_roi_spad_en_21;
  *(pbuffer +  30) =
    pdata->dss_calc__user_roi_spad_en_22;
  *(pbuffer +  31) =
    pdata->dss_calc__user_roi_spad_en_23;
  *(pbuffer +  32) =
    pdata->dss_calc__user_roi_spad_en_24;
  *(pbuffer +  33) =
    pdata->dss_calc__user_roi_spad_en_25;
  *(pbuffer +  34) =
    pdata->dss_calc__user_roi_spad_en_26;
  *(pbuffer +  35) =
    pdata->dss_calc__user_roi_spad_en_27;
  *(pbuffer +  36) =
    pdata->dss_calc__user_roi_spad_en_28;
  *(pbuffer +  37) =
    pdata->dss_calc__user_roi_spad_en_29;
  *(pbuffer +  38) =
    pdata->dss_calc__user_roi_spad_en_30;
  *(pbuffer +  39) =
    pdata->dss_calc__user_roi_spad_en_31;
  *(pbuffer +  40) =
    pdata->dss_calc__user_roi_0;
  *(pbuffer +  41) =
    pdata->dss_calc__user_roi_1;
  *(pbuffer +  42) =
    pdata->dss_calc__mode_roi_0;
  *(pbuffer +  43) =
    pdata->dss_calc__mode_roi_1;
  *(pbuffer +  44) =
    pdata->sigma_estimator_calc__spare_0;
  VL53LX_i2c_encode_uint16_t(
    pdata->vhv_result__peak_signal_rate_mcps,
    2,
    pbuffer +  46);
  VL53LX_i2c_encode_uint32_t(
    pdata->vhv_result__signal_total_events_ref,
    4,
    pbuffer +  48);
  VL53LX_i2c_encode_uint16_t(
    pdata->phasecal_result__phase_output_ref,
    2,
    pbuffer +  52);
  VL53LX_i2c_encode_uint16_t(
    pdata->dss_result__total_rate_per_spad,
    2,
    pbuffer +  54);
  *(pbuffer +  56) =
    pdata->dss_result__enabled_blocks;
  VL53LX_i2c_encode_uint16_t(
    pdata->dss_result__num_requested_spads,
    2,
    pbuffer +  58);
  VL53LX_i2c_encode_uint16_t(
    pdata->mm_result__inner_intersection_rate,
    2,
    pbuffer +  62);
  VL53LX_i2c_encode_uint16_t(
    pdata->mm_result__outer_complement_rate,
    2,
    pbuffer +  64);
  VL53LX_i2c_encode_uint16_t(
    pdata->mm_result__total_offset,
    2,
    pbuffer +  66);
  VL53LX_i2c_encode_uint32_t(
    pdata->xtalk_calc__xtalk_for_enabled_spads & 0xFFFFFF,
    4,
    pbuffer +  68);
  VL53LX_i2c_encode_uint32_t(
    pdata->xtalk_result__avg_xtalk_user_roi_kcps & 0xFFFFFF,
    4,
    pbuffer +  72);
  VL53LX_i2c_encode_uint32_t(
    pdata->xtalk_result__avg_xtalk_mm_inner_roi_kcps & 0xFFFFFF,
    4,
    pbuffer +  76);
  VL53LX_i2c_encode_uint32_t(
    pdata->xtalk_result__avg_xtalk_mm_outer_roi_kcps & 0xFFFFFF,
    4,
    pbuffer +  80);
  VL53LX_i2c_encode_uint32_t(
    pdata->range_result__accum_phase,
    4,
    pbuffer +  84);
  VL53LX_i2c_encode_uint16_t(
    pdata->range_result__offset_corrected_range,
    2,
    pbuffer +  88);

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_patch_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_patch_results_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->dss_calc__roi_ctrl =
    (*(pbuffer +   0)) & 0x3;
  pdata->dss_calc__spare_1 =
    (*(pbuffer +   1));
  pdata->dss_calc__spare_2 =
    (*(pbuffer +   2));
  pdata->dss_calc__spare_3 =
    (*(pbuffer +   3));
  pdata->dss_calc__spare_4 =
    (*(pbuffer +   4));
  pdata->dss_calc__spare_5 =
    (*(pbuffer +   5));
  pdata->dss_calc__spare_6 =
    (*(pbuffer +   6));
  pdata->dss_calc__spare_7 =
    (*(pbuffer +   7));
  pdata->dss_calc__user_roi_spad_en_0 =
    (*(pbuffer +   8));
  pdata->dss_calc__user_roi_spad_en_1 =
    (*(pbuffer +   9));
  pdata->dss_calc__user_roi_spad_en_2 =
    (*(pbuffer +  10));
  pdata->dss_calc__user_roi_spad_en_3 =
    (*(pbuffer +  11));
  pdata->dss_calc__user_roi_spad_en_4 =
    (*(pbuffer +  12));
  pdata->dss_calc__user_roi_spad_en_5 =
    (*(pbuffer +  13));
  pdata->dss_calc__user_roi_spad_en_6 =
    (*(pbuffer +  14));
  pdata->dss_calc__user_roi_spad_en_7 =
    (*(pbuffer +  15));
  pdata->dss_calc__user_roi_spad_en_8 =
    (*(pbuffer +  16));
  pdata->dss_calc__user_roi_spad_en_9 =
    (*(pbuffer +  17));
  pdata->dss_calc__user_roi_spad_en_10 =
    (*(pbuffer +  18));
  pdata->dss_calc__user_roi_spad_en_11 =
    (*(pbuffer +  19));
  pdata->dss_calc__user_roi_spad_en_12 =
    (*(pbuffer +  20));
  pdata->dss_calc__user_roi_spad_en_13 =
    (*(pbuffer +  21));
  pdata->dss_calc__user_roi_spad_en_14 =
    (*(pbuffer +  22));
  pdata->dss_calc__user_roi_spad_en_15 =
    (*(pbuffer +  23));
  pdata->dss_calc__user_roi_spad_en_16 =
    (*(pbuffer +  24));
  pdata->dss_calc__user_roi_spad_en_17 =
    (*(pbuffer +  25));
  pdata->dss_calc__user_roi_spad_en_18 =
    (*(pbuffer +  26));
  pdata->dss_calc__user_roi_spad_en_19 =
    (*(pbuffer +  27));
  pdata->dss_calc__user_roi_spad_en_20 =
    (*(pbuffer +  28));
  pdata->dss_calc__user_roi_spad_en_21 =
    (*(pbuffer +  29));
  pdata->dss_calc__user_roi_spad_en_22 =
    (*(pbuffer +  30));
  pdata->dss_calc__user_roi_spad_en_23 =
    (*(pbuffer +  31));
  pdata->dss_calc__user_roi_spad_en_24 =
    (*(pbuffer +  32));
  pdata->dss_calc__user_roi_spad_en_25 =
    (*(pbuffer +  33));
  pdata->dss_calc__user_roi_spad_en_26 =
    (*(pbuffer +  34));
  pdata->dss_calc__user_roi_spad_en_27 =
    (*(pbuffer +  35));
  pdata->dss_calc__user_roi_spad_en_28 =
    (*(pbuffer +  36));
  pdata->dss_calc__user_roi_spad_en_29 =
    (*(pbuffer +  37));
  pdata->dss_calc__user_roi_spad_en_30 =
    (*(pbuffer +  38));
  pdata->dss_calc__user_roi_spad_en_31 =
    (*(pbuffer +  39));
  pdata->dss_calc__user_roi_0 =
    (*(pbuffer +  40));
  pdata->dss_calc__user_roi_1 =
    (*(pbuffer +  41));
  pdata->dss_calc__mode_roi_0 =
    (*(pbuffer +  42));
  pdata->dss_calc__mode_roi_1 =
    (*(pbuffer +  43));
  pdata->sigma_estimator_calc__spare_0 =
    (*(pbuffer +  44));
  pdata->vhv_result__peak_signal_rate_mcps =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  46));
  pdata->vhv_result__signal_total_events_ref =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  48));
  pdata->phasecal_result__phase_output_ref =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  52));
  pdata->dss_result__total_rate_per_spad =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  54));
  pdata->dss_result__enabled_blocks =
    (*(pbuffer +  56));
  pdata->dss_result__num_requested_spads =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  58));
  pdata->mm_result__inner_intersection_rate =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  62));
  pdata->mm_result__outer_complement_rate =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  64));
  pdata->mm_result__total_offset =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  66));
  pdata->xtalk_calc__xtalk_for_enabled_spads =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  68)) & 0xFFFFFF;
  pdata->xtalk_result__avg_xtalk_user_roi_kcps =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  72)) & 0xFFFFFF;
  pdata->xtalk_result__avg_xtalk_mm_inner_roi_kcps =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  76)) & 0xFFFFFF;
  pdata->xtalk_result__avg_xtalk_mm_outer_roi_kcps =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  80)) & 0xFFFFFF;
  pdata->range_result__accum_phase =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  84));
  pdata->range_result__offset_corrected_range =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  88));


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_patch_results(
  VL53LX_patch_results_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_patch_results(
               pdata,
               VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_DSS_CALC__ROI_CTRL,
               comms_buffer,
               VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_get_patch_results(
  VL53LX_patch_results_t    *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_DSS_CALC__ROI_CTRL,
               comms_buffer,
               VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_patch_results(
               VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_encode_shadow_system_results(
  VL53LX_shadow_system_results_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  *(pbuffer +   0) =
    pdata->shadow_phasecal_result__vcsel_start;
  *(pbuffer +   2) =
    pdata->shadow_result__interrupt_status & 0x3F;
  *(pbuffer +   3) =
    pdata->shadow_result__range_status;
  *(pbuffer +   4) =
    pdata->shadow_result__report_status & 0xF;
  *(pbuffer +   5) =
    pdata->shadow_result__stream_count;
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__dss_actual_effective_spads_sd0,
    2,
    pbuffer +   6);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__peak_signal_count_rate_mcps_sd0,
    2,
    pbuffer +   8);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__ambient_count_rate_mcps_sd0,
    2,
    pbuffer +  10);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__sigma_sd0,
    2,
    pbuffer +  12);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__phase_sd0,
    2,
    pbuffer +  14);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__final_crosstalk_corrected_range_mm_sd0,
    2,
    pbuffer +  16);
  VL53LX_i2c_encode_uint16_t(
    pdata->shr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0,
    2,
    pbuffer +  18);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__mm_inner_actual_effective_spads_sd0,
    2,
    pbuffer +  20);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__mm_outer_actual_effective_spads_sd0,
    2,
    pbuffer +  22);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__avg_signal_count_rate_mcps_sd0,
    2,
    pbuffer +  24);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__dss_actual_effective_spads_sd1,
    2,
    pbuffer +  26);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__peak_signal_count_rate_mcps_sd1,
    2,
    pbuffer +  28);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__ambient_count_rate_mcps_sd1,
    2,
    pbuffer +  30);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__sigma_sd1,
    2,
    pbuffer +  32);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__phase_sd1,
    2,
    pbuffer +  34);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__final_crosstalk_corrected_range_mm_sd1,
    2,
    pbuffer +  36);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__spare_0_sd1,
    2,
    pbuffer +  38);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__spare_1_sd1,
    2,
    pbuffer +  40);
  VL53LX_i2c_encode_uint16_t(
    pdata->shadow_result__spare_2_sd1,
    2,
    pbuffer +  42);
  *(pbuffer +  44) =
    pdata->shadow_result__spare_3_sd1;
  *(pbuffer +  45) =
    pdata->shadow_result__thresh_info;
  *(pbuffer +  80) =
    pdata->shadow_phasecal_result__reference_phase_hi;
  *(pbuffer +  81) =
    pdata->shadow_phasecal_result__reference_phase_lo;

  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_shadow_system_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_shadow_system_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->shadow_phasecal_result__vcsel_start =
    (*(pbuffer +   0));
  pdata->shadow_result__interrupt_status =
    (*(pbuffer +   2)) & 0x3F;
  pdata->shadow_result__range_status =
    (*(pbuffer +   3));
  pdata->shadow_result__report_status =
    (*(pbuffer +   4)) & 0xF;
  pdata->shadow_result__stream_count =
    (*(pbuffer +   5));
  pdata->shadow_result__dss_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   6));
  pdata->shadow_result__peak_signal_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +   8));
  pdata->shadow_result__ambient_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  10));
  pdata->shadow_result__sigma_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  12));
  pdata->shadow_result__phase_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  14));
  pdata->shadow_result__final_crosstalk_corrected_range_mm_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  16));
  pdata->shr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  18));
  pdata->shadow_result__mm_inner_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  20));
  pdata->shadow_result__mm_outer_actual_effective_spads_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  22));
  pdata->shadow_result__avg_signal_count_rate_mcps_sd0 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  24));
  pdata->shadow_result__dss_actual_effective_spads_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  26));
  pdata->shadow_result__peak_signal_count_rate_mcps_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  28));
  pdata->shadow_result__ambient_count_rate_mcps_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  30));
  pdata->shadow_result__sigma_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  32));
  pdata->shadow_result__phase_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  34));
  pdata->shadow_result__final_crosstalk_corrected_range_mm_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  36));
  pdata->shadow_result__spare_0_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  38));
  pdata->shadow_result__spare_1_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  40));
  pdata->shadow_result__spare_2_sd1 =
    (VL53LX_i2c_decode_uint16_t(2, pbuffer +  42));
  pdata->shadow_result__spare_3_sd1 =
    (*(pbuffer +  44));
  pdata->shadow_result__thresh_info =
    (*(pbuffer +  45));
  pdata->shadow_phasecal_result__reference_phase_hi =
    (*(pbuffer +  80));
  pdata->shadow_phasecal_result__reference_phase_lo =
    (*(pbuffer +  81));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_shadow_system_results(
  VL53LX_shadow_system_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_shadow_system_results(
               pdata,
               VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_SHADOW_PHASECAL_RESULT__VCSEL_START,
               comms_buffer,
               VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_get_shadow_system_results(
  VL53LX_shadow_system_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_SHADOW_PHASECAL_RESULT__VCSEL_START,
               comms_buffer,
               VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_shadow_system_results(
               VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}
VL53LX_Error VL53LX::VL53LX_i2c_encode_shadow_core_results(
  VL53LX_shadow_core_results_t *pdata,
  uint16_t                  buf_size,
  uint8_t                  *pbuffer)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  VL53LX_i2c_encode_uint32_t(
    pdata->shadow_result_core__ambient_window_events_sd0,
    4,
    pbuffer +   0);
  VL53LX_i2c_encode_uint32_t(
    pdata->shadow_result_core__ranging_total_events_sd0,
    4,
    pbuffer +   4);
  VL53LX_i2c_encode_int32_t(
    pdata->shadow_result_core__signal_total_events_sd0,
    4,
    pbuffer +   8);
  VL53LX_i2c_encode_uint32_t(
    pdata->shadow_result_core__total_periods_elapsed_sd0,
    4,
    pbuffer +  12);
  VL53LX_i2c_encode_uint32_t(
    pdata->shadow_result_core__ambient_window_events_sd1,
    4,
    pbuffer +  16);
  VL53LX_i2c_encode_uint32_t(
    pdata->shadow_result_core__ranging_total_events_sd1,
    4,
    pbuffer +  20);
  VL53LX_i2c_encode_int32_t(
    pdata->shadow_result_core__signal_total_events_sd1,
    4,
    pbuffer +  24);
  VL53LX_i2c_encode_uint32_t(
    pdata->shadow_result_core__total_periods_elapsed_sd1,
    4,
    pbuffer +  28);
  *(pbuffer +  32) =
    pdata->shadow_result_core__spare_0;



  return status;
}

VL53LX_Error VL53LX::VL53LX_i2c_decode_shadow_core_results(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_shadow_core_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (buf_size < VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES) {
    return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
  }

  pdata->shadow_result_core__ambient_window_events_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +   0));
  pdata->shadow_result_core__ranging_total_events_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +   4));
  pdata->shadow_result_core__signal_total_events_sd0 =
    (VL53LX_i2c_decode_int32_t(4, pbuffer +   8));
  pdata->shadow_result_core__total_periods_elapsed_sd0 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  12));
  pdata->shadow_result_core__ambient_window_events_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  16));
  pdata->shadow_result_core__ranging_total_events_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  20));
  pdata->shadow_result_core__signal_total_events_sd1 =
    (VL53LX_i2c_decode_int32_t(4, pbuffer +  24));
  pdata->shadow_result_core__total_periods_elapsed_sd1 =
    (VL53LX_i2c_decode_uint32_t(4, pbuffer +  28));
  pdata->shadow_result_core__spare_0 =
    (*(pbuffer +  32));


  return status;
}

VL53LX_Error VL53LX::VL53LX_set_shadow_core_results(
  VL53LX_shadow_core_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_encode_shadow_core_results(
               pdata,
               VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES,
               comms_buffer);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WriteMulti(
               Dev,
               VL53LX_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0,
               comms_buffer,
               VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_get_shadow_core_results(
  VL53LX_shadow_core_results_t  *pdata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t comms_buffer[VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES];


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_ReadMulti(
               Dev,
               VL53LX_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0,
               comms_buffer,
               VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_i2c_decode_shadow_core_results(
               VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES,
               comms_buffer,
               pdata);


  return status;
}


/* vl53lx_nvm.c */

VL53LX_Error VL53LX::VL53LX_nvm_enable(
  uint16_t        nvm_ctrl_pulse_width,
  int32_t         nvm_power_up_delay_us)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }




  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_powerforce();
  }



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WaitUs(
               Dev,
               VL53LX_ENABLE_POWERFORCE_SETTLING_TIME_US);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_RANGING_CORE__NVM_CTRL__PDN,
               0x01);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_RANGING_CORE__CLK_CTRL1,
               0x05);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WaitUs(
               Dev,
               nvm_power_up_delay_us);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_RANGING_CORE__NVM_CTRL__MODE,
               0x01);

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrWord(
               Dev,
               VL53LX_RANGING_CORE__NVM_CTRL__PULSE_WIDTH_MSB,
               nvm_ctrl_pulse_width);

  return status;

}

VL53LX_Error VL53LX::VL53LX_nvm_read(
  uint8_t       start_address,
  uint8_t       count,
  uint8_t      *pdata)
{


  VL53LX_Error status   = VL53LX_ERROR_NONE;
  uint8_t      nvm_addr = 0;



  for (nvm_addr = start_address;
       nvm_addr < (start_address + count) ; nvm_addr++) {



    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_WrByte(
                 Dev,
                 VL53LX_RANGING_CORE__NVM_CTRL__ADDR,
                 nvm_addr);



    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_WrByte(
                 Dev,
                 VL53LX_RANGING_CORE__NVM_CTRL__READN,
                 0x00);



    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_WaitUs(
                 Dev,
                 VL53LX_NVM_READ_TRIGGER_DELAY_US);

    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_WrByte(
                 Dev,
                 VL53LX_RANGING_CORE__NVM_CTRL__READN,
                 0x01);


    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_ReadMulti(
                 Dev,
                 VL53LX_RANGING_CORE__NVM_CTRL__DATAOUT_MMM,
                 pdata,
                 4);


    pdata = pdata + 4;


  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_nvm_disable()
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_RANGING_CORE__NVM_CTRL__READN,
               0x01);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_WrByte(
               Dev,
               VL53LX_RANGING_CORE__NVM_CTRL__PDN,
               0x00);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_powerforce();
  }



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }


  return status;

}


VL53LX_Error VL53LX::VL53LX_nvm_format_decode(
  uint16_t                   buf_size,
  uint8_t                   *pbuffer,
  VL53LX_decoded_nvm_data_t *pdata)
{



  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t    i        = 0;
  uint8_t   *ptmp     = NULL;
  int        pptmp[VL53LX_NVM_MAX_FMT_RANGE_DATA];

  if (buf_size < VL53LX_NVM_SIZE_IN_BYTES) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }

  pdata->nvm__identification_model_id =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__IDENTIFICATION__MODEL_ID,
      0x000000FF,
      0,
      0);
  pdata->nvm__identification_module_type =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__IDENTIFICATION__MODULE_TYPE,
      0x000000FF,
      0,
      0);
  pdata->nvm__identification_revision_id =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__IDENTIFICATION__REVISION_ID,
      0x0000000F,
      0,
      0);
  pdata->nvm__identification_module_id =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__IDENTIFICATION__MODULE_ID,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__i2c_valid =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__I2C_VALID,
      0x000000FF,
      0,
      0);
  pdata->nvm__i2c_device_address_ews =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__I2C_SLAVE__DEVICE_ADDRESS,
      0x000000FF,
      0,
      0);
  pdata->nvm__ews__fast_osc_frequency =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__EWS__OSC_MEASURED__FAST_OSC_FREQUENCY,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__ews__fast_osc_trim_max =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__FAST_OSC_TRIM_MAX,
      0x0000007F,
      0,
      0);
  pdata->nvm__ews__fast_osc_freq_set =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__FAST_OSC_FREQ_SET,
      0x00000007,
      0,
      0);
  pdata->nvm__ews__slow_osc_calibration =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__EWS__SLOW_OSC_CALIBRATION,
      0x000003FF,
      0,
      0);
  pdata->nvm__fmt__fast_osc_frequency =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__FMT__OSC_MEASURED__FAST_OSC_FREQUENCY,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__fast_osc_trim_max =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FAST_OSC_TRIM_MAX,
      0x0000007F,
      0,
      0);
  pdata->nvm__fmt__fast_osc_freq_set =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FAST_OSC_FREQ_SET,
      0x00000007,
      0,
      0);
  pdata->nvm__fmt__slow_osc_calibration =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__FMT__SLOW_OSC_CALIBRATION,
      0x000003FF,
      0,
      0);
  pdata->nvm__vhv_config_unlock =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__VHV_CONFIG_UNLOCK,
      0x000000FF,
      0,
      0);
  pdata->nvm__ref_selvddpix =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__REF_SELVDDPIX,
      0x0000000F,
      0,
      0);
  pdata->nvm__ref_selvquench =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__REF_SELVQUENCH,
      0x00000078,
      3,
      0);
  pdata->nvm__regavdd1v2_sel =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__REGAVDD1V2_SEL_REGDVDD1V2_SEL,
      0x0000000C,
      2,
      0);
  pdata->nvm__regdvdd1v2_sel =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__REGAVDD1V2_SEL_REGDVDD1V2_SEL,
      0x00000003,
      0,
      0);
  pdata->nvm__vhv_timeout__macrop =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
      0x00000003,
      0,
      0);
  pdata->nvm__vhv_loop_bound =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
      0x000000FC,
      2,
      0);
  pdata->nvm__vhv_count_threshold =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__VHV_CONFIG__COUNT_THRESH,
      0x000000FF,
      0,
      0);
  pdata->nvm__vhv_offset =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__VHV_CONFIG__OFFSET,
      0x0000003F,
      0,
      0);
  pdata->nvm__vhv_init_enable =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__VHV_CONFIG__INIT,
      0x00000080,
      7,
      0);
  pdata->nvm__vhv_init_value =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__VHV_CONFIG__INIT,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_vcsel_trim_ll =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_TRIM_LL,
      0x00000007,
      0,
      0);
  pdata->nvm__laser_safety_vcsel_selion_ll =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_LL,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_vcsel_selion_max_ll =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_MAX_LL,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_mult_ll =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__MULT_LL,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_clip_ll =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__CLIP_LL,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_vcsel_trim_ld =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_TRIM_LD,
      0x00000007,
      0,
      0);
  pdata->nvm__laser_safety_vcsel_selion_ld =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_LD,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_vcsel_selion_max_ld =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_MAX_LD,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_mult_ld =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__MULT_LD,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_clip_ld =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY__CLIP_LD,
      0x0000003F,
      0,
      0);
  pdata->nvm__laser_safety_lock_byte =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY_LOCK_BYTE,
      0x000000FF,
      0,
      0);
  pdata->nvm__laser_safety_unlock_byte =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__LASER_SAFETY_UNLOCK_BYTE,
      0x000000FF,
      0,
      0);



  ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_RTN_0_;
  for (i = 0 ; i < VL53LX_RTN_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__ews__spad_enables_rtn[i] = *ptmp++;
  }

  ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_REF__LOC1_0_;
  for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__ews__spad_enables_ref__loc1[i] = *ptmp++;
  }

  ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_REF__LOC2_0_;
  for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__ews__spad_enables_ref__loc2[i] = *ptmp++;
  }

  ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_REF__LOC3_0_;
  for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__ews__spad_enables_ref__loc3[i] = *ptmp++;
  }



  ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_RTN_0_;
  for (i = 0 ; i < VL53LX_RTN_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__fmt__spad_enables_rtn[i] = *ptmp++;
  }

  ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_REF__LOC1_0_;
  for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__fmt__spad_enables_ref__loc1[i] = *ptmp++;
  }

  ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_REF__LOC2_0_;
  for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__fmt__spad_enables_ref__loc2[i] = *ptmp++;
  }

  ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_REF__LOC3_0_;
  for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
    pdata->nvm__fmt__spad_enables_ref__loc3[i] = *ptmp++;
  }


  pdata->nvm__fmt__roi_config__mode_roi_centre_spad =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__FMT__ROI_CONFIG__MODE_ROI_CENTRE_SPAD,
      0x000000FF,
      0,
      0);
  pdata->nvm__fmt__roi_config__mode_roi_x_size =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__FMT__ROI_CONFIG__MODE_ROI_XY_SIZE,
      0x000000F0,
      4,
      0);
  pdata->nvm__fmt__roi_config__mode_roi_y_size =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__ROI_CONFIG__MODE_ROI_XY_SIZE,
      0x0000000F,
      0,
      0);
  pdata->nvm__fmt__ref_spad_apply__num_requested_ref_spad =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__FMT__REF_SPAD_APPLY__NUM_REQUESTED_REF_SPAD,
      0x000000FF,
      0,
      0);
  pdata->nvm__fmt__ref_spad_man__ref_location =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__REF_SPAD_MAN__REF_LOCATION,
      0x00000003,
      0,
      0);
  pdata->nvm__fmt__mm_config__inner_offset_mm =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__FMT__MM_CONFIG__INNER_OFFSET_MM,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__mm_config__outer_offset_mm =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__FMT__MM_CONFIG__OUTER_OFFSET_MM,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__algo_part_to_part_range_offset_mm =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__FMT__ALGO__PART_TO_PART_RANGE_OFFSET_MM,
      0x00000FFF,
      0,
      0);
  pdata->nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__spare__host_config__nvm_config_spare_0 =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__FMT__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_0,
      0x000000FF,
      0,
      0);
  pdata->nvm__fmt__spare__host_config__nvm_config_spare_1 =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__FMT__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_1,
      0x000000FF,
      0,
      0);
  pdata->nvm__customer_space_programmed =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__CUSTOMER_NVM_SPACE_PROGRAMMED,
      0x000000FF,
      0,
      0);
  pdata->nvm__cust__i2c_device_address =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__CUST__I2C_SLAVE__DEVICE_ADDRESS,
      0x000000FF,
      0,
      0);
  pdata->nvm__cust__ref_spad_apply__num_requested_ref_spad =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__CUST__REF_SPAD_APPLY__NUM_REQUESTED_REF_SPAD,
      0x000000FF,
      0,
      0);
  pdata->nvm__cust__ref_spad_man__ref_location =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__CUST__REF_SPAD_MAN__REF_LOCATION,
      0x00000003,
      0,
      0);
  pdata->nvm__cust__mm_config__inner_offset_mm =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__CUST__MM_CONFIG__INNER_OFFSET_MM,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__cust__mm_config__outer_offset_mm =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__CUST__MM_CONFIG__OUTER_OFFSET_MM,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__cust__algo_part_to_part_range_offset_mm =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__CUST__ALGO__PART_TO_PART_RANGE_OFFSET_MM,
      0x00000FFF,
      0,
      0);
  pdata->nvm__cust__algo__crosstalk_compensation_plane_offset_kcps =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer +
      VL53LX_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__cust__spare__host_config__nvm_config_spare_0 =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__CUST__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_0,
      0x000000FF,
      0,
      0);
  pdata->nvm__cust__spare__host_config__nvm_config_spare_1 =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer +
      VL53LX_NVM__CUST__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_1,
      0x000000FF,
      0,
      0);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_optical_centre(
        buf_size,
        pbuffer + VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_INDEX,
        &(pdata->fmt_optical_centre));



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_cal_peak_rate_map(
        buf_size,
        pbuffer + VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_INDEX,
        &(pdata->fmt_peak_rate_map));



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_additional_offset_cal_data(
        buf_size,
        pbuffer +
        VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_INDEX,
        &(pdata->fmt_add_offset_data));



  pptmp[0] = VL53LX_NVM__FMT__RANGE_RESULTS__140MM_MM_PRE_RANGE;
  pptmp[1] = VL53LX_NVM__FMT__RANGE_RESULTS__140MM_DARK;
  pptmp[2] = VL53LX_NVM__FMT__RANGE_RESULTS__400MM_DARK;
  pptmp[3] = VL53LX_NVM__FMT__RANGE_RESULTS__400MM_AMBIENT;

  for (i = 0 ; i < VL53LX_NVM_MAX_FMT_RANGE_DATA ; i++) {
    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_nvm_decode_fmt_range_results_data(
          buf_size,
          pbuffer + pptmp[i],
          &(pdata->fmt_range_data[i]));
  }


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_fmt_info(
        buf_size,
        pbuffer,
        &(pdata->fmt_info));

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_ews_info(
        buf_size,
        pbuffer,
        &(pdata->ews_info));

  return status;

}
VL53LX_Error VL53LX::VL53LX_nvm_decode_optical_centre(
  uint16_t                    buf_size,
  uint8_t                    *pbuffer,
  VL53LX_optical_centre_t    *pdata)
{

  VL53LX_Error status   = VL53LX_ERROR_NONE;

  uint16_t  tmp = 0;

  if (buf_size < VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }


  tmp  = 0x0100;
  tmp -= (uint16_t) * (pbuffer + 2);
  if (tmp > 0x0FF) {
    tmp = 0;
  }

  pdata->x_centre = (uint8_t)tmp;
  pdata->y_centre = *(pbuffer + 3);

  return status;
}

VL53LX_Error VL53LX::VL53LX_nvm_decode_cal_peak_rate_map(
  uint16_t                    buf_size,
  uint8_t                    *pbuffer,
  VL53LX_cal_peak_rate_map_t *pdata)
{

  VL53LX_Error status   = VL53LX_ERROR_NONE;

  uint8_t   *ptmp = NULL;
  uint8_t       i = 0;

  if (buf_size < VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }

  pdata->cal_distance_mm =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer);

  pdata->cal_reflectance_pc =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 2);
  pdata->cal_reflectance_pc =
    pdata->cal_reflectance_pc >> 6;

  pdata->max_samples = VL53LX_NVM_PEAK_RATE_MAP_SAMPLES;
  pdata->width       = VL53LX_NVM_PEAK_RATE_MAP_WIDTH;
  pdata->height      = VL53LX_NVM_PEAK_RATE_MAP_HEIGHT;

  ptmp = pbuffer + 4;
  for (i = 0 ; i < VL53LX_NVM_PEAK_RATE_MAP_SAMPLES ; i++) {
    pdata->peak_rate_mcps[i] =
      (uint16_t)VL53LX_i2c_decode_uint16_t(2, ptmp);
    ptmp += 2;
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_nvm_decode_additional_offset_cal_data(
  uint16_t                             buf_size,
  uint8_t                             *pbuffer,
  VL53LX_additional_offset_cal_data_t *pdata)
{

  VL53LX_Error status   = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }

  pdata->result__mm_inner_actual_effective_spads =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer);

  pdata->result__mm_outer_actual_effective_spads =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 2);

  pdata->result__mm_inner_peak_signal_count_rtn_mcps =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 4);

  pdata->result__mm_outer_peak_signal_count_rtn_mcps =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 6);

  return status;
}


VL53LX_Error VL53LX::VL53LX_nvm_decode_fmt_range_results_data(
  uint16_t                             buf_size,
  uint8_t                             *pbuffer,
  VL53LX_decoded_nvm_fmt_range_data_t *pdata)
{

  VL53LX_Error status   = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }

  pdata->result__actual_effective_rtn_spads =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer);

  pdata->ref_spad_array__num_requested_ref_spads =
    *(pbuffer + 2);

  pdata->ref_spad_array__ref_location =
    *(pbuffer + 3);

  pdata->result__peak_signal_count_rate_rtn_mcps =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 4);

  pdata->result__ambient_count_rate_rtn_mcps =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 6);

  pdata->result__peak_signal_count_rate_ref_mcps =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 8);

  pdata->result__ambient_count_rate_ref_mcps =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 10);

  pdata->measured_distance_mm =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 12);

  pdata->measured_distance_stdev_mm =
    (uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 14);

  return status;
}


VL53LX_Error VL53LX::VL53LX_nvm_decode_fmt_info(
  uint16_t                       buf_size,
  uint8_t                       *pbuffer,
  VL53LX_decoded_nvm_fmt_info_t *pdata)
{

  VL53LX_Error status   = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_NVM_SIZE_IN_BYTES) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }

  pdata->nvm__fmt__fgc[0] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_0,
      0x000000FE,
      1,
      0);
  pdata->nvm__fmt__fgc[1] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_1,
      0x000001FC,
      2,
      0);
  pdata->nvm__fmt__fgc[2] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_2 - 1,
      0x000003F8,
      3,
      0);
  pdata->nvm__fmt__fgc[3] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_3 - 1,
      0x000007F0,
      4,
      0);
  pdata->nvm__fmt__fgc[4] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_4 - 1,
      0x00000FE0,
      5,
      0);
  pdata->nvm__fmt__fgc[5] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_5 - 1,
      0x00001FC0,
      6,
      0);
  pdata->nvm__fmt__fgc[6] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_6 - 1,
      0x00003F80,
      7,
      0);
  pdata->nvm__fmt__fgc[7] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_6,
      0x0000007F,
      0,
      0);
  pdata->nvm__fmt__fgc[8] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_7,
      0x000000FE,
      1,
      0);
  pdata->nvm__fmt__fgc[9] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_8,
      0x000001FC,
      2,
      0);
  pdata->nvm__fmt__fgc[10] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_9 - 1,
      0x000003F8,
      3,
      0);
  pdata->nvm__fmt__fgc[11] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_10 - 1,
      0x000007F0,
      4,
      0);
  pdata->nvm__fmt__fgc[12] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_11 - 1,
      0x00000FE0,
      5,
      0);
  pdata->nvm__fmt__fgc[13] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_12 - 1,
      0x00001FC0,
      6,
      0);
  pdata->nvm__fmt__fgc[14] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_13 - 1,
      0x00003F80,
      7,
      0);
  pdata->nvm__fmt__fgc[15] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_13,
      0x0000007F,
      0,
      0);
  pdata->nvm__fmt__fgc[16] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_14,
      0x000000FE,
      1,
      0);
  pdata->nvm__fmt__fgc[17] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__FGC__BYTE_15,
      0x000001FC,
      2,
      0);
  pdata->nvm__fmt__fgc[18] = 0x00;

  pdata->nvm__fmt__test_program_major =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__TEST_PROGRAM_MAJOR_MINOR,
      0x000000E0,
      5,
      0);
  pdata->nvm__fmt__test_program_minor =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__TEST_PROGRAM_MAJOR_MINOR,
      0x0000001F,
      0,
      0);
  pdata->nvm__fmt__map_major =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__MAP_MAJOR_MINOR,
      0x000000E0,
      5,
      0);
  pdata->nvm__fmt__map_minor =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__MAP_MAJOR_MINOR,
      0x0000001F,
      0,
      0);
  pdata->nvm__fmt__year =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__YEAR_MONTH,
      0x000000F0,
      4,
      0);
  pdata->nvm__fmt__month =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__YEAR_MONTH,
      0x0000000F,
      0,
      0);
  pdata->nvm__fmt__day =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__DAY_MODULE_DATE_PHASE,
      0x000000F8,
      3,
      0);
  pdata->nvm__fmt__module_date_phase =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__DAY_MODULE_DATE_PHASE,
      0x00000007,
      0,
      0);
  pdata->nvm__fmt__time =
    (uint16_t)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__FMT__TIME,
      0x0000FFFF,
      0,
      0);
  pdata->nvm__fmt__tester_id =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__TESTER_ID,
      0x000000FF,
      0,
      0);
  pdata->nvm__fmt__site_id =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__FMT__SITE_ID,
      0x000000FF,
      0,
      0);

  return status;
}


VL53LX_Error VL53LX::VL53LX_nvm_decode_ews_info(
  uint16_t                       buf_size,
  uint8_t                       *pbuffer,
  VL53LX_decoded_nvm_ews_info_t *pdata)
{

  VL53LX_Error status   = VL53LX_ERROR_NONE;

  if (buf_size < VL53LX_NVM_SIZE_IN_BYTES) {
    return VL53LX_ERROR_BUFFER_TOO_SMALL;
  }

  pdata->nvm__ews__test_program_major =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__TEST_PROGRAM_MAJOR_MINOR,
      0x000000E0,
      5,
      0);
  pdata->nvm__ews__test_program_minor =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__TEST_PROGRAM_MAJOR_MINOR,
      0x0000001F,
      0,
      0);
  pdata->nvm__ews__probe_card_major =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__PROBE_CARD_MAJOR_MINOR,
      0x000000F0,
      4,
      0);
  pdata->nvm__ews__probe_card_minor =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__PROBE_CARD_MAJOR_MINOR,
      0x0000000F,
      0,
      0);
  pdata->nvm__ews__tester_id =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__TESTER_ID,
      0x000000FF,
      0,
      0);
  pdata->nvm__ews__lot[0] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_0,
      0x000000FC,
      2,
      32);
  pdata->nvm__ews__lot[1] =
    (char)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_1 - 1,
      0x000003F0,
      4,
      32);
  pdata->nvm__ews__lot[2] =
    (char)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_2 - 1,
      0x00000FC0,
      6,
      32);
  pdata->nvm__ews__lot[3] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_2,
      0x0000003F,
      0,
      32);
  pdata->nvm__ews__lot[4] =
    (char)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_3,
      0x000000FC,
      2,
      32);
  pdata->nvm__ews__lot[5] =
    (char)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_4 - 1,
      0x000003F0,
      4,
      32);
  pdata->nvm__ews__lot[6] =
    (char)VL53LX_i2c_decode_with_mask(
      2,
      pbuffer + VL53LX_NVM__EWS__LOT__BYTE_5 - 1,
      0x00000FC0,
      6,
      32);

  pdata->nvm__ews__lot[7] = 0x00;

  pdata->nvm__ews__wafer =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__WAFER,
      0x0000001F,
      0,
      0);
  pdata->nvm__ews__xcoord =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__XCOORD,
      0x000000FF,
      0,
      0);
  pdata->nvm__ews__ycoord =
    (uint8_t)VL53LX_i2c_decode_with_mask(
      1,
      pbuffer + VL53LX_NVM__EWS__YCOORD,
      0x000000FF,
      0,
      0);

  return status;

}


void VL53LX::VL53LX_nvm_format_encode(
  VL53LX_decoded_nvm_data_t *pnvm_info,
  uint8_t                   *pnvm_data)
{
  SUPPRESS_UNUSED_WARNING(pnvm_info);
  SUPPRESS_UNUSED_WARNING(pnvm_data);
}

VL53LX_Error VL53LX::VL53LX_read_nvm_raw_data(
  uint8_t        start_address,
  uint8_t        count,
  uint8_t       *pnvm_raw_data)
{



  VL53LX_Error status = VL53LX_ERROR_NONE;




  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_nvm_enable(
               0x0004,
               VL53LX_NVM_POWER_UP_DELAY_US);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_nvm_read(
               start_address,
               count,
               pnvm_raw_data);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_nvm_disable();
  }


  return status;

}

VL53LX_Error VL53LX::VL53LX_read_nvm(
  uint8_t                    nvm_format,
  VL53LX_decoded_nvm_data_t *pnvm_info)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;


  uint8_t nvm_data[2 * VL53LX_NVM_SIZE_IN_BYTES];


  SUPPRESS_UNUSED_WARNING(nvm_format);



  status = VL53LX_read_nvm_raw_data(
             0,
             VL53LX_NVM_SIZE_IN_BYTES >> 2,
             nvm_data);





  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_nvm_format_decode(
               VL53LX_NVM_SIZE_IN_BYTES,
               nvm_data,
               pnvm_info);

  return status;

}

VL53LX_Error VL53LX::VL53LX_read_nvm_optical_centre(
  VL53LX_optical_centre_t          *pcentre)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  uint8_t nvm_data[2 * VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE];

  status =
    VL53LX_read_nvm_raw_data(
      (uint8_t)(VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_INDEX
                >> 2),
      (uint8_t)(VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE
                >> 2),
      nvm_data);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_optical_centre(
        VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE,
        nvm_data,
        pcentre);

  return status;
}
VL53LX_Error VL53LX::VL53LX_read_nvm_cal_peak_rate_map(
  VL53LX_cal_peak_rate_map_t          *pcal_data)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  uint8_t nvm_data[2 * VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE];


  status =
    VL53LX_read_nvm_raw_data(
      (uint8_t)(VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_INDEX
                >> 2),
      (uint8_t)(VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE
                >> 2),
      nvm_data);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_cal_peak_rate_map(
        VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE,
        nvm_data,
        pcal_data);


  return status;
}


VL53LX_Error VL53LX::VL53LX_read_nvm_additional_offset_cal_data(
  VL53LX_additional_offset_cal_data_t *pcal_data)
{



  VL53LX_Error status = VL53LX_ERROR_NONE;


  uint8_t nvm_data[2 * VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE];

  status =
    VL53LX_read_nvm_raw_data(
      (uint8_t)(
        VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_INDEX >> 2),
      (uint8_t)(
        VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE >> 2),
      nvm_data);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_nvm_decode_additional_offset_cal_data(
               VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE,
               nvm_data,
               pcal_data);


  return status;

}

VL53LX_Error VL53LX::VL53LX_read_nvm_fmt_range_results_data(
  uint16_t                             range_results_select,
  VL53LX_decoded_nvm_fmt_range_data_t *prange_data)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;


  uint8_t nvm_data[2 * VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES];

  status = VL53LX_read_nvm_raw_data(
             (uint8_t)(range_results_select >> 2),
             (uint8_t)(VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES >> 2),
             nvm_data);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_nvm_decode_fmt_range_results_data(
        VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES,
        nvm_data,
        prange_data);

  return status;

}

/* vl53lx_platform_ipp.c */

VL53LX_Error VL53LX::VL53LX_ipp_hist_process_data(
  VL53LX_dmax_calibration_data_t    *pdmax_cal,
  VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
  VL53LX_hist_post_process_config_t *ppost_cfg,
  VL53LX_histogram_bin_data_t       *pbins,
  VL53LX_xtalk_histogram_data_t     *pxtalk,
  uint8_t                           *pArea1,
  uint8_t                           *pArea2,
  uint8_t                           *phisto_merge_nb,
  VL53LX_range_results_t            *presults)
{

  VL53LX_Error status         = VL53LX_ERROR_NONE;

  SUPPRESS_UNUSED_WARNING(Dev);

  status =
    VL53LX_hist_process_data(
      pdmax_cal,
      pdmax_cfg,
      ppost_cfg,
      pbins,
      pxtalk,
      pArea1,
      pArea2,
      presults,
      phisto_merge_nb);

  return status;
}


VL53LX_Error VL53LX::VL53LX_ipp_hist_ambient_dmax(
  uint16_t                           target_reflectance,
  VL53LX_dmax_calibration_data_t    *pdmax_cal,
  VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
  VL53LX_histogram_bin_data_t       *pbins,
  int16_t                           *pambient_dmax_mm)
{











  VL53LX_Error status         = VL53LX_ERROR_NONE;

  SUPPRESS_UNUSED_WARNING(Dev);

  status =
    VL53LX_hist_ambient_dmax(
      target_reflectance,
      pdmax_cal,
      pdmax_cfg,
      pbins,
      pambient_dmax_mm);

  return status;
}


VL53LX_Error VL53LX::VL53LX_ipp_xtalk_calibration_process_data(
  VL53LX_xtalk_range_results_t       *pxtalk_ranges,
  VL53LX_xtalk_histogram_data_t      *pxtalk_shape,
  VL53LX_xtalk_calibration_results_t *pxtalk_cal)
{






  VL53LX_Error status         = VL53LX_ERROR_NONE;

  SUPPRESS_UNUSED_WARNING(Dev);

  status =
    VL53LX_xtalk_calibration_process_data(
      pxtalk_ranges,
      pxtalk_shape,
      pxtalk_cal);

  return status;
}


VL53LX_Error VL53LX::VL53LX_ipp_hist_xtalk_correction(
  VL53LX_customer_nvm_managed_t *pcustomer,
  VL53LX_dynamic_config_t       *pdyn_cfg,
  VL53LX_xtalk_histogram_data_t *pxtalk_shape,
  VL53LX_histogram_bin_data_t   *pip_hist_data,
  VL53LX_histogram_bin_data_t   *pop_hist_data,
  VL53LX_histogram_bin_data_t   *pxtalk_count_data)
{
  VL53LX_Error status         = VL53LX_ERROR_NONE;

  SUPPRESS_UNUSED_WARNING(Dev);

  status =
    VL53LX_f_046(
      pcustomer,
      pdyn_cfg,
      pxtalk_shape,
      pip_hist_data,
      pop_hist_data,
      pxtalk_count_data);

  return status;
}

VL53LX_Error VL53LX::VL53LX_ipp_generate_dual_reflectance_xtalk_samples(
  VL53LX_xtalk_range_results_t  *pxtalk_results,
  uint16_t                 expected_target_distance_mm,
  uint8_t                        higher_reflectance,
  VL53LX_histogram_bin_data_t   *pxtalk_avg_samples)
{
  VL53LX_Error status         = VL53LX_ERROR_NONE;

  SUPPRESS_UNUSED_WARNING(Dev);

  status = VL53LX_generate_dual_reflectance_xtalk_samples(
             pxtalk_results,
             expected_target_distance_mm,
             higher_reflectance,
             pxtalk_avg_samples);

  return status;

}

/* vl53lx_hist_funcs.c */

VL53LX_Error VL53LX::VL53LX_hist_process_data(
  VL53LX_dmax_calibration_data_t     *pdmax_cal,
  VL53LX_hist_gen3_dmax_config_t     *pdmax_cfg,
  VL53LX_hist_post_process_config_t  *ppost_cfg,
  VL53LX_histogram_bin_data_t        *pbins_input,
  VL53LX_xtalk_histogram_data_t      *pxtalk_shape,
  uint8_t                            *pArea1,
  uint8_t                            *pArea2,
  VL53LX_range_results_t             *presults,
  uint8_t                            *HistMergeNumber)
{

  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  VL53LX_hist_gen3_algo_private_data_t  *palgo_gen3 =
    (VL53LX_hist_gen3_algo_private_data_t *) pArea1;
  VL53LX_hist_gen4_algo_filtered_data_t *pfiltered4 =
    (VL53LX_hist_gen4_algo_filtered_data_t *) pArea2;

  VL53LX_hist_gen3_dmax_private_data_t   dmax_algo_gen3;
  VL53LX_hist_gen3_dmax_private_data_t  *pdmax_algo_gen3 =
    &dmax_algo_gen3;

  VL53LX_histogram_bin_data_t             bins_averaged;
  VL53LX_histogram_bin_data_t           *pbins_averaged = &bins_averaged;

  VL53LX_range_data_t                   *pdata;

  uint32_t xtalk_rate_kcps               = 0;
  uint32_t max_xtalk_rate_per_spad_kcps  = 0;
  uint8_t  xtalk_enable                  = 0;
  uint8_t  r                             = 0;
  uint8_t  t                             = 0;
  uint32_t XtalkDetectMaxSigma           = 0;



  int16_t  delta_mm                      = 0;


  VL53LX_f_031(
    pbins_input,
    pbins_averaged);






  VL53LX_init_histogram_bin_data_struct(
    0,
    pxtalk_shape->xtalk_shape.VL53LX_p_021,
    &(pxtalk_shape->xtalk_hist_removed));







  VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
    &(pxtalk_shape->xtalk_shape),
    &(pxtalk_shape->xtalk_hist_removed));







  if ((status == VL53LX_ERROR_NONE) &&
      (ppost_cfg->algo__crosstalk_compensation_enable > 0))

    status =
      VL53LX_f_032(
        ppost_cfg->algo__crosstalk_compensation_plane_offset_kcps,
        ppost_cfg->algo__crosstalk_compensation_x_plane_gradient_kcps,
        ppost_cfg->algo__crosstalk_compensation_y_plane_gradient_kcps,
        0,
        0,
        pbins_input->result__dss_actual_effective_spads,
        pbins_input->roi_config__user_roi_centre_spad,
        pbins_input->roi_config__user_roi_requested_global_xy_size,
        &(xtalk_rate_kcps));








  if ((status == VL53LX_ERROR_NONE) &&
      (ppost_cfg->algo__crosstalk_compensation_enable > 0))
    status =
      VL53LX_f_033(
        pbins_averaged,
        &(pxtalk_shape->xtalk_shape),
        xtalk_rate_kcps,
        &(pxtalk_shape->xtalk_hist_removed));













  presults->xmonitor.total_periods_elapsed =
    pbins_averaged->total_periods_elapsed;
  presults->xmonitor.VL53LX_p_004 =
    pbins_averaged->result__dss_actual_effective_spads;

  presults->xmonitor.peak_signal_count_rate_mcps = 0;
  presults->xmonitor.VL53LX_p_009     = 0;

  presults->xmonitor.range_id     = 0;
  presults->xmonitor.range_status = VL53LX_DEVICEERROR_NOUPDATE;






  xtalk_enable = 0;
  if (ppost_cfg->algo__crosstalk_compensation_enable > 0) {
    xtalk_enable = 1;
  }









  for (r = 0 ; r <= xtalk_enable ; r++) {





    ppost_cfg->algo__crosstalk_compensation_enable = r;






    status =
      VL53LX_f_025(
        pdmax_cal,
        pdmax_cfg,
        ppost_cfg,
        pbins_averaged,
        &(pxtalk_shape->xtalk_hist_removed),
        palgo_gen3,
        pfiltered4,
        pdmax_algo_gen3,
        presults);





    if (!(status == VL53LX_ERROR_NONE && r == 0)) {
      continue;
    }






    if (presults->active_results == 0) {
      pdata = &(presults->VL53LX_p_003[0]);
      pdata->ambient_count_rate_mcps =
        pdmax_algo_gen3->VL53LX_p_034;
      pdata->VL53LX_p_004 =
        pdmax_algo_gen3->VL53LX_p_004;
    }







    max_xtalk_rate_per_spad_kcps = (uint32_t)(
                                     ppost_cfg->algo__crosstalk_detect_max_valid_rate_kcps);
    max_xtalk_rate_per_spad_kcps *= (uint32_t)(*HistMergeNumber);
    max_xtalk_rate_per_spad_kcps <<= 4;

    for (t = 0 ; t < presults->active_results ; t++) {

      pdata = &(presults->VL53LX_p_003[t]);




      if (pdata->max_range_mm > pdata->min_range_mm)
        delta_mm =
          pdata->max_range_mm -
          pdata->min_range_mm;
      else
        delta_mm =
          pdata->min_range_mm -
          pdata->max_range_mm;

      XtalkDetectMaxSigma =
        ppost_cfg->algo__crosstalk_detect_max_sigma_mm;
      XtalkDetectMaxSigma *= (uint32_t)(*HistMergeNumber);
      XtalkDetectMaxSigma <<= 5;
      if (pdata->median_range_mm  >
          ppost_cfg->algo__crosstalk_detect_min_valid_range_mm &&
          pdata->median_range_mm  <
          ppost_cfg->algo__crosstalk_detect_max_valid_range_mm &&
          pdata->VL53LX_p_009 <
          max_xtalk_rate_per_spad_kcps &&
          pdata->VL53LX_p_002 < XtalkDetectMaxSigma &&
          delta_mm <
          ppost_cfg->algo__crosstalk_detect_min_max_tolerance) {




        memcpy(
          &(presults->xmonitor),
          pdata,
          sizeof(VL53LX_range_data_t));

      }
    }

  }




  ppost_cfg->algo__crosstalk_compensation_enable = xtalk_enable;


  return status;
}

VL53LX_Error VL53LX::VL53LX_hist_ambient_dmax(
  uint16_t                            target_reflectance,
  VL53LX_dmax_calibration_data_t     *pdmax_cal,
  VL53LX_hist_gen3_dmax_config_t     *pdmax_cfg,
  VL53LX_histogram_bin_data_t        *pbins,
  int16_t                            *pambient_dmax_mm)
{

  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  VL53LX_hist_gen3_dmax_private_data_t   dmax_algo;
  VL53LX_hist_gen3_dmax_private_data_t  *pdmax_algo = &dmax_algo;

  status =
    VL53LX_f_001(
      target_reflectance,
      pdmax_cal,
      pdmax_cfg,
      pbins,
      pdmax_algo,
      pambient_dmax_mm);

  return status;
}

/* vl53lx_core_support.c */

uint32_t VL53LX::VL53LX_calc_pll_period_us(
  uint16_t  fast_osc_frequency)
{
  uint32_t  pll_period_us        = 0;


  if (fast_osc_frequency > 0) {
    pll_period_us = (0x01 << 30) / fast_osc_frequency;
  }


  return pll_period_us;
}

uint32_t  VL53LX::VL53LX_duration_maths(
  uint32_t  pll_period_us,
  uint32_t  vcsel_parm_pclks,
  uint32_t  window_vclks,
  uint32_t  elapsed_mclks)
{
  uint64_t  tmp_long_int = 0;
  uint32_t  duration_us  = 0;


  duration_us = window_vclks * pll_period_us;


  duration_us = duration_us >> 12;


  tmp_long_int = (uint64_t)duration_us;


  duration_us = elapsed_mclks * vcsel_parm_pclks;


  duration_us = duration_us >> 4;


  tmp_long_int = tmp_long_int * (uint64_t)duration_us;


  tmp_long_int = tmp_long_int >> 12;


  if (tmp_long_int > 0xFFFFFFFF) {
    tmp_long_int = 0xFFFFFFFF;
  }

  duration_us  = (uint32_t)tmp_long_int;

  return duration_us;
}

uint32_t VL53LX::VL53LX_events_per_spad_maths(
  int32_t   VL53LX_p_010,
  uint16_t  num_spads,
  uint32_t  duration)
{
  uint64_t total_hist_counts  = 0;
  uint64_t xtalk_per_spad     = 0;
  uint32_t rate_per_spad_kcps = 0;

  uint64_t dividend = ((uint64_t)VL53LX_p_010
                       * 1000 * 256);

  if (num_spads != 0)
    total_hist_counts = do_division_u(
                          dividend, (uint64_t)num_spads);



  if (duration > 0) {


    uint64_t dividend = (((uint64_t)(total_hist_counts << 11))
                         + ((uint64_t)duration / 2));

    xtalk_per_spad = do_division_u(dividend, (uint64_t)duration);
  } else {
    xtalk_per_spad = (uint64_t)(total_hist_counts << 11);
  }

  rate_per_spad_kcps = (uint32_t)xtalk_per_spad;

  return rate_per_spad_kcps;
}

uint32_t VL53LX::VL53LX_isqrt(uint32_t num)
{



  uint32_t  res = 0;
  uint32_t  bit = 1 << 30;


  while (bit > num) {
    bit >>= 2;
  }

  while (bit != 0) {
    if (num >= res + bit)  {
      num -= res + bit;
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }

  return res;
}

void  VL53LX::VL53LX_hist_calc_zero_distance_phase(
  VL53LX_histogram_bin_data_t   *pdata)
{
  uint32_t  period        = 0;
  uint32_t  VL53LX_p_014         = 0;


  period = 2048 *
           (uint32_t)VL53LX_decode_vcsel_period(pdata->VL53LX_p_005);

  VL53LX_p_014  = period;
  VL53LX_p_014 += (uint32_t)pdata->phasecal_result__reference_phase;
  VL53LX_p_014 += (2048 * (uint32_t)pdata->phasecal_result__vcsel_start);
  VL53LX_p_014 -= (2048 * (uint32_t)pdata->cal_config__vcsel_start);

  VL53LX_p_014  = VL53LX_p_014 % period;

  pdata->zero_distance_phase = (uint16_t)VL53LX_p_014;

}

void  VL53LX::VL53LX_hist_estimate_ambient_from_thresholded_bins(
  int32_t                        ambient_threshold_sigma,
  VL53LX_histogram_bin_data_t   *pdata)
{


  uint8_t  bin                      = 0;
  int32_t  VL53LX_p_031 = 0;

  VL53LX_hist_find_min_max_bin_values(pdata);



  VL53LX_p_031  =
    (int32_t)VL53LX_isqrt((uint32_t)pdata->min_bin_value);
  VL53LX_p_031 *= ambient_threshold_sigma;
  VL53LX_p_031 += 0x07;
  VL53LX_p_031  = VL53LX_p_031 >> 4;
  VL53LX_p_031 += pdata->min_bin_value;



  pdata->number_of_ambient_samples = 0;
  pdata->ambient_events_sum        = 0;

  for (bin = 0; bin < pdata->VL53LX_p_021; bin++)
    if (pdata->bin_data[bin] < VL53LX_p_031) {
      pdata->ambient_events_sum += pdata->bin_data[bin];
      pdata->number_of_ambient_samples++;
    }



  if (pdata->number_of_ambient_samples > 0) {
    pdata->VL53LX_p_028 =
      pdata->ambient_events_sum;
    pdata->VL53LX_p_028 +=
      ((int32_t)pdata->number_of_ambient_samples / 2);
    pdata->VL53LX_p_028 /=
      (int32_t)pdata->number_of_ambient_samples;
  }

}


void  VL53LX::VL53LX_hist_remove_ambient_bins(
  VL53LX_histogram_bin_data_t   *pdata)
{

  uint8_t bin = 0;
  uint8_t lc = 0;
  uint8_t i = 0;



  if ((pdata->bin_seq[0] & 0x07) == 0x07) {

    i = 0;
    for (lc = 0; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH; lc++) {
      if ((pdata->bin_seq[lc] & 0x07) != 0x07) {
        pdata->bin_seq[i] = pdata->bin_seq[lc];
        pdata->bin_rep[i] = pdata->bin_rep[lc];
        i++;
      }
    }



    for (lc = i; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH; lc++) {
      pdata->bin_seq[lc] = VL53LX_MAX_BIN_SEQUENCE_CODE + 1;
      pdata->bin_rep[lc] = 0;
    }
  }

  if (pdata->number_of_ambient_bins > 0) {


    for (bin = pdata->number_of_ambient_bins;
         bin < pdata->VL53LX_p_020; bin++) {
      pdata->bin_data[bin - pdata->number_of_ambient_bins] =
        pdata->bin_data[bin];
    }


    pdata->VL53LX_p_021 =
      pdata->VL53LX_p_021 -
      pdata->number_of_ambient_bins;
    pdata->number_of_ambient_bins = 0;
  }
}


uint32_t VL53LX::VL53LX_calc_pll_period_mm(
  uint16_t fast_osc_frequency)
{


  uint32_t pll_period_us = 0;
  uint32_t pll_period_mm = 0;

  pll_period_us  = VL53LX_calc_pll_period_us(fast_osc_frequency);




  pll_period_mm =
    VL53LX_SPEED_OF_LIGHT_IN_AIR_DIV_8 *
    (pll_period_us >> 2);


  pll_period_mm = (pll_period_mm + (0x01 << 15)) >> 16;

  return pll_period_mm;
}


uint16_t VL53LX::VL53LX_rate_maths(
  int32_t   VL53LX_p_018,
  uint32_t  time_us)
{


  uint32_t  tmp_int   = 0;
  uint32_t  frac_bits = 7;
  uint16_t  rate_mcps = 0;



  if (VL53LX_p_018 > VL53LX_SPAD_TOTAL_COUNT_MAX) {
    tmp_int = VL53LX_SPAD_TOTAL_COUNT_MAX;
  } else if (VL53LX_p_018 > 0) {
    tmp_int = (uint32_t)VL53LX_p_018;
  }




  if (VL53LX_p_018 > VL53LX_SPAD_TOTAL_COUNT_RES_THRES) {
    frac_bits = 3;
  } else {
    frac_bits = 7;
  }


  if (time_us > 0) {
    tmp_int = ((tmp_int << frac_bits) + (time_us / 2)) / time_us;
  }


  if (VL53LX_p_018 > VL53LX_SPAD_TOTAL_COUNT_RES_THRES) {
    tmp_int = tmp_int << 4;
  }



  if (tmp_int > 0xFFFF) {
    tmp_int = 0xFFFF;
  }

  rate_mcps = (uint16_t)tmp_int;

  return rate_mcps;
}

uint16_t VL53LX::VL53LX_rate_per_spad_maths(
  uint32_t  frac_bits,
  uint32_t  peak_count_rate,
  uint16_t  num_spads,
  uint32_t  max_output_value)
{

  uint32_t  tmp_int   = 0;


  uint16_t  rate_per_spad = 0;





  if (num_spads > 0) {
    tmp_int = (peak_count_rate << 8) << frac_bits;
    tmp_int = (tmp_int +
               ((uint32_t)num_spads / 2)) /
              (uint32_t)num_spads;
  } else {
    tmp_int = ((peak_count_rate) << frac_bits);
  }



  if (tmp_int > max_output_value) {
    tmp_int = max_output_value;
  }

  rate_per_spad = (uint16_t)tmp_int;

  return rate_per_spad;
}

int32_t VL53LX::VL53LX_range_maths(
  uint16_t  fast_osc_frequency,
  uint16_t  VL53LX_p_014,
  uint16_t  zero_distance_phase,
  uint8_t   fractional_bits,
  int32_t   gain_factor,
  int32_t   range_offset_mm)
{


  uint32_t    pll_period_us = 0;
  int64_t     tmp_long_int  = 0;
  int32_t     range_mm      = 0;
  int32_t     range_mm_10   = 0;



  pll_period_us  = VL53LX_calc_pll_period_us(fast_osc_frequency);



  tmp_long_int = (int64_t)VL53LX_p_014 - (int64_t)zero_distance_phase;



  tmp_long_int =  tmp_long_int * (int64_t)pll_period_us;



  tmp_long_int =  tmp_long_int / (0x01 << 9);



  tmp_long_int =  tmp_long_int * VL53LX_SPEED_OF_LIGHT_IN_AIR_DIV_8;



  tmp_long_int =  tmp_long_int / (0x01 << 22);


  range_mm  = (int32_t)tmp_long_int + range_offset_mm;


  range_mm *= gain_factor;
  range_mm += 0x0400;
  range_mm /= 0x0800;


  if (fractional_bits == 0) {
    range_mm_10 = range_mm * 10;
    range_mm_10 = range_mm_10 / (0x01 << 2);
    if ((range_mm_10 % 10) < 5) {
      range_mm = (int16_t)(range_mm_10 / 10);
    } else {
      range_mm = (int16_t)(range_mm_10 / 10 + 1);
    }
  } else if (fractional_bits == 1) {
    range_mm = range_mm / (0x01 << 1);
  }

  return range_mm;
}

uint8_t VL53LX::VL53LX_decode_vcsel_period(uint8_t vcsel_period_reg)
{


  uint8_t VL53LX_p_030 = 0;

  VL53LX_p_030 = (vcsel_period_reg + 1) << 1;

  return VL53LX_p_030;
}


void VL53LX::VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
  VL53LX_xtalk_histogram_shape_t *pxtalk,
  VL53LX_histogram_bin_data_t    *phist)
{


  phist->cal_config__vcsel_start =
    pxtalk->cal_config__vcsel_start;
  phist->VL53LX_p_015 =
    pxtalk->VL53LX_p_015;
  phist->VL53LX_p_019 =
    pxtalk->VL53LX_p_019;

  phist->phasecal_result__reference_phase   =
    pxtalk->phasecal_result__reference_phase;
  phist->phasecal_result__vcsel_start       =
    pxtalk->phasecal_result__vcsel_start;

  phist->vcsel_width =
    pxtalk->vcsel_width;
  phist->zero_distance_phase =
    pxtalk->zero_distance_phase;

  phist->zone_id      = pxtalk->zone_id;
  phist->VL53LX_p_020  = pxtalk->VL53LX_p_020;
  phist->time_stamp   = pxtalk->time_stamp;
}

void VL53LX::VL53LX_init_histogram_bin_data_struct(
  int32_t                      bin_value,
  uint16_t                     VL53LX_p_021,
  VL53LX_histogram_bin_data_t *pdata)
{
  uint16_t          i = 0;

  pdata->cfg_device_state          = VL53LX_DEVICESTATE_SW_STANDBY;
  pdata->rd_device_state           = VL53LX_DEVICESTATE_SW_STANDBY;

  pdata->zone_id                   = 0;
  pdata->time_stamp                = 0;

  pdata->VL53LX_p_019                 = 0;
  pdata->VL53LX_p_020               = VL53LX_HISTOGRAM_BUFFER_SIZE;
  pdata->VL53LX_p_021            = (uint8_t)VL53LX_p_021;
  pdata->number_of_ambient_bins    = 0;

  pdata->result__interrupt_status           = 0;
  pdata->result__range_status               = 0;
  pdata->result__report_status              = 0;
  pdata->result__stream_count               = 0;

  pdata->result__dss_actual_effective_spads = 0;
  pdata->phasecal_result__reference_phase   = 0;
  pdata->phasecal_result__vcsel_start       = 0;
  pdata->cal_config__vcsel_start            = 0;

  pdata->vcsel_width                        = 0;
  pdata->VL53LX_p_005                       = 0;
  pdata->VL53LX_p_015                = 0;
  pdata->total_periods_elapsed              = 0;

  pdata->min_bin_value                      = 0;
  pdata->max_bin_value                      = 0;

  pdata->zero_distance_phase                = 0;
  pdata->number_of_ambient_samples          = 0;
  pdata->ambient_events_sum                 = 0;
  pdata->VL53LX_p_028             = 0;

  for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++) {
    pdata->bin_seq[i] = (uint8_t)i;
  }

  for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++) {
    pdata->bin_rep[i] = 1;
  }


  for (i = 0; i < VL53LX_HISTOGRAM_BUFFER_SIZE; i++)
    if (i < VL53LX_p_021) {
      pdata->bin_data[i] = bin_value;
    } else {
      pdata->bin_data[i] = 0;
    }


}

void VL53LX::VL53LX_decode_row_col(
  uint8_t  spad_number,
  uint8_t  *prow,
  uint8_t  *pcol)
{



  if (spad_number > 127) {
    *prow = 8 + ((255 - spad_number) & 0x07);
    *pcol = (spad_number - 128) >> 3;
  } else {
    *prow = spad_number & 0x07;
    *pcol = (127 - spad_number) >> 3;
  }
}

void  VL53LX::VL53LX_hist_find_min_max_bin_values(
  VL53LX_histogram_bin_data_t   *pdata)
{
  uint8_t  bin            = 0;

  for (bin = 0; bin < pdata->VL53LX_p_021; bin++) {

    if (bin == 0 || pdata->min_bin_value >= pdata->bin_data[bin]) {
      pdata->min_bin_value = pdata->bin_data[bin];
    }

    if (bin == 0 || pdata->max_bin_value <= pdata->bin_data[bin]) {
      pdata->max_bin_value = pdata->bin_data[bin];
    }

  }


}

void  VL53LX::VL53LX_hist_estimate_ambient_from_ambient_bins(
  VL53LX_histogram_bin_data_t   *pdata)
{

  uint8_t  bin            = 0;


  if (pdata->number_of_ambient_bins > 0) {

    pdata->number_of_ambient_samples =
      pdata->number_of_ambient_bins;



    pdata->ambient_events_sum = 0;
    for (bin = 0; bin < pdata->number_of_ambient_bins; bin++) {
      pdata->ambient_events_sum += pdata->bin_data[bin];
    }

    pdata->VL53LX_p_028 = pdata->ambient_events_sum;
    pdata->VL53LX_p_028 +=
      ((int32_t)pdata->number_of_ambient_bins / 2);
    pdata->VL53LX_p_028 /=
      (int32_t)pdata->number_of_ambient_bins;

  }

}
/* vl53lx_core.c */
void  VL53LX::VL53LX_init_version()
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->version.ll_major    = VL53LX_LL_API_IMPLEMENTATION_VER_MAJOR;
  pdev->version.ll_minor    = VL53LX_LL_API_IMPLEMENTATION_VER_MINOR;
  pdev->version.ll_build    = VL53LX_LL_API_IMPLEMENTATION_VER_SUB;
  pdev->version.ll_revision = VL53LX_LL_API_IMPLEMENTATION_VER_REVISION;
}

void  VL53LX::VL53LX_init_ll_driver_state(
  VL53LX_DeviceState device_state)
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);

  pstate->cfg_device_state  = device_state;
  pstate->cfg_stream_count  = 0;
  pstate->cfg_gph_id        = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
  pstate->cfg_timing_status = 0;
  pstate->cfg_zone_id       = 0;

  pstate->rd_device_state   = device_state;
  pstate->rd_stream_count   = 0;
  pstate->rd_gph_id         = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
  pstate->rd_timing_status  = 0;
  pstate->rd_zone_id        = 0;

}


VL53LX_Error  VL53LX::VL53LX_update_ll_driver_rd_state()
{


  VL53LX_Error        status  = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);


  if ((pdev->sys_ctrl.system__mode_start &
       VL53LX_DEVICEMEASUREMENTMODE_MODE_MASK) == 0x00) {

    pstate->rd_device_state  = VL53LX_DEVICESTATE_SW_STANDBY;
    pstate->rd_stream_count  = 0;
    pstate->rd_internal_stream_count = 0;
    pstate->rd_internal_stream_count_val = 0;
    pstate->rd_gph_id = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
    pstate->rd_timing_status = 0;
    pstate->rd_zone_id       = 0;

  } else {



    if (pstate->rd_stream_count == 0xFF) {
      pstate->rd_stream_count = 0x80;
    } else {
      pstate->rd_stream_count++;
    }


    status = VL53LX_update_internal_stream_counters(
               pstate->rd_stream_count,
               &(pstate->rd_internal_stream_count),
               &(pstate->rd_internal_stream_count_val));



    pstate->rd_gph_id ^= VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;



    switch (pstate->rd_device_state) {

      case VL53LX_DEVICESTATE_SW_STANDBY:

        if ((pdev->dyn_cfg.system__grouped_parameter_hold &
             VL53LX_GROUPEDPARAMETERHOLD_ID_MASK) > 0) {
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC;
        } else {
          if (pstate->rd_zone_id >=
              pdev->zone_cfg.active_zones)
            pstate->rd_device_state =
              VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
          else
            pstate->rd_device_state =
              VL53LX_DEVICESTATE_RANGING_GATHER_DATA;
        }

        pstate->rd_stream_count  = 0;
        pstate->rd_internal_stream_count = 0;
        pstate->rd_internal_stream_count_val = 0;
        pstate->rd_timing_status = 0;
        pstate->rd_zone_id       = 0;

        break;

      case VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC:
        pstate->rd_stream_count = 0;
        pstate->rd_internal_stream_count = 0;
        pstate->rd_internal_stream_count_val = 0;
        pstate->rd_zone_id      = 0;
        if (pstate->rd_zone_id >=
            pdev->zone_cfg.active_zones)
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
        else
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_GATHER_DATA;

        break;

      case VL53LX_DEVICESTATE_RANGING_GATHER_DATA:
        pstate->rd_zone_id++;
        if (pstate->rd_zone_id >=
            pdev->zone_cfg.active_zones)
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
        else
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_GATHER_DATA;

        break;

      case VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA:
        pstate->rd_zone_id        = 0;
        pstate->rd_timing_status ^= 0x01;

        if (pstate->rd_zone_id >=
            pdev->zone_cfg.active_zones)
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
        else
          pstate->rd_device_state =
            VL53LX_DEVICESTATE_RANGING_GATHER_DATA;
        break;

      default:
        pstate->rd_device_state  =
          VL53LX_DEVICESTATE_SW_STANDBY;
        pstate->rd_stream_count  = 0;
        pstate->rd_internal_stream_count = 0;
        pstate->rd_internal_stream_count_val = 0;
        pstate->rd_gph_id = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
        pstate->rd_timing_status = 0;
        pstate->rd_zone_id       = 0;
        break;
    }
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_check_ll_driver_rd_state()
{


  VL53LX_Error         status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t  *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t  *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_ll_driver_state_t  *pstate       = &(pdev->ll_state);
  VL53LX_system_results_t   *psys_results = &(pdev->sys_results);
  VL53LX_histogram_bin_data_t *phist_data = &(pdev->hist_data);
  VL53LX_zone_private_dyn_cfgs_t *pZ = &(pres->zone_dyn_cfgs);

  uint8_t   device_range_status   = 0;
  uint8_t   device_stream_count   = 0;
  uint8_t   device_gph_id         = 0;
  uint8_t   histogram_mode        = 0;
  uint8_t   expected_stream_count = 0;
  uint8_t   expected_gph_id       = 0;


  device_range_status =
    psys_results->result__range_status &
    VL53LX_RANGE_STATUS__RANGE_STATUS_MASK;

  device_stream_count = psys_results->result__stream_count;



  histogram_mode =
    (pdev->sys_ctrl.system__mode_start &
     VL53LX_DEVICESCHEDULERMODE_HISTOGRAM) ==
    VL53LX_DEVICESCHEDULERMODE_HISTOGRAM;


  device_gph_id = (psys_results->result__interrupt_status &
                   VL53LX_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK) >> 4;

  if (histogram_mode)
    device_gph_id = (phist_data->result__interrupt_status &
                     VL53LX_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK) >> 4;



  if (!((pdev->sys_ctrl.system__mode_start &
         VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK) ==
        VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK)) {
    goto ENDFUNC;
  }



  if (pstate->rd_device_state ==
      VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {

    if (histogram_mode == 0) {
      if (device_range_status !=
          VL53LX_DEVICEERROR_GPHSTREAMCOUNT0READY)
        status =
          VL53LX_ERROR_GPH_SYNC_CHECK_FAIL;

    }
  } else {
    if (pstate->rd_stream_count != device_stream_count) {
      status = VL53LX_ERROR_STREAM_COUNT_CHECK_FAIL;
    }


    if (pstate->rd_gph_id != device_gph_id) {
      status = VL53LX_ERROR_GPH_ID_CHECK_FAIL;
    }




    expected_stream_count =
      pZ->VL53LX_p_003[pstate->rd_zone_id].expected_stream_count;
    expected_gph_id =
      pZ->VL53LX_p_003[pstate->rd_zone_id].expected_gph_id;



    if (expected_stream_count != device_stream_count) {


      if (!((pdev->zone_cfg.active_zones == 0) &&
            (device_stream_count == 255)))
        status =
          VL53LX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL;


    }



    if (expected_gph_id != device_gph_id) {
      status = VL53LX_ERROR_ZONE_GPH_ID_CHECK_FAIL;
    }

  }
ENDFUNC:

  return status;
}


VL53LX_Error  VL53LX::VL53LX_update_ll_driver_cfg_state()
{


  VL53LX_Error         status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t  *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t  *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);
  VL53LX_zone_private_dyn_cfgs_t *pZ = &(pres->zone_dyn_cfgs);

  uint8_t prev_cfg_zone_id;
  uint8_t prev_cfg_gph_id;
  uint8_t prev_cfg_stream_count;

  if ((pdev->sys_ctrl.system__mode_start &
       VL53LX_DEVICEMEASUREMENTMODE_MODE_MASK) == 0x00) {

    pstate->cfg_device_state  = VL53LX_DEVICESTATE_SW_STANDBY;
    pstate->cfg_stream_count  = 0;
    pstate->cfg_internal_stream_count = 0;
    pstate->cfg_internal_stream_count_val = 0;
    pstate->cfg_gph_id = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
    pstate->cfg_timing_status = 0;
    pstate->cfg_zone_id       = 0;
    prev_cfg_zone_id          = 0;
    prev_cfg_gph_id           = 0;
    prev_cfg_stream_count     = 0;

  } else {

    prev_cfg_gph_id           = pstate->cfg_gph_id;
    prev_cfg_zone_id          = pstate->cfg_zone_id;
    prev_cfg_stream_count     = pstate->cfg_stream_count;



    if (pstate->cfg_stream_count == 0xFF) {
      pstate->cfg_stream_count = 0x80;
    } else {
      pstate->cfg_stream_count++;
    }


    status = VL53LX_update_internal_stream_counters(
               pstate->cfg_stream_count,
               &(pstate->cfg_internal_stream_count),
               &(pstate->cfg_internal_stream_count_val));



    pstate->cfg_gph_id ^= VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;



    switch (pstate->cfg_device_state) {

      case VL53LX_DEVICESTATE_SW_STANDBY:
        pstate->cfg_zone_id = 1;
        if (pstate->cfg_zone_id >
            pdev->zone_cfg.active_zones) {
          pstate->cfg_zone_id = 0;
          pstate->cfg_timing_status ^= 0x01;
        }
        pstate->cfg_stream_count = 1;

        if (pdev->gen_cfg.global_config__stream_divider == 0) {
          pstate->cfg_internal_stream_count = 1;
          pstate->cfg_internal_stream_count_val = 0;
        } else {
          pstate->cfg_internal_stream_count = 0;
          pstate->cfg_internal_stream_count_val = 1;
        }
        pstate->cfg_device_state =
          VL53LX_DEVICESTATE_RANGING_DSS_AUTO;
        break;

      case VL53LX_DEVICESTATE_RANGING_DSS_AUTO:
        pstate->cfg_zone_id++;
        if (pstate->cfg_zone_id >
            pdev->zone_cfg.active_zones) {

          pstate->cfg_zone_id = 0;
          pstate->cfg_timing_status ^= 0x01;




          if (pdev->zone_cfg.active_zones > 0) {
            pstate->cfg_device_state =
              VL53LX_DEVICESTATE_RANGING_DSS_MANUAL;
          }
        }
        break;

      case VL53LX_DEVICESTATE_RANGING_DSS_MANUAL:
        pstate->cfg_zone_id++;
        if (pstate->cfg_zone_id >
            pdev->zone_cfg.active_zones) {
          pstate->cfg_zone_id = 0;
          pstate->cfg_timing_status ^= 0x01;
        }
        break;

      default:
        pstate->cfg_device_state =
          VL53LX_DEVICESTATE_SW_STANDBY;
        pstate->cfg_stream_count = 0;
        pstate->cfg_internal_stream_count = 0;
        pstate->cfg_internal_stream_count_val = 0;
        pstate->cfg_gph_id =
          VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
        pstate->cfg_timing_status = 0;
        pstate->cfg_zone_id       = 0;
        break;
    }
  }


  if (pdev->zone_cfg.active_zones == 0) {

    pZ->VL53LX_p_003[prev_cfg_zone_id].expected_stream_count
      = prev_cfg_stream_count - 1;

    pZ->VL53LX_p_003[pstate->rd_zone_id].expected_gph_id =
      prev_cfg_gph_id ^ VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
  } else {
    pZ->VL53LX_p_003[prev_cfg_zone_id].expected_stream_count
      = prev_cfg_stream_count;
    pZ->VL53LX_p_003[prev_cfg_zone_id].expected_gph_id =
      prev_cfg_gph_id;
  }



  return status;
}

void VL53LX::VL53LX_copy_rtn_good_spads_to_buffer(
  VL53LX_nvm_copy_data_t  *pdata,
  uint8_t                 *pbuffer)
{


  *(pbuffer +  0) = pdata->global_config__spad_enables_rtn_0;
  *(pbuffer +  1) = pdata->global_config__spad_enables_rtn_1;
  *(pbuffer +  2) = pdata->global_config__spad_enables_rtn_2;
  *(pbuffer +  3) = pdata->global_config__spad_enables_rtn_3;
  *(pbuffer +  4) = pdata->global_config__spad_enables_rtn_4;
  *(pbuffer +  5) = pdata->global_config__spad_enables_rtn_5;
  *(pbuffer +  6) = pdata->global_config__spad_enables_rtn_6;
  *(pbuffer +  7) = pdata->global_config__spad_enables_rtn_7;
  *(pbuffer +  8) = pdata->global_config__spad_enables_rtn_8;
  *(pbuffer +  9) = pdata->global_config__spad_enables_rtn_9;
  *(pbuffer + 10) = pdata->global_config__spad_enables_rtn_10;
  *(pbuffer + 11) = pdata->global_config__spad_enables_rtn_11;
  *(pbuffer + 12) = pdata->global_config__spad_enables_rtn_12;
  *(pbuffer + 13) = pdata->global_config__spad_enables_rtn_13;
  *(pbuffer + 14) = pdata->global_config__spad_enables_rtn_14;
  *(pbuffer + 15) = pdata->global_config__spad_enables_rtn_15;
  *(pbuffer + 16) = pdata->global_config__spad_enables_rtn_16;
  *(pbuffer + 17) = pdata->global_config__spad_enables_rtn_17;
  *(pbuffer + 18) = pdata->global_config__spad_enables_rtn_18;
  *(pbuffer + 19) = pdata->global_config__spad_enables_rtn_19;
  *(pbuffer + 20) = pdata->global_config__spad_enables_rtn_20;
  *(pbuffer + 21) = pdata->global_config__spad_enables_rtn_21;
  *(pbuffer + 22) = pdata->global_config__spad_enables_rtn_22;
  *(pbuffer + 23) = pdata->global_config__spad_enables_rtn_23;
  *(pbuffer + 24) = pdata->global_config__spad_enables_rtn_24;
  *(pbuffer + 25) = pdata->global_config__spad_enables_rtn_25;
  *(pbuffer + 26) = pdata->global_config__spad_enables_rtn_26;
  *(pbuffer + 27) = pdata->global_config__spad_enables_rtn_27;
  *(pbuffer + 28) = pdata->global_config__spad_enables_rtn_28;
  *(pbuffer + 29) = pdata->global_config__spad_enables_rtn_29;
  *(pbuffer + 30) = pdata->global_config__spad_enables_rtn_30;
  *(pbuffer + 31) = pdata->global_config__spad_enables_rtn_31;
}

void VL53LX::VL53LX_init_system_results(
  VL53LX_system_results_t  *pdata)
{


  pdata->result__interrupt_status                       = 0xFF;
  pdata->result__range_status                           = 0xFF;
  pdata->result__report_status                          = 0xFF;
  pdata->result__stream_count                           = 0xFF;

  pdata->result__dss_actual_effective_spads_sd0         = 0xFFFF;
  pdata->result__peak_signal_count_rate_mcps_sd0        = 0xFFFF;
  pdata->result__ambient_count_rate_mcps_sd0            = 0xFFFF;
  pdata->result__sigma_sd0                              = 0xFFFF;
  pdata->result__phase_sd0                              = 0xFFFF;
  pdata->result__final_crosstalk_corrected_range_mm_sd0 = 0xFFFF;
  pdata->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
    0xFFFF;
  pdata->result__mm_inner_actual_effective_spads_sd0    = 0xFFFF;
  pdata->result__mm_outer_actual_effective_spads_sd0    = 0xFFFF;
  pdata->result__avg_signal_count_rate_mcps_sd0         = 0xFFFF;

  pdata->result__dss_actual_effective_spads_sd1         = 0xFFFF;
  pdata->result__peak_signal_count_rate_mcps_sd1        = 0xFFFF;
  pdata->result__ambient_count_rate_mcps_sd1            = 0xFFFF;
  pdata->result__sigma_sd1                              = 0xFFFF;
  pdata->result__phase_sd1                              = 0xFFFF;
  pdata->result__final_crosstalk_corrected_range_mm_sd1 = 0xFFFF;
  pdata->result__spare_0_sd1                            = 0xFFFF;
  pdata->result__spare_1_sd1                            = 0xFFFF;
  pdata->result__spare_2_sd1                            = 0xFFFF;
  pdata->result__spare_3_sd1                            = 0xFF;

}
void VL53LX::V53L1_init_zone_results_structure(
  uint8_t                 active_zones,
  VL53LX_zone_results_t  *pdata)
{



  uint8_t  z = 0;
  VL53LX_zone_objects_t *pobjects;

  pdata->max_zones    = VL53LX_MAX_USER_ZONES;
  pdata->active_zones = active_zones;

  for (z = 0; z < pdata->max_zones; z++) {
    pobjects = &(pdata->VL53LX_p_003[z]);
    pobjects->cfg_device_state = VL53LX_DEVICESTATE_SW_STANDBY;
    pobjects->rd_device_state  = VL53LX_DEVICESTATE_SW_STANDBY;
    pobjects->max_objects      = VL53LX_MAX_RANGE_RESULTS;
    pobjects->active_objects   = 0;
  }
}

void VL53LX::V53L1_init_zone_dss_configs()
{

  VL53LX_LLDriverResults_t  *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);
  uint8_t  z = 0;
  uint8_t max_zones    = VL53LX_MAX_USER_ZONES;
  VL53LX_zone_private_dyn_cfgs_t *pdata = &(pres->zone_dyn_cfgs);

  for (z = 0; z < max_zones; z++) {
    pdata->VL53LX_p_003[z].dss_mode =
      VL53LX_DSS_CONTROL__MODE_TARGET_RATE;
    pdata->VL53LX_p_003[z].dss_requested_effective_spad_count = 0;
  }
}

void VL53LX::VL53LX_init_histogram_config_structure(
  uint8_t   even_bin0,
  uint8_t   even_bin1,
  uint8_t   even_bin2,
  uint8_t   even_bin3,
  uint8_t   even_bin4,
  uint8_t   even_bin5,
  uint8_t   odd_bin0,
  uint8_t   odd_bin1,
  uint8_t   odd_bin2,
  uint8_t   odd_bin3,
  uint8_t   odd_bin4,
  uint8_t   odd_bin5,
  VL53LX_histogram_config_t  *pdata)
{


  pdata->histogram_config__low_amb_even_bin_0_1  =
    (even_bin1 << 4) + even_bin0;
  pdata->histogram_config__low_amb_even_bin_2_3  =
    (even_bin3 << 4) + even_bin2;
  pdata->histogram_config__low_amb_even_bin_4_5  =
    (even_bin5 << 4) + even_bin4;

  pdata->histogram_config__low_amb_odd_bin_0_1   =
    (odd_bin1 << 4) + odd_bin0;
  pdata->histogram_config__low_amb_odd_bin_2_3   =
    (odd_bin3 << 4) + odd_bin2;
  pdata->histogram_config__low_amb_odd_bin_4_5   =
    (odd_bin5 << 4) + odd_bin4;

  pdata->histogram_config__mid_amb_even_bin_0_1  =
    pdata->histogram_config__low_amb_even_bin_0_1;
  pdata->histogram_config__mid_amb_even_bin_2_3  =
    pdata->histogram_config__low_amb_even_bin_2_3;
  pdata->histogram_config__mid_amb_even_bin_4_5  =
    pdata->histogram_config__low_amb_even_bin_4_5;

  pdata->histogram_config__mid_amb_odd_bin_0_1   =
    pdata->histogram_config__low_amb_odd_bin_0_1;
  pdata->histogram_config__mid_amb_odd_bin_2     = odd_bin2;
  pdata->histogram_config__mid_amb_odd_bin_3_4   =
    (odd_bin4 << 4) + odd_bin3;
  pdata->histogram_config__mid_amb_odd_bin_5     = odd_bin5;

  pdata->histogram_config__user_bin_offset       = 0x00;

  pdata->histogram_config__high_amb_even_bin_0_1 =
    pdata->histogram_config__low_amb_even_bin_0_1;
  pdata->histogram_config__high_amb_even_bin_2_3 =
    pdata->histogram_config__low_amb_even_bin_2_3;
  pdata->histogram_config__high_amb_even_bin_4_5 =
    pdata->histogram_config__low_amb_even_bin_4_5;

  pdata->histogram_config__high_amb_odd_bin_0_1  =
    pdata->histogram_config__low_amb_odd_bin_0_1;
  pdata->histogram_config__high_amb_odd_bin_2_3  =
    pdata->histogram_config__low_amb_odd_bin_2_3;
  pdata->histogram_config__high_amb_odd_bin_4_5  =
    pdata->histogram_config__low_amb_odd_bin_4_5;



  pdata->histogram_config__amb_thresh_low        = 0xFFFF;
  pdata->histogram_config__amb_thresh_high       = 0xFFFF;



  pdata->histogram_config__spad_array_selection  = 0x00;

}

void VL53LX::VL53LX_init_histogram_multizone_config_structure(
  uint8_t   even_bin0,
  uint8_t   even_bin1,
  uint8_t   even_bin2,
  uint8_t   even_bin3,
  uint8_t   even_bin4,
  uint8_t   even_bin5,
  uint8_t   odd_bin0,
  uint8_t   odd_bin1,
  uint8_t   odd_bin2,
  uint8_t   odd_bin3,
  uint8_t   odd_bin4,
  uint8_t   odd_bin5,
  VL53LX_histogram_config_t  *pdata)
{


  pdata->histogram_config__low_amb_even_bin_0_1  =
    (even_bin1 << 4) + even_bin0;
  pdata->histogram_config__low_amb_even_bin_2_3  =
    (even_bin3 << 4) + even_bin2;
  pdata->histogram_config__low_amb_even_bin_4_5  =
    (even_bin5 << 4) + even_bin4;

  pdata->histogram_config__low_amb_odd_bin_0_1   =
    pdata->histogram_config__low_amb_even_bin_0_1;
  pdata->histogram_config__low_amb_odd_bin_2_3
    = pdata->histogram_config__low_amb_even_bin_2_3;
  pdata->histogram_config__low_amb_odd_bin_4_5
    = pdata->histogram_config__low_amb_even_bin_4_5;

  pdata->histogram_config__mid_amb_even_bin_0_1  =
    pdata->histogram_config__low_amb_even_bin_0_1;
  pdata->histogram_config__mid_amb_even_bin_2_3
    = pdata->histogram_config__low_amb_even_bin_2_3;
  pdata->histogram_config__mid_amb_even_bin_4_5
    = pdata->histogram_config__low_amb_even_bin_4_5;

  pdata->histogram_config__mid_amb_odd_bin_0_1
    = pdata->histogram_config__low_amb_odd_bin_0_1;
  pdata->histogram_config__mid_amb_odd_bin_2     = odd_bin2;
  pdata->histogram_config__mid_amb_odd_bin_3_4   =
    (odd_bin4 << 4) + odd_bin3;
  pdata->histogram_config__mid_amb_odd_bin_5     = odd_bin5;

  pdata->histogram_config__user_bin_offset       = 0x00;

  pdata->histogram_config__high_amb_even_bin_0_1 =
    (odd_bin1 << 4) + odd_bin0;
  pdata->histogram_config__high_amb_even_bin_2_3 =
    (odd_bin3 << 4) + odd_bin2;
  pdata->histogram_config__high_amb_even_bin_4_5 =
    (odd_bin5 << 4) + odd_bin4;

  pdata->histogram_config__high_amb_odd_bin_0_1
    = pdata->histogram_config__high_amb_even_bin_0_1;
  pdata->histogram_config__high_amb_odd_bin_2_3
    = pdata->histogram_config__high_amb_even_bin_2_3;
  pdata->histogram_config__high_amb_odd_bin_4_5
    = pdata->histogram_config__high_amb_even_bin_4_5;



  pdata->histogram_config__amb_thresh_low        = 0xFFFF;
  pdata->histogram_config__amb_thresh_high       = 0xFFFF;



  pdata->histogram_config__spad_array_selection  = 0x00;
}

void VL53LX::VL53LX_init_xtalk_bin_data_struct(
  uint32_t                        bin_value,
  uint16_t                        VL53LX_p_021,
  VL53LX_xtalk_histogram_shape_t *pdata)
{



  uint16_t          i = 0;

  pdata->zone_id                   = 0;
  pdata->time_stamp                = 0;

  pdata->VL53LX_p_019                 = 0;
  pdata->VL53LX_p_020               = VL53LX_XTALK_HISTO_BINS;
  pdata->VL53LX_p_021            = (uint8_t)VL53LX_p_021;

  pdata->phasecal_result__reference_phase   = 0;
  pdata->phasecal_result__vcsel_start       = 0;
  pdata->cal_config__vcsel_start            = 0;

  pdata->vcsel_width                        = 0;
  pdata->VL53LX_p_015                = 0;

  pdata->zero_distance_phase                = 0;

  for (i = 0; i < VL53LX_XTALK_HISTO_BINS; i++) {
    if (i < VL53LX_p_021) {
      pdata->bin_data[i] = bin_value;
    } else {
      pdata->bin_data[i] = 0;
    }
  }
}
void VL53LX::VL53LX_i2c_encode_uint16_t(
  uint16_t    ip_value,
  uint16_t    count,
  uint8_t    *pbuffer)
{


  uint16_t   i    = 0;
  uint16_t   VL53LX_p_003 = 0;

  VL53LX_p_003 =  ip_value;

  for (i = 0; i < count; i++) {
    pbuffer[count - i - 1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
    VL53LX_p_003 = VL53LX_p_003 >> 8;
  }
}

uint16_t VL53LX::VL53LX_i2c_decode_uint16_t(
  uint16_t    count,
  uint8_t    *pbuffer)
{


  uint16_t   value = 0x00;

  while (count-- > 0) {
    value = (value << 8) | (uint16_t) * pbuffer++;
  }

  return value;
}

void VL53LX::VL53LX_i2c_encode_int16_t(
  int16_t     ip_value,
  uint16_t    count,
  uint8_t    *pbuffer)
{


  uint16_t   i    = 0;
  int16_t    VL53LX_p_003 = 0;

  VL53LX_p_003 =  ip_value;

  for (i = 0; i < count; i++) {
    pbuffer[count - i - 1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
    VL53LX_p_003 = VL53LX_p_003 >> 8;
  }
}
int16_t VL53LX::VL53LX_i2c_decode_int16_t(
  uint16_t    count,
  uint8_t    *pbuffer)
{


  int16_t    value = 0x00;


  if (*pbuffer >= 0x80) {
    value = 0xFFFF;
  }

  while (count-- > 0) {
    value = (value << 8) | (int16_t) * pbuffer++;
  }

  return value;
}

void VL53LX::VL53LX_i2c_encode_uint32_t(
  uint32_t    ip_value,
  uint16_t    count,
  uint8_t    *pbuffer)
{


  uint16_t   i    = 0;
  uint32_t   VL53LX_p_003 = 0;

  VL53LX_p_003 =  ip_value;

  for (i = 0; i < count; i++) {
    pbuffer[count - i - 1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
    VL53LX_p_003 = VL53LX_p_003 >> 8;
  }
}


uint32_t VL53LX::VL53LX_i2c_decode_uint32_t(
  uint16_t    count,
  uint8_t    *pbuffer)
{


  uint32_t   value = 0x00;

  while (count-- > 0) {
    value = (value << 8) | (uint32_t) * pbuffer++;
  }

  return value;
}
uint32_t VL53LX::VL53LX_i2c_decode_with_mask(
  uint16_t    count,
  uint8_t    *pbuffer,
  uint32_t    bit_mask,
  uint32_t    down_shift,
  uint32_t    offset)
{


  uint32_t   value = 0x00;


  while (count-- > 0) {
    value = (value << 8) | (uint32_t) * pbuffer++;
  }


  value =  value & bit_mask;
  if (down_shift > 0) {
    value = value >> down_shift;
  }


  value = value + offset;

  return value;
}

void VL53LX::VL53LX_i2c_encode_int32_t(
  int32_t     ip_value,
  uint16_t    count,
  uint8_t    *pbuffer)
{


  uint16_t   i    = 0;
  int32_t    VL53LX_p_003 = 0;

  VL53LX_p_003 =  ip_value;

  for (i = 0; i < count; i++) {
    pbuffer[count - i - 1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
    VL53LX_p_003 = VL53LX_p_003 >> 8;
  }
}

int32_t VL53LX::VL53LX_i2c_decode_int32_t(
  uint16_t    count,
  uint8_t    *pbuffer)
{


  int32_t    value = 0x00;


  if (*pbuffer >= 0x80) {
    value = 0xFFFFFFFF;
  }

  while (count-- > 0) {
    value = (value << 8) | (int32_t) * pbuffer++;
  }

  return value;
}
VL53LX_Error VL53LX::VL53LX_start_test(
  uint8_t       test_mode__ctrl)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_WrByte(
               Dev,
               VL53LX_TEST_MODE__CTRL,
               test_mode__ctrl);
  }


  return status;
}
VL53LX_Error VL53LX::VL53LX_set_firmware_enable_register(uint8_t  value)
{


  VL53LX_Error status         = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->sys_ctrl.firmware__enable = value;

  status = VL53LX_WrByte(
             Dev,
             VL53LX_FIRMWARE__ENABLE,
             pdev->sys_ctrl.firmware__enable);

  return status;
}

VL53LX_Error VL53LX::VL53LX_enable_firmware()
{
  VL53LX_Error status       = VL53LX_ERROR_NONE;

  status = VL53LX_set_firmware_enable_register(0x01);


  return status;
}

VL53LX_Error VL53LX::VL53LX_disable_firmware()
{
  VL53LX_Error status       = VL53LX_ERROR_NONE;

  status = VL53LX_set_firmware_enable_register(0x00);

  return status;
}


VL53LX_Error VL53LX::VL53LX_set_powerforce_register(
  uint8_t       value)
{


  VL53LX_Error status       = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->sys_ctrl.power_management__go1_power_force = value;

  status = VL53LX_WrByte(
             Dev,
             VL53LX_POWER_MANAGEMENT__GO1_POWER_FORCE,
             pdev->sys_ctrl.power_management__go1_power_force);

  return status;
}


VL53LX_Error VL53LX::VL53LX_enable_powerforce()
{
  VL53LX_Error status       = VL53LX_ERROR_NONE;

  status = VL53LX_set_powerforce_register(0x01);

  return status;
}
VL53LX_Error VL53LX::VL53LX_disable_powerforce()
{

  VL53LX_Error status       = VL53LX_ERROR_NONE;

  status = VL53LX_set_powerforce_register(0x00);

  return status;
}

VL53LX_Error VL53LX::VL53LX_clear_interrupt()
{


  VL53LX_Error status       = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->sys_ctrl.system__interrupt_clear = VL53LX_CLEAR_RANGE_INT;

  status = VL53LX_WrByte(
             Dev,
             VL53LX_SYSTEM__INTERRUPT_CLEAR,
             pdev->sys_ctrl.system__interrupt_clear);


  return status;
}


VL53LX_Error VL53LX::VL53LX_force_shadow_stream_count_to_zero()
{


  VL53LX_Error status       = VL53LX_ERROR_NONE;

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_WrByte(
               Dev,
               VL53LX_SHADOW_RESULT__STREAM_COUNT,
               0x00);
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}
uint32_t VL53LX::VL53LX_calc_macro_period_us(
  uint16_t  fast_osc_frequency,
  uint8_t   VL53LX_p_005)
{

  uint32_t  pll_period_us        = 0;
  uint8_t   VL53LX_p_030   = 0;
  uint32_t  macro_period_us      = 0;


  pll_period_us = VL53LX_calc_pll_period_us(fast_osc_frequency);



  VL53LX_p_030 = VL53LX_decode_vcsel_period(VL53LX_p_005);



  macro_period_us =
    (uint32_t)VL53LX_MACRO_PERIOD_VCSEL_PERIODS *
    pll_period_us;
  macro_period_us = macro_period_us >> 6;

  macro_period_us = macro_period_us * (uint32_t)VL53LX_p_030;
  macro_period_us = macro_period_us >> 6;

  return macro_period_us;
}

uint16_t VL53LX::VL53LX_calc_range_ignore_threshold(
  uint32_t central_rate,
  int16_t  x_gradient,
  int16_t  y_gradient,
  uint8_t  rate_mult)
{


  int32_t    range_ignore_thresh_int  = 0;
  uint16_t   range_ignore_thresh_kcps = 0;
  int32_t    central_rate_int         = 0;
  int16_t    x_gradient_int           = 0;
  int16_t    y_gradient_int           = 0;

  central_rate_int = ((int32_t)central_rate * (1 << 4)) / (1000);

  if (x_gradient < 0) {
    x_gradient_int = x_gradient * -1;
  }

  if (y_gradient < 0) {
    y_gradient_int = y_gradient * -1;
  }





  range_ignore_thresh_int = (8 * x_gradient_int * 4) +
                            (8 * y_gradient_int * 4);



  range_ignore_thresh_int = range_ignore_thresh_int / 1000;



  range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;



  range_ignore_thresh_int = (int32_t)rate_mult * range_ignore_thresh_int;

  range_ignore_thresh_int = (range_ignore_thresh_int + (1 << 4)) / (1 << 5);



  if (range_ignore_thresh_int > 0xFFFF) {
    range_ignore_thresh_kcps = 0xFFFF;
  } else {
    range_ignore_thresh_kcps = (uint16_t)range_ignore_thresh_int;
  }

  return range_ignore_thresh_kcps;
}

uint32_t VL53LX::VL53LX_calc_timeout_mclks(
  uint32_t timeout_us,
  uint32_t macro_period_us)
{
  uint32_t timeout_mclks   = 0;

  timeout_mclks   =
    ((timeout_us << 12) + (macro_period_us >> 1)) /
    macro_period_us;

  return timeout_mclks;
}
uint16_t VL53LX::VL53LX_calc_encoded_timeout(
  uint32_t timeout_us,
  uint32_t macro_period_us)
{


  uint32_t timeout_mclks   = 0;
  uint16_t timeout_encoded = 0;


  timeout_mclks   =
    VL53LX_calc_timeout_mclks(timeout_us, macro_period_us);

  timeout_encoded =
    VL53LX_encode_timeout(timeout_mclks);

  return timeout_encoded;
}

uint32_t VL53LX::VL53LX_calc_timeout_us(
  uint32_t timeout_mclks,
  uint32_t macro_period_us)
{


  uint32_t timeout_us     = 0;
  uint64_t tmp            = 0;


  tmp  = (uint64_t)timeout_mclks * (uint64_t)macro_period_us;
  tmp += 0x00800;
  tmp  = tmp >> 12;

  timeout_us = (uint32_t)tmp;


  return timeout_us;
}

uint32_t VL53LX::VL53LX_calc_crosstalk_plane_offset_with_margin(
  uint32_t     plane_offset_kcps,
  int16_t      margin_offset_kcps)
{
  uint32_t plane_offset_with_margin = 0;
  int32_t  plane_offset_kcps_temp   = 0;

  plane_offset_kcps_temp =
    (int32_t)plane_offset_kcps +
    (int32_t)margin_offset_kcps;

  if (plane_offset_kcps_temp < 0) {
    plane_offset_kcps_temp = 0;
  } else if (plane_offset_kcps_temp > 0x3FFFF) {
    plane_offset_kcps_temp = 0x3FFFF;
  }

  plane_offset_with_margin = (uint32_t) plane_offset_kcps_temp;

  return plane_offset_with_margin;

}

uint32_t VL53LX::VL53LX_calc_decoded_timeout_us(
  uint16_t timeout_encoded,
  uint32_t macro_period_us)
{


  uint32_t timeout_mclks  = 0;
  uint32_t timeout_us     = 0;

  timeout_mclks =
    VL53LX_decode_timeout(timeout_encoded);

  timeout_us    =
    VL53LX_calc_timeout_us(timeout_mclks, macro_period_us);

  return timeout_us;
}

uint16_t VL53LX::VL53LX_encode_timeout(uint32_t timeout_mclks)
{


  uint16_t encoded_timeout = 0;
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte = ls_byte >> 1;
      ms_byte++;
    }

    encoded_timeout = (ms_byte << 8)
                      + (uint16_t)(ls_byte & 0x000000FF);
  }

  return encoded_timeout;
}

uint32_t VL53LX::VL53LX_decode_timeout(uint16_t encoded_timeout)
{


  uint32_t timeout_macro_clks = 0;

  timeout_macro_clks = ((uint32_t)(encoded_timeout & 0x00FF)
                        << (uint32_t)((encoded_timeout & 0xFF00) >> 8)) + 1;

  return timeout_macro_clks;
}




VL53LX_Error VL53LX::VL53LX_calc_timeout_register_values(
  uint32_t                 phasecal_config_timeout_us,
  uint32_t                 mm_config_timeout_us,
  uint32_t                 range_config_timeout_us,
  uint16_t                 fast_osc_frequency,
  VL53LX_general_config_t *pgeneral,
  VL53LX_timing_config_t  *ptiming)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint32_t macro_period_us    = 0;
  uint32_t timeout_mclks      = 0;
  uint16_t timeout_encoded    = 0;

  if (fast_osc_frequency == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  } else {

    macro_period_us =
      VL53LX_calc_macro_period_us(
        fast_osc_frequency,
        ptiming->range_config__vcsel_period_a);


    timeout_mclks =
      VL53LX_calc_timeout_mclks(
        phasecal_config_timeout_us,
        macro_period_us);


    if (timeout_mclks > 0xFF) {
      timeout_mclks = 0xFF;
    }

    pgeneral->phasecal_config__timeout_macrop =
      (uint8_t)timeout_mclks;


    timeout_encoded =
      VL53LX_calc_encoded_timeout(
        mm_config_timeout_us,
        macro_period_us);

    ptiming->mm_config__timeout_macrop_a_hi =
      (uint8_t)((timeout_encoded & 0xFF00) >> 8);
    ptiming->mm_config__timeout_macrop_a_lo =
      (uint8_t)(timeout_encoded & 0x00FF);


    timeout_encoded =
      VL53LX_calc_encoded_timeout(
        range_config_timeout_us,
        macro_period_us);

    ptiming->range_config__timeout_macrop_a_hi =
      (uint8_t)((timeout_encoded & 0xFF00) >> 8);
    ptiming->range_config__timeout_macrop_a_lo =
      (uint8_t)(timeout_encoded & 0x00FF);


    macro_period_us =
      VL53LX_calc_macro_period_us(
        fast_osc_frequency,
        ptiming->range_config__vcsel_period_b);


    timeout_encoded =
      VL53LX_calc_encoded_timeout(
        mm_config_timeout_us,
        macro_period_us);

    ptiming->mm_config__timeout_macrop_b_hi =
      (uint8_t)((timeout_encoded & 0xFF00) >> 8);
    ptiming->mm_config__timeout_macrop_b_lo =
      (uint8_t)(timeout_encoded & 0x00FF);


    timeout_encoded = VL53LX_calc_encoded_timeout(
                        range_config_timeout_us,
                        macro_period_us);

    ptiming->range_config__timeout_macrop_b_hi =
      (uint8_t)((timeout_encoded & 0xFF00) >> 8);
    ptiming->range_config__timeout_macrop_b_lo =
      (uint8_t)(timeout_encoded & 0x00FF);
  }

  return status;

}

uint8_t VL53LX::VL53LX_encode_vcsel_period(uint8_t VL53LX_p_030)
{


  uint8_t vcsel_period_reg = 0;

  vcsel_period_reg = (VL53LX_p_030 >> 1) - 1;

  return vcsel_period_reg;
}

uint32_t VL53LX::VL53LX_decode_unsigned_integer(
  uint8_t  *pbuffer,
  uint8_t   no_of_bytes)
{


  uint8_t   i = 0;
  uint32_t  decoded_value = 0;

  for (i = 0; i < no_of_bytes; i++) {
    decoded_value = (decoded_value << 8) + (uint32_t)pbuffer[i];
  }

  return decoded_value;
}


void VL53LX::VL53LX_encode_unsigned_integer(
  uint32_t  ip_value,
  uint8_t   no_of_bytes,
  uint8_t  *pbuffer)
{


  uint8_t   i    = 0;
  uint32_t  VL53LX_p_003 = 0;

  VL53LX_p_003 = ip_value;
  for (i = 0; i < no_of_bytes; i++) {
    pbuffer[no_of_bytes - i - 1] = VL53LX_p_003 & 0x00FF;
    VL53LX_p_003 = VL53LX_p_003 >> 8;
  }
}

VL53LX_Error  VL53LX::VL53LX_hist_copy_and_scale_ambient_info(
  VL53LX_zone_hist_info_t       *pidata,
  VL53LX_histogram_bin_data_t   *podata)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  int64_t  evts              = 0;
  int64_t  tmpi              = 0;
  int64_t  tmpo              = 0;


  if (pidata->result__dss_actual_effective_spads == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  } else {
    if (pidata->number_of_ambient_bins >  0 &&
        podata->number_of_ambient_bins == 0) {



      tmpo    = 1 + (int64_t)podata->total_periods_elapsed;
      tmpo   *=
        (int64_t)podata->result__dss_actual_effective_spads;

      tmpi    = 1 + (int64_t)pidata->total_periods_elapsed;
      tmpi   *=
        (int64_t)pidata->result__dss_actual_effective_spads;

      evts  = tmpo *
              (int64_t)pidata->ambient_events_sum;
      evts += (tmpi / 2);


      if (tmpi != 0) {
        evts = do_division_s(evts, tmpi);
      }

      podata->ambient_events_sum = (int32_t)evts;



      podata->VL53LX_p_028 =
        podata->ambient_events_sum;
      podata->VL53LX_p_028 +=
        ((int32_t)pidata->number_of_ambient_bins / 2);
      podata->VL53LX_p_028 /=
        (int32_t)pidata->number_of_ambient_bins;
    }
  }


  return status;
}


void  VL53LX::VL53LX_hist_get_bin_sequence_config(
  VL53LX_histogram_bin_data_t   *pdata)
{


  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  int32_t amb_thresh_low   = 0;
  int32_t amb_thresh_high  = 0;

  uint8_t i = 0;

  amb_thresh_low  = 1024 *
                    (int32_t)pdev->hist_cfg.histogram_config__amb_thresh_low;
  amb_thresh_high = 1024 *
                    (int32_t)pdev->hist_cfg.histogram_config__amb_thresh_high;



  if ((pdev->ll_state.rd_stream_count & 0x01) == 0) {

    pdata->bin_seq[5] =
      pdev->hist_cfg.histogram_config__mid_amb_even_bin_4_5 >> 4;
    pdata->bin_seq[4] =
      pdev->hist_cfg.histogram_config__mid_amb_even_bin_4_5 & 0x0F;
    pdata->bin_seq[3] =
      pdev->hist_cfg.histogram_config__mid_amb_even_bin_2_3 >> 4;
    pdata->bin_seq[2] =
      pdev->hist_cfg.histogram_config__mid_amb_even_bin_2_3 & 0x0F;
    pdata->bin_seq[1] =
      pdev->hist_cfg.histogram_config__mid_amb_even_bin_0_1 >> 4;
    pdata->bin_seq[0] =
      pdev->hist_cfg.histogram_config__mid_amb_even_bin_0_1 & 0x0F;

    if (pdata->ambient_events_sum > amb_thresh_high) {
      pdata->bin_seq[5] =
        pdev->hist_cfg.histogram_config__high_amb_even_bin_4_5
        >> 4;
      pdata->bin_seq[4] =
        pdev->hist_cfg.histogram_config__high_amb_even_bin_4_5
        & 0x0F;
      pdata->bin_seq[3] =
        pdev->hist_cfg.histogram_config__high_amb_even_bin_2_3
        >> 4;
      pdata->bin_seq[2] =
        pdev->hist_cfg.histogram_config__high_amb_even_bin_2_3
        & 0x0F;
      pdata->bin_seq[1] =
        pdev->hist_cfg.histogram_config__high_amb_even_bin_0_1
        >> 4;
      pdata->bin_seq[0] =
        pdev->hist_cfg.histogram_config__high_amb_even_bin_0_1
        & 0x0F;
    }

    if (pdata->ambient_events_sum < amb_thresh_low) {
      pdata->bin_seq[5] =
        pdev->hist_cfg.histogram_config__low_amb_even_bin_4_5
        >> 4;
      pdata->bin_seq[4] =
        pdev->hist_cfg.histogram_config__low_amb_even_bin_4_5
        & 0x0F;
      pdata->bin_seq[3] =
        pdev->hist_cfg.histogram_config__low_amb_even_bin_2_3
        >> 4;
      pdata->bin_seq[2] =
        pdev->hist_cfg.histogram_config__low_amb_even_bin_2_3
        & 0x0F;
      pdata->bin_seq[1] =
        pdev->hist_cfg.histogram_config__low_amb_even_bin_0_1
        >> 4;
      pdata->bin_seq[0] =
        pdev->hist_cfg.histogram_config__low_amb_even_bin_0_1
        & 0x0F;
    }

  } else {
    pdata->bin_seq[5] =
      pdev->hist_cfg.histogram_config__mid_amb_odd_bin_5
      & 0x0F;
    pdata->bin_seq[4] =
      pdev->hist_cfg.histogram_config__mid_amb_odd_bin_3_4
      & 0x0F;
    pdata->bin_seq[3] =
      pdev->hist_cfg.histogram_config__mid_amb_odd_bin_3_4
      >> 4;
    pdata->bin_seq[2] =
      pdev->hist_cfg.histogram_config__mid_amb_odd_bin_2 &
      0x0F;
    pdata->bin_seq[1] =
      pdev->hist_cfg.histogram_config__mid_amb_odd_bin_0_1
      >> 4;
    pdata->bin_seq[0] =
      pdev->hist_cfg.histogram_config__mid_amb_odd_bin_0_1
      & 0x0F;

    if (pdata->ambient_events_sum > amb_thresh_high) {
      pdata->bin_seq[5] =
        pdev->hist_cfg.histogram_config__high_amb_odd_bin_4_5
        >> 4;
      pdata->bin_seq[4] =
        pdev->hist_cfg.histogram_config__high_amb_odd_bin_4_5
        & 0x0F;
      pdata->bin_seq[3] =
        pdev->hist_cfg.histogram_config__high_amb_odd_bin_2_3
        >> 4;
      pdata->bin_seq[2] =
        pdev->hist_cfg.histogram_config__high_amb_odd_bin_2_3
        & 0x0F;
      pdata->bin_seq[1] =
        pdev->hist_cfg.histogram_config__high_amb_odd_bin_0_1
        >> 4;
      pdata->bin_seq[0] =
        pdev->hist_cfg.histogram_config__high_amb_odd_bin_0_1
        & 0x0F;
    }

    if (pdata->ambient_events_sum < amb_thresh_low) {
      pdata->bin_seq[5] =
        pdev->hist_cfg.histogram_config__low_amb_odd_bin_4_5
        >> 4;
      pdata->bin_seq[4] =
        pdev->hist_cfg.histogram_config__low_amb_odd_bin_4_5
        & 0x0F;
      pdata->bin_seq[3] =
        pdev->hist_cfg.histogram_config__low_amb_odd_bin_2_3
        >> 4;
      pdata->bin_seq[2] =
        pdev->hist_cfg.histogram_config__low_amb_odd_bin_2_3
        & 0x0F;
      pdata->bin_seq[1] =
        pdev->hist_cfg.histogram_config__low_amb_odd_bin_0_1
        >> 4;
      pdata->bin_seq[0] =
        pdev->hist_cfg.histogram_config__low_amb_odd_bin_0_1
        & 0x0F;
    }
  }



  for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++) {
    pdata->bin_rep[i] = 1;
  }

}


VL53LX_Error  VL53LX::VL53LX_hist_phase_consistency_check(
  VL53LX_zone_hist_info_t     *phist_prev,
  VL53LX_zone_objects_t       *prange_prev,
  VL53LX_range_results_t      *prange_curr)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t   lc = 0;
  uint8_t   p = 0;

  uint16_t  phase_delta      = 0;
  uint16_t  phase_tolerance  = 0;

  int32_t   events_delta     = 0;
  int32_t   events_tolerance = 0;


  uint8_t event_sigma;
  uint16_t event_min_spad_count;
  uint16_t min_max_tolerance;
  uint8_t pht;

  VL53LX_DeviceError  range_status = 0;


  event_sigma =
    pdev->histpostprocess.algo__consistency_check__event_sigma;
  event_min_spad_count =
    pdev->histpostprocess.algo__consistency_check__event_min_spad_count;
  min_max_tolerance =
    pdev->histpostprocess.algo__consistency_check__min_max_tolerance;


  pht = pdev->histpostprocess.algo__consistency_check__phase_tolerance;
  phase_tolerance = (uint16_t)pht;
  phase_tolerance = phase_tolerance << 8;



  if (prange_prev->rd_device_state !=
      VL53LX_DEVICESTATE_RANGING_GATHER_DATA &&
      prange_prev->rd_device_state !=
      VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA) {
    return status;
  }



  if (phase_tolerance == 0) {
    return status;
  }

  for (lc = 0; lc < prange_curr->active_results; lc++) {

    if (!((prange_curr->VL53LX_p_003[lc].range_status ==
           VL53LX_DEVICEERROR_RANGECOMPLETE) ||
          (prange_curr->VL53LX_p_003[lc].range_status ==
           VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK))) {
      continue;
    }






    if (prange_prev->active_objects == 0)
      prange_curr->VL53LX_p_003[lc].range_status =
        VL53LX_DEVICEERROR_PREV_RANGE_NO_TARGETS;
    else
      prange_curr->VL53LX_p_003[lc].range_status =
        VL53LX_DEVICEERROR_PHASECONSISTENCY;





    for (p = 0; p < prange_prev->active_objects; p++) {

      if (prange_curr->VL53LX_p_003[lc].VL53LX_p_011 >
          prange_prev->VL53LX_p_003[p].VL53LX_p_011) {
        phase_delta =
          prange_curr->VL53LX_p_003[lc].VL53LX_p_011 -
          prange_prev->VL53LX_p_003[p].VL53LX_p_011;
      } else {
        phase_delta =
          prange_prev->VL53LX_p_003[p].VL53LX_p_011 -
          prange_curr->VL53LX_p_003[lc].VL53LX_p_011;
      }

      if (phase_delta < phase_tolerance) {





        if (status == VL53LX_ERROR_NONE)
          status =
            VL53LX_hist_events_consistency_check(
              event_sigma,
              event_min_spad_count,
              phist_prev,
              &(prange_prev->VL53LX_p_003[p]),
              &(prange_curr->VL53LX_p_003[lc]),
              &events_tolerance,
              &events_delta,
              &range_status);




        if (status == VL53LX_ERROR_NONE &&
            range_status ==
            VL53LX_DEVICEERROR_RANGECOMPLETE)
          status =
            VL53LX_hist_merged_pulse_check(
              min_max_tolerance,
              &(prange_curr->VL53LX_p_003[lc]),
              &range_status);

        prange_curr->VL53LX_p_003[lc].range_status =
          range_status;
      }
    }

  }

  return status;
}



VL53LX_Error  VL53LX::VL53LX_hist_events_consistency_check(
  uint8_t                      event_sigma,
  uint16_t                     min_effective_spad_count,
  VL53LX_zone_hist_info_t     *phist_prev,
  VL53LX_object_data_t        *prange_prev,
  VL53LX_range_data_t         *prange_curr,
  int32_t                     *pevents_tolerance,
  int32_t                     *pevents_delta,
  VL53LX_DeviceError          *prange_status)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  int64_t   tmpp                   = 0;
  int64_t   tmpc                   = 0;
  int64_t   events_scaler          = 0;
  int64_t   events_scaler_sq       = 0;
  int64_t   c_signal_events        = 0;
  int64_t   c_sig_noise_sq         = 0;
  int64_t   c_amb_noise_sq         = 0;
  int64_t   p_amb_noise_sq         = 0;

  int32_t   p_signal_events        = 0;
  uint32_t  noise_sq_sum           = 0;



  if (event_sigma == 0) {
    *prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE;
    return status;
  }



  tmpp  = 1 + (int64_t)phist_prev->total_periods_elapsed;
  tmpp *= (int64_t)phist_prev->result__dss_actual_effective_spads;



  tmpc  = 1 + (int64_t)prange_curr->total_periods_elapsed;
  tmpc *= (int64_t)prange_curr->VL53LX_p_004;



  events_scaler  = tmpp * 4096;
  if (tmpc != 0) {
    events_scaler += (tmpc / 2);
    events_scaler  = do_division_s(events_scaler, tmpc);
  }

  events_scaler_sq  = events_scaler * events_scaler;
  events_scaler_sq += 2048;
  events_scaler_sq /= 4096;



  c_signal_events  = (int64_t)prange_curr->VL53LX_p_017;
  c_signal_events -= (int64_t)prange_curr->VL53LX_p_016;
  c_signal_events *= (int64_t)events_scaler;
  c_signal_events += 2048;
  c_signal_events /= 4096;

  c_sig_noise_sq  = (int64_t)events_scaler_sq;
  c_sig_noise_sq *= (int64_t)prange_curr->VL53LX_p_017;
  c_sig_noise_sq += 2048;
  c_sig_noise_sq /= 4096;

  c_amb_noise_sq  = (int64_t)events_scaler_sq;
  c_amb_noise_sq *= (int64_t)prange_curr->VL53LX_p_016;
  c_amb_noise_sq += 2048;
  c_amb_noise_sq /= 4096;


  c_amb_noise_sq += 2;
  c_amb_noise_sq /= 4;



  p_amb_noise_sq  =
    (int64_t)prange_prev->VL53LX_p_016;


  p_amb_noise_sq += 2;
  p_amb_noise_sq /= 4;

  noise_sq_sum =
    (uint32_t)prange_prev->VL53LX_p_017 +
    (uint32_t)c_sig_noise_sq +
    (uint32_t)p_amb_noise_sq +
    (uint32_t)c_amb_noise_sq;

  *pevents_tolerance =
    (int32_t)VL53LX_isqrt(noise_sq_sum * 16);

  *pevents_tolerance *= (int32_t)event_sigma;
  *pevents_tolerance += 32;
  *pevents_tolerance /= 64;

  p_signal_events  = (int32_t)prange_prev->VL53LX_p_017;
  p_signal_events -= (int32_t)prange_prev->VL53LX_p_016;

  if ((int32_t)c_signal_events > p_signal_events)
    *pevents_delta =
      (int32_t)c_signal_events - p_signal_events;
  else
    *pevents_delta =
      p_signal_events - (int32_t)c_signal_events;

  if (*pevents_delta > *pevents_tolerance &&
      prange_curr->VL53LX_p_004 > min_effective_spad_count) {
    *prange_status = VL53LX_DEVICEERROR_EVENTCONSISTENCY;
  } else {
    *prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE;
  }
  return status;
}


VL53LX_Error  VL53LX::VL53LX_hist_merged_pulse_check(
  int16_t                      min_max_tolerance_mm,
  VL53LX_range_data_t         *pdata,
  VL53LX_DeviceError          *prange_status)
{


  VL53LX_Error  status   = VL53LX_ERROR_NONE;
  int16_t       delta_mm = 0;

  if (pdata->max_range_mm > pdata->min_range_mm)
    delta_mm =
      pdata->max_range_mm - pdata->min_range_mm;
  else
    delta_mm =
      pdata->min_range_mm - pdata->max_range_mm;

  if (min_max_tolerance_mm > 0 &&
      delta_mm > min_max_tolerance_mm) {
    *prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE;
  } else {
    *prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE;
  }

  return status;
}


VL53LX_Error  VL53LX::VL53LX_hist_xmonitor_consistency_check(
  VL53LX_zone_hist_info_t     *phist_prev,
  VL53LX_zone_objects_t       *prange_prev,
  VL53LX_range_data_t         *prange_curr)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  int32_t   events_delta     = 0;
  int32_t   events_tolerance = 0;
  uint8_t event_sigma;
  uint16_t min_spad_count;

  event_sigma = pdev->histpostprocess.algo__crosstalk_detect_event_sigma;
  min_spad_count =
    pdev->histpostprocess.algo__consistency_check__event_min_spad_count;

  if (prange_curr->range_status == VL53LX_DEVICEERROR_RANGECOMPLETE ||
      prange_curr->range_status ==
      VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK ||
      prange_curr->range_status ==
      VL53LX_DEVICEERROR_EVENTCONSISTENCY) {

    if (prange_prev->xmonitor.range_status ==
        VL53LX_DEVICEERROR_RANGECOMPLETE ||
        prange_prev->xmonitor.range_status ==
        VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK ||
        prange_prev->xmonitor.range_status ==
        VL53LX_DEVICEERROR_EVENTCONSISTENCY) {

      prange_curr->range_status =
        VL53LX_DEVICEERROR_RANGECOMPLETE;

      status =
        VL53LX_hist_events_consistency_check(
          event_sigma,
          min_spad_count,
          phist_prev,
          &(prange_prev->xmonitor),
          prange_curr,
          &events_tolerance,
          &events_delta,
          &(prange_curr->range_status));

    }
  }

  return status;
}

VL53LX_Error  VL53LX::VL53LX_hist_wrap_dmax(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_histogram_bin_data_t        *pcurrent,
  int16_t                            *pwrap_dmax_mm)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint32_t  pll_period_mm        = 0;
  uint32_t  wrap_dmax_phase      = 0;
  uint32_t  range_mm             = 0;

  *pwrap_dmax_mm = 0;


  if (pcurrent->VL53LX_p_015 != 0) {



    pll_period_mm =
      VL53LX_calc_pll_period_mm(
        pcurrent->VL53LX_p_015);



    wrap_dmax_phase =
      (uint32_t)phistpostprocess->valid_phase_high << 8;



    range_mm = wrap_dmax_phase * pll_period_mm;
    range_mm = (range_mm + (1 << 14)) >> 15;

    *pwrap_dmax_mm = (int16_t)range_mm;
  }

  return status;
}

void VL53LX::VL53LX_hist_combine_mm1_mm2_offsets(
  int16_t                               mm1_offset_mm,
  int16_t                               mm2_offset_mm,
  uint8_t                               encoded_mm_roi_centre,
  uint8_t                               encoded_mm_roi_size,
  uint8_t                               encoded_zone_centre,
  uint8_t                               encoded_zone_size,
  VL53LX_additional_offset_cal_data_t  *pcal_data,
  uint8_t                              *pgood_spads,
  uint16_t                              aperture_attenuation,
  int16_t                               *prange_offset_mm)
{



  uint16_t max_mm_inner_effective_spads = 0;
  uint16_t max_mm_outer_effective_spads = 0;
  uint16_t mm_inner_effective_spads     = 0;
  uint16_t mm_outer_effective_spads     = 0;

  uint32_t scaled_mm1_peak_rate_mcps    = 0;
  uint32_t scaled_mm2_peak_rate_mcps    = 0;

  int32_t tmp0 = 0;
  int32_t tmp1 = 0;



  VL53LX_calc_mm_effective_spads(
    encoded_mm_roi_centre,
    encoded_mm_roi_size,
    0xC7,
    0xFF,
    pgood_spads,
    aperture_attenuation,
    &max_mm_inner_effective_spads,
    &max_mm_outer_effective_spads);

  if ((max_mm_inner_effective_spads == 0) ||
      (max_mm_outer_effective_spads == 0)) {
    goto FAIL;
  }


  VL53LX_calc_mm_effective_spads(
    encoded_mm_roi_centre,
    encoded_mm_roi_size,
    encoded_zone_centre,
    encoded_zone_size,
    pgood_spads,
    aperture_attenuation,
    &mm_inner_effective_spads,
    &mm_outer_effective_spads);



  scaled_mm1_peak_rate_mcps  =
    (uint32_t)pcal_data->result__mm_inner_peak_signal_count_rtn_mcps;
  scaled_mm1_peak_rate_mcps *= (uint32_t)mm_inner_effective_spads;
  scaled_mm1_peak_rate_mcps /= (uint32_t)max_mm_inner_effective_spads;

  scaled_mm2_peak_rate_mcps  =
    (uint32_t)pcal_data->result__mm_outer_peak_signal_count_rtn_mcps;
  scaled_mm2_peak_rate_mcps *= (uint32_t)mm_outer_effective_spads;
  scaled_mm2_peak_rate_mcps /= (uint32_t)max_mm_outer_effective_spads;



  tmp0  = ((int32_t)mm1_offset_mm * (int32_t)scaled_mm1_peak_rate_mcps);
  tmp0 += ((int32_t)mm2_offset_mm * (int32_t)scaled_mm2_peak_rate_mcps);

  tmp1 = (int32_t)scaled_mm1_peak_rate_mcps +
         (int32_t)scaled_mm2_peak_rate_mcps;



  if (tmp1 != 0) {
    tmp0 = (tmp0 * 4) / tmp1;
  }
FAIL:
  *prange_offset_mm = (int16_t)tmp0;

}

VL53LX_Error VL53LX::VL53LX_hist_xtalk_extract_calc_window(
  int16_t                             target_distance_mm,
  uint16_t                            target_width_oversize,
  VL53LX_histogram_bin_data_t        *phist_bins,
  VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pxtalk_data->pll_period_mm =
    VL53LX_calc_pll_period_mm(phist_bins->VL53LX_p_015);
  if (pxtalk_data->pll_period_mm == 0) {
    pxtalk_data->pll_period_mm = 1;
  }


  pxtalk_data->xtalk_width_phase =
    (int32_t)phist_bins->vcsel_width * 128;
  pxtalk_data->target_width_phase =
    pxtalk_data->xtalk_width_phase +
    (int32_t)target_width_oversize * 128;



  pxtalk_data->xtalk_start_phase =
    (int32_t)phist_bins->zero_distance_phase -
    (pxtalk_data->xtalk_width_phase / 2);
  pxtalk_data->xtalk_end_phase  =
    (int32_t)pxtalk_data->xtalk_start_phase +
    pxtalk_data->xtalk_width_phase;

  if (pxtalk_data->xtalk_start_phase < 0) {
    pxtalk_data->xtalk_start_phase = 0;
  }




  pxtalk_data->VL53LX_p_012 =
    (uint8_t)(pxtalk_data->xtalk_start_phase / 2048);


  pxtalk_data->VL53LX_p_013 =
    (uint8_t)((pxtalk_data->xtalk_end_phase + 2047) / 2048);



  pxtalk_data->target_start_phase  =
    (int32_t)target_distance_mm * 2048 * 16;
  pxtalk_data->target_start_phase +=
    ((int32_t)pxtalk_data->pll_period_mm / 2);
  pxtalk_data->target_start_phase /= (int32_t)pxtalk_data->pll_period_mm;
  pxtalk_data->target_start_phase +=
    (int32_t)phist_bins->zero_distance_phase;



  pxtalk_data->target_start_phase -=
    (pxtalk_data->target_width_phase / 2);
  pxtalk_data->target_end_phase  =
    (int32_t)pxtalk_data->target_start_phase +
    pxtalk_data->target_width_phase;

  if (pxtalk_data->target_start_phase < 0) {
    pxtalk_data->target_start_phase = 0;
  }


  pxtalk_data->target_start =
    (uint8_t)(pxtalk_data->target_start_phase / 2048);


  if (pxtalk_data->VL53LX_p_013 > (pxtalk_data->target_start - 1)) {
    pxtalk_data->VL53LX_p_013 = pxtalk_data->target_start - 1;
  }


  pxtalk_data->effective_width =
    (2048 * ((int32_t)pxtalk_data->VL53LX_p_013 + 1));
  pxtalk_data->effective_width -= pxtalk_data->xtalk_start_phase;


  if (pxtalk_data->effective_width > pxtalk_data->xtalk_width_phase) {
    pxtalk_data->effective_width = pxtalk_data->xtalk_width_phase;
  }

  if (pxtalk_data->effective_width < 1) {
    pxtalk_data->effective_width = 1;
  }


  pxtalk_data->event_scaler  =  pxtalk_data->xtalk_width_phase * 1000;
  pxtalk_data->event_scaler += (pxtalk_data->effective_width / 2);
  pxtalk_data->event_scaler /=  pxtalk_data->effective_width;


  if (pxtalk_data->event_scaler < 1000) {
    pxtalk_data->event_scaler = 1000;
  }

  if (pxtalk_data->event_scaler > 4000) {
    pxtalk_data->event_scaler = 4000;
  }


  pxtalk_data->event_scaler_sum += pxtalk_data->event_scaler;


  pxtalk_data->peak_duration_us_sum +=
    (uint32_t)phist_bins->peak_duration_us;


  pxtalk_data->effective_spad_count_sum +=
    (uint32_t)phist_bins->result__dss_actual_effective_spads;


  pxtalk_data->zero_distance_phase_sum +=
    (uint32_t)phist_bins->zero_distance_phase;

  return status;
}


VL53LX_Error VL53LX::VL53LX_hist_xtalk_extract_calc_event_sums(
  VL53LX_histogram_bin_data_t        *phist_bins,
  VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint8_t   lb = 0;
  uint8_t   i = 0;


  for (lb  = pxtalk_data->VL53LX_p_012;
       lb <= pxtalk_data->VL53LX_p_013;
       lb++) {


    i = (lb + phist_bins->number_of_ambient_bins +
         phist_bins->VL53LX_p_021) %
        phist_bins->VL53LX_p_021;


    pxtalk_data->signal_events_sum += phist_bins->bin_data[i];
    pxtalk_data->signal_events_sum -=
      phist_bins->VL53LX_p_028;
  }



  for (lb  = 0; lb < VL53LX_XTALK_HISTO_BINS  &&
       lb < phist_bins->VL53LX_p_021; lb++) {


    i = (lb + phist_bins->number_of_ambient_bins +
         phist_bins->VL53LX_p_021) %
        phist_bins->VL53LX_p_021;


    pxtalk_data->bin_data_sums[lb] += phist_bins->bin_data[i];
    pxtalk_data->bin_data_sums[lb] -=
      phist_bins->VL53LX_p_028;
  }

  pxtalk_data->sample_count += 1;


  return status;
}
VL53LX_Error VL53LX::VL53LX_hist_xtalk_extract_calc_rate_per_spad(
  VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint64_t tmp64_0        = 0;
  uint64_t tmp64_1        = 0;
  uint64_t xtalk_per_spad = 0;


  if (pxtalk_data->signal_events_sum > 0) {
    tmp64_0 =
      ((uint64_t)pxtalk_data->signal_events_sum *
       (uint64_t)pxtalk_data->sample_count *
       (uint64_t)pxtalk_data->event_scaler_avg * 256U) << 9U;
    tmp64_1 =
      (uint64_t)pxtalk_data->effective_spad_count_sum *
      (uint64_t)pxtalk_data->peak_duration_us_sum;



    if (tmp64_1 > 0U) {

      tmp64_0 = tmp64_0 + (tmp64_1 >> 1U);
      xtalk_per_spad = do_division_u(tmp64_0, tmp64_1);
    } else {
      xtalk_per_spad = (uint64_t)tmp64_0;
    }

  } else {
    status = VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL;
  }

  pxtalk_data->xtalk_rate_kcps_per_spad = (uint32_t)xtalk_per_spad;


  return status;
}

VL53LX_Error VL53LX::VL53LX_hist_xtalk_extract_calc_shape(
  VL53LX_hist_xtalk_extract_data_t  *pxtalk_data,
  VL53LX_xtalk_histogram_shape_t    *pxtalk_shape)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  int32_t  lb = 0;
  uint64_t total_events    = 0U;
  uint64_t tmp64_0         = 0U;
  int32_t  remaining_area  = 1024;

  pxtalk_shape->VL53LX_p_019      = 0;
  pxtalk_shape->VL53LX_p_020    = VL53LX_XTALK_HISTO_BINS;
  pxtalk_shape->VL53LX_p_021 = VL53LX_XTALK_HISTO_BINS;

  pxtalk_shape->zero_distance_phase =
    (uint16_t)pxtalk_data->zero_distance_phase_avg;
  pxtalk_shape->phasecal_result__reference_phase =
    (uint16_t)pxtalk_data->zero_distance_phase_avg + (3 * 2048);



  if (pxtalk_data->signal_events_sum > 0)
    total_events =
      (uint64_t)pxtalk_data->signal_events_sum *
      (uint64_t)pxtalk_data->event_scaler_avg;
  else {
    total_events = 1;
  }
  if (total_events == 0) {
    total_events = 1;
  }


  remaining_area  = 1024;
  pxtalk_data->max_shape_value = 0;

  for (lb = 0; lb < VL53LX_XTALK_HISTO_BINS; lb++) {

    if ((lb < (int32_t)pxtalk_data->VL53LX_p_012 ||
         lb > (int32_t)pxtalk_data->VL53LX_p_013)  ||
        pxtalk_data->bin_data_sums[lb] < 0) {


      if (remaining_area > 0 && remaining_area < 1024) {
        if (remaining_area >
            pxtalk_data->max_shape_value) {
          pxtalk_shape->bin_data[lb] =
            (uint32_t)pxtalk_data->max_shape_value;
          remaining_area -=
            pxtalk_data->max_shape_value;
        } else {
          pxtalk_shape->bin_data[lb] =
            (uint32_t)remaining_area;
          remaining_area = 0;
        }
      } else {
        pxtalk_shape->bin_data[lb] = 0;
      }

    } else {

      tmp64_0 =
        (uint64_t)pxtalk_data->bin_data_sums[lb]
        * 1024U * 1000U;
      tmp64_0 += (total_events >> 1);
      tmp64_0 = do_division_u(tmp64_0, total_events);
      if (tmp64_0 > 0xFFFFU) {
        tmp64_0 = 0xFFFFU;
      }

      pxtalk_shape->bin_data[lb] = (uint32_t)tmp64_0;


      if ((int32_t)pxtalk_shape->bin_data[lb] >
          pxtalk_data->max_shape_value)
        pxtalk_data->max_shape_value =
          (int32_t)pxtalk_shape->bin_data[lb];

      remaining_area -= (int32_t)pxtalk_shape->bin_data[lb];
    }
  }


  return status;
}



VL53LX_Error VL53LX::VL53LX_hist_xtalk_shape_model(
  uint16_t                         events_per_bin,
  uint16_t                         pulse_centre,
  uint16_t                         pulse_width,
  VL53LX_xtalk_histogram_shape_t  *pxtalk_shape)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint32_t phase_start  = 0;
  uint32_t phase_stop   = 0;
  uint32_t phase_bin    = 0;

  uint32_t bin_start    = 0;
  uint32_t bin_stop     = 0;

  uint32_t  lb           = 0;
  uint16_t  VL53LX_p_018      = 0;

  pxtalk_shape->VL53LX_p_019      = 0;
  pxtalk_shape->VL53LX_p_020    = VL53LX_XTALK_HISTO_BINS;
  pxtalk_shape->VL53LX_p_021 = VL53LX_XTALK_HISTO_BINS;

  pxtalk_shape->zero_distance_phase              = pulse_centre;
  pxtalk_shape->phasecal_result__reference_phase =
    pulse_centre + (3 * 2048);


  if (pulse_centre > (pulse_width >> 1))
    phase_start = (uint32_t)pulse_centre -
                  ((uint32_t)pulse_width >> 1);
  else {
    phase_start = 0;
  }

  phase_stop = (uint32_t)pulse_centre  +
               ((uint32_t)pulse_width >> 1);


  bin_start = (phase_start / 2048);
  bin_stop  = (phase_stop  / 2048);

  for (lb = 0; lb < VL53LX_XTALK_HISTO_BINS; lb++) {
    VL53LX_p_018 = 0;


    if (lb == bin_start && lb == bin_stop) {
      VL53LX_p_018 =
        VL53LX_hist_xtalk_shape_model_interp(
          events_per_bin,
          phase_stop - phase_start);

    } else if (lb > bin_start && lb < bin_stop) {


      VL53LX_p_018 = events_per_bin;

    } else if (lb == bin_start) {


      phase_bin = (lb + 1) * 2048;
      VL53LX_p_018 =
        VL53LX_hist_xtalk_shape_model_interp(
          events_per_bin,
          (phase_bin - phase_start));

    } else if (lb == bin_stop) {


      phase_bin = lb * 2048;
      VL53LX_p_018 =
        VL53LX_hist_xtalk_shape_model_interp(
          events_per_bin,
          (phase_stop - phase_bin));
    }

    pxtalk_shape->bin_data[lb] = VL53LX_p_018;
  }


  return status;
}

uint16_t VL53LX::VL53LX_hist_xtalk_shape_model_interp(
  uint16_t      events_per_bin,
  uint32_t      phase_delta)
{

  uint32_t  VL53LX_p_018  = 0;

  VL53LX_p_018  = (uint32_t)events_per_bin * phase_delta;
  VL53LX_p_018 +=  1024;
  VL53LX_p_018 /=  2048;


  if (VL53LX_p_018 > 0xFFFFU) {
    VL53LX_p_018 = 0xFFFFU;
  }

  return (uint16_t)VL53LX_p_018;
}

void VL53LX::VL53LX_spad_number_to_byte_bit_index(
  uint8_t  spad_number,
  uint8_t *pbyte_index,
  uint8_t *pbit_index,
  uint8_t *pbit_mask)
{



  *pbyte_index  = spad_number >> 3;
  *pbit_index   = spad_number & 0x07;
  *pbit_mask    = 0x01 << *pbit_index;

}

void VL53LX::VL53LX_encode_row_col(
  uint8_t  row,
  uint8_t  col,
  uint8_t *pspad_number)
{


  if (row > 7) {
    *pspad_number = 128 + (col << 3) + (15 - row);
  } else {
    *pspad_number = ((15 - col) << 3) + row;
  }

}
void VL53LX::VL53LX_decode_zone_size(
  uint8_t  encoded_xy_size,
  uint8_t  *pwidth,
  uint8_t  *pheight)
{



  *pheight = encoded_xy_size >> 4;
  *pwidth  = encoded_xy_size & 0x0F;

}

void VL53LX::VL53LX_encode_zone_size(
  uint8_t  width,
  uint8_t  height,
  uint8_t *pencoded_xy_size)
{


  *pencoded_xy_size = (height << 4) + width;

}

void VL53LX::VL53LX_decode_zone_limits(
  uint8_t   encoded_xy_centre,
  uint8_t   encoded_xy_size,
  int16_t  *px_ll,
  int16_t  *py_ll,
  int16_t  *px_ur,
  int16_t  *py_ur)
{



  uint8_t x_centre = 0;
  uint8_t y_centre = 0;
  uint8_t width    = 0;
  uint8_t height   = 0;



  VL53LX_decode_row_col(
    encoded_xy_centre,
    &y_centre,
    &x_centre);

  VL53LX_decode_zone_size(
    encoded_xy_size,
    &width,
    &height);



  *px_ll = (int16_t)x_centre - ((int16_t)width + 1) / 2;
  if (*px_ll < 0) {
    *px_ll = 0;
  }

  *px_ur = *px_ll + (int16_t)width;
  if (*px_ur > (VL53LX_SPAD_ARRAY_WIDTH - 1)) {
    *px_ur = VL53LX_SPAD_ARRAY_WIDTH - 1;
  }

  *py_ll = (int16_t)y_centre - ((int16_t)height + 1) / 2;
  if (*py_ll < 0) {
    *py_ll = 0;
  }

  *py_ur = *py_ll + (int16_t)height;
  if (*py_ur > (VL53LX_SPAD_ARRAY_HEIGHT - 1)) {
    *py_ur = VL53LX_SPAD_ARRAY_HEIGHT - 1;
  }
}

uint8_t VL53LX::VL53LX_is_aperture_location(
  uint8_t row,
  uint8_t col)
{


  uint8_t is_aperture = 0;
  uint8_t mod_row     = row % 4;
  uint8_t mod_col     = col % 4;

  if (mod_row == 0 && mod_col == 2) {
    is_aperture = 1;
  }

  if (mod_row == 2 && mod_col == 0) {
    is_aperture = 1;
  }

  return is_aperture;
}

void VL53LX::VL53LX_calc_max_effective_spads(
  uint8_t     encoded_zone_centre,
  uint8_t     encoded_zone_size,
  uint8_t    *pgood_spads,
  uint16_t    aperture_attenuation,
  uint16_t   *pmax_effective_spads)
{



  int16_t   x         = 0;
  int16_t   y         = 0;

  int16_t   zone_x_ll = 0;
  int16_t   zone_y_ll = 0;
  int16_t   zone_x_ur = 0;
  int16_t   zone_y_ur = 0;

  uint8_t   spad_number = 0;
  uint8_t   byte_index  = 0;
  uint8_t   bit_index   = 0;
  uint8_t   bit_mask    = 0;

  uint8_t   is_aperture = 0;



  VL53LX_decode_zone_limits(
    encoded_zone_centre,
    encoded_zone_size,
    &zone_x_ll,
    &zone_y_ll,
    &zone_x_ur,
    &zone_y_ur);



  *pmax_effective_spads = 0;

  for (y = zone_y_ll; y <= zone_y_ur; y++) {
    for (x = zone_x_ll; x <= zone_x_ur; x++) {



      VL53LX_encode_row_col(
        (uint8_t)y,
        (uint8_t)x,
        &spad_number);



      VL53LX_spad_number_to_byte_bit_index(
        spad_number,
        &byte_index,
        &bit_index,
        &bit_mask);



      if ((pgood_spads[byte_index] & bit_mask) > 0) {


        is_aperture = VL53LX_is_aperture_location(
                        (uint8_t)y,
                        (uint8_t)x);

        if (is_aperture > 0)
          *pmax_effective_spads +=
            aperture_attenuation;
        else {
          *pmax_effective_spads += 0x0100;
        }

      }
    }
  }
}




void VL53LX::VL53LX_calc_mm_effective_spads(
  uint8_t     encoded_mm_roi_centre,
  uint8_t     encoded_mm_roi_size,
  uint8_t     encoded_zone_centre,
  uint8_t     encoded_zone_size,
  uint8_t    *pgood_spads,
  uint16_t    aperture_attenuation,
  uint16_t   *pmm_inner_effective_spads,
  uint16_t   *pmm_outer_effective_spads)
{

  int16_t   x         = 0;
  int16_t   y         = 0;

  int16_t   mm_x_ll   = 0;
  int16_t   mm_y_ll   = 0;
  int16_t   mm_x_ur   = 0;
  int16_t   mm_y_ur   = 0;

  int16_t   zone_x_ll = 0;
  int16_t   zone_y_ll = 0;
  int16_t   zone_x_ur = 0;
  int16_t   zone_y_ur = 0;

  uint8_t   spad_number = 0;
  uint8_t   byte_index  = 0;
  uint8_t   bit_index   = 0;
  uint8_t   bit_mask    = 0;

  uint8_t   is_aperture = 0;
  uint16_t  spad_attenuation = 0;



  VL53LX_decode_zone_limits(
    encoded_mm_roi_centre,
    encoded_mm_roi_size,
    &mm_x_ll,
    &mm_y_ll,
    &mm_x_ur,
    &mm_y_ur);

  VL53LX_decode_zone_limits(
    encoded_zone_centre,
    encoded_zone_size,
    &zone_x_ll,
    &zone_y_ll,
    &zone_x_ur,
    &zone_y_ur);



  *pmm_inner_effective_spads = 0;
  *pmm_outer_effective_spads = 0;

  for (y = zone_y_ll; y <= zone_y_ur; y++) {
    for (x = zone_x_ll; x <= zone_x_ur; x++) {



      VL53LX_encode_row_col(
        (uint8_t)y,
        (uint8_t)x,
        &spad_number);



      VL53LX_spad_number_to_byte_bit_index(
        spad_number,
        &byte_index,
        &bit_index,
        &bit_mask);



      if ((pgood_spads[byte_index] & bit_mask) > 0) {


        is_aperture = VL53LX_is_aperture_location(
                        (uint8_t)y,
                        (uint8_t)x);

        if (is_aperture > 0) {
          spad_attenuation = aperture_attenuation;
        } else {
          spad_attenuation = 0x0100;
        }



        if (x >= mm_x_ll && x <= mm_x_ur &&
            y >= mm_y_ll && y <= mm_y_ur)
          *pmm_inner_effective_spads +=
            spad_attenuation;
        else
          *pmm_outer_effective_spads +=
            spad_attenuation;
      }
    }
  }
}


void VL53LX::VL53LX_hist_copy_results_to_sys_and_core(
  VL53LX_histogram_bin_data_t      *pbins,
  VL53LX_range_results_t           *phist,
  VL53LX_system_results_t          *psys,
  VL53LX_core_results_t            *pcore)
{


  uint8_t  i = 0;

  VL53LX_range_data_t  *pdata;

  VL53LX_init_system_results(psys);

  psys->result__interrupt_status = pbins->result__interrupt_status;
  psys->result__range_status     = phist->active_results;
  psys->result__report_status    = pbins->result__report_status;
  psys->result__stream_count     = pbins->result__stream_count;

  pdata = &(phist->VL53LX_p_003[0]);

  for (i = 0; i < phist->active_results; i++) {

    switch (i) {
      case 0:
        psys->result__dss_actual_effective_spads_sd0 =
          pdata->VL53LX_p_004;
        psys->result__peak_signal_count_rate_mcps_sd0 =
          pdata->peak_signal_count_rate_mcps;
        psys->result__avg_signal_count_rate_mcps_sd0 =
          pdata->avg_signal_count_rate_mcps;
        psys->result__ambient_count_rate_mcps_sd0 =
          pdata->ambient_count_rate_mcps;

        psys->result__sigma_sd0 = pdata->VL53LX_p_002;
        psys->result__phase_sd0 = pdata->VL53LX_p_011;

        psys->result__final_crosstalk_corrected_range_mm_sd0 =
          (uint16_t)pdata->median_range_mm;

        psys->result__phase_sd1  = pdata->zero_distance_phase;

        pcore->result_core__ranging_total_events_sd0 =
          pdata->VL53LX_p_017;
        pcore->result_core__signal_total_events_sd0 =
          pdata->VL53LX_p_010;
        pcore->result_core__total_periods_elapsed_sd0 =
          pdata->total_periods_elapsed;
        pcore->result_core__ambient_window_events_sd0 =
          pdata->VL53LX_p_016;

        break;
      case 1:
        psys->result__dss_actual_effective_spads_sd1 =
          pdata->VL53LX_p_004;
        psys->result__peak_signal_count_rate_mcps_sd1 =
          pdata->peak_signal_count_rate_mcps;
        psys->result__ambient_count_rate_mcps_sd1 =
          pdata->ambient_count_rate_mcps;

        psys->result__sigma_sd1 = pdata->VL53LX_p_002;
        psys->result__phase_sd1 = pdata->VL53LX_p_011;

        psys->result__final_crosstalk_corrected_range_mm_sd1 =
          (uint16_t)pdata->median_range_mm;

        pcore->result_core__ranging_total_events_sd1 =
          pdata->VL53LX_p_017;
        pcore->result_core__signal_total_events_sd1 =
          pdata->VL53LX_p_010;
        pcore->result_core__total_periods_elapsed_sd1 =
          pdata->total_periods_elapsed;
        pcore->result_core__ambient_window_events_sd1 =
          pdata->VL53LX_p_016;
        break;
    }

    pdata++;
  }

}

VL53LX_Error VL53LX::VL53LX_sum_histogram_data(
  VL53LX_histogram_bin_data_t *phist_input,
  VL53LX_histogram_bin_data_t *phist_output)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t i = 0;
  uint8_t smallest_bin_num = 0;


  if (status == VL53LX_ERROR_NONE) {
    if (phist_output->VL53LX_p_021 >=
        phist_input->VL53LX_p_021) {
      smallest_bin_num = phist_input->VL53LX_p_021;
    } else {
      smallest_bin_num = phist_output->VL53LX_p_021;
    }
  }





  if (status == VL53LX_ERROR_NONE)
    for (i = 0; i < smallest_bin_num; i++)

    {
      phist_output->bin_data[i] += phist_input->bin_data[i];
    }

  if (status == VL53LX_ERROR_NONE)
    phist_output->VL53LX_p_028 +=
      phist_input->VL53LX_p_028;


  return status;
}



VL53LX_Error VL53LX::VL53LX_avg_histogram_data(
  uint8_t no_of_samples,
  VL53LX_histogram_bin_data_t *phist_sum,
  VL53LX_histogram_bin_data_t *phist_avg)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t i = 0;


  if (status == VL53LX_ERROR_NONE) {
    for (i = 0; i < phist_sum->VL53LX_p_021; i++) {



      if (no_of_samples > 0)
        phist_avg->bin_data[i] =
          phist_sum->bin_data[i] /
          (int32_t)no_of_samples;
      else {
        phist_avg->bin_data[i] = phist_sum->bin_data[i];
      }
    }
  }

  if (status == VL53LX_ERROR_NONE) {
    if (no_of_samples > 0)
      phist_avg->VL53LX_p_028 =
        phist_sum->VL53LX_p_028 /
        (int32_t)no_of_samples;
    else
      phist_avg->VL53LX_p_028 =
        phist_sum->VL53LX_p_028;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_save_cfg_data()
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t  *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t  *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_zone_private_dyn_cfg_t *pzone_dyn_cfg;
  VL53LX_dynamic_config_t       *pdynamic = &(pdev->dyn_cfg);


  pzone_dyn_cfg =
    &(pres->zone_dyn_cfgs.VL53LX_p_003[pdev->ll_state.cfg_zone_id]);

  pzone_dyn_cfg->expected_stream_count =
    pdev->ll_state.cfg_stream_count;

  pzone_dyn_cfg->expected_gph_id =
    pdev->ll_state.cfg_gph_id;

  pzone_dyn_cfg->roi_config__user_roi_centre_spad =
    pdynamic->roi_config__user_roi_centre_spad;

  pzone_dyn_cfg->roi_config__user_roi_requested_global_xy_size =
    pdynamic->roi_config__user_roi_requested_global_xy_size;

  return status;
}


VL53LX_Error VL53LX::VL53LX_dynamic_zone_update(
  VL53LX_range_results_t *presults)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t  *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t  *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);
  VL53LX_zone_private_dyn_cfgs_t *pZ = &(pres->zone_dyn_cfgs);

  uint8_t   zone_id = pdev->ll_state.rd_zone_id;
  uint8_t   i;
  uint16_t  max_total_rate_per_spads;
  uint16_t  target_rate =
    pdev->stat_cfg.dss_config__target_total_rate_mcps;
  uint32_t  temp = 0xFFFF;

  pZ->VL53LX_p_003[zone_id].dss_requested_effective_spad_count = 0;


  max_total_rate_per_spads =
    presults->VL53LX_p_003[0].total_rate_per_spad_mcps;


  for (i = 1; i < presults->active_results; i++) {


    if (presults->VL53LX_p_003[i].total_rate_per_spad_mcps >
        max_total_rate_per_spads)
      max_total_rate_per_spads =
        presults->VL53LX_p_003[i].total_rate_per_spad_mcps;

  }

  if (max_total_rate_per_spads == 0) {

    temp = 0xFFFF;
  } else {

    temp = target_rate << 14;

    temp = temp / max_total_rate_per_spads;


    if (temp > 0xFFFF) {
      temp = 0xFFFF;
    }

  }

  pZ->VL53LX_p_003[zone_id].dss_requested_effective_spad_count =
    (uint16_t)temp;


  return status;
}

VL53LX_Error VL53LX::VL53LX_multizone_hist_bins_update()
{


  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);
  VL53LX_zone_config_t *pzone_cfg = &(pdev->zone_cfg);
  VL53LX_histogram_config_t *phist_cfg = &(pdev->hist_cfg);
  VL53LX_histogram_config_t *pmulti_hist =
    &(pzone_cfg->multizone_hist_cfg);

  uint8_t   next_range_is_odd_timing = (pstate->cfg_stream_count) % 2;

  if (pzone_cfg->bin_config[pdev->ll_state.cfg_zone_id] ==
      VL53LX_ZONECONFIG_BINCONFIG__LOWAMB) {
    if (!next_range_is_odd_timing) {
      phist_cfg->histogram_config__low_amb_even_bin_0_1  =
        pmulti_hist->histogram_config__low_amb_even_bin_0_1;
      phist_cfg->histogram_config__low_amb_even_bin_2_3  =
        pmulti_hist->histogram_config__low_amb_even_bin_2_3;
      phist_cfg->histogram_config__low_amb_even_bin_4_5  =
        pmulti_hist->histogram_config__low_amb_even_bin_4_5;
    }

    if (next_range_is_odd_timing) {
      phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
        pmulti_hist->histogram_config__low_amb_even_bin_0_1;
      phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
        pmulti_hist->histogram_config__low_amb_even_bin_2_3;
      phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
        pmulti_hist->histogram_config__low_amb_even_bin_4_5;
    }
  } else if (pzone_cfg->bin_config[pdev->ll_state.cfg_zone_id] ==
             VL53LX_ZONECONFIG_BINCONFIG__MIDAMB) {
    if (!next_range_is_odd_timing) {
      phist_cfg->histogram_config__low_amb_even_bin_0_1  =
        pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
      phist_cfg->histogram_config__low_amb_even_bin_2_3  =
        pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
      phist_cfg->histogram_config__low_amb_even_bin_4_5  =
        pmulti_hist->histogram_config__mid_amb_even_bin_4_5;
    }

    if (next_range_is_odd_timing) {
      phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
        pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
      phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
        pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
      phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
        pmulti_hist->histogram_config__mid_amb_even_bin_4_5;
    }
  } else if (pzone_cfg->bin_config[pdev->ll_state.cfg_zone_id] ==
             VL53LX_ZONECONFIG_BINCONFIG__HIGHAMB) {
    if (!next_range_is_odd_timing) {
      phist_cfg->histogram_config__low_amb_even_bin_0_1  =
        pmulti_hist->histogram_config__high_amb_even_bin_0_1;
      phist_cfg->histogram_config__low_amb_even_bin_2_3  =
        pmulti_hist->histogram_config__high_amb_even_bin_2_3;
      phist_cfg->histogram_config__low_amb_even_bin_4_5  =
        pmulti_hist->histogram_config__high_amb_even_bin_4_5;
    }

    if (next_range_is_odd_timing) {
      phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
        pmulti_hist->histogram_config__high_amb_even_bin_0_1;
      phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
        pmulti_hist->histogram_config__high_amb_even_bin_2_3;
      phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
        pmulti_hist->histogram_config__high_amb_even_bin_4_5;
    }
  }



  if (status == VL53LX_ERROR_NONE) {
    VL53LX_copy_hist_bins_to_static_cfg(
      phist_cfg,
      &(pdev->stat_cfg),
      &(pdev->tim_cfg));
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_update_internal_stream_counters(
  uint8_t     external_stream_count,
  uint8_t    *pinternal_stream_count,
  uint8_t    *pinternal_stream_count_val)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t stream_divider;

  VL53LX_LLDriverData_t  *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  stream_divider = pdev->gen_cfg.global_config__stream_divider;

  if (stream_divider == 0) {


    *pinternal_stream_count = external_stream_count;

  } else if (*pinternal_stream_count_val == (stream_divider - 1)) {


    if (*pinternal_stream_count == 0xFF) {
      *pinternal_stream_count = 0x80;
    } else {
      *pinternal_stream_count = *pinternal_stream_count + 1;
    }


    *pinternal_stream_count_val = 0;

  } else {


    *pinternal_stream_count_val = *pinternal_stream_count_val + 1;
  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_set_histogram_multizone_initial_bin_config(
  VL53LX_zone_config_t    *pzone_cfg,
  VL53LX_histogram_config_t *phist_cfg,
  VL53LX_histogram_config_t *pmulti_hist)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  if (pzone_cfg->bin_config[0] ==
      VL53LX_ZONECONFIG_BINCONFIG__LOWAMB) {
    phist_cfg->histogram_config__low_amb_even_bin_0_1  =
      pmulti_hist->histogram_config__low_amb_even_bin_0_1;
    phist_cfg->histogram_config__low_amb_even_bin_2_3  =
      pmulti_hist->histogram_config__low_amb_even_bin_2_3;
    phist_cfg->histogram_config__low_amb_even_bin_4_5  =
      pmulti_hist->histogram_config__low_amb_even_bin_4_5;

    phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
      pmulti_hist->histogram_config__low_amb_even_bin_0_1;
    phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
      pmulti_hist->histogram_config__low_amb_even_bin_2_3;
    phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
      pmulti_hist->histogram_config__low_amb_even_bin_4_5;
  } else if (pzone_cfg->bin_config[0] ==
             VL53LX_ZONECONFIG_BINCONFIG__MIDAMB) {
    phist_cfg->histogram_config__low_amb_even_bin_0_1  =
      pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
    phist_cfg->histogram_config__low_amb_even_bin_2_3  =
      pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
    phist_cfg->histogram_config__low_amb_even_bin_4_5  =
      pmulti_hist->histogram_config__mid_amb_even_bin_4_5;

    phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
      pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
    phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
      pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
    phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
      pmulti_hist->histogram_config__mid_amb_even_bin_4_5;
  } else if (pzone_cfg->bin_config[0] ==
             VL53LX_ZONECONFIG_BINCONFIG__HIGHAMB) {
    phist_cfg->histogram_config__low_amb_even_bin_0_1  =
      pmulti_hist->histogram_config__high_amb_even_bin_0_1;
    phist_cfg->histogram_config__low_amb_even_bin_2_3  =
      pmulti_hist->histogram_config__high_amb_even_bin_2_3;
    phist_cfg->histogram_config__low_amb_even_bin_4_5  =
      pmulti_hist->histogram_config__high_amb_even_bin_4_5;
    phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
      pmulti_hist->histogram_config__high_amb_even_bin_0_1;
    phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
      pmulti_hist->histogram_config__high_amb_even_bin_2_3;
    phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
      pmulti_hist->histogram_config__high_amb_even_bin_4_5;
  }

  return status;
}

uint8_t VL53LX::VL53LX_encode_GPIO_interrupt_config(
  VL53LX_GPIO_interrupt_config_t  *pintconf)
{
  uint8_t system__interrupt_config;

  system__interrupt_config = pintconf->intr_mode_distance;
  system__interrupt_config |= ((pintconf->intr_mode_rate) << 2);
  system__interrupt_config |= ((pintconf->intr_new_measure_ready) << 5);
  system__interrupt_config |= ((pintconf->intr_no_target) << 6);
  system__interrupt_config |= ((pintconf->intr_combined_mode) << 7);

  return system__interrupt_config;
}

VL53LX_GPIO_interrupt_config_t VL53LX::VL53LX_decode_GPIO_interrupt_config(
  uint8_t   system__interrupt_config)
{
  VL53LX_GPIO_interrupt_config_t  intconf;

  intconf.intr_mode_distance = system__interrupt_config & 0x03;
  intconf.intr_mode_rate = (system__interrupt_config >> 2) & 0x03;
  intconf.intr_new_measure_ready = (system__interrupt_config >> 5) & 0x01;
  intconf.intr_no_target = (system__interrupt_config >> 6) & 0x01;
  intconf.intr_combined_mode = (system__interrupt_config >> 7) & 0x01;


  intconf.threshold_rate_low = 0;
  intconf.threshold_rate_high = 0;
  intconf.threshold_distance_low = 0;
  intconf.threshold_distance_high = 0;

  return intconf;
}

VL53LX_Error VL53LX::VL53LX_set_GPIO_distance_threshold(
  uint16_t      threshold_high,
  uint16_t      threshold_low)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  pdev->dyn_cfg.system__thresh_high = threshold_high;
  pdev->dyn_cfg.system__thresh_low = threshold_low;

  return status;
}


VL53LX_Error VL53LX::VL53LX_set_GPIO_rate_threshold(
  uint16_t      threshold_high,
  uint16_t      threshold_low)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


  pdev->gen_cfg.system__thresh_rate_high = threshold_high;
  pdev->gen_cfg.system__thresh_rate_low = threshold_low;

  return status;
}
VL53LX_Error VL53LX::VL53LX_set_GPIO_thresholds_from_struct(
  VL53LX_GPIO_interrupt_config_t *pintconf)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_set_GPIO_distance_threshold(
             pintconf->threshold_distance_high,
             pintconf->threshold_distance_low);

  if (status == VL53LX_ERROR_NONE) {
    status =
      VL53LX_set_GPIO_rate_threshold(
        pintconf->threshold_rate_high,
        pintconf->threshold_rate_low);
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_set_ref_spad_char_config(
  uint8_t       vcsel_period_a,
  uint32_t      phasecal_timeout_us,
  uint16_t      total_rate_target_mcps,
  uint16_t      max_count_rate_rtn_limit_mcps,
  uint16_t      min_count_rate_rtn_limit_mcps,
  uint16_t      fast_osc_frequency)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t buffer[2];

  uint32_t macro_period_us = 0;
  uint32_t timeout_mclks   = 0;

  macro_period_us =
    VL53LX_calc_macro_period_us(
      fast_osc_frequency,
      vcsel_period_a);
  if (macro_period_us == 0) {
    macro_period_us = 1;
  }


  timeout_mclks = phasecal_timeout_us << 12;
  timeout_mclks = timeout_mclks + (macro_period_us >> 1);
  timeout_mclks = timeout_mclks / macro_period_us;

  if (timeout_mclks > 0xFF) {
    pdev->gen_cfg.phasecal_config__timeout_macrop = 0xFF;
  } else
    pdev->gen_cfg.phasecal_config__timeout_macrop =
      (uint8_t)timeout_mclks;

  pdev->tim_cfg.range_config__vcsel_period_a = vcsel_period_a;



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrByte(
        Dev,
        VL53LX_PHASECAL_CONFIG__TIMEOUT_MACROP,
        pdev->gen_cfg.phasecal_config__timeout_macrop);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrByte(
        Dev,
        VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A,
        pdev->tim_cfg.range_config__vcsel_period_a);



  buffer[0] = pdev->tim_cfg.range_config__vcsel_period_a;
  buffer[1] = pdev->tim_cfg.range_config__vcsel_period_a;

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WriteMulti(
        Dev,
        VL53LX_SD_CONFIG__WOI_SD0,
        buffer,
        2);



  pdev->customer.ref_spad_char__total_rate_target_mcps =
    total_rate_target_mcps;

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrWord(
        Dev,
        VL53LX_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS,
        total_rate_target_mcps);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrWord(
        Dev,
        VL53LX_RANGE_CONFIG__SIGMA_THRESH,
        max_count_rate_rtn_limit_mcps);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrWord(
        Dev,
        VL53LX_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
        min_count_rate_rtn_limit_mcps);

  return status;
}

VL53LX_Error VL53LX::VL53LX_set_ssc_config(
  VL53LX_ssc_config_t  *pssc_cfg,
  uint16_t              fast_osc_frequency)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t buffer[5];

  uint32_t macro_period_us = 0;
  uint16_t timeout_encoded = 0;

  macro_period_us =
    VL53LX_calc_macro_period_us(
      fast_osc_frequency,
      pssc_cfg->VL53LX_p_005);


  timeout_encoded =
    VL53LX_calc_encoded_timeout(
      pssc_cfg->timeout_us,
      macro_period_us);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrByte(
        Dev,
        VL53LX_CAL_CONFIG__VCSEL_START,
        pssc_cfg->vcsel_start);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrByte(
        Dev,
        VL53LX_GLOBAL_CONFIG__VCSEL_WIDTH,
        pssc_cfg->vcsel_width);



  buffer[0] = (uint8_t)((timeout_encoded &  0x0000FF00) >> 8);
  buffer[1] = (uint8_t)(timeout_encoded &  0x000000FF);
  buffer[2] = pssc_cfg->VL53LX_p_005;
  buffer[3] = (uint8_t)((pssc_cfg->rate_limit_mcps &  0x0000FF00) >> 8);
  buffer[4] = (uint8_t)(pssc_cfg->rate_limit_mcps &  0x000000FF);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WriteMulti(
        Dev,
        VL53LX_RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
        buffer,
        5);



  buffer[0] = pssc_cfg->VL53LX_p_005;
  buffer[1] = pssc_cfg->VL53LX_p_005;

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WriteMulti(
        Dev,
        VL53LX_SD_CONFIG__WOI_SD0,
        buffer,
        2);


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WrByte(
        Dev,
        VL53LX_NVM_BIST__CTRL,
        pssc_cfg->array_select);

  return status;
}


VL53LX_Error VL53LX::VL53LX_get_spad_rate_data(
  VL53LX_spad_rate_data_t  *pspad_rates)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  int               i = 0;

  uint8_t  VL53LX_p_003[512]{};
  uint8_t *pdata = &VL53LX_p_003[0];

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_firmware();
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_ReadMulti(
        Dev,
        VL53LX_PRIVATE__PATCH_BASE_ADDR_RSLV,
        pdata,
        512);


  pdata = &VL53LX_p_003[0];
  for (i = 0; i < VL53LX_NO_OF_SPAD_ENABLES; i++) {
    pspad_rates->rate_data[i] =
      (uint16_t)VL53LX_decode_unsigned_integer(pdata, 2);
    pdata += 2;
  }



  pspad_rates->VL53LX_p_020     = VL53LX_NO_OF_SPAD_ENABLES;
  pspad_rates->no_of_values    = VL53LX_NO_OF_SPAD_ENABLES;
  pspad_rates->fractional_bits = 15;



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_firmware();
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_calc_required_samples()
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);
  VL53LX_smudge_corrector_config_t *pconfig =
    &(pdev->smudge_correct_config);
  VL53LX_smudge_corrector_internals_t *pint =
    &(pdev->smudge_corrector_internals);

  VL53LX_range_results_t *presults = &(pres->range_results);
  VL53LX_range_data_t *pxmonitor = &(presults->xmonitor);

  uint32_t peak_duration_us = pxmonitor->peak_duration_us;

  uint64_t temp64a;
  uint64_t temp64z;

  temp64a = pxmonitor->VL53LX_p_017 +
            pxmonitor->VL53LX_p_016;
  if (peak_duration_us == 0) {
    peak_duration_us = 1000;
  }
  temp64a = do_division_u((temp64a * 1000), peak_duration_us);
  temp64a = do_division_u((temp64a * 1000), peak_duration_us);

  temp64z = pconfig->noise_margin * pxmonitor->VL53LX_p_004;
  if (temp64z == 0) {
    temp64z = 1;
  }
  temp64a = temp64a * 1000 * 256;
  temp64a = do_division_u(temp64a, temp64z);
  temp64a = temp64a * 1000 * 256;
  temp64a = do_division_u(temp64a, temp64z);

  pint->required_samples = (uint32_t)temp64a;


  if (pint->required_samples < 2) {
    pint->required_samples = 2;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
  uint32_t        xtalk_offset_out,
  VL53LX_smudge_corrector_config_t  *pconfig,
  VL53LX_smudge_corrector_data_t    *pout,
  uint8_t         add_smudge,
  uint8_t         soft_update
)
{



  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  int16_t  x_gradient_scaler;
  int16_t  y_gradient_scaler;
  uint32_t orig_xtalk_offset;
  int16_t  orig_x_gradient;
  int16_t  orig_y_gradient;
  uint8_t  histo_merge_nb;
  uint8_t  i;
  int32_t  itemp32;
  uint32_t SmudgeFactor;
  VL53LX_xtalk_config_t  *pX = &(pdev->xtalk_cfg);
  VL53LX_xtalk_calibration_results_t  *pC = &(pdev->xtalk_cal);
  uint32_t *pcpo;
  uint32_t max, nXtalk, cXtalk;

  if (add_smudge == 1) {
    pout->algo__crosstalk_compensation_plane_offset_kcps =
      (uint32_t)xtalk_offset_out +
      (uint32_t)pconfig->smudge_margin;
  } else {
    pout->algo__crosstalk_compensation_plane_offset_kcps =
      (uint32_t)xtalk_offset_out;
  }


  orig_xtalk_offset =
    pX->nvm_default__crosstalk_compensation_plane_offset_kcps;

  orig_x_gradient =
    pX->nvm_default__crosstalk_compensation_x_plane_gradient_kcps;

  orig_y_gradient =
    pX->nvm_default__crosstalk_compensation_y_plane_gradient_kcps;

  if (((pconfig->user_scaler_set == 0) ||
       (pconfig->scaler_calc_method == 1)) &&
      (pC->algo__crosstalk_compensation_plane_offset_kcps != 0)) {

    VL53LX_compute_histo_merge_nb(&histo_merge_nb);

    if (histo_merge_nb == 0) {
      histo_merge_nb = 1;
    }
    if (pdev->tuning_parms.tp_hist_merge != 1)
      orig_xtalk_offset =
        pC->algo__crosstalk_compensation_plane_offset_kcps;
    else
      orig_xtalk_offset =
        pC->algo__xtalk_cpo_HistoMerge_kcps[histo_merge_nb - 1];

    orig_x_gradient =
      pC->algo__crosstalk_compensation_x_plane_gradient_kcps;

    orig_y_gradient =
      pC->algo__crosstalk_compensation_y_plane_gradient_kcps;
  }


  if ((pconfig->user_scaler_set == 0) && (orig_x_gradient == 0)) {
    pout->gradient_zero_flag |= 0x01;
  }

  if ((pconfig->user_scaler_set == 0) && (orig_y_gradient == 0)) {
    pout->gradient_zero_flag |= 0x02;
  }



  if (orig_xtalk_offset == 0) {
    orig_xtalk_offset = 1;
  }



  if (pconfig->user_scaler_set == 1) {
    x_gradient_scaler = pconfig->x_gradient_scaler;
    y_gradient_scaler = pconfig->y_gradient_scaler;
  } else {

    x_gradient_scaler = (int16_t)do_division_s(
                          (((int32_t)orig_x_gradient) << 6),
                          orig_xtalk_offset);
    pconfig->x_gradient_scaler = x_gradient_scaler;
    y_gradient_scaler = (int16_t)do_division_s(
                          (((int32_t)orig_y_gradient) << 6),
                          orig_xtalk_offset);
    pconfig->y_gradient_scaler = y_gradient_scaler;
  }



  if (pconfig->scaler_calc_method == 0) {


    itemp32 = (int32_t)(
                pout->algo__crosstalk_compensation_plane_offset_kcps *
                x_gradient_scaler);
    itemp32 = itemp32 >> 6;
    if (itemp32 > 0xFFFF) {
      itemp32 = 0xFFFF;
    }

    pout->algo__crosstalk_compensation_x_plane_gradient_kcps =
      (int16_t)itemp32;

    itemp32 = (int32_t)(
                pout->algo__crosstalk_compensation_plane_offset_kcps *
                y_gradient_scaler);
    itemp32 = itemp32 >> 6;
    if (itemp32 > 0xFFFF) {
      itemp32 = 0xFFFF;
    }

    pout->algo__crosstalk_compensation_y_plane_gradient_kcps =
      (int16_t)itemp32;
  } else if (pconfig->scaler_calc_method == 1) {


    itemp32 = (int32_t)(orig_xtalk_offset -
                        pout->algo__crosstalk_compensation_plane_offset_kcps);
    itemp32 = (int32_t)(do_division_s(itemp32, 16));
    itemp32 = itemp32 << 2;
    itemp32 = itemp32 + (int32_t)(orig_x_gradient);
    if (itemp32 > 0xFFFF) {
      itemp32 = 0xFFFF;
    }

    pout->algo__crosstalk_compensation_x_plane_gradient_kcps =
      (int16_t)itemp32;

    itemp32 = (int32_t)(orig_xtalk_offset -
                        pout->algo__crosstalk_compensation_plane_offset_kcps);
    itemp32 = (int32_t)(do_division_s(itemp32, 80));
    itemp32 = itemp32 << 2;
    itemp32 = itemp32 + (int32_t)(orig_y_gradient);
    if (itemp32 > 0xFFFF) {
      itemp32 = 0xFFFF;
    }

    pout->algo__crosstalk_compensation_y_plane_gradient_kcps =
      (int16_t)itemp32;
  }


  if ((pconfig->smudge_corr_apply_enabled == 1) &&
      (soft_update != 1)) {

    pout->new_xtalk_applied_flag = 1;
    nXtalk = pout->algo__crosstalk_compensation_plane_offset_kcps;

    VL53LX_compute_histo_merge_nb(&histo_merge_nb);
    max = pdev->tuning_parms.tp_hist_merge_max_size;
    pcpo = &(pC->algo__xtalk_cpo_HistoMerge_kcps[0]);
    if ((histo_merge_nb > 0) &&
        (pdev->tuning_parms.tp_hist_merge == 1) &&
        (nXtalk != 0)) {
      cXtalk =
        pC->algo__xtalk_cpo_HistoMerge_kcps[histo_merge_nb - 1];
      SmudgeFactor = cXtalk * 1000 / nXtalk;
      if (SmudgeFactor >= pconfig->max_smudge_factor) {
        pout->new_xtalk_applied_flag = 0;
      } else if (SmudgeFactor > 0)
        for (i = 0; i < max; i++) {
          *pcpo *= 1000;
          *pcpo /= SmudgeFactor;
          pcpo++;
        }
    }
    if (pout->new_xtalk_applied_flag) {

      pX->algo__crosstalk_compensation_plane_offset_kcps =
        pout->algo__crosstalk_compensation_plane_offset_kcps;
      pX->algo__crosstalk_compensation_x_plane_gradient_kcps =
        pout->algo__crosstalk_compensation_x_plane_gradient_kcps;
      pX->algo__crosstalk_compensation_y_plane_gradient_kcps =
        pout->algo__crosstalk_compensation_y_plane_gradient_kcps;

      if (pconfig->smudge_corr_single_apply == 1) {

        pconfig->smudge_corr_apply_enabled = 0;
        pconfig->smudge_corr_single_apply = 0;
      }
    }
  }


  if (soft_update != 1) {
    pout->smudge_corr_valid = 1;
  }

  return status;
}



VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_corrector()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);
  VL53LX_smudge_corrector_config_t *pconfig =
    &(pdev->smudge_correct_config);
  VL53LX_smudge_corrector_internals_t *pint =
    &(pdev->smudge_corrector_internals);
  VL53LX_smudge_corrector_data_t *pout =
    &(pres->range_results.smudge_corrector_data);
  VL53LX_range_results_t  *pR = &(pres->range_results);
  VL53LX_xtalk_config_t  *pX = &(pdev->xtalk_cfg);

  uint8_t run_smudge_detection = 0;
  uint8_t merging_complete = 0;
  uint8_t run_nodetect = 0;
  uint8_t ambient_check = 0;
  int32_t itemp32 = 0;
  uint64_t utemp64 = 0;
  uint8_t continue_processing = CONT_CONTINUE;
  uint32_t xtalk_offset_out = 0;
  uint32_t xtalk_offset_in = 0;
  uint32_t current_xtalk = 0;
  uint32_t smudge_margin_adjusted = 0;
  uint8_t i = 0;
  uint8_t nodetect_index = 0;
  uint16_t    amr;
  uint32_t    cco;
  uint8_t histo_merge_nb;


  VL53LX_compute_histo_merge_nb(&histo_merge_nb);
  if ((histo_merge_nb == 0) ||
      (pdev->tuning_parms.tp_hist_merge != 1)) {
    histo_merge_nb = 1;
  }


  VL53LX_dynamic_xtalk_correction_output_init(pres);


  ambient_check = (pconfig->smudge_corr_ambient_threshold == 0) ||
                  ((pconfig->smudge_corr_ambient_threshold * histo_merge_nb)  >
                   ((uint32_t)pR->xmonitor.ambient_count_rate_mcps));


  merging_complete =
    ((pdev->tuning_parms.tp_hist_merge != 1) ||
     (histo_merge_nb == pdev->tuning_parms.tp_hist_merge_max_size));
  run_smudge_detection =
    (pconfig->smudge_corr_enabled == 1) &&
    ambient_check &&
    (pR->xmonitor.range_status
     == VL53LX_DEVICEERROR_RANGECOMPLETE) &&
    merging_complete;


  if ((pR->xmonitor.range_status
       != VL53LX_DEVICEERROR_RANGECOMPLETE) &&
      (pconfig->smudge_corr_enabled == 1)) {

    run_nodetect = 2;
    for (i = 0; i < pR->active_results; i++) {
      if (pR->VL53LX_p_003[i].range_status ==
          VL53LX_DEVICEERROR_RANGECOMPLETE) {
        if (pR->VL53LX_p_003[i].median_range_mm
            <=
            pconfig->nodetect_min_range_mm) {
          run_nodetect = 0;
        } else {
          if (run_nodetect == 2) {
            run_nodetect = 1;
            nodetect_index = i;
          }
        }
      }
    }

    if (run_nodetect == 2)

    {
      run_nodetect = 0;
    }

    amr =
      pR->VL53LX_p_003[nodetect_index].ambient_count_rate_mcps;

    if (run_nodetect == 1) {




      utemp64 = 1000 * ((uint64_t)amr);


      utemp64 = utemp64 << 9;


      if (utemp64 < pconfig->nodetect_ambient_threshold) {
        run_nodetect = 1;
      } else {
        run_nodetect = 0;
      }

    }
  }


  if (run_smudge_detection) {

    pint->nodetect_counter = 0;


    VL53LX_dynamic_xtalk_correction_calc_required_samples();


    xtalk_offset_in =
      pR->xmonitor.VL53LX_p_009;


    cco = pX->algo__crosstalk_compensation_plane_offset_kcps;
    current_xtalk = ((uint32_t)cco) << 2;


    smudge_margin_adjusted =
      ((uint32_t)(pconfig->smudge_margin)) << 2;


    itemp32 = xtalk_offset_in - current_xtalk +
              smudge_margin_adjusted;

    if (itemp32 < 0) {
      itemp32 = itemp32 * (-1);
    }


    if (itemp32 > ((int32_t)pconfig->single_xtalk_delta)) {
      if ((int32_t)xtalk_offset_in >
          ((int32_t)current_xtalk -
           (int32_t)smudge_margin_adjusted)) {
        pout->single_xtalk_delta_flag = 1;
      } else {
        pout->single_xtalk_delta_flag = 2;
      }
    }


    pint->current_samples = pint->current_samples + 1;


    if (pint->current_samples > pconfig->sample_limit) {
      pout->sample_limit_exceeded_flag = 1;
      continue_processing = CONT_RESET;
    } else {
      pint->accumulator = pint->accumulator +
                          xtalk_offset_in;
    }

    if (pint->current_samples < pint->required_samples) {
      continue_processing = CONT_NEXT_LOOP;
    }


    xtalk_offset_out =
      (uint32_t)(do_division_u(pint->accumulator,
                               pint->current_samples));


    itemp32 = xtalk_offset_out - current_xtalk +
              smudge_margin_adjusted;

    if (itemp32 < 0) {
      itemp32 = itemp32 * (-1);
    }

    if (continue_processing == CONT_CONTINUE &&
        (itemp32 >= ((int32_t)(pconfig->averaged_xtalk_delta)))
       ) {
      if ((int32_t)xtalk_offset_out >
          ((int32_t)current_xtalk -
           (int32_t)smudge_margin_adjusted)) {
        pout->averaged_xtalk_delta_flag = 1;
      } else {
        pout->averaged_xtalk_delta_flag = 2;
      }
    }

    if (continue_processing == CONT_CONTINUE &&
        (itemp32 < ((int32_t)(pconfig->averaged_xtalk_delta)))
       )

    {
      continue_processing = CONT_RESET;
    }



    pout->smudge_corr_clipped = 0;
    if ((continue_processing == CONT_CONTINUE) &&
        (pconfig->smudge_corr_clip_limit != 0)) {
      if (xtalk_offset_out >
          (pconfig->smudge_corr_clip_limit * histo_merge_nb)) {
        pout->smudge_corr_clipped = 1;
        continue_processing = CONT_RESET;
      }
    }



    if (pconfig->user_xtalk_offset_limit_hi &&
        (xtalk_offset_out >
         pconfig->user_xtalk_offset_limit))
      xtalk_offset_out =
        pconfig->user_xtalk_offset_limit;



    if ((pconfig->user_xtalk_offset_limit_hi == 0) &&
        (xtalk_offset_out <
         pconfig->user_xtalk_offset_limit))
      xtalk_offset_out =
        pconfig->user_xtalk_offset_limit;



    xtalk_offset_out = xtalk_offset_out >> 2;
    if (xtalk_offset_out > 0x3FFFF) {
      xtalk_offset_out = 0x3FFFF;
    }


    if (continue_processing == CONT_CONTINUE) {

      VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
        xtalk_offset_out,
        pconfig,
        pout,
        1,
        0
      );


      continue_processing = CONT_RESET;
    } else {

      VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
        xtalk_offset_out,
        pconfig,
        pout,
        1,
        1
      );
    }


    if (continue_processing == CONT_RESET) {
      pint->accumulator = 0;
      pint->current_samples = 0;
      pint->nodetect_counter = 0;
    }

  }

  continue_processing = CONT_CONTINUE;
  if (run_nodetect == 1) {

    pint->nodetect_counter += 1;


    if (pint->nodetect_counter < pconfig->nodetect_sample_limit) {
      continue_processing = CONT_NEXT_LOOP;
    }


    xtalk_offset_out = (uint32_t)(pconfig->nodetect_xtalk_offset);

    if (continue_processing == CONT_CONTINUE) {

      VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
        xtalk_offset_out,
        pconfig,
        pout,
        0,
        0
      );


      pout->smudge_corr_valid = 2;


      continue_processing = CONT_RESET;
    } else {

      VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
        xtalk_offset_out,
        pconfig,
        pout,
        0,
        1
      );
    }


    if (continue_processing == CONT_RESET) {
      pint->accumulator = 0;
      pint->current_samples = 0;
      pint->nodetect_counter = 0;
    }
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_data_init()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);

  pdev->smudge_correct_config.smudge_corr_enabled       = 1;
  pdev->smudge_correct_config.smudge_corr_apply_enabled = 1;
  pdev->smudge_correct_config.smudge_corr_single_apply  =
    VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY_DEFAULT;

  pdev->smudge_correct_config.smudge_margin =
    VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN_DEFAULT;
  pdev->smudge_correct_config.noise_margin =
    VL53LX_TUNINGPARM_DYNXTALK_NOISE_MARGIN_DEFAULT;
  pdev->smudge_correct_config.user_xtalk_offset_limit =
    VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_DEFAULT;
  pdev->smudge_correct_config.user_xtalk_offset_limit_hi =
    VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI_DEFAULT;
  pdev->smudge_correct_config.sample_limit =
    VL53LX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT_DEFAULT;
  pdev->smudge_correct_config.single_xtalk_delta =
    VL53LX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA_DEFAULT;
  pdev->smudge_correct_config.averaged_xtalk_delta =
    VL53LX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA_DEFAULT;
  pdev->smudge_correct_config.smudge_corr_clip_limit =
    VL53LX_TUNINGPARM_DYNXTALK_CLIP_LIMIT_DEFAULT;
  pdev->smudge_correct_config.smudge_corr_ambient_threshold =
    VL53LX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD_DEFAULT;
  pdev->smudge_correct_config.scaler_calc_method =
    0;
  pdev->smudge_correct_config.x_gradient_scaler =
    VL53LX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER_DEFAULT;
  pdev->smudge_correct_config.y_gradient_scaler =
    VL53LX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER_DEFAULT;
  pdev->smudge_correct_config.user_scaler_set =
    VL53LX_TUNINGPARM_DYNXTALK_USER_SCALER_SET_DEFAULT;
  pdev->smudge_correct_config.nodetect_ambient_threshold =
    VL53LX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS_DEFAULT;
  pdev->smudge_correct_config.nodetect_sample_limit =
    VL53LX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT_DEFAULT;
  pdev->smudge_correct_config.nodetect_xtalk_offset =
    VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS_DEFAULT;
  pdev->smudge_correct_config.nodetect_min_range_mm =
    VL53LX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM_DEFAULT;
  pdev->smudge_correct_config.max_smudge_factor =
    VL53LX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR_DEFAULT;


  pdev->smudge_corrector_internals.current_samples = 0;
  pdev->smudge_corrector_internals.required_samples = 0;
  pdev->smudge_corrector_internals.accumulator = 0;
  pdev->smudge_corrector_internals.nodetect_counter = 0;


  VL53LX_dynamic_xtalk_correction_output_init(pres);

  return status;
}


VL53LX_Error VL53LX::VL53LX_dynamic_xtalk_correction_output_init(
  VL53LX_LLDriverResults_t *pres
)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_smudge_corrector_data_t *pdata;

  pdata = &(pres->range_results.smudge_corrector_data);

  pdata->smudge_corr_valid = 0;
  pdata->smudge_corr_clipped = 0;
  pdata->single_xtalk_delta_flag = 0;
  pdata->averaged_xtalk_delta_flag = 0;
  pdata->sample_limit_exceeded_flag = 0;
  pdata->gradient_zero_flag = 0;
  pdata->new_xtalk_applied_flag = 0;

  pdata->algo__crosstalk_compensation_plane_offset_kcps = 0;
  pdata->algo__crosstalk_compensation_x_plane_gradient_kcps = 0;
  pdata->algo__crosstalk_compensation_y_plane_gradient_kcps = 0;

  return status;
}

VL53LX_Error VL53LX::VL53LX_xtalk_cal_data_init()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps = 0;
  pdev->xtalk_cal.algo__crosstalk_compensation_x_plane_gradient_kcps = 0;
  pdev->xtalk_cal.algo__crosstalk_compensation_y_plane_gradient_kcps = 0;
  memset(&pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[0], 0,
         sizeof(pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps));

  return status;
}

VL53LX_Error VL53LX::VL53LX_low_power_auto_data_init()
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->low_power_auto_data.vhv_loop_bound =
    VL53LX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT;
  pdev->low_power_auto_data.is_low_power_auto_mode = 0;
  pdev->low_power_auto_data.low_power_auto_range_count = 0;
  pdev->low_power_auto_data.saved_interrupt_config = 0;
  pdev->low_power_auto_data.saved_vhv_init = 0;
  pdev->low_power_auto_data.saved_vhv_timeout = 0;
  pdev->low_power_auto_data.first_run_phasecal_result = 0;
  pdev->low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
  pdev->low_power_auto_data.dss__required_spads = 0;

  return status;
}
VL53LX_Error VL53LX::VL53LX_low_power_auto_data_stop_range()
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  pdev->low_power_auto_data.low_power_auto_range_count = 0xFF;

  pdev->low_power_auto_data.first_run_phasecal_result = 0;
  pdev->low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
  pdev->low_power_auto_data.dss__required_spads = 0;


  if (pdev->low_power_auto_data.saved_vhv_init != 0)
    pdev->stat_nvm.vhv_config__init =
      pdev->low_power_auto_data.saved_vhv_init;
  if (pdev->low_power_auto_data.saved_vhv_timeout != 0)
    pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
      pdev->low_power_auto_data.saved_vhv_timeout;

  pdev->gen_cfg.phasecal_config__override = 0x00;

  return status;
}

VL53LX_Error VL53LX::VL53LX_config_low_power_auto_mode(
  VL53LX_general_config_t   *pgeneral,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_low_power_auto_data_t *plpadata
)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  plpadata->is_low_power_auto_mode = 1;


  plpadata->low_power_auto_range_count = 0;


  pdynamic->system__sequence_config =
    VL53LX_SEQUENCE_VHV_EN |
    VL53LX_SEQUENCE_PHASECAL_EN |
    VL53LX_SEQUENCE_DSS1_EN |



    VL53LX_SEQUENCE_RANGE_EN;


  pgeneral->dss_config__manual_effective_spads_select = 200 << 8;
  pgeneral->dss_config__roi_mode_control =
    VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;

  return status;
}


VL53LX_Error VL53LX::VL53LX_low_power_auto_setup_manual_calibration()
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdev->low_power_auto_data.saved_vhv_init =
    pdev->stat_nvm.vhv_config__init;
  pdev->low_power_auto_data.saved_vhv_timeout =
    pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;


  pdev->stat_nvm.vhv_config__init &= 0x7F;

  pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
    (pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03) +
    (pdev->low_power_auto_data.vhv_loop_bound << 2);

  pdev->gen_cfg.phasecal_config__override = 0x01;
  pdev->low_power_auto_data.first_run_phasecal_result =
    pdev->dbg_results.phasecal_result__vcsel_start;
  pdev->gen_cfg.cal_config__vcsel_start =
    pdev->low_power_auto_data.first_run_phasecal_result;

  return status;
}


VL53LX_Error VL53LX::VL53LX_low_power_auto_update_DSS()
{



  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_system_results_t *pS = &(pdev->sys_results);


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint32_t utemp32a;

  utemp32a =
    pS->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0
    + pS->result__ambient_count_rate_mcps_sd0;


  if (utemp32a > 0xFFFF) {
    utemp32a = 0xFFFF;
  }



  utemp32a = utemp32a << 16;


  if (pdev->sys_results.result__dss_actual_effective_spads_sd0 == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  } else {

    utemp32a = utemp32a /
               pdev->sys_results.result__dss_actual_effective_spads_sd0;

    pdev->low_power_auto_data.dss__total_rate_per_spad_mcps =
      utemp32a;


    utemp32a = pdev->stat_cfg.dss_config__target_total_rate_mcps <<
               16;


    if (pdev->low_power_auto_data.dss__total_rate_per_spad_mcps
        == 0) {
      status = VL53LX_ERROR_DIVISION_BY_ZERO;
    } else {

      utemp32a = utemp32a /
                 pdev->low_power_auto_data.dss__total_rate_per_spad_mcps;


      if (utemp32a > 0xFFFF) {
        utemp32a = 0xFFFF;
      }


      pdev->low_power_auto_data.dss__required_spads =
        (uint16_t)utemp32a;


      pdev->gen_cfg.dss_config__manual_effective_spads_select
        = pdev->low_power_auto_data.dss__required_spads;
      pdev->gen_cfg.dss_config__roi_mode_control =
        VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
    }

  }

  if (status == VL53LX_ERROR_DIVISION_BY_ZERO) {



    pdev->low_power_auto_data.dss__required_spads = 0x8000;


    pdev->gen_cfg.dss_config__manual_effective_spads_select =
      pdev->low_power_auto_data.dss__required_spads;
    pdev->gen_cfg.dss_config__roi_mode_control =
      VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;


    status = VL53LX_ERROR_NONE;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_compute_histo_merge_nb(uint8_t *histo_merge_nb)
{
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_Error  status = VL53LX_ERROR_NONE;
  uint8_t i, timing;
  uint8_t sum = 0;

  timing = (pdev->hist_data.bin_seq[0] == 7 ? 1 : 0);
  for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
    if (pdev->multi_bins_rec[i][timing][7] > 0) {
      sum++;
    }
  *histo_merge_nb = sum;

  return status;
}

/* vl53lx_wait.c */


VL53LX_Error VL53LX::VL53LX_wait_for_boot_completion()
{

  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t      fw_ready  = 0;

  if (pdev->wait_method == VL53LX_WAIT_METHOD_BLOCKING) {



    status =
      VL53LX_poll_for_boot_completion(VL53LX_BOOT_COMPLETION_POLLING_TIMEOUT_MS);

  } else {



    fw_ready = 0;
    while (fw_ready == 0x00 && status == VL53LX_ERROR_NONE) {
      status = VL53LX_is_boot_complete(
                 &fw_ready);

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_WaitMs(
                   Dev,
                   VL53LX_POLLING_DELAY_MS);
      }
    }
  }

  return status;

}

VL53LX_Error VL53LX::VL53LX_wait_for_firmware_ready()
{

  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t      fw_ready  = 0;
  uint8_t      mode_start  = 0;

  mode_start =
    pdev->sys_ctrl.system__mode_start &
    VL53LX_DEVICEMEASUREMENTMODE_MODE_MASK;



  if ((mode_start == VL53LX_DEVICEMEASUREMENTMODE_TIMED) ||
      (mode_start == VL53LX_DEVICEMEASUREMENTMODE_SINGLESHOT)) {

    if (pdev->wait_method == VL53LX_WAIT_METHOD_BLOCKING) {



      status =
        VL53LX_poll_for_firmware_ready(
          VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS);

    } else {



      fw_ready = 0;
      while (fw_ready == 0x00 && status ==
             VL53LX_ERROR_NONE) {
        status = VL53LX_is_firmware_ready(
                   &fw_ready);

        if (status == VL53LX_ERROR_NONE) {
          status = VL53LX_WaitMs(
                     Dev,
                     VL53LX_POLLING_DELAY_MS);
        }
      }
    }
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_wait_for_range_completion()
{

  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t      data_ready  = 0;

  if (pdev->wait_method == VL53LX_WAIT_METHOD_BLOCKING) {



    status =
      VL53LX_poll_for_range_completion(
        VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS);

  } else {



    data_ready = 0;
    while (data_ready == 0x00 && status == VL53LX_ERROR_NONE) {
      status = VL53LX_is_new_data_ready(
                 &data_ready);

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_WaitMs(
                   Dev,
                   VL53LX_POLLING_DELAY_MS);
      }
    }
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_wait_for_test_completion()
{

  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t      data_ready  = 0;


  if (pdev->wait_method == VL53LX_WAIT_METHOD_BLOCKING) {



    status =
      VL53LX_poll_for_range_completion(
        VL53LX_TEST_COMPLETION_POLLING_TIMEOUT_MS);

  } else {



    data_ready = 0;
    while (data_ready == 0x00 && status == VL53LX_ERROR_NONE) {
      status = VL53LX_is_new_data_ready(
                 &data_ready);

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_WaitMs(
                   Dev,
                   VL53LX_POLLING_DELAY_MS);
      }
    }
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_is_boot_complete(
  uint8_t       *pready)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  uint8_t  firmware__system_status = 0;

  status =
    VL53LX_RdByte(
      Dev,
      VL53LX_FIRMWARE__SYSTEM_STATUS,
      &firmware__system_status);



  if ((firmware__system_status & 0x01) == 0x01) {
    *pready = 0x01;
    VL53LX_init_ll_driver_state(
      VL53LX_DEVICESTATE_SW_STANDBY);
  } else {
    *pready = 0x00;
    VL53LX_init_ll_driver_state(
      VL53LX_DEVICESTATE_FW_COLDBOOT);
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_is_firmware_ready(
  uint8_t       *pready)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  status = VL53LX_is_firmware_ready_silicon(
             pready);

  pdev->fw_ready = *pready;

  return status;
}
VL53LX_Error VL53LX::VL53LX_is_new_data_ready(
  uint8_t       *pready)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t  gpio__mux_active_high_hv = 0;
  uint8_t  gpio__tio_hv_status      = 0;
  uint8_t  interrupt_ready          = 0;

  gpio__mux_active_high_hv =
    pdev->stat_cfg.gpio_hv_mux__ctrl &
    VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_MASK;

  if (gpio__mux_active_high_hv == VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_HIGH) {
    interrupt_ready = 0x01;
  } else {
    interrupt_ready = 0x00;
  }


  status = VL53LX_RdByte(
             Dev,
             VL53LX_GPIO__TIO_HV_STATUS,
             &gpio__tio_hv_status);



  if ((gpio__tio_hv_status & 0x01) == interrupt_ready) {
    *pready = 0x01;
  } else {
    *pready = 0x00;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_poll_for_boot_completion(
  uint32_t      timeout_ms)
{


  VL53LX_Error status       = VL53LX_ERROR_NONE;


  status = VL53LX_WaitUs(
             Dev,
             VL53LX_FIRMWARE_BOOT_TIME_US);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WaitValueMaskEx(
        Dev,
        timeout_ms,
        VL53LX_FIRMWARE__SYSTEM_STATUS,
        0x01,
        0x01,
        VL53LX_POLLING_DELAY_MS);

  if (status == VL53LX_ERROR_NONE) {
    VL53LX_init_ll_driver_state(VL53LX_DEVICESTATE_SW_STANDBY);
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_poll_for_firmware_ready(
  uint32_t      timeout_ms)
{

  VL53LX_Error status          = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint32_t     start_time_ms   = 0;
  uint32_t     current_time_ms = 0;
  int32_t      poll_delay_ms   = VL53LX_POLLING_DELAY_MS;
  uint8_t      fw_ready        = 0;



  VL53LX_GetTickCount(&start_time_ms);
  pdev->fw_ready_poll_duration_ms = 0;

  while ((status == VL53LX_ERROR_NONE) &&
         (pdev->fw_ready_poll_duration_ms < timeout_ms) &&
         (fw_ready == 0)) {

    status = VL53LX_is_firmware_ready(
               &fw_ready);

    if (status == VL53LX_ERROR_NONE &&
        fw_ready == 0 &&
        poll_delay_ms > 0) {
      status = VL53LX_WaitMs(
                 Dev,
                 poll_delay_ms);
    }


    VL53LX_GetTickCount(&current_time_ms);
    pdev->fw_ready_poll_duration_ms =
      current_time_ms - start_time_ms;
  }

  if (fw_ready == 0 && status == VL53LX_ERROR_NONE) {
    status = VL53LX_ERROR_TIME_OUT;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_poll_for_range_completion(
  uint32_t       timeout_ms)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t  gpio__mux_active_high_hv = 0;
  uint8_t  interrupt_ready          = 0;

  gpio__mux_active_high_hv =
    pdev->stat_cfg.gpio_hv_mux__ctrl &
    VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_MASK;

  if (gpio__mux_active_high_hv == VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_HIGH) {
    interrupt_ready = 0x01;
  } else {
    interrupt_ready = 0x00;
  }

  status =
    VL53LX_WaitValueMaskEx(
      Dev,
      timeout_ms,
      VL53LX_GPIO__TIO_HV_STATUS,
      interrupt_ready,
      0x01,
      VL53LX_POLLING_DELAY_MS);

  return status;
}

/* vl53lx_zone_presets.c */

VL53LX_Error VL53LX::VL53LX_init_zone_config_structure(
  uint8_t x_off,
  uint8_t x_inc,
  uint8_t x_zones,
  uint8_t y_off,
  uint8_t y_inc,
  uint8_t y_zones,
  uint8_t width,
  uint8_t height,
  VL53LX_zone_config_t   *pdata)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint8_t  x  = 0;
  uint8_t  y  = 0;
  uint16_t  i  = 0;


  pdata->max_zones = VL53LX_MAX_USER_ZONES;

  i = 0;

  for (x = 0 ; x < x_zones ; x++) {
    for (y = 0 ; y <  y_zones ; y++) {

      if (i < VL53LX_MAX_USER_ZONES) {

        pdata->active_zones = (uint8_t)i;
        pdata->user_zones[i].height   = height;
        pdata->user_zones[i].width    = width;
        pdata->user_zones[i].x_centre =
          x_off + (x * x_inc);
        pdata->user_zones[i].y_centre =
          y_off + (y * y_inc);
      }

      i++;
    }
  }

  status = VL53LX_init_zone_config_histogram_bins(pdata);

  return status;
}

VL53LX_Error VL53LX::VL53LX_zone_preset_xtalk_planar(
  VL53LX_general_config_t *pgeneral,
  VL53LX_zone_config_t    *pzone_cfg)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pgeneral->global_config__stream_divider = 0x05;


  pzone_cfg->active_zones                 = 0x04;

  pzone_cfg->user_zones[0].height         = 15;
  pzone_cfg->user_zones[0].width          = 7;
  pzone_cfg->user_zones[0].x_centre       = 4;
  pzone_cfg->user_zones[0].y_centre       = 8;

  pzone_cfg->user_zones[1].height         = 15;
  pzone_cfg->user_zones[1].width          = 7;
  pzone_cfg->user_zones[1].x_centre       = 12;
  pzone_cfg->user_zones[1].y_centre       = 8;

  pzone_cfg->user_zones[2].height         = 7;
  pzone_cfg->user_zones[2].width          = 15;
  pzone_cfg->user_zones[2].x_centre       = 8;
  pzone_cfg->user_zones[2].y_centre       = 4;

  pzone_cfg->user_zones[3].height         = 7;
  pzone_cfg->user_zones[3].width          = 15;
  pzone_cfg->user_zones[3].x_centre       = 8;
  pzone_cfg->user_zones[3].y_centre       = 12;



  pzone_cfg->user_zones[4].height         = 15;
  pzone_cfg->user_zones[4].width          = 15;
  pzone_cfg->user_zones[4].x_centre       = 8;
  pzone_cfg->user_zones[4].y_centre       = 8;

  status = VL53LX_init_zone_config_histogram_bins(pzone_cfg);

  return status;
}

VL53LX_Error VL53LX::VL53LX_init_zone_config_histogram_bins(
  VL53LX_zone_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  uint8_t i;


  for (i = 0; i < pdata->max_zones; i++) {
    pdata->bin_config[i] = VL53LX_ZONECONFIG_BINCONFIG__LOWAMB;
  }

  return status;
}

/* vl53lx_api_preset_modes.h */

VL53LX_Error VL53LX::VL53LX_init_refspadchar_config_struct(
  VL53LX_refspadchar_config_t   *pdata)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->device_test_mode =
    VL53LX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT;
  pdata->VL53LX_p_005              =
    VL53LX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT;
  pdata->timeout_us                =
    VL53LX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT;
  pdata->target_count_rate_mcps    =
    VL53LX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT;
  pdata->min_count_rate_limit_mcps =
    VL53LX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT;
  pdata->max_count_rate_limit_mcps =
    VL53LX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT;

  return status;
}

VL53LX_Error VL53LX::VL53LX_init_ssc_config_struct(
  VL53LX_ssc_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->array_select = VL53LX_DEVICESSCARRAY_RTN;


  pdata->VL53LX_p_005 =
    VL53LX_TUNINGPARM_SPADMAP_VCSEL_PERIOD_DEFAULT;


  pdata->vcsel_start  =
    VL53LX_TUNINGPARM_SPADMAP_VCSEL_START_DEFAULT;


  pdata->vcsel_width = 0x02;


  pdata->timeout_us   = 36000;


  pdata->rate_limit_mcps =
    VL53LX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT;


  return status;
}


VL53LX_Error VL53LX::VL53LX_init_xtalk_config_struct(
  VL53LX_customer_nvm_managed_t *pnvm,
  VL53LX_xtalk_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->algo__crosstalk_compensation_plane_offset_kcps      =
    pnvm->algo__crosstalk_compensation_plane_offset_kcps;
  pdata->algo__crosstalk_compensation_x_plane_gradient_kcps  =
    pnvm->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pdata->algo__crosstalk_compensation_y_plane_gradient_kcps  =
    pnvm->algo__crosstalk_compensation_y_plane_gradient_kcps;



  pdata->nvm_default__crosstalk_compensation_plane_offset_kcps      =
    (uint32_t)pnvm->algo__crosstalk_compensation_plane_offset_kcps;
  pdata->nvm_default__crosstalk_compensation_x_plane_gradient_kcps  =
    pnvm->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pdata->nvm_default__crosstalk_compensation_y_plane_gradient_kcps  =
    pnvm->algo__crosstalk_compensation_y_plane_gradient_kcps;

  pdata->histogram_mode_crosstalk_margin_kcps                =
    VL53LX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS_DEFAULT;
  pdata->lite_mode_crosstalk_margin_kcps                     =
    VL53LX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS_DEFAULT;



  pdata->crosstalk_range_ignore_threshold_mult =
    VL53LX_TUNINGPARM_LITE_RIT_MULT_DEFAULT;

  if ((pdata->algo__crosstalk_compensation_plane_offset_kcps == 0x00)
      && (pdata->algo__crosstalk_compensation_x_plane_gradient_kcps
          == 0x00)
      && (pdata->algo__crosstalk_compensation_y_plane_gradient_kcps
          == 0x00)) {
    pdata->global_crosstalk_compensation_enable = 0x00;
  } else {
    pdata->global_crosstalk_compensation_enable = 0x01;
  }


  if ((status == VL53LX_ERROR_NONE) &&
      (pdata->global_crosstalk_compensation_enable == 0x01)) {
    pdata->crosstalk_range_ignore_threshold_rate_mcps =
      VL53LX_calc_range_ignore_threshold(
        pdata->algo__crosstalk_compensation_plane_offset_kcps,
        pdata->algo__crosstalk_compensation_x_plane_gradient_kcps,
        pdata->algo__crosstalk_compensation_y_plane_gradient_kcps,
        pdata->crosstalk_range_ignore_threshold_mult);
  } else {
    pdata->crosstalk_range_ignore_threshold_rate_mcps = 0;
  }




  pdata->algo__crosstalk_detect_min_valid_range_mm  =
    VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM_DEFAULT;
  pdata->algo__crosstalk_detect_max_valid_range_mm  =
    VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM_DEFAULT;
  pdata->algo__crosstalk_detect_max_valid_rate_kcps =
    VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS_DEFAULT;
  pdata->algo__crosstalk_detect_max_sigma_mm        =
    VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM_DEFAULT;

  return status;
}

VL53LX_Error VL53LX::VL53LX_init_xtalk_extract_config_struct(
  VL53LX_xtalkextract_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->dss_config__target_total_rate_mcps          =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS_DEFAULT;

  pdata->mm_config_timeout_us                        =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US_DEFAULT;

  pdata->num_of_samples                              =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES_DEFAULT;

  pdata->phasecal_config_timeout_us                  =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US_DEFAULT;

  pdata->range_config_timeout_us                     =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US_DEFAULT;



  pdata->algo__crosstalk_extract_min_valid_range_mm  =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM_DEFAULT;
  pdata->algo__crosstalk_extract_max_valid_range_mm  =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM_DEFAULT;
  pdata->algo__crosstalk_extract_max_valid_rate_kcps =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS_DEFAULT;
  pdata->algo__crosstalk_extract_max_sigma_mm        =
    VL53LX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM_DEFAULT;


  return status;
}


VL53LX_Error VL53LX::VL53LX_init_offset_cal_config_struct(
  VL53LX_offsetcal_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->dss_config__target_total_rate_mcps          =
    VL53LX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT;

  pdata->phasecal_config_timeout_us                  =
    VL53LX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT;

  pdata->range_config_timeout_us                     =
    VL53LX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT;

  pdata->mm_config_timeout_us                        =
    VL53LX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT;


  pdata->pre_num_of_samples                          =
    VL53LX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES_DEFAULT;
  pdata->mm1_num_of_samples                          =
    VL53LX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES_DEFAULT;
  pdata->mm2_num_of_samples                          =
    VL53LX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES_DEFAULT;

  return status;
}

VL53LX_Error VL53LX::VL53LX_init_zone_cal_config_struct(
  VL53LX_zonecal_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->dss_config__target_total_rate_mcps          =
    VL53LX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS_DEFAULT;

  pdata->phasecal_config_timeout_us                  =
    VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US_DEFAULT;

  pdata->range_config_timeout_us                     =
    VL53LX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US_DEFAULT;

  pdata->mm_config_timeout_us                        =
    VL53LX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US_DEFAULT;

  pdata->phasecal_num_of_samples                     =
    VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES_DEFAULT;
  pdata->zone_num_of_samples                         =
    VL53LX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES_DEFAULT;


  return status;
}

VL53LX_Error VL53LX::VL53LX_init_hist_post_process_config_struct(
  uint8_t                             xtalk_compensation_enable,
  VL53LX_hist_post_process_config_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->hist_algo_select =
    VL53LX_TUNINGPARM_HIST_ALGO_SELECT_DEFAULT;



  pdata->hist_target_order =
    VL53LX_TUNINGPARM_HIST_TARGET_ORDER_DEFAULT;



  pdata->filter_woi0                   =
    VL53LX_TUNINGPARM_HIST_FILTER_WOI_0_DEFAULT;
  pdata->filter_woi1                   =
    VL53LX_TUNINGPARM_HIST_FILTER_WOI_1_DEFAULT;


  pdata->hist_amb_est_method =
    VL53LX_TUNINGPARM_HIST_AMB_EST_METHOD_DEFAULT;

  pdata->ambient_thresh_sigma0         =
    VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0_DEFAULT;
  pdata->ambient_thresh_sigma1         =
    VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1_DEFAULT;


  pdata->ambient_thresh_events_scaler     =
    VL53LX_TUNINGPARM_HIST_AMB_EVENTS_SCALER_DEFAULT;


  pdata->min_ambient_thresh_events     =
    VL53LX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS_DEFAULT;

  pdata->noise_threshold               =
    VL53LX_TUNINGPARM_HIST_NOISE_THRESHOLD_DEFAULT;

  pdata->signal_total_events_limit     =
    VL53LX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT_DEFAULT;
  pdata->sigma_estimator__sigma_ref_mm =
    VL53LX_TUNINGPARM_HIST_SIGMA_EST_REF_MM_DEFAULT;


  pdata->sigma_thresh                  =
    VL53LX_TUNINGPARM_HIST_SIGMA_THRESH_MM_DEFAULT;

  pdata->range_offset_mm            =      0;

  pdata->gain_factor                =
    VL53LX_TUNINGPARM_HIST_GAIN_FACTOR_DEFAULT;



  pdata->valid_phase_low = 0x08;
  pdata->valid_phase_high = 0x88;



  pdata->algo__consistency_check__phase_tolerance =
    VL53LX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE_DEFAULT;



  pdata->algo__consistency_check__event_sigma =
    VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_DEFAULT;


  pdata->algo__consistency_check__event_min_spad_count =
    VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT_DEFAULT;



  pdata->algo__consistency_check__min_max_tolerance =
    VL53LX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM_DEFAULT;


  pdata->algo__crosstalk_compensation_enable = xtalk_compensation_enable;


  pdata->algo__crosstalk_detect_min_valid_range_mm  =
    VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM_DEFAULT;
  pdata->algo__crosstalk_detect_max_valid_range_mm  =
    VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM_DEFAULT;
  pdata->algo__crosstalk_detect_max_valid_rate_kcps =
    VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS_DEFAULT;
  pdata->algo__crosstalk_detect_max_sigma_mm        =
    VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM_DEFAULT;





  pdata->algo__crosstalk_detect_event_sigma       =
    VL53LX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA_DEFAULT;



  pdata->algo__crosstalk_detect_min_max_tolerance   =
    VL53LX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE_DEFAULT;


  return status;
}


VL53LX_Error VL53LX::VL53LX_init_dmax_calibration_data_struct(
  VL53LX_dmax_calibration_data_t   *pdata)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->ref__actual_effective_spads = 0x5F2D;

  pdata->ref__peak_signal_count_rate_mcps = 0x0844;

  pdata->ref__distance_mm = 0x08A5;


  pdata->ref_reflectance_pc = 0x0014;

  pdata->coverglass_transmission = 0x0100;

  return status;
}

VL53LX_Error VL53LX::VL53LX_init_tuning_parm_storage_struct(
  VL53LX_tuning_parm_storage_t   *pdata)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->tp_tuning_parm_version              =
    VL53LX_TUNINGPARM_VERSION_DEFAULT;
  pdata->tp_tuning_parm_key_table_version    =
    VL53LX_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT;
  pdata->tp_tuning_parm_lld_version          =
    VL53LX_TUNINGPARM_LLD_VERSION_DEFAULT;
  pdata->tp_init_phase_rtn_lite_long         =
    VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT;
  pdata->tp_init_phase_rtn_lite_med          =
    VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT;
  pdata->tp_init_phase_rtn_lite_short        =
    VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT;
  pdata->tp_init_phase_ref_lite_long         =
    VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT;
  pdata->tp_init_phase_ref_lite_med          =
    VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT;
  pdata->tp_init_phase_ref_lite_short        =
    VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT;
  pdata->tp_init_phase_rtn_hist_long         =
    VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE_DEFAULT;
  pdata->tp_init_phase_rtn_hist_med          =
    VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE_DEFAULT;
  pdata->tp_init_phase_rtn_hist_short        =
    VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE_DEFAULT;
  pdata->tp_init_phase_ref_hist_long         =
    VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE_DEFAULT;
  pdata->tp_init_phase_ref_hist_med          =
    VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE_DEFAULT;
  pdata->tp_init_phase_ref_hist_short        =
    VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE_DEFAULT;
  pdata->tp_consistency_lite_phase_tolerance =
    VL53LX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT;
  pdata->tp_phasecal_target                  =
    VL53LX_TUNINGPARM_PHASECAL_TARGET_DEFAULT;
  pdata->tp_cal_repeat_rate                  =
    VL53LX_TUNINGPARM_LITE_CAL_REPEAT_RATE_DEFAULT;
  pdata->tp_lite_min_clip                    =
    VL53LX_TUNINGPARM_LITE_MIN_CLIP_MM_DEFAULT;
  pdata->tp_lite_long_sigma_thresh_mm        =
    VL53LX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT;
  pdata->tp_lite_med_sigma_thresh_mm         =
    VL53LX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM_DEFAULT;
  pdata->tp_lite_short_sigma_thresh_mm       =
    VL53LX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT;
  pdata->tp_lite_long_min_count_rate_rtn_mcps  =
    VL53LX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
  pdata->tp_lite_med_min_count_rate_rtn_mcps   =
    VL53LX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
  pdata->tp_lite_short_min_count_rate_rtn_mcps =
    VL53LX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
  pdata->tp_lite_sigma_est_pulse_width_ns      =
    VL53LX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT;
  pdata->tp_lite_sigma_est_amb_width_ns        =
    VL53LX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT;
  pdata->tp_lite_sigma_ref_mm                  =
    VL53LX_TUNINGPARM_LITE_SIGMA_REF_MM_DEFAULT;
  pdata->tp_lite_seed_cfg                      =
    VL53LX_TUNINGPARM_LITE_SEED_CONFIG_DEFAULT;
  pdata->tp_timed_seed_cfg                     =
    VL53LX_TUNINGPARM_TIMED_SEED_CONFIG_DEFAULT;
  pdata->tp_lite_quantifier                    =
    VL53LX_TUNINGPARM_LITE_QUANTIFIER_DEFAULT;
  pdata->tp_lite_first_order_select            =
    VL53LX_TUNINGPARM_LITE_FIRST_ORDER_SELECT_DEFAULT;




  pdata->tp_dss_target_lite_mcps               =
    VL53LX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
  pdata->tp_dss_target_histo_mcps              =
    VL53LX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
  pdata->tp_dss_target_histo_mz_mcps           =
    VL53LX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
  pdata->tp_dss_target_timed_mcps              =
    VL53LX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
  pdata->tp_phasecal_timeout_lite_us           =
    VL53LX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_hist_long_us      =
    VL53LX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_hist_med_us       =
    VL53LX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_hist_short_us     =
    VL53LX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_mz_long_us        =
    VL53LX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_mz_med_us         =
    VL53LX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_mz_short_us       =
    VL53LX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_phasecal_timeout_timed_us          =
    VL53LX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_mm_timeout_lite_us                 =
    VL53LX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_mm_timeout_histo_us                =
    VL53LX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_mm_timeout_mz_us                   =
    VL53LX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_mm_timeout_timed_us                =
    VL53LX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_range_timeout_lite_us              =
    VL53LX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_range_timeout_histo_us             =
    VL53LX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_range_timeout_mz_us                =
    VL53LX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_range_timeout_timed_us             =
    VL53LX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT;



  pdata->tp_mm_timeout_lpa_us =
    VL53LX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT;
  pdata->tp_range_timeout_lpa_us =
    VL53LX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

  pdata->tp_dss_target_very_short_mcps =
    VL53LX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS_DEFAULT;

  pdata->tp_phasecal_patch_power =
    VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER_DEFAULT;

  pdata->tp_hist_merge =
    VL53LX_TUNINGPARM_HIST_MERGE_DEFAULT;

  pdata->tp_reset_merge_threshold =
    VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD_DEFAULT;

  pdata->tp_hist_merge_max_size =
    VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE_DEFAULT;

  return status;
}

VL53LX_Error VL53LX::VL53LX_init_hist_gen3_dmax_config_struct(
  VL53LX_hist_gen3_dmax_config_t   *pdata)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pdata->dss_config__target_total_rate_mcps = 0x1400;
  pdata->dss_config__aperture_attenuation = 0x38;

  pdata->signal_thresh_sigma                 =
    VL53LX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA_DEFAULT;
  pdata->ambient_thresh_sigma = 0x70;
  pdata->min_ambient_thresh_events           = 16;
  pdata->signal_total_events_limit           = 100;
  pdata->max_effective_spads = 0xFFFF;



  pdata->target_reflectance_for_dmax_calc[0] =
    VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0_DEFAULT;
  pdata->target_reflectance_for_dmax_calc[1] =
    VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1_DEFAULT;
  pdata->target_reflectance_for_dmax_calc[2] =
    VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2_DEFAULT;
  pdata->target_reflectance_for_dmax_calc[3] =
    VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3_DEFAULT;
  pdata->target_reflectance_for_dmax_calc[4] =
    VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4_DEFAULT;

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_standard_ranging(
  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  pstatic->dss_config__target_total_rate_mcps = 0x0A00;
  pstatic->debug__ctrl = 0x00;
  pstatic->test_mode__ctrl = 0x00;
  pstatic->clk_gating__ctrl = 0x00;
  pstatic->nvm_bist__ctrl = 0x00;
  pstatic->nvm_bist__num_nvm_words = 0x00;
  pstatic->nvm_bist__start_address = 0x00;
  pstatic->host_if__status = 0x00;
  pstatic->pad_i2c_hv__config = 0x00;
  pstatic->pad_i2c_hv__extsup_config = 0x00;


  pstatic->gpio_hv_pad__ctrl = 0x00;


  pstatic->gpio_hv_mux__ctrl  =
    VL53LX_DEVICEINTERRUPTPOLARITY_ACTIVE_LOW |
    VL53LX_DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS;

  pstatic->gpio__tio_hv_status = 0x02;
  pstatic->gpio__fio_hv_status = 0x00;
  pstatic->ana_config__spad_sel_pswidth = 0x02;
  pstatic->ana_config__vcsel_pulse_width_offset = 0x08;
  pstatic->ana_config__fast_osc__config_ctrl = 0x00;

  pstatic->sigma_estimator__effective_pulse_width_ns        =
    ptuning_parms->tp_lite_sigma_est_pulse_width_ns;
  pstatic->sigma_estimator__effective_ambient_width_ns      =
    ptuning_parms->tp_lite_sigma_est_amb_width_ns;
  pstatic->sigma_estimator__sigma_ref_mm                    =
    ptuning_parms->tp_lite_sigma_ref_mm;

  pstatic->algo__crosstalk_compensation_valid_height_mm = 0x01;
  pstatic->spare_host_config__static_config_spare_0 = 0x00;
  pstatic->spare_host_config__static_config_spare_1 = 0x00;

  pstatic->algo__range_ignore_threshold_mcps = 0x0000;


  pstatic->algo__range_ignore_valid_height_mm = 0xff;
  pstatic->algo__range_min_clip                             =
    ptuning_parms->tp_lite_min_clip;

  pstatic->algo__consistency_check__tolerance               =
    ptuning_parms->tp_consistency_lite_phase_tolerance;
  pstatic->spare_host_config__static_config_spare_2 = 0x00;
  pstatic->sd_config__reset_stages_msb = 0x00;
  pstatic->sd_config__reset_stages_lsb = 0x00;

  pgeneral->gph_config__stream_count_update_value = 0x00;
  pgeneral->global_config__stream_divider = 0x00;
  pgeneral->system__interrupt_config_gpio =
    VL53LX_INTERRUPT_CONFIG_NEW_SAMPLE_READY;
  pgeneral->cal_config__vcsel_start = 0x0B;


  pgeneral->cal_config__repeat_rate                         =
    ptuning_parms->tp_cal_repeat_rate;
  pgeneral->global_config__vcsel_width = 0x02;

  pgeneral->phasecal_config__timeout_macrop = 0x0D;

  pgeneral->phasecal_config__target                         =
    ptuning_parms->tp_phasecal_target;
  pgeneral->phasecal_config__override = 0x00;
  pgeneral->dss_config__roi_mode_control =
    VL53LX_DEVICEDSSMODE__TARGET_RATE;

  pgeneral->system__thresh_rate_high = 0x0000;
  pgeneral->system__thresh_rate_low = 0x0000;

  pgeneral->dss_config__manual_effective_spads_select = 0x8C00;
  pgeneral->dss_config__manual_block_select = 0x00;


  pgeneral->dss_config__aperture_attenuation = 0x38;
  pgeneral->dss_config__max_spads_limit = 0xFF;
  pgeneral->dss_config__min_spads_limit = 0x01;




  ptiming->mm_config__timeout_macrop_a_hi = 0x00;
  ptiming->mm_config__timeout_macrop_a_lo = 0x1a;
  ptiming->mm_config__timeout_macrop_b_hi = 0x00;
  ptiming->mm_config__timeout_macrop_b_lo = 0x20;

  ptiming->range_config__timeout_macrop_a_hi = 0x01;
  ptiming->range_config__timeout_macrop_a_lo = 0xCC;

  ptiming->range_config__vcsel_period_a = 0x0B;

  ptiming->range_config__timeout_macrop_b_hi = 0x01;
  ptiming->range_config__timeout_macrop_b_lo = 0xF5;

  ptiming->range_config__vcsel_period_b = 0x09;

  ptiming->range_config__sigma_thresh                       =
    ptuning_parms->tp_lite_med_sigma_thresh_mm;

  ptiming->range_config__min_count_rate_rtn_limit_mcps      =
    ptuning_parms->tp_lite_med_min_count_rate_rtn_mcps;


  ptiming->range_config__valid_phase_low = 0x08;
  ptiming->range_config__valid_phase_high = 0x78;
  ptiming->system__intermeasurement_period = 0x00000000;
  ptiming->system__fractional_enable = 0x00;



  phistogram->histogram_config__low_amb_even_bin_0_1 = 0x07;
  phistogram->histogram_config__low_amb_even_bin_2_3 = 0x21;
  phistogram->histogram_config__low_amb_even_bin_4_5 = 0x43;

  phistogram->histogram_config__low_amb_odd_bin_0_1 = 0x10;
  phistogram->histogram_config__low_amb_odd_bin_2_3 = 0x32;
  phistogram->histogram_config__low_amb_odd_bin_4_5 = 0x54;

  phistogram->histogram_config__mid_amb_even_bin_0_1 = 0x07;
  phistogram->histogram_config__mid_amb_even_bin_2_3 = 0x21;
  phistogram->histogram_config__mid_amb_even_bin_4_5 = 0x43;

  phistogram->histogram_config__mid_amb_odd_bin_0_1 = 0x10;
  phistogram->histogram_config__mid_amb_odd_bin_2 = 0x02;
  phistogram->histogram_config__mid_amb_odd_bin_3_4 = 0x43;
  phistogram->histogram_config__mid_amb_odd_bin_5 = 0x05;

  phistogram->histogram_config__user_bin_offset = 0x00;

  phistogram->histogram_config__high_amb_even_bin_0_1 = 0x07;
  phistogram->histogram_config__high_amb_even_bin_2_3 = 0x21;
  phistogram->histogram_config__high_amb_even_bin_4_5 = 0x43;

  phistogram->histogram_config__high_amb_odd_bin_0_1 = 0x10;
  phistogram->histogram_config__high_amb_odd_bin_2_3 = 0x32;
  phistogram->histogram_config__high_amb_odd_bin_4_5 = 0x54;

  phistogram->histogram_config__amb_thresh_low = 0xFFFF;
  phistogram->histogram_config__amb_thresh_high = 0xFFFF;

  phistogram->histogram_config__spad_array_selection = 0x00;


  pzone_cfg->max_zones                     = VL53LX_MAX_USER_ZONES;
  pzone_cfg->active_zones = 0x00;
  pzone_cfg->user_zones[0].height = 0x0f;
  pzone_cfg->user_zones[0].width = 0x0f;
  pzone_cfg->user_zones[0].x_centre = 0x08;
  pzone_cfg->user_zones[0].y_centre = 0x08;



  pdynamic->system__grouped_parameter_hold_0 = 0x01;

  pdynamic->system__thresh_high = 0x0000;
  pdynamic->system__thresh_low = 0x0000;
  pdynamic->system__enable_xtalk_per_quadrant = 0x00;
  pdynamic->system__seed_config =
    ptuning_parms->tp_lite_seed_cfg;


  pdynamic->sd_config__woi_sd0 = 0x0B;

  pdynamic->sd_config__woi_sd1 = 0x09;

  pdynamic->sd_config__initial_phase_sd0                     =
    ptuning_parms->tp_init_phase_rtn_lite_med;
  pdynamic->sd_config__initial_phase_sd1                     =
    ptuning_parms->tp_init_phase_ref_lite_med;

  pdynamic->system__grouped_parameter_hold_1 = 0x01;



  pdynamic->sd_config__first_order_select =
    ptuning_parms->tp_lite_first_order_select;
  pdynamic->sd_config__quantifier         =
    ptuning_parms->tp_lite_quantifier;


  pdynamic->roi_config__user_roi_centre_spad = 0xC7;

  pdynamic->roi_config__user_roi_requested_global_xy_size = 0xFF;


  pdynamic->system__sequence_config                          =
    VL53LX_SEQUENCE_VHV_EN |
    VL53LX_SEQUENCE_PHASECAL_EN |
    VL53LX_SEQUENCE_DSS1_EN |
    VL53LX_SEQUENCE_DSS2_EN |
    VL53LX_SEQUENCE_MM2_EN |
    VL53LX_SEQUENCE_RANGE_EN;

  pdynamic->system__grouped_parameter_hold = 0x02;




  psystem->system__stream_count_ctrl = 0x00;
  psystem->firmware__enable = 0x01;
  psystem->system__interrupt_clear                           =
    VL53LX_CLEAR_RANGE_INT;

  psystem->system__mode_start                                =
    VL53LX_DEVICESCHEDULERMODE_STREAMING |
    VL53LX_DEVICEREADOUTMODE_SINGLE_SD |
    VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_standard_ranging_short_range(
  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    ptiming->range_config__vcsel_period_a = 0x07;
    ptiming->range_config__vcsel_period_b = 0x05;
    ptiming->range_config__sigma_thresh                  =
      ptuning_parms->tp_lite_short_sigma_thresh_mm;
    ptiming->range_config__min_count_rate_rtn_limit_mcps =
      ptuning_parms->tp_lite_short_min_count_rate_rtn_mcps;
    ptiming->range_config__valid_phase_low = 0x08;
    ptiming->range_config__valid_phase_high = 0x38;



    pdynamic->sd_config__woi_sd0 = 0x07;
    pdynamic->sd_config__woi_sd1 = 0x05;
    pdynamic->sd_config__initial_phase_sd0               =
      ptuning_parms->tp_init_phase_rtn_lite_short;
    pdynamic->sd_config__initial_phase_sd1               =
      ptuning_parms->tp_init_phase_ref_lite_short;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_standard_ranging_long_range(
  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    ptiming->range_config__vcsel_period_a = 0x0F;
    ptiming->range_config__vcsel_period_b = 0x0D;
    ptiming->range_config__sigma_thresh                  =
      ptuning_parms->tp_lite_long_sigma_thresh_mm;
    ptiming->range_config__min_count_rate_rtn_limit_mcps =
      ptuning_parms->tp_lite_long_min_count_rate_rtn_mcps;
    ptiming->range_config__valid_phase_low = 0x08;
    ptiming->range_config__valid_phase_high = 0xB8;



    pdynamic->sd_config__woi_sd0 = 0x0F;
    pdynamic->sd_config__woi_sd1 = 0x0D;
    pdynamic->sd_config__initial_phase_sd0               =
      ptuning_parms->tp_init_phase_rtn_lite_long;
    pdynamic->sd_config__initial_phase_sd1               =
      ptuning_parms->tp_init_phase_ref_lite_long;
  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_standard_ranging_mm1_cal(
  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {

    pgeneral->dss_config__roi_mode_control =
      VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;

    pdynamic->system__sequence_config  =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_standard_ranging_mm2_cal(
  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;



  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {

    pgeneral->dss_config__roi_mode_control =
      VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;

    pdynamic->system__sequence_config  =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM2_EN;
  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_timed_ranging(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {




    pdynamic->system__grouped_parameter_hold = 0x00;


    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0xB1;

    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0xD4;



    ptiming->system__intermeasurement_period = 0x00000600;
    pdynamic->system__seed_config =
      ptuning_parms->tp_timed_seed_cfg;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_PSEUDO_SOLO |
      VL53LX_DEVICEREADOUTMODE_SINGLE_SD     |
      VL53LX_DEVICEMEASUREMENTMODE_TIMED;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_timed_ranging_short_range(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;


  status = VL53LX_preset_mode_standard_ranging_short_range(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {




    pdynamic->system__grouped_parameter_hold = 0x00;





    ptiming->range_config__timeout_macrop_a_hi = 0x01;
    ptiming->range_config__timeout_macrop_a_lo = 0x84;

    ptiming->range_config__timeout_macrop_b_hi = 0x01;
    ptiming->range_config__timeout_macrop_b_lo = 0xB1;

    ptiming->system__intermeasurement_period = 0x00000600;
    pdynamic->system__seed_config =
      ptuning_parms->tp_timed_seed_cfg;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_PSEUDO_SOLO |
      VL53LX_DEVICEREADOUTMODE_SINGLE_SD     |
      VL53LX_DEVICEMEASUREMENTMODE_TIMED;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_timed_ranging_long_range(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  status = VL53LX_preset_mode_standard_ranging_long_range(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {




    pdynamic->system__grouped_parameter_hold = 0x00;





    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x97;

    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0xB1;

    ptiming->system__intermeasurement_period = 0x00000600;
    pdynamic->system__seed_config =
      ptuning_parms->tp_timed_seed_cfg;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_PSEUDO_SOLO |
      VL53LX_DEVICEREADOUTMODE_SINGLE_SD     |
      VL53LX_DEVICEMEASUREMENTMODE_TIMED;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_low_power_auto_ranging(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg,
  VL53LX_low_power_auto_data_t *plpadata)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_timed_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_config_low_power_auto_mode(
               pgeneral,
               pdynamic,
               plpadata
             );
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_preset_mode_low_power_auto_short_ranging(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg,
  VL53LX_low_power_auto_data_t *plpadata)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_timed_ranging_short_range(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_config_low_power_auto_mode(
               pgeneral,
               pdynamic,
               plpadata
             );
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_low_power_auto_long_ranging(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg,
  VL53LX_low_power_auto_data_t *plpadata)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_timed_ranging_long_range(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_config_low_power_auto_mode(
               pgeneral,
               pdynamic,
               plpadata
             );
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_singleshot_ranging(

  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {

    pdynamic->system__grouped_parameter_hold = 0x00;




    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0xB1;

    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0xD4;

    pdynamic->system__seed_config =
      ptuning_parms->tp_timed_seed_cfg;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_PSEUDO_SOLO |
      VL53LX_DEVICEREADOUTMODE_SINGLE_SD     |
      VL53LX_DEVICEMEASUREMENTMODE_SINGLESHOT;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_standard_ranging(
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pstatic->dss_config__target_total_rate_mcps = 0x1400;



    VL53LX_init_histogram_config_structure(
      7, 0, 1, 2, 3, 4,
      0, 1, 2, 3, 4, 5,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 0, 1, 2, 3, 4,
      0, 1, 2, 3, 4, 5,
      &(pzone_cfg->multizone_hist_cfg));




    ptiming->range_config__vcsel_period_a = 0x09;
    ptiming->range_config__vcsel_period_b = 0x0B;
    pdynamic->sd_config__woi_sd0 = 0x09;
    pdynamic->sd_config__woi_sd1 = 0x0B;




    ptiming->mm_config__timeout_macrop_a_hi = 0x00;
    ptiming->mm_config__timeout_macrop_a_lo = 0x20;
    ptiming->mm_config__timeout_macrop_b_hi = 0x00;
    ptiming->mm_config__timeout_macrop_b_lo = 0x1A;


    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x28;


    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0x21;


    pgeneral->phasecal_config__timeout_macrop = 0xF5;



    phistpostprocess->valid_phase_low = 0x08;
    phistpostprocess->valid_phase_high = 0x88;



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);




    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |


      VL53LX_SEQUENCE_RANGE_EN;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging_with_mm1(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    VL53LX_init_histogram_config_structure(
      7,   0,   1, 2, 3, 4,
      8 + 0, 8 + 1, 8 + 2, 3, 4, 5,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7,  0,    1, 2, 3, 4,
      8 + 0, 8 + 1, 8 + 2, 3, 4, 5,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN |
      VL53LX_SEQUENCE_RANGE_EN;



    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging_with_mm2(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging_with_mm1(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM2_EN |
      VL53LX_SEQUENCE_RANGE_EN;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging_mm1_cal(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    VL53LX_init_histogram_config_structure(
      7, 8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4,
      8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4, 8 + 5,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4,
      8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4, 8 + 5,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    pgeneral->dss_config__roi_mode_control =
      VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN |
      VL53LX_SEQUENCE_RANGE_EN;

  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging_mm2_cal(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging_mm1_cal(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);

  if (status == VL53LX_ERROR_NONE) {



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM2_EN |
      VL53LX_SEQUENCE_RANGE_EN;

  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging_short_timing(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pstatic->dss_config__target_total_rate_mcps = 0x1400;



    VL53LX_init_histogram_config_structure(
      7, 0, 1, 2, 3, 4,
      7, 0, 1, 2, 3, 4,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 0, 1, 2, 3, 4,
      7, 0, 1, 2, 3, 4,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    ptiming->range_config__vcsel_period_a = 0x04;
    ptiming->range_config__vcsel_period_b = 0x03;
    ptiming->mm_config__timeout_macrop_a_hi = 0x00;
    ptiming->mm_config__timeout_macrop_a_lo = 0x42;
    ptiming->mm_config__timeout_macrop_b_hi = 0x00;
    ptiming->mm_config__timeout_macrop_b_lo = 0x42;
    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x52;
    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0x66;

    pgeneral->cal_config__vcsel_start = 0x04;



    pgeneral->phasecal_config__timeout_macrop = 0xa4;



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |


      VL53LX_SEQUENCE_RANGE_EN;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }

  return status;
}



VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_long_range(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      7, 0, 1, 2, 3, 4,
      0, 1, 2, 3, 4, 5,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 0, 1, 2, 3, 4,
      0, 1, 2, 3, 4, 5,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    ptiming->range_config__vcsel_period_a = 0x09;
    ptiming->range_config__vcsel_period_b = 0x0b;



    ptiming->mm_config__timeout_macrop_a_hi = 0x00;
    ptiming->mm_config__timeout_macrop_a_lo = 0x21;
    ptiming->mm_config__timeout_macrop_b_hi = 0x00;
    ptiming->mm_config__timeout_macrop_b_lo = 0x1b;



    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x29;
    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0x22;



    pgeneral->cal_config__vcsel_start = 0x09;



    pgeneral->phasecal_config__timeout_macrop = 0xF5;



    pdynamic->sd_config__woi_sd0 = 0x09;
    pdynamic->sd_config__woi_sd1 = 0x0B;
    pdynamic->sd_config__initial_phase_sd0            =
      ptuning_parms->tp_init_phase_rtn_hist_long;
    pdynamic->sd_config__initial_phase_sd1            =
      ptuning_parms->tp_init_phase_ref_hist_long;



    phistpostprocess->valid_phase_low = 0x08;
    phistpostprocess->valid_phase_high = 0x88;

    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_RANGE_EN;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_long_range_mm1(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_long_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      7,   0,   1, 2, 3, 4,
      8 + 0, 8 + 1, 8 + 2, 3, 4, 5,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7,   0,   1, 2, 3, 4,
      8 + 0, 8 + 1, 8 + 2, 3, 4, 5,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN |
      VL53LX_SEQUENCE_RANGE_EN;
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_long_range_mm2(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t      *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;


  status =
    VL53LX_preset_mode_histogram_long_range_mm1(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM2_EN |
      VL53LX_SEQUENCE_RANGE_EN;
  }

  return status;
}



VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_medium_range(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      7, 0, 1, 1, 2, 2,
      0, 1, 2, 1, 2, 3,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 0, 1, 1, 2, 2,
      0, 1, 2, 1, 2, 3,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    ptiming->range_config__vcsel_period_a = 0x05;
    ptiming->range_config__vcsel_period_b = 0x07;



    ptiming->mm_config__timeout_macrop_a_hi = 0x00;
    ptiming->mm_config__timeout_macrop_a_lo = 0x36;
    ptiming->mm_config__timeout_macrop_b_hi = 0x00;
    ptiming->mm_config__timeout_macrop_b_lo = 0x28;



    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x44;
    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0x33;



    pgeneral->cal_config__vcsel_start = 0x05;



    pgeneral->phasecal_config__timeout_macrop = 0xF5;



    pdynamic->sd_config__woi_sd0 = 0x05;
    pdynamic->sd_config__woi_sd1 = 0x07;
    pdynamic->sd_config__initial_phase_sd0            =
      ptuning_parms->tp_init_phase_rtn_hist_med;
    pdynamic->sd_config__initial_phase_sd1            =
      ptuning_parms->tp_init_phase_ref_hist_med;



    phistpostprocess->valid_phase_low = 0x08;
    phistpostprocess->valid_phase_high = 0x48;

    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_RANGE_EN;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_medium_range_mm1(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_medium_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    VL53LX_init_histogram_config_structure(
      7,   0,   1, 1, 2, 2,
      8 + 0, 8 + 1, 8 + 2, 1, 2, 3,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7,   0,   1, 1, 2, 2,
      8 + 0, 8 + 1, 8 + 2, 1, 2, 3,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN  |
      VL53LX_SEQUENCE_RANGE_EN;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_medium_range_mm2(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_medium_range_mm1(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM2_EN |
      VL53LX_SEQUENCE_RANGE_EN;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_short_range(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      7, 7, 0, 1, 1, 1,
      0, 1, 1, 1, 2, 2,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 7, 0, 1, 1, 1,
      0, 1, 1, 1, 2, 2,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    ptiming->range_config__vcsel_period_a = 0x03;
    ptiming->range_config__vcsel_period_b = 0x05;



    ptiming->mm_config__timeout_macrop_a_hi = 0x00;
    ptiming->mm_config__timeout_macrop_a_lo = 0x52;
    ptiming->mm_config__timeout_macrop_b_hi = 0x00;
    ptiming->mm_config__timeout_macrop_b_lo = 0x37;



    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x66;
    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0x44;



    pgeneral->cal_config__vcsel_start = 0x03;



    pgeneral->phasecal_config__timeout_macrop = 0xF5;



    pdynamic->sd_config__woi_sd0 = 0x03;
    pdynamic->sd_config__woi_sd1 = 0x05;
    pdynamic->sd_config__initial_phase_sd0            =
      ptuning_parms->tp_init_phase_rtn_hist_short;
    pdynamic->sd_config__initial_phase_sd1            =
      ptuning_parms->tp_init_phase_ref_hist_short;


    phistpostprocess->valid_phase_low = 0x08;
    phistpostprocess->valid_phase_high = 0x28;

    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN  |

      VL53LX_SEQUENCE_RANGE_EN;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }

  return status;
}



VL53LX_Error VL53LX::VL53LX_preset_mode_special_histogram_short_range(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;


  status =
    VL53LX_preset_mode_histogram_short_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      7, 7, 0, 0, 1, 1,
      0, 0, 0, 1, 1, 1,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7, 7, 0, 0, 1, 1,
      0, 0, 0, 1, 1, 1,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    ptiming->range_config__vcsel_period_a = 0x02;
    ptiming->range_config__vcsel_period_b = 0x03;



    pgeneral->cal_config__vcsel_start = 0x00;



    pgeneral->phasecal_config__target = 0x31;



    pdynamic->sd_config__woi_sd0 = 0x02;
    pdynamic->sd_config__woi_sd1 = 0x03;
    pdynamic->sd_config__initial_phase_sd0            =
      ptuning_parms->tp_init_phase_rtn_hist_short;
    pdynamic->sd_config__initial_phase_sd1            =
      ptuning_parms->tp_init_phase_ref_hist_short;



    phistpostprocess->valid_phase_low = 0x10;
    phistpostprocess->valid_phase_high = 0x18;

  }

  return status;
}



VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_short_range_mm1(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_short_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      7,   7, 0, 1, 1, 1,
      8 + 0, 8 + 1, 1, 1, 2, 2,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      7,   7, 0, 1, 1, 1,
      8 + 0, 8 + 1, 1, 1, 2, 2,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN  |
      VL53LX_SEQUENCE_RANGE_EN;

  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_short_range_mm2(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_short_range_mm1(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM2_EN |
      VL53LX_SEQUENCE_RANGE_EN;
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_characterisation(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    pstatic->debug__ctrl = 0x01;
    psystem->power_management__go1_power_force = 0x01;

    pdynamic->system__sequence_config               =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_RANGE_EN;

    psystem->system__mode_start                     =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM    |
      VL53LX_DEVICEREADOUTMODE_SPLIT_MANUAL   |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_xtalk_planar(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;


  status =
    VL53LX_preset_mode_histogram_multizone_long_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    status =
      VL53LX_zone_preset_xtalk_planar(
        pgeneral,
        pzone_cfg);



    ptiming->range_config__vcsel_period_a = 0x09;
    ptiming->range_config__vcsel_period_b = 0x09;



    VL53LX_init_histogram_config_structure(
      7, 0, 1, 2, 3, 4,
      7, 0, 1, 2, 3, 4,
      phistogram);



    VL53LX_init_histogram_multizone_config_structure(
      7, 0, 1, 2, 3, 4,
      7, 0, 1, 2, 3, 4,
      &(pzone_cfg->multizone_hist_cfg));




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_set_histogram_multizone_initial_bin_config(
          pzone_cfg,
          phistogram,
          &(pzone_cfg->multizone_hist_cfg));
    }



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);

  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_xtalk_mm1(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;


  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);




  if (status == VL53LX_ERROR_NONE) {





    VL53LX_init_histogram_config_structure(
      8 + 7, 8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4,
      8 + 7, 8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4,
      phistogram);


    VL53LX_init_histogram_multizone_config_structure(
      8 + 7, 8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4,
      8 + 7, 8 + 0, 8 + 1, 8 + 2, 8 + 3, 8 + 4,
      &(pzone_cfg->multizone_hist_cfg));



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);



    ptiming->range_config__vcsel_period_a = 0x09;
    ptiming->range_config__vcsel_period_b = 0x09;



    ptiming->mm_config__timeout_macrop_a_hi = 0x00;
    ptiming->mm_config__timeout_macrop_a_lo = 0x21;
    ptiming->mm_config__timeout_macrop_b_hi = 0x00;
    ptiming->mm_config__timeout_macrop_b_lo = 0x21;



    ptiming->range_config__timeout_macrop_a_hi = 0x00;
    ptiming->range_config__timeout_macrop_a_lo = 0x29;
    ptiming->range_config__timeout_macrop_b_hi = 0x00;
    ptiming->range_config__timeout_macrop_b_lo = 0x29;



    pgeneral->cal_config__vcsel_start = 0x09;



    pgeneral->phasecal_config__timeout_macrop = 0xF5;



    pdynamic->sd_config__woi_sd0 = 0x09;
    pdynamic->sd_config__woi_sd1 = 0x09;
    pdynamic->sd_config__initial_phase_sd0 = 0x09;
    pdynamic->sd_config__initial_phase_sd1 = 0x06;

    pdynamic->system__sequence_config =
      VL53LX_SEQUENCE_VHV_EN |
      VL53LX_SEQUENCE_PHASECAL_EN |
      VL53LX_SEQUENCE_DSS1_EN |
      VL53LX_SEQUENCE_DSS2_EN |
      VL53LX_SEQUENCE_MM1_EN |
      VL53LX_SEQUENCE_RANGE_EN;




    psystem->system__mode_start =
      VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
      VL53LX_DEVICEREADOUTMODE_DUAL_SD |
      VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }

  return status;
}
VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_xtalk_mm2(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_xtalk_mm1(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);


  pdynamic->system__sequence_config =
    VL53LX_SEQUENCE_VHV_EN |
    VL53LX_SEQUENCE_PHASECAL_EN |
    VL53LX_SEQUENCE_DSS1_EN |
    VL53LX_SEQUENCE_DSS2_EN |
    VL53LX_SEQUENCE_MM2_EN |
    VL53LX_SEQUENCE_RANGE_EN;


  return status;
}


VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_multizone(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_medium_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    status =
      VL53LX_init_zone_config_structure(
        4, 8, 2,
        4, 8, 2,
        7, 7,
        pzone_cfg);

    pgeneral->global_config__stream_divider =
      pzone_cfg->active_zones + 1;



    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_set_histogram_multizone_initial_bin_config(
          pzone_cfg,
          phistogram,
          &(pzone_cfg->multizone_hist_cfg));
    }

    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_multizone_short_range(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_short_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    status =
      VL53LX_init_zone_config_structure(
        4, 8, 2,
        4, 8, 2,
        7, 7,
        pzone_cfg);

    pgeneral->global_config__stream_divider =
      pzone_cfg->active_zones + 1;



    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_set_histogram_multizone_initial_bin_config(
          pzone_cfg,
          phistogram,
          &(pzone_cfg->multizone_hist_cfg)
        );
    }



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_multizone_long_range(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_long_range(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {



    status =
      VL53LX_init_zone_config_structure(
        4, 8, 2,
        4, 8, 2,
        7, 7,
        pzone_cfg);

    pgeneral->global_config__stream_divider =
      pzone_cfg->active_zones + 1;



    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_set_histogram_multizone_initial_bin_config(
          pzone_cfg,
          phistogram,
          &(pzone_cfg->multizone_hist_cfg));
    }



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_preset_mode_olt(
  VL53LX_static_config_t    *pstatic,
  VL53LX_histogram_config_t *phistogram,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic,
  VL53LX_system_control_t   *psystem,
  VL53LX_tuning_parm_storage_t *ptuning_parms,
  VL53LX_zone_config_t      *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status = VL53LX_preset_mode_standard_ranging(
             pstatic,
             phistogram,
             pgeneral,
             ptiming,
             pdynamic,
             psystem,
             ptuning_parms,
             pzone_cfg);



  if (status == VL53LX_ERROR_NONE)

  {
    psystem->system__stream_count_ctrl = 0x01;
  }


  return status;
}

void VL53LX::VL53LX_copy_hist_cfg_to_static_cfg(
  VL53LX_histogram_config_t *phistogram,
  VL53LX_static_config_t    *pstatic,
  VL53LX_general_config_t   *pgeneral,
  VL53LX_timing_config_t    *ptiming,
  VL53LX_dynamic_config_t   *pdynamic)
{

  SUPPRESS_UNUSED_WARNING(pgeneral);

  pstatic->sigma_estimator__effective_pulse_width_ns =
    phistogram->histogram_config__high_amb_even_bin_0_1;
  pstatic->sigma_estimator__effective_ambient_width_ns =
    phistogram->histogram_config__high_amb_even_bin_2_3;
  pstatic->sigma_estimator__sigma_ref_mm =
    phistogram->histogram_config__high_amb_even_bin_4_5;

  pstatic->algo__crosstalk_compensation_valid_height_mm =
    phistogram->histogram_config__high_amb_odd_bin_0_1;

  pstatic->spare_host_config__static_config_spare_0 =
    phistogram->histogram_config__high_amb_odd_bin_2_3;
  pstatic->spare_host_config__static_config_spare_1 =
    phistogram->histogram_config__high_amb_odd_bin_4_5;

  pstatic->algo__range_ignore_threshold_mcps =
    (((uint16_t)phistogram->histogram_config__mid_amb_even_bin_0_1)
     << 8)
    + (uint16_t)phistogram->histogram_config__mid_amb_even_bin_2_3;

  pstatic->algo__range_ignore_valid_height_mm =
    phistogram->histogram_config__mid_amb_even_bin_4_5;
  pstatic->algo__range_min_clip =
    phistogram->histogram_config__mid_amb_odd_bin_0_1;
  pstatic->algo__consistency_check__tolerance =
    phistogram->histogram_config__mid_amb_odd_bin_2;

  pstatic->spare_host_config__static_config_spare_2 =
    phistogram->histogram_config__mid_amb_odd_bin_3_4;
  pstatic->sd_config__reset_stages_msb =
    phistogram->histogram_config__mid_amb_odd_bin_5;

  pstatic->sd_config__reset_stages_lsb =
    phistogram->histogram_config__user_bin_offset;

  ptiming->range_config__sigma_thresh =
    (((uint16_t)phistogram->histogram_config__low_amb_even_bin_0_1)
     << 8)
    + (uint16_t)phistogram->histogram_config__low_amb_even_bin_2_3;

  ptiming->range_config__min_count_rate_rtn_limit_mcps =
    (((uint16_t)phistogram->histogram_config__low_amb_even_bin_4_5)
     << 8)
    + (uint16_t)phistogram->histogram_config__low_amb_odd_bin_0_1;

  ptiming->range_config__valid_phase_low =
    phistogram->histogram_config__low_amb_odd_bin_2_3;
  ptiming->range_config__valid_phase_high =
    phistogram->histogram_config__low_amb_odd_bin_4_5;

  pdynamic->system__thresh_high =
    phistogram->histogram_config__amb_thresh_low;

  pdynamic->system__thresh_low =
    phistogram->histogram_config__amb_thresh_high;

  pdynamic->system__enable_xtalk_per_quadrant =
    phistogram->histogram_config__spad_array_selection;


}

void VL53LX::VL53LX_copy_hist_bins_to_static_cfg(
  VL53LX_histogram_config_t *phistogram,
  VL53LX_static_config_t    *pstatic,
  VL53LX_timing_config_t    *ptiming)
{

  pstatic->sigma_estimator__effective_pulse_width_ns =
    phistogram->histogram_config__high_amb_even_bin_0_1;
  pstatic->sigma_estimator__effective_ambient_width_ns =
    phistogram->histogram_config__high_amb_even_bin_2_3;
  pstatic->sigma_estimator__sigma_ref_mm =
    phistogram->histogram_config__high_amb_even_bin_4_5;

  pstatic->algo__crosstalk_compensation_valid_height_mm =
    phistogram->histogram_config__high_amb_odd_bin_0_1;

  pstatic->spare_host_config__static_config_spare_0 =
    phistogram->histogram_config__high_amb_odd_bin_2_3;
  pstatic->spare_host_config__static_config_spare_1 =
    phistogram->histogram_config__high_amb_odd_bin_4_5;

  pstatic->algo__range_ignore_threshold_mcps =
    (((uint16_t)phistogram->histogram_config__mid_amb_even_bin_0_1)
     << 8)
    + (uint16_t)phistogram->histogram_config__mid_amb_even_bin_2_3;

  pstatic->algo__range_ignore_valid_height_mm =
    phistogram->histogram_config__mid_amb_even_bin_4_5;
  pstatic->algo__range_min_clip =
    phistogram->histogram_config__mid_amb_odd_bin_0_1;
  pstatic->algo__consistency_check__tolerance =
    phistogram->histogram_config__mid_amb_odd_bin_2;

  pstatic->spare_host_config__static_config_spare_2 =
    phistogram->histogram_config__mid_amb_odd_bin_3_4;
  pstatic->sd_config__reset_stages_msb =
    phistogram->histogram_config__mid_amb_odd_bin_5;

  ptiming->range_config__sigma_thresh =
    (((uint16_t)phistogram->histogram_config__low_amb_even_bin_0_1)
     << 8)
    + (uint16_t)phistogram->histogram_config__low_amb_even_bin_2_3;

  ptiming->range_config__min_count_rate_rtn_limit_mcps =
    (((uint16_t)phistogram->histogram_config__low_amb_even_bin_4_5)
     << 8)
    + (uint16_t)phistogram->histogram_config__low_amb_odd_bin_0_1;

  ptiming->range_config__valid_phase_low =
    phistogram->histogram_config__low_amb_odd_bin_2_3;
  ptiming->range_config__valid_phase_high =
    phistogram->histogram_config__low_amb_odd_bin_4_5;


}

VL53LX_Error VL53LX::VL53LX_preset_mode_histogram_ranging_ref(
  VL53LX_hist_post_process_config_t  *phistpostprocess,
  VL53LX_static_config_t             *pstatic,
  VL53LX_histogram_config_t          *phistogram,
  VL53LX_general_config_t            *pgeneral,
  VL53LX_timing_config_t             *ptiming,
  VL53LX_dynamic_config_t            *pdynamic,
  VL53LX_system_control_t            *psystem,
  VL53LX_tuning_parm_storage_t       *ptuning_parms,
  VL53LX_zone_config_t               *pzone_cfg)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_preset_mode_histogram_ranging(
      phistpostprocess,
      pstatic,
      phistogram,
      pgeneral,
      ptiming,
      pdynamic,
      psystem,
      ptuning_parms,
      pzone_cfg);



  if (status == VL53LX_ERROR_NONE) {

    phistogram->histogram_config__spad_array_selection = 0x01;



    VL53LX_copy_hist_cfg_to_static_cfg(
      phistogram,
      pstatic,
      pgeneral,
      ptiming,
      pdynamic);
  }

  return status;
}

/* vl53lx_silicon_core.c */

VL53LX_Error VL53LX::VL53LX_is_firmware_ready_silicon(
  uint8_t       *pready)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t  comms_buffer[5];

  status = VL53LX_ReadMulti(
             Dev,
             VL53LX_INTERRUPT_MANAGER__ENABLES,
             comms_buffer,
             5);

  if (status != VL53LX_ERROR_NONE) {
    goto ENDFUNC;
  }

  pdev->dbg_results.interrupt_manager__enables =
    comms_buffer[0];
  pdev->dbg_results.interrupt_manager__clear =
    comms_buffer[1];
  pdev->dbg_results.interrupt_manager__status =
    comms_buffer[2];
  pdev->dbg_results.mcu_to_host_bank__wr_access_en =
    comms_buffer[3];
  pdev->dbg_results.power_management__go1_reset_status =
    comms_buffer[4];

  if ((pdev->sys_ctrl.power_management__go1_power_force & 0x01)
      == 0x01) {

    if (((pdev->dbg_results.interrupt_manager__enables &
          0x1F) == 0x1F) &&
        ((pdev->dbg_results.interrupt_manager__clear
          & 0x1F) == 0x1F)) {
      *pready = 0x01;
    } else {
      *pready = 0x00;
    }

  } else {


    if ((pdev->dbg_results.power_management__go1_reset_status
         & 0x01) == 0x00) {
      *pready = 0x01;
    } else {
      *pready = 0x00;
    }
  }
ENDFUNC:
  return status;
}

/* vl53lx_hist_core.c */
void VL53LX::VL53LX_f_022(
  uint8_t                         VL53LX_p_032,
  uint8_t                         filter_woi,
  VL53LX_histogram_bin_data_t    *pbins,
  int32_t                        *pa,
  int32_t                        *pb,
  int32_t                        *pc)
{


  uint8_t w = 0;
  uint8_t j = 0;

  *pa = 0;
  *pb = pbins->bin_data[VL53LX_p_032];
  *pc = 0;

  for (w = 0 ; w < ((filter_woi << 1) + 1) ; w++) {


    j = ((VL53LX_p_032 + w + pbins->VL53LX_p_021) -
         filter_woi) % pbins->VL53LX_p_021;

    if (w < filter_woi) {
      *pa += pbins->bin_data[j];
    } else if (w > filter_woi) {
      *pc += pbins->bin_data[j];
    }
  }
}


VL53LX_Error VL53LX::VL53LX_f_018(
  uint16_t           vcsel_width,
  uint16_t           fast_osc_frequency,
  uint32_t           total_periods_elapsed,
  uint16_t           VL53LX_p_004,
  VL53LX_range_data_t  *pdata)
{
  VL53LX_Error     status = VL53LX_ERROR_NONE;

  uint32_t    pll_period_us       = 0;
  uint32_t    periods_elapsed     = 0;
  uint32_t    count_rate_total    = 0;


  pdata->width                  = vcsel_width;
  pdata->fast_osc_frequency     = fast_osc_frequency;
  pdata->total_periods_elapsed  = total_periods_elapsed;
  pdata->VL53LX_p_004 = VL53LX_p_004;




  if (pdata->fast_osc_frequency == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  }

  if (pdata->total_periods_elapsed == 0) {
    status = VL53LX_ERROR_DIVISION_BY_ZERO;
  }

  if (status == VL53LX_ERROR_NONE) {




    pll_period_us =
      VL53LX_calc_pll_period_us(pdata->fast_osc_frequency);

    periods_elapsed      = pdata->total_periods_elapsed + 1;

    pdata->peak_duration_us    = VL53LX_duration_maths(
                                   pll_period_us,
                                   (uint32_t)pdata->width,
                                   VL53LX_RANGING_WINDOW_VCSEL_PERIODS,
                                   periods_elapsed);

    pdata->woi_duration_us     = VL53LX_duration_maths(
                                   pll_period_us,
                                   ((uint32_t)pdata->VL53LX_p_029) << 4,
                                   VL53LX_RANGING_WINDOW_VCSEL_PERIODS,
                                   periods_elapsed);




    pdata->peak_signal_count_rate_mcps = VL53LX_rate_maths(
                                           (int32_t)pdata->VL53LX_p_010,
                                           pdata->peak_duration_us);

    pdata->avg_signal_count_rate_mcps = VL53LX_rate_maths(
                                          (int32_t)pdata->VL53LX_p_010,
                                          pdata->woi_duration_us);

    pdata->ambient_count_rate_mcps    = VL53LX_rate_maths(
                                          (int32_t)pdata->VL53LX_p_016,
                                          pdata->woi_duration_us);




    count_rate_total =
      (uint32_t)pdata->peak_signal_count_rate_mcps +
      (uint32_t)pdata->ambient_count_rate_mcps;

    pdata->total_rate_per_spad_mcps   =
      VL53LX_rate_per_spad_maths(
        0x06,
        count_rate_total,
        pdata->VL53LX_p_004,
        0xFFFF);




    pdata->VL53LX_p_009   =
      VL53LX_events_per_spad_maths(
        pdata->VL53LX_p_010,
        pdata->VL53LX_p_004,
        pdata->peak_duration_us);



  }

  return status;
}


void VL53LX::VL53LX_f_019(
  uint16_t             gain_factor,
  int16_t              range_offset_mm,
  VL53LX_range_data_t *pdata)
{

  pdata->min_range_mm =
    (int16_t)VL53LX_range_maths(
      pdata->fast_osc_frequency,
      pdata->VL53LX_p_026,
      pdata->zero_distance_phase,
      0,

      (int32_t)gain_factor,
      (int32_t)range_offset_mm);

  pdata->median_range_mm =
    (int16_t)VL53LX_range_maths(
      pdata->fast_osc_frequency,
      pdata->VL53LX_p_011,
      pdata->zero_distance_phase,
      0,

      (int32_t)gain_factor,
      (int32_t)range_offset_mm);

  pdata->max_range_mm =
    (int16_t)VL53LX_range_maths(
      pdata->fast_osc_frequency,
      pdata->VL53LX_p_027,
      pdata->zero_distance_phase,
      0,

      (int32_t)gain_factor,
      (int32_t)range_offset_mm);

}

void  VL53LX::VL53LX_f_029(
  VL53LX_histogram_bin_data_t   *pdata,
  int32_t                        ambient_estimate_counts_per_bin)
{
  uint8_t i = 0;

  for (i = 0 ; i <  pdata->VL53LX_p_021 ; i++)
    pdata->bin_data[i] = pdata->bin_data[i] -
                         ambient_estimate_counts_per_bin;
}




void  VL53LX::VL53LX_f_005(
  VL53LX_histogram_bin_data_t   *pxtalk,
  VL53LX_histogram_bin_data_t   *pbins,
  VL53LX_histogram_bin_data_t   *pxtalk_realigned)
{


  uint8_t i          = 0;
  uint8_t min_bins   = 0;
  int8_t  bin_offset = 0;
  int8_t  bin_access = 0;


  memcpy(
    pxtalk_realigned,
    pbins,
    sizeof(VL53LX_histogram_bin_data_t));

  for (i = 0 ; i < pxtalk_realigned->VL53LX_p_020 ; i++) {
    pxtalk_realigned->bin_data[i] = 0;
  }


  bin_offset =  VL53LX_f_030(
                  pbins,
                  pxtalk);


  if (pxtalk->VL53LX_p_021 < pbins->VL53LX_p_021) {
    min_bins = pxtalk->VL53LX_p_021;
  } else {
    min_bins = pbins->VL53LX_p_021;
  }


  for (i = 0 ; i <  min_bins ; i++) {




    if (bin_offset >= 0)
      bin_access = ((int8_t)i + (int8_t)bin_offset)
                   % (int8_t)pbins->VL53LX_p_021;
    else
      bin_access = ((int8_t)pbins->VL53LX_p_021 +
                    ((int8_t)i + (int8_t)bin_offset))
                   % (int8_t)pbins->VL53LX_p_021;

    if (pbins->bin_data[(uint8_t)bin_access] >
        pxtalk->bin_data[i]) {

      pbins->bin_data[(uint8_t)bin_access] =
        pbins->bin_data[(uint8_t)bin_access]
        - pxtalk->bin_data[i];

    } else {
      pbins->bin_data[(uint8_t)bin_access] = 0;
    }


    pxtalk_realigned->bin_data[(uint8_t)bin_access] =
      pxtalk->bin_data[i];


  }


}




int8_t  VL53LX::VL53LX_f_030(
  VL53LX_histogram_bin_data_t   *pdata1,
  VL53LX_histogram_bin_data_t   *pdata2)
{


  int32_t  phase_delta      = 0;
  int8_t   bin_offset       = 0;
  uint32_t period           = 0;
  uint32_t remapped_phase   = 0;

  period = 2048 *
           (uint32_t)VL53LX_decode_vcsel_period(pdata1->VL53LX_p_005);

  remapped_phase = (uint32_t)pdata2->zero_distance_phase % period;


  phase_delta = (int32_t)pdata1->zero_distance_phase
                - (int32_t)remapped_phase;


  if (phase_delta > 0) {
    bin_offset = (int8_t)((phase_delta + 1024) / 2048);
  } else {
    bin_offset = (int8_t)((phase_delta - 1024) / 2048);
  }


  return bin_offset;
}


VL53LX_Error  VL53LX::VL53LX_f_031(
  VL53LX_histogram_bin_data_t   *pidata,
  VL53LX_histogram_bin_data_t   *podata)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t  bin_initial_index[VL53LX_MAX_BIN_SEQUENCE_CODE + 1];
  uint8_t  bin_repeat_count[VL53LX_MAX_BIN_SEQUENCE_CODE + 1];

  uint8_t  bin_cfg        = 0;
  uint8_t  bin_seq_length = 0;
  int32_t  repeat_count   = 0;

  uint8_t  VL53LX_p_032       = 0;
  uint8_t  lc       = 0;
  uint8_t  i       = 0;

  memcpy(podata, pidata, sizeof(VL53LX_histogram_bin_data_t));

  podata->VL53LX_p_021 = 0;

  for (lc = 0 ; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH ; lc++) {
    podata->bin_seq[lc] = VL53LX_MAX_BIN_SEQUENCE_CODE + 1;
  }

  for (lc = 0 ; lc < podata->VL53LX_p_020 ; lc++) {
    podata->bin_data[lc] = 0;
  }




  for (lc = 0 ; lc <= VL53LX_MAX_BIN_SEQUENCE_CODE ; lc++) {
    bin_initial_index[lc] = 0x00;
    bin_repeat_count[lc]  = 0x00;
  }





  bin_seq_length = 0x00;

  for (lc = 0 ; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH ; lc++) {

    bin_cfg = pidata->bin_seq[lc];


    if (bin_repeat_count[bin_cfg] == 0) {
      bin_initial_index[bin_cfg]      = bin_seq_length * 4;
      podata->bin_seq[bin_seq_length] = bin_cfg;
      bin_seq_length++;
    }

    bin_repeat_count[bin_cfg]++;


    VL53LX_p_032 = bin_initial_index[bin_cfg];

    for (i = 0 ; i < 4 ; i++)
      podata->bin_data[VL53LX_p_032 + i] +=
        pidata->bin_data[lc * 4 + i];

  }




  for (lc = 0 ; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH ; lc++) {

    bin_cfg = podata->bin_seq[lc];

    if (bin_cfg <= VL53LX_MAX_BIN_SEQUENCE_CODE)
      podata->bin_rep[lc] =
        bin_repeat_count[bin_cfg];
    else {
      podata->bin_rep[lc] = 0;
    }
  }

  podata->VL53LX_p_021 = bin_seq_length * 4;


  for (lc = 0 ; lc <= VL53LX_MAX_BIN_SEQUENCE_CODE ; lc++) {

    repeat_count = (int32_t)bin_repeat_count[lc];

    if (repeat_count > 0) {

      VL53LX_p_032 = bin_initial_index[lc];

      for (i = 0 ; i < 4 ; i++) {
        podata->bin_data[VL53LX_p_032 + i] +=
          (repeat_count / 2);
        podata->bin_data[VL53LX_p_032 + i] /=
          repeat_count;
      }
    }
  }

  podata->number_of_ambient_bins = 0;
  if ((bin_repeat_count[7] > 0) ||
      (bin_repeat_count[15] > 0)) {
    podata->number_of_ambient_bins = 4;
  }

  return status;
}


/* vl53lx_xtalk.c */


VL53LX_Error VL53LX::VL53LX_xtalk_calibration_process_data(
  VL53LX_xtalk_range_results_t    *pxtalk_results,
  VL53LX_xtalk_histogram_data_t   *pxtalk_shape,
  VL53LX_xtalk_calibration_results_t  *pxtalk_cal)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_xtalk_algo_data_t xtalk_debug;
  VL53LX_xtalk_algo_data_t *pdebug      = &xtalk_debug;
  VL53LX_xtalk_range_data_t *pxtalk_data = NULL;

  VL53LX_histogram_bin_data_t avg_bins;
  VL53LX_histogram_bin_data_t *pavg_bins   = &avg_bins;

  memcpy(pavg_bins, &(pxtalk_results->central_histogram_avg),
         sizeof(VL53LX_histogram_bin_data_t));




  if (status == VL53LX_ERROR_NONE)

    VL53LX_init_histogram_bin_data_struct(
      0, 0, &(pdebug->VL53LX_p_056));

  if (status == VL53LX_ERROR_NONE)

    VL53LX_init_histogram_bin_data_struct(
      0, 0, &(pdebug->VL53LX_p_057));






  if (status == VL53LX_ERROR_NONE)

    status = VL53LX_f_039(
               pxtalk_results,
               pdebug,
               &(pxtalk_cal->algo__crosstalk_compensation_x_plane_gradient_kcps
                ),
               &(pxtalk_cal->algo__crosstalk_compensation_y_plane_gradient_kcps
                ));







  if (status != VL53LX_ERROR_NONE) {
    goto ENDFUNC;
  }

  pxtalk_data = &(pxtalk_results->VL53LX_p_003[4]);

  if (pxtalk_data->no_of_samples >  0) {




    if (status == VL53LX_ERROR_NONE) {

      memcpy(&(pdebug->VL53LX_p_056),
             pavg_bins,
             sizeof(VL53LX_histogram_bin_data_t));
    }




    status = VL53LX_f_040(
               pxtalk_data,
               pdebug,
               &(pxtalk_cal->algo__crosstalk_compensation_plane_offset_kcps));




    if (status == VL53LX_ERROR_NONE)
      status = VL53LX_f_041(
                 pavg_bins,
                 pdebug,
                 pxtalk_data,
                 pxtalk_results->central_histogram__window_start,
                 pxtalk_results->central_histogram__window_end,
                 &(pxtalk_shape->xtalk_shape));

  } else {




    pxtalk_cal->algo__crosstalk_compensation_plane_offset_kcps = 0;




    pdebug->VL53LX_p_058 = 0;



  }
ENDFUNC:
  return status;
}


VL53LX_Error VL53LX::VL53LX_generate_dual_reflectance_xtalk_samples(
  VL53LX_xtalk_range_results_t  *pxtalk_results,
  uint16_t      expected_target_distance_mm,
  uint8_t         higher_reflectance,
  VL53LX_histogram_bin_data_t *pxtalk_avg_samples
)
{

  VL53LX_Error status        = VL53LX_ERROR_NONE;

  VL53LX_histogram_bin_data_t *pzone_avg_1   =
    &(pxtalk_results->histogram_avg_1[0]);
  VL53LX_histogram_bin_data_t *pzone_avg_2   =
    &(pxtalk_results->histogram_avg_2[0]);

  VL53LX_histogram_bin_data_t *pxtalk_output = pxtalk_avg_samples;




  int i = 0;




  for (i = 0 ; i < 5 ; i++) {

    if (status == VL53LX_ERROR_NONE)

      VL53LX_init_histogram_bin_data_struct(
        0, 0, pzone_avg_1);

    if (status == VL53LX_ERROR_NONE)

      VL53LX_init_histogram_bin_data_struct(
        0, 0, pzone_avg_2);

    pzone_avg_1++;
    pzone_avg_2++;
  }





  pzone_avg_1 = &(pxtalk_results->histogram_avg_1[0]);
  pzone_avg_2 = &(pxtalk_results->histogram_avg_2[0]);

  for (i = 0 ; i < 5 ; i++) {

    if (status == VL53LX_ERROR_NONE) {

      status = VL53LX_f_042(
                 pzone_avg_1,
                 pzone_avg_2,
                 expected_target_distance_mm,
                 0x01,

                 higher_reflectance,
                 pxtalk_output
               );




      pzone_avg_1++;
      pzone_avg_2++;
      pxtalk_output++;

    }
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_042(
  VL53LX_histogram_bin_data_t *pzone_avg_1,
  VL53LX_histogram_bin_data_t *pzone_avg_2,
  uint16_t      expected_target_distance,
  uint8_t       subtract_amb,
  uint8_t       higher_reflectance,
  VL53LX_histogram_bin_data_t *pxtalk_output
)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_histogram_bin_data_t  zone_avg_realigned;



  SUPPRESS_UNUSED_WARNING(pxtalk_output);
  SUPPRESS_UNUSED_WARNING(expected_target_distance);




  if ((status == VL53LX_ERROR_NONE) && (subtract_amb == 0x01)) {
    VL53LX_f_029(
      pzone_avg_1,
      pzone_avg_1->VL53LX_p_028);



    pzone_avg_1->VL53LX_p_028 = 0x0;
  }

  if ((status == VL53LX_ERROR_NONE) && (subtract_amb == 0x01)) {
    VL53LX_f_029(
      pzone_avg_2,
      pzone_avg_2->VL53LX_p_028);



    pzone_avg_2->VL53LX_p_028 = 0x0;
  }







  if (status == VL53LX_ERROR_NONE) {
    if (higher_reflectance == 0x01) {
      VL53LX_f_005(
        pzone_avg_2,
        pzone_avg_1,
        &zone_avg_realigned);
    } else {




      VL53LX_f_005(
        pzone_avg_1,
        pzone_avg_2,
        &zone_avg_realigned);





    }
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_041(
  VL53LX_histogram_bin_data_t        *pavg_bins,
  VL53LX_xtalk_algo_data_t           *pdebug,
  VL53LX_xtalk_range_data_t          *pxtalk_data,
  uint8_t                             histogram__window_start,
  uint8_t                             histogram__window_end,
  VL53LX_xtalk_histogram_shape_t     *pxtalk_shape)
{

  VL53LX_Error status        = VL53LX_ERROR_NONE;




  uint32_t ambient_thresh         = 0;




  if (status == VL53LX_ERROR_NONE)

    VL53LX_f_029(
      pavg_bins,
      pavg_bins->VL53LX_p_028);




  if (status == VL53LX_ERROR_NONE)

    VL53LX_f_043(
      6,
      pavg_bins->VL53LX_p_028,
      &ambient_thresh);








  if (status == VL53LX_ERROR_NONE)

    status = VL53LX_f_044(
               pavg_bins,
               ambient_thresh,
               histogram__window_start,
               histogram__window_end);




  if (status == VL53LX_ERROR_NONE)
    status =  VL53LX_f_045(
                pavg_bins,
                pxtalk_data,
                pdebug,
                pxtalk_shape);


  return status;

}


VL53LX_Error VL53LX::VL53LX_f_039(
  VL53LX_xtalk_range_results_t   *pxtalk_results,
  VL53LX_xtalk_algo_data_t       *pdebug,
  int16_t                        *xgradient,
  int16_t                        *ygradient
)
{


  VL53LX_Error status        = VL53LX_ERROR_NONE;

  VL53LX_xtalk_range_data_t  *presults_int = NULL;

  int          i                   = 0;

  uint32_t xtalk_per_spad[4];
  int32_t  VL53LX_p_059         = 0;
  int32_t  VL53LX_p_060         = 0;

  uint8_t  result_invalid          = 0;



  *xgradient = 0;
  *ygradient = 0;





  for (i = 0; i < 4; i++) {
    xtalk_per_spad[i] = 0;
  }




  for (i = 0; i < 4; i++) {

    if (status == VL53LX_ERROR_NONE) {

      presults_int = &(pxtalk_results->VL53LX_p_003[i]);






      if (presults_int->no_of_samples == 0) {



        result_invalid = 1;
        pdebug->VL53LX_p_061[i] = 0;


      } else {

        xtalk_per_spad[i] =
          presults_int->rate_per_spad_kcps_avg;




        pdebug->VL53LX_p_061[i] =
          (uint32_t)xtalk_per_spad[i];

      }
    }

  }








  if ((status == VL53LX_ERROR_NONE) && (result_invalid == 0)) {




    if (status == VL53LX_ERROR_NONE) {

      VL53LX_p_059 = ((int32_t)xtalk_per_spad[1]
                      - (int32_t)xtalk_per_spad[0]) / (8);
      VL53LX_p_060 = ((int32_t)xtalk_per_spad[3]
                      - (int32_t)xtalk_per_spad[2]) / (8);
    }






    if (status == VL53LX_ERROR_NONE) {

      if (VL53LX_p_059 < -32767) {
        VL53LX_p_059 = -32767;
      } else {
        if (VL53LX_p_059 > 32767) {
          VL53LX_p_059 = 32767;
        }
      }

      if (VL53LX_p_060 < -32767) {
        VL53LX_p_060 = -32767;
      } else {
        if (VL53LX_p_060 > 32767) {
          VL53LX_p_060 = 32767;
        }
      }




      pdebug->VL53LX_p_059 = (int16_t)VL53LX_p_059;
      pdebug->VL53LX_p_060 = (int16_t)VL53LX_p_060;
    }

  } else {




    VL53LX_p_059 = 0;
    VL53LX_p_060 = 0;

    pdebug->VL53LX_p_059 = 0;
    pdebug->VL53LX_p_060 = 0;
  }




  if (status == VL53LX_ERROR_NONE) {
    *xgradient = (int16_t)VL53LX_p_059;
    *ygradient = (int16_t)VL53LX_p_060;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_040(
  VL53LX_xtalk_range_data_t *pxtalk_data,
  VL53LX_xtalk_algo_data_t  *pdebug,
  uint32_t                 *xtalk_mean_offset_kcps
)
{

  VL53LX_Error status        = VL53LX_ERROR_NONE;

  uint32_t xtalk_per_spad          = 0;
  uint8_t  result_invalid          = 0;


  *xtalk_mean_offset_kcps          = 0;



  if (pxtalk_data->no_of_samples == 0) {




    result_invalid = 1;




    pdebug->VL53LX_p_058 = 0;

  }





  if ((status == VL53LX_ERROR_NONE) && (result_invalid == 0)) {





    xtalk_per_spad = pxtalk_data->rate_per_spad_kcps_avg >> 2;




    pdebug->VL53LX_p_058 = xtalk_per_spad;



    if (xtalk_per_spad < 0x3FFFF) {
      *xtalk_mean_offset_kcps     = (uint32_t)xtalk_per_spad;
    } else {
      *xtalk_mean_offset_kcps     = 0x3FFFF;
    }

  } else {




    *xtalk_mean_offset_kcps     = 0;
  }


  return status;

}



VL53LX_Error VL53LX::VL53LX_f_045(
  VL53LX_histogram_bin_data_t *phist_data,
  VL53LX_xtalk_range_data_t      *pxtalk_data,
  VL53LX_xtalk_algo_data_t       *pdebug,
  VL53LX_xtalk_histogram_shape_t *pxtalk_histo
)
{

  VL53LX_Error status            = VL53LX_ERROR_NONE;

  int          i                 = 0;
  uint64_t     bin_data[VL53LX_XTALK_HISTO_BINS]{};

  pxtalk_histo->VL53LX_p_020             =
    phist_data->VL53LX_p_020;
  pxtalk_histo->cal_config__vcsel_start =
    phist_data->cal_config__vcsel_start;
  pxtalk_histo->VL53LX_p_015     =
    phist_data->VL53LX_p_015;
  pxtalk_histo->VL53LX_p_019               =
    phist_data->VL53LX_p_019;
  pxtalk_histo->time_stamp              =
    phist_data->time_stamp;
  pxtalk_histo->vcsel_width             =
    phist_data->vcsel_width;
  pxtalk_histo->zero_distance_phase     =
    phist_data->zero_distance_phase;
  pxtalk_histo->zone_id                 =
    phist_data->zone_id;
  pxtalk_histo->VL53LX_p_021          =
    VL53LX_XTALK_HISTO_BINS;
  pxtalk_histo->phasecal_result__reference_phase =
    phist_data->phasecal_result__reference_phase;
  pxtalk_histo->phasecal_result__vcsel_start     =
    phist_data->phasecal_result__vcsel_start;





  memcpy(&(pdebug->VL53LX_p_057),
         phist_data, sizeof(VL53LX_histogram_bin_data_t));










  if (pxtalk_data->signal_total_events_avg != 0) {
    for (i = 0; i < pxtalk_histo->VL53LX_p_021; i++) {
      if (phist_data->bin_data[i +
                                 phist_data->number_of_ambient_bins] > 0) {
        bin_data[i] =
          (((uint64_t)phist_data->bin_data[i +
                                             phist_data->number_of_ambient_bins] << 10)
           + ((uint64_t)pxtalk_data->signal_total_events_avg
              / 2))
          / (uint64_t)pxtalk_data->signal_total_events_avg;
      }
    }
  }





  for (i = 0; i < VL53LX_XTALK_HISTO_BINS; i++) {
    pxtalk_histo->bin_data[i] = (uint32_t)bin_data[i];
  }




  for (i = 0; i < pxtalk_histo->VL53LX_p_021; i++) {
    pdebug->VL53LX_p_062[i] = pxtalk_histo->bin_data[i];
  }



  return status;

}



VL53LX_Error VL53LX::VL53LX_f_046(
  VL53LX_customer_nvm_managed_t *pcustomer,
  VL53LX_dynamic_config_t       *pdyn_cfg,
  VL53LX_xtalk_histogram_data_t *pxtalk_shape,
  VL53LX_histogram_bin_data_t   *pip_hist_data,
  VL53LX_histogram_bin_data_t   *pop_hist_data,
  VL53LX_histogram_bin_data_t   *pxtalk_count_data)
{







  VL53LX_Error status = VL53LX_ERROR_NONE;




  uint32_t xtalk_rate_kcps = 0;



  memcpy(pop_hist_data, pip_hist_data,
         sizeof(VL53LX_histogram_bin_data_t));




  status =
    VL53LX_f_032(
      pcustomer->algo__crosstalk_compensation_plane_offset_kcps,
      pcustomer->algo__crosstalk_compensation_x_plane_gradient_kcps,
      pcustomer->algo__crosstalk_compensation_y_plane_gradient_kcps,
      0,

      0,

      pip_hist_data->result__dss_actual_effective_spads,


      pdyn_cfg->roi_config__user_roi_centre_spad,
      pdyn_cfg->roi_config__user_roi_requested_global_xy_size,
      &(xtalk_rate_kcps));




  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_033(
        pip_hist_data,
        &(pxtalk_shape->xtalk_shape),
        xtalk_rate_kcps,
        pxtalk_count_data);




  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_047(
        pop_hist_data,
        pxtalk_count_data,
        pip_hist_data->number_of_ambient_bins);

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_032(
  uint32_t                       mean_offset,
  int16_t                        xgradient,
  int16_t                        ygradient,
  int8_t                         centre_offset_x,
  int8_t                         centre_offset_y,
  uint16_t                       roi_effective_spads,
  uint8_t                        roi_centre_spad,
  uint8_t                        roi_xy_size,
  uint32_t                      *xtalk_rate_kcps
)
{







  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t row = 0;
  uint8_t col = 0;




  int16_t  bound_l_x = 0;

  int16_t  bound_r_x = 0;

  int16_t  bound_u_y = 0;

  int16_t  bound_d_y = 0;


  int64_t xtalk_rate_ll = 0;

  int64_t xtalk_rate_ur = 0;


  int64_t xtalk_avg = 0;

  SUPPRESS_UNUSED_WARNING(roi_effective_spads);



  if (status == VL53LX_ERROR_NONE) {

    VL53LX_decode_row_col(
      roi_centre_spad,
      &row,
      &col);
  }


  if (status == VL53LX_ERROR_NONE) {

    if ((((int16_t)roi_xy_size / 16) & 0x01) == 1)
      bound_l_x = (int16_t) col -
                  (((int16_t)roi_xy_size / 32) + 1);
    else
      bound_l_x = (int16_t) col -
                  ((int16_t)roi_xy_size / 32);

    bound_r_x = (int16_t) col + ((int16_t)roi_xy_size / 32);

    if ((((int16_t)roi_xy_size) & 0x01) == 1)
      bound_d_y = (int16_t) row -
                  ((((int16_t)roi_xy_size & 0x0f) / 2) + 1);
    else
      bound_d_y = (int16_t) row -
                  (((int16_t)roi_xy_size & 0x0f) / 2);

    bound_u_y = (int16_t) row +
                (((int16_t)roi_xy_size & 0xf) / 2);
  }


  if (status == VL53LX_ERROR_NONE) {

    bound_l_x = (2 * bound_l_x) - 15 +
                (2 * (int16_t)centre_offset_x);
    bound_r_x = (2 * bound_r_x) - 15 +
                (2 * (int16_t)centre_offset_x);
    bound_u_y = (2 * bound_u_y) - 15 +
                (2 * (int16_t)centre_offset_y);
    bound_d_y = (2 * bound_d_y) - 15 +
                (2 * (int16_t)centre_offset_y);
  }




  if (status == VL53LX_ERROR_NONE) {

    xtalk_rate_ll  = ((int64_t)bound_l_x *
                      ((int64_t)xgradient)) + ((int64_t)bound_d_y *
                                               ((int64_t)ygradient));
    xtalk_rate_ll  = (xtalk_rate_ll + 1) / 2;

    xtalk_rate_ll += ((int64_t)mean_offset * 4);

    xtalk_rate_ur  = ((int64_t)bound_r_x *
                      ((int64_t)xgradient)) + ((int64_t)bound_u_y *
                                               ((int64_t)ygradient));
    xtalk_rate_ur  = (xtalk_rate_ur + 1) / 2;

    xtalk_rate_ur += ((int64_t)mean_offset * 4);
  }


  if (status == VL53LX_ERROR_NONE)

  {
    xtalk_avg = ((xtalk_rate_ll + xtalk_rate_ur) + 1) / 2;
  }



  if (status == VL53LX_ERROR_NONE)

    if (xtalk_avg < 0) {
      xtalk_avg = 0;
    }

  *xtalk_rate_kcps = (uint32_t) xtalk_avg;

  return status;
}



VL53LX_Error VL53LX::VL53LX_f_033(
  VL53LX_histogram_bin_data_t    *phist_data,
  VL53LX_xtalk_histogram_shape_t *pxtalk_data,
  uint32_t                        xtalk_rate_kcps,
  VL53LX_histogram_bin_data_t    *pxtalkcount_data
)
{

  VL53LX_Error status              = VL53LX_ERROR_NONE;

  uint64_t xtalk_events_per_spad = 0;
  uint64_t xtalk_total_events = 0;
  uint64_t xtalk_temp_bin = 0;

  uint8_t  i = 0;

  xtalk_events_per_spad = ((((uint64_t)xtalk_rate_kcps *
                             (uint64_t)phist_data->peak_duration_us) + 500) / 1000);

  xtalk_total_events = xtalk_events_per_spad *
                       (uint64_t)phist_data->result__dss_actual_effective_spads;


  xtalk_total_events = (xtalk_total_events)  / 256;


  xtalk_total_events = (xtalk_total_events + 1024) / 2048;


  if (xtalk_total_events > 0xFFFFFFFF) {
    xtalk_total_events = 0xFFFFFFFF;
  }

  for (i = 0; i < pxtalk_data->VL53LX_p_021; i++) {
    xtalk_temp_bin = (uint64_t)pxtalk_data->bin_data[i] *
                     (uint64_t)xtalk_total_events;
    xtalk_temp_bin = (xtalk_temp_bin + 512) / 1024;

    pxtalkcount_data->bin_data[i] = (uint32_t)xtalk_temp_bin;

  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_f_047(
  VL53LX_histogram_bin_data_t *phist_data,
  VL53LX_histogram_bin_data_t *pxtalk_data,
  uint8_t         xtalk_bin_offset)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t  i = 0;

  int32_t  temp_bin;

  if (status == VL53LX_ERROR_NONE)

    for (i = xtalk_bin_offset;
         i < pxtalk_data->VL53LX_p_021; i++) {


      temp_bin = (int32_t)phist_data->bin_data[i] -
                 (int32_t)pxtalk_data->bin_data[i - xtalk_bin_offset];

      if (temp_bin < 0) {
        temp_bin = 0;
      }

      phist_data->bin_data[i] = (uint32_t)temp_bin;
    }


  return status;
}


VL53LX_Error VL53LX::VL53LX_f_044(
  VL53LX_histogram_bin_data_t   *pxtalk_data,
  uint32_t            amb_threshold,
  uint8_t           VL53LX_p_019,
  uint8_t           VL53LX_p_024)
{

  VL53LX_Error status = VL53LX_ERROR_NONE;

  uint8_t i = 0;
  uint8_t first_bin_int = 0;
  uint8_t first_bin_inc = 0;
  uint8_t last_bin_int  = 0;
  uint8_t realign_bin   = 0;
  uint8_t realign_index = 0;
  int32_t realign_bin_data[VL53LX_HISTOGRAM_BUFFER_SIZE];


  for (i = 0 ; i < VL53LX_HISTOGRAM_BUFFER_SIZE ; i++) {
    realign_bin_data[i] = 0;
  }

  first_bin_int = VL53LX_p_019;
  last_bin_int  = VL53LX_p_024;

  VL53LX_hist_remove_ambient_bins(pxtalk_data);

  first_bin_int = (first_bin_int) %
                  pxtalk_data->VL53LX_p_021;

  last_bin_int = (last_bin_int) %
                 pxtalk_data->VL53LX_p_021;

  first_bin_inc = (first_bin_int + 1) % pxtalk_data->VL53LX_p_021;

  if (first_bin_inc > last_bin_int) {



    realign_bin = pxtalk_data->VL53LX_p_021 - first_bin_inc;



    first_bin_int = (first_bin_int + realign_bin) %
                    pxtalk_data->VL53LX_p_021;
    last_bin_int = (last_bin_int + realign_bin) %
                   pxtalk_data->VL53LX_p_021;



    pxtalk_data->zero_distance_phase =
      pxtalk_data->zero_distance_phase +
      ((uint16_t)realign_bin * 2048);
  }

  if (realign_bin > 0) {


    for (i = 0; i < pxtalk_data->VL53LX_p_021; i++) {
      realign_bin_data[i] = pxtalk_data->bin_data[i];
    }



    for (i = 0; i < pxtalk_data->VL53LX_p_021; i++) {
      realign_index = (pxtalk_data->VL53LX_p_021 -
                       realign_bin + i)
                      % pxtalk_data->VL53LX_p_021;

      pxtalk_data->bin_data[i] =
        realign_bin_data[realign_index];
    }
  }


  for (i = 0; i < pxtalk_data->VL53LX_p_021; i++) {


    if (first_bin_int <= last_bin_int) {
      if ((i >= first_bin_int) && (i <= last_bin_int)) {
        if (pxtalk_data->bin_data[i] <
            (int32_t)amb_threshold) {
          pxtalk_data->bin_data[i] = 0;
        }
      } else {
        pxtalk_data->bin_data[i] = 0;
      }
    } else {
      if ((i >= first_bin_int) || (i <= last_bin_int)) {
        if (pxtalk_data->bin_data[i] <
            (int32_t)amb_threshold) {
          pxtalk_data->bin_data[i] = 0;
        }
      } else {
        pxtalk_data->bin_data[i] = 0;
      }
    }
  }


  return status;
}

VL53LX_Error VL53LX::VL53LX_f_043(
  uint8_t                      sigma_mult,
  int32_t                      VL53LX_p_028,
  uint32_t                    *ambient_noise)
{

  VL53LX_Error status              = VL53LX_ERROR_NONE;

  uint32_t ambient_events_per_bin_int = 0;

  if (VL53LX_p_028 <= 0) {
    ambient_events_per_bin_int = 1;
  } else {
    ambient_events_per_bin_int = (uint32_t)VL53LX_p_028;
  }

  *ambient_noise =  VL53LX_isqrt(ambient_events_per_bin_int);

  *ambient_noise = *ambient_noise * (uint32_t)sigma_mult;

  return status;
}

/* vl53lx_sigma_estimate.c */


uint16_t  VL53LX::VL53LX_f_034(
  uint8_t  sigma_estimator__effective_pulse_width_ns,
  uint8_t  sigma_estimator__effective_ambient_width_ns,
  uint8_t  sigma_estimator__sigma_ref_mm,
  VL53LX_range_data_t *pdata)
{
  uint16_t    sigma_est  = VL53LX_D_002;

  uint32_t    tmp0 = 0;
  uint32_t    tmp1 = 0;
  uint32_t    tmp2 = 0;

  uint32_t    sigma_est__rtn_array  = 0;
  uint32_t    sigma_est__ref_array  = 0;

  if (pdata->peak_signal_count_rate_mcps  > 0 &&
      pdata->VL53LX_p_010 > 0) {

    tmp0 =  100 *
            (uint32_t)sigma_estimator__effective_pulse_width_ns;

    tmp1 = ((uint32_t)sigma_estimator__effective_pulse_width_ns *
            100 *
            (uint32_t)sigma_estimator__effective_ambient_width_ns);

    tmp1 = (tmp1 +
            (uint32_t)pdata->peak_signal_count_rate_mcps / 2) /
           (uint32_t)pdata->peak_signal_count_rate_mcps;

    sigma_est__rtn_array =
      VL53LX_f_035(tmp0, tmp1);

    sigma_est__rtn_array =
      ((VL53LX_SPEED_OF_LIGHT_IN_AIR + 1000) / 2000) *
      sigma_est__rtn_array;



    tmp2 =
      VL53LX_isqrt(12 * (uint32_t)pdata->VL53LX_p_010);

    if (tmp2 > 0) {

      sigma_est__rtn_array =
        (sigma_est__rtn_array + tmp2 / 2) / tmp2;



      sigma_est__ref_array =
        100 * (uint32_t)sigma_estimator__sigma_ref_mm;

      sigma_est =
        (uint16_t)VL53LX_f_035(
          (uint32_t)sigma_est__ref_array,
          sigma_est__rtn_array);

    } else {
      sigma_est = VL53LX_D_002;
    }

  }

  pdata->VL53LX_p_002  = sigma_est;

  return sigma_est;

}


uint16_t VL53LX::VL53LX_f_036(
  uint8_t  sigma_estimator__effective_pulse_width_ns,
  uint8_t  sigma_estimator__effective_ambient_width_ns,
  uint8_t  sigma_estimator__sigma_ref_mm,
  VL53LX_range_data_t *pdata)
{


  uint16_t    sigma_est  = VL53LX_D_002;

  uint32_t    eqn7 = 0;
  uint32_t    sigma_est__ref_sq  = 0;
  uint32_t    sigma_est__rtn_sq  = 0;

  uint64_t    tmp0 = 0;
  uint64_t    tmp1 = 0;


  if (pdata->peak_signal_count_rate_mcps > 0 &&
      pdata->VL53LX_p_010         > 0) {


    eqn7 =  4573 * 4573;
    eqn7 =  eqn7 / (3 * (uint32_t)pdata->VL53LX_p_010);


    tmp0 = ((uint64_t)sigma_estimator__effective_pulse_width_ns)
           << 8;



    tmp1 = ((uint64_t)pdata->ambient_count_rate_mcps *
            (uint64_t)sigma_estimator__effective_ambient_width_ns)
           << 8;

    tmp1 = tmp1 / (uint64_t)pdata->peak_signal_count_rate_mcps;


    tmp1 = 16 * (uint64_t)eqn7 * (tmp0 * tmp0 + tmp1 * tmp1);
    tmp1 = tmp1 / (15625 * 15625);
    sigma_est__rtn_sq = (uint32_t)tmp1;




    sigma_est__ref_sq = ((uint32_t)sigma_estimator__sigma_ref_mm)
                        << 2;

    sigma_est__ref_sq = sigma_est__ref_sq * sigma_est__ref_sq;




    sigma_est = (uint16_t)VL53LX_isqrt(sigma_est__ref_sq +
                                       sigma_est__rtn_sq);

  }

  pdata->VL53LX_p_002  = sigma_est;

  return sigma_est;

}



VL53LX_Error VL53LX::VL53LX_f_037(
  uint8_t  sigma_estimator__sigma_ref_mm,
  uint32_t VL53LX_p_007,
  uint32_t VL53LX_p_032,
  uint32_t VL53LX_p_001,
  uint32_t a_zp,
  uint32_t c_zp,
  uint32_t bx,
  uint32_t ax_zp,
  uint32_t cx_zp,
  uint32_t VL53LX_p_028,
  uint16_t fast_osc_frequency,
  uint16_t *psigma_est)
{



  VL53LX_Error status = VL53LX_ERROR_DIVISION_BY_ZERO;
  uint32_t sigma_int  = VL53LX_D_002;

  uint32_t pll_period_mm  = 0;

  uint64_t tmp0        = 0;
  uint64_t tmp1        = 0;
  uint64_t b_minus_amb = 0;
  uint64_t VL53LX_p_055   = 0;

  *psigma_est  = VL53LX_D_002;

  if (fast_osc_frequency != 0) {





    pll_period_mm = VL53LX_calc_pll_period_mm(fast_osc_frequency);



    pll_period_mm = (pll_period_mm + 0x02) >> 2;




    if (VL53LX_p_028 > VL53LX_p_032)
      b_minus_amb = (uint64_t)VL53LX_p_028 -
                    (uint64_t)VL53LX_p_032;
    else
      b_minus_amb = (uint64_t)VL53LX_p_032 -
                    (uint64_t)VL53LX_p_028;




    if (VL53LX_p_007 > VL53LX_p_001)
      VL53LX_p_055 = (uint64_t)VL53LX_p_007 -
                     (uint64_t)VL53LX_p_001;
    else
      VL53LX_p_055 = (uint64_t)VL53LX_p_001 -
                     (uint64_t)VL53LX_p_007;


    if (b_minus_amb != 0) {



      tmp0 = (uint64_t)pll_period_mm *
             (uint64_t)pll_period_mm;
      tmp0 = tmp0 * ((uint64_t)c_zp +
                     (uint64_t)cx_zp + (uint64_t)a_zp +
                     (uint64_t)ax_zp);
      tmp0 = (tmp0 + (b_minus_amb >> 1)) / b_minus_amb;

      tmp1 = (uint64_t)pll_period_mm *
             (uint64_t)pll_period_mm * VL53LX_p_055;
      tmp1 = (tmp1 + (b_minus_amb >> 1)) / b_minus_amb;

      tmp1 =  tmp1 * VL53LX_p_055;
      tmp1 = (tmp1 + (b_minus_amb >> 1)) / b_minus_amb;

      tmp1 =  tmp1 * ((uint64_t)VL53LX_p_032 + (uint64_t)bx +
                      (uint64_t)VL53LX_p_028);
      tmp1 = (tmp1 + (b_minus_amb >> 1)) / b_minus_amb;



      tmp0 = tmp0 + tmp1;
      tmp0 = (tmp0 + (b_minus_amb >> 1)) / b_minus_amb;
      tmp0 = (tmp0 + 0x01) >> 2;






      tmp1 = (uint64_t)sigma_estimator__sigma_ref_mm << 2;
      tmp1 = tmp1 * tmp1;
      tmp0 = tmp0 + tmp1;






      if (tmp0 > 0xFFFFFFFF) {
        tmp0 =  0xFFFFFFFF;
      }

      sigma_int = VL53LX_isqrt((uint32_t)tmp0);






      if (sigma_int > VL53LX_D_002)
        *psigma_est =
          (uint16_t)VL53LX_D_002;
      else {
        *psigma_est = (uint16_t)sigma_int;
      }

      status = VL53LX_ERROR_NONE;
    }

  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_023(
  uint8_t  sigma_estimator__sigma_ref_mm,
  uint32_t VL53LX_p_007,
  uint32_t VL53LX_p_032,
  uint32_t VL53LX_p_001,
  uint32_t a_zp,
  uint32_t c_zp,
  uint32_t bx,
  uint32_t ax_zp,
  uint32_t cx_zp,
  uint32_t VL53LX_p_028,
  uint16_t fast_osc_frequency,
  uint16_t *psigma_est)
{

  VL53LX_Error status = VL53LX_ERROR_DIVISION_BY_ZERO;
  uint32_t sigma_int  = VL53LX_D_002;

  uint32_t pll_period_mm  = 0;

  uint64_t tmp0        = 0;
  uint64_t tmp1        = 0;
  uint64_t b_minus_amb = 0;
  uint64_t VL53LX_p_055   = 0;

  *psigma_est  = VL53LX_D_002;



  if (fast_osc_frequency != 0) {


    pll_period_mm = VL53LX_calc_pll_period_mm(fast_osc_frequency);




    if (VL53LX_p_028 > VL53LX_p_032)
      b_minus_amb = (uint64_t)VL53LX_p_028 -
                    (uint64_t)VL53LX_p_032;
    else
      b_minus_amb = (uint64_t)VL53LX_p_032 -
                    (uint64_t)VL53LX_p_028;




    if (VL53LX_p_007 > VL53LX_p_001)
      VL53LX_p_055 = (uint64_t)VL53LX_p_007 -
                     (uint64_t)VL53LX_p_001;
    else
      VL53LX_p_055 = (uint64_t)VL53LX_p_001 -
                     (uint64_t)VL53LX_p_007;




    if (b_minus_amb != 0) {


      tmp0 = (uint64_t)VL53LX_p_032 + (uint64_t)bx +
             (uint64_t)VL53LX_p_028;
      if (tmp0 > VL53LX_D_003) {
        tmp0 = VL53LX_D_003;
      }




      tmp1 = (uint64_t)VL53LX_p_055 * (uint64_t)VL53LX_p_055;
      tmp1 = tmp1 << 8;



      if (tmp1 > VL53LX_D_004) {
        tmp1 = VL53LX_D_004;
      }



      tmp1 = tmp1 / b_minus_amb;
      tmp1 = tmp1 / b_minus_amb;



      if (tmp1 > (uint64_t)VL53LX_D_005) {
        tmp1 = (uint64_t)VL53LX_D_005;
      }



      tmp0 = tmp1 * tmp0;





      tmp1 = (uint64_t)c_zp + (uint64_t)cx_zp +
             (uint64_t)a_zp + (uint64_t)ax_zp;



      if (tmp1 > (uint64_t)VL53LX_D_003) {
        tmp1 = (uint64_t)VL53LX_D_003;
      }

      tmp1 = tmp1 << 8;




      tmp0 = tmp1 + tmp0;
      if (tmp0 > (uint64_t)VL53LX_D_006) {
        tmp0 = (uint64_t)VL53LX_D_006;
      }

      if (tmp0 > (uint64_t)VL53LX_D_007) {
        tmp0 = tmp0 / b_minus_amb;
        tmp0 = tmp0 * pll_period_mm;
      } else {
        tmp0 = tmp0 * pll_period_mm;
        tmp0 = tmp0 / b_minus_amb;
      }



      if (tmp0 > (uint64_t)VL53LX_D_006) {
        tmp0 = (uint64_t)VL53LX_D_006;
      }




      if (tmp0 > (uint64_t)VL53LX_D_007) {
        tmp0 = tmp0 / b_minus_amb;
        tmp0 = tmp0 / 4;
        tmp0 = tmp0 * pll_period_mm;
      } else {
        tmp0 = tmp0 * pll_period_mm;
        tmp0 = tmp0 / b_minus_amb;
        tmp0 = tmp0 / 4;
      }



      if (tmp0 > (uint64_t)VL53LX_D_006) {
        tmp0 = (uint64_t)VL53LX_D_006;
      }



      tmp0 = tmp0 >> 2;



      if (tmp0 > (uint64_t)VL53LX_D_007) {
        tmp0 = (uint64_t)VL53LX_D_007;
      }



      tmp1 = (uint64_t)sigma_estimator__sigma_ref_mm << 7;
      tmp1 = tmp1 * tmp1;
      tmp0 = tmp0 + tmp1;



      if (tmp0 > (uint64_t)VL53LX_D_007) {
        tmp0 = (uint64_t)VL53LX_D_007;
      }



      sigma_int = VL53LX_isqrt((uint32_t)tmp0);

      *psigma_est = (uint16_t)sigma_int;

      status = VL53LX_ERROR_NONE;
    }

  }

  return status;
}


uint32_t VL53LX::VL53LX_f_038(
  uint64_t VL53LX_p_007,
  uint32_t size
)
{

  uint64_t next;
  uint64_t upper;
  uint64_t lower;
  uint32_t stepsize;
  uint32_t count;


  next = VL53LX_p_007;
  upper = 0;
  lower = 0;
  stepsize = size / 2;
  count = 0;

  while (1) {
    upper = next >> stepsize;
    lower = next & ((1 << stepsize) - 1);

    if (upper != 0) {
      count += stepsize;
      next = upper;
    } else {
      next = lower;
    }

    stepsize = stepsize / 2;
    if (stepsize == 0) {
      break;
    }
  }

  return count;
}


uint32_t VL53LX::VL53LX_f_035(
  uint32_t VL53LX_p_007,
  uint32_t VL53LX_p_032)
{

  uint32_t  res = 0;

  if (VL53LX_p_007 > 65535 || VL53LX_p_032 > 65535) {
    res = 65535;
  } else
    res = VL53LX_isqrt(VL53LX_p_007 * VL53LX_p_007 +
                       VL53LX_p_032 * VL53LX_p_032);

  return res;
}

/* vl53lx_hist_algos_gen3.c */

void VL53LX::VL53LX_f_003(
  VL53LX_hist_gen3_algo_private_data_t   *palgo)
{
  uint8_t  lb                 = 0;

  palgo->VL53LX_p_020              = VL53LX_HISTOGRAM_BUFFER_SIZE;
  palgo->VL53LX_p_019                = 0;
  palgo->VL53LX_p_021           = 0;
  palgo->VL53LX_p_039         = 0;
  palgo->VL53LX_p_028   = 0;
  palgo->VL53LX_p_031 = 0;

  for (lb = palgo->VL53LX_p_019; lb < palgo->VL53LX_p_020; lb++) {
    palgo->VL53LX_p_040[lb]      = 0;
    palgo->VL53LX_p_041[lb] = 0;
    palgo->VL53LX_p_042[lb]     = 0;
    palgo->VL53LX_p_043[lb]      = 0;
    palgo->VL53LX_p_018[lb]     = 0;
  }

  palgo->VL53LX_p_044 = 0;
  palgo->VL53LX_p_045               = VL53LX_D_001;
  palgo->VL53LX_p_046             = 0;




  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(palgo->VL53LX_p_006));
  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(palgo->VL53LX_p_047));
  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(palgo->VL53LX_p_048));
  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(palgo->VL53LX_p_049));
  VL53LX_init_histogram_bin_data_struct(
    0,
    VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(palgo->VL53LX_p_050));
}


VL53LX_Error VL53LX::VL53LX_f_004(
  VL53LX_dmax_calibration_data_t         *pdmax_cal,
  VL53LX_hist_gen3_dmax_config_t         *pdmax_cfg,
  VL53LX_hist_post_process_config_t      *ppost_cfg,
  VL53LX_histogram_bin_data_t            *pbins_input,
  VL53LX_histogram_bin_data_t            *pxtalk,
  VL53LX_hist_gen3_algo_private_data_t   *palgo,
  VL53LX_hist_gen3_dmax_private_data_t   *pdmax_algo,
  VL53LX_range_results_t                 *presults)
{





  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t     *ppulse_data;
  VL53LX_range_data_t          *prange_data;

  uint8_t                       p = 0;

  VL53LX_f_003(palgo);






  memcpy(
    &(palgo->VL53LX_p_006),
    pbins_input,
    sizeof(VL53LX_histogram_bin_data_t));






  presults->cfg_device_state = pbins_input->cfg_device_state;
  presults->rd_device_state  = pbins_input->rd_device_state;
  presults->zone_id          = pbins_input->zone_id;
  presults->stream_count     = pbins_input->result__stream_count;
  presults->wrap_dmax_mm     = 0;
  presults->max_results      = VL53LX_MAX_RANGE_RESULTS;
  presults->active_results   = 0;

  for (p = 0; p < VL53LX_MAX_AMBIENT_DMAX_VALUES; p++) {
    presults->VL53LX_p_022[p] = 0;
  }




  VL53LX_hist_calc_zero_distance_phase(&(palgo->VL53LX_p_006));






  if (ppost_cfg->hist_amb_est_method ==
      VL53LX_HIST_AMB_EST_METHOD__THRESHOLDED_BINS) {
    VL53LX_hist_estimate_ambient_from_thresholded_bins(
      (int32_t)ppost_cfg->ambient_thresh_sigma0,
      &(palgo->VL53LX_p_006));
  } else {
    VL53LX_hist_estimate_ambient_from_ambient_bins(
      &(palgo->VL53LX_p_006));
  }





  VL53LX_hist_remove_ambient_bins(&(palgo->VL53LX_p_006));





  if (ppost_cfg->algo__crosstalk_compensation_enable > 0) {
    VL53LX_f_005(
      pxtalk,
      &(palgo->VL53LX_p_006),
      &(palgo->VL53LX_p_047));
  }






  pdmax_cfg->ambient_thresh_sigma =
    ppost_cfg->ambient_thresh_sigma1;

  for (p = 0; p < VL53LX_MAX_AMBIENT_DMAX_VALUES; p++) {
    if (status == VL53LX_ERROR_NONE) {

      status =
        VL53LX_f_001(
          pdmax_cfg->target_reflectance_for_dmax_calc[p],
          pdmax_cal,
          pdmax_cfg,
          &(palgo->VL53LX_p_006),
          pdmax_algo,
          &(presults->VL53LX_p_022[p]));
    }
  }









  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_006(
        ppost_cfg->ambient_thresh_events_scaler,
        (int32_t)ppost_cfg->ambient_thresh_sigma1,
        (int32_t)ppost_cfg->min_ambient_thresh_events,
        ppost_cfg->algo__crosstalk_compensation_enable,
        &(palgo->VL53LX_p_006),
        &(palgo->VL53LX_p_047),
        palgo);









  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_007(palgo);






  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_008(palgo);






  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_009(palgo);







  for (p = 0; p < palgo->VL53LX_p_046; p++) {

    ppulse_data = &(palgo->VL53LX_p_003[p]);




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_010(
          p,
          &(palgo->VL53LX_p_006),
          palgo);
    }




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_011(
          p,
          &(palgo->VL53LX_p_006),
          palgo,
          palgo->VL53LX_p_006.VL53LX_p_028,
          &(palgo->VL53LX_p_048));
    }




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_011(
          p,
          &(palgo->VL53LX_p_006),
          palgo,
          0,
          &(palgo->VL53LX_p_049));
    }




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_011(
          p,
          &(palgo->VL53LX_p_047),
          palgo,
          0,
          &(palgo->VL53LX_p_050));
    }




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_012(
          p,
          &(palgo->VL53LX_p_048),
          palgo);
    }




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_013(
          p,
          ppost_cfg->noise_threshold,
          palgo);
    }

    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_014(
          ppulse_data->VL53LX_p_023,
          ppost_cfg->sigma_estimator__sigma_ref_mm,
          palgo->VL53LX_p_030,
          ppulse_data->VL53LX_p_051,
          ppost_cfg->algo__crosstalk_compensation_enable,
          &(palgo->VL53LX_p_048),
          &(palgo->VL53LX_p_049),
          &(palgo->VL53LX_p_050),
          &(ppulse_data->VL53LX_p_002));
    }









    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_015(
          p,
          1,
          &(palgo->VL53LX_p_006),
          palgo);

  }






  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_016(
        ppost_cfg->hist_target_order,
        palgo);






  for (p = 0; p < palgo->VL53LX_p_046; p++) {

    ppulse_data = &(palgo->VL53LX_p_003[p]);



    if (!(presults->active_results < presults->max_results)) {
      continue;
    }






    if (!(ppulse_data->VL53LX_p_010 >
          ppost_cfg->signal_total_events_limit &&
          ppulse_data->VL53LX_p_023 < 0xFF)) {
      continue;
    }

    prange_data =
      &(presults->VL53LX_p_003[presults->active_results]);

    if (status == VL53LX_ERROR_NONE)
      VL53LX_f_017(
        presults->active_results,
        ppost_cfg->valid_phase_low,
        ppost_cfg->valid_phase_high,
        ppost_cfg->sigma_thresh,
        &(palgo->VL53LX_p_006),
        ppulse_data,
        prange_data);


    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_018(
          palgo->VL53LX_p_006.vcsel_width,
          palgo->VL53LX_p_006.VL53LX_p_015,
          palgo->VL53LX_p_006.total_periods_elapsed,
          palgo->VL53LX_p_006.result__dss_actual_effective_spads,
          prange_data);


    if (status == VL53LX_ERROR_NONE)
      VL53LX_f_019(
        ppost_cfg->gain_factor,
        ppost_cfg->range_offset_mm,
        prange_data);

    presults->active_results++;


  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_f_006(
  uint16_t                          ambient_threshold_events_scaler,
  int32_t                           ambient_threshold_sigma,
  int32_t                           min_ambient_threshold_events,
  uint8_t                           algo__crosstalk_compensation_enable,
  VL53LX_histogram_bin_data_t           *pbins,
  VL53LX_histogram_bin_data_t           *pxtalk,
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{



  VL53LX_Error  status  = VL53LX_ERROR_NONE;
  uint8_t  lb            = 0;
  uint8_t  VL53LX_p_001            = 0;
  int64_t  tmp          = 0;
  int32_t  amb_events   = 0;
  int32_t  VL53LX_p_018       = 0;
  int32_t  samples      = 0;

  palgo->VL53LX_p_020            = pbins->VL53LX_p_020;
  palgo->VL53LX_p_019              = pbins->VL53LX_p_019;
  palgo->VL53LX_p_021         = pbins->VL53LX_p_021;
  palgo->VL53LX_p_028 = pbins->VL53LX_p_028;






  palgo->VL53LX_p_030 =
    VL53LX_decode_vcsel_period(pbins->VL53LX_p_005);

  tmp  = (int64_t)pbins->VL53LX_p_028;
  tmp *= (int64_t)ambient_threshold_events_scaler;
  tmp += 2048;
  tmp = do_division_s(tmp, 4096);
  amb_events = (int32_t)tmp;


  for (lb = 0; lb < pbins->VL53LX_p_021; lb++) {

    VL53LX_p_001 = lb >> 2;
    samples = (int32_t)pbins->bin_rep[VL53LX_p_001];

    if (samples > 0) {

      if (lb < pxtalk->VL53LX_p_021 &&
          algo__crosstalk_compensation_enable > 0)
        VL53LX_p_018 = samples * (amb_events +
                                  pxtalk->bin_data[lb]);
      else {
        VL53LX_p_018 = samples *  amb_events;
      }

      VL53LX_p_018  = VL53LX_isqrt(VL53LX_p_018);

      VL53LX_p_018 += (samples / 2);
      VL53LX_p_018 /= samples;
      VL53LX_p_018 *= ambient_threshold_sigma;
      VL53LX_p_018 += 8;
      VL53LX_p_018 /= 16;
      VL53LX_p_018 += amb_events;

      if (VL53LX_p_018 < min_ambient_threshold_events) {
        VL53LX_p_018 = min_ambient_threshold_events;
      }

      palgo->VL53LX_p_052[lb]             = VL53LX_p_018;
      palgo->VL53LX_p_031 = VL53LX_p_018;
    }

  }

  palgo->VL53LX_p_039 = 0;

  for (lb = pbins->VL53LX_p_019; lb < pbins->VL53LX_p_021; lb++) {

    if (pbins->bin_data[lb] > palgo->VL53LX_p_052[lb]) {
      palgo->VL53LX_p_040[lb]      = 1;
      palgo->VL53LX_p_041[lb] = 1;
      palgo->VL53LX_p_039++;
    } else {
      palgo->VL53LX_p_040[lb]      = 0;
      palgo->VL53LX_p_041[lb] = 0;
    }
  }



  return status;

}



VL53LX_Error VL53LX::VL53LX_f_007(
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  uint8_t  i            = 0;
  uint8_t  j            = 0;
  uint8_t  found        = 0;


  palgo->VL53LX_p_044 = 0;

  for (i = 0; i < palgo->VL53LX_p_030; i++) {

    j = (i + 1) % palgo->VL53LX_p_030;



    if (i < palgo->VL53LX_p_021 && j < palgo->VL53LX_p_021) {
      if (palgo->VL53LX_p_041[i] == 0 &&
          palgo->VL53LX_p_041[j] == 1 &&
          found == 0) {
        palgo->VL53LX_p_044 = i;
        found = 1;
      }
    }
  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_f_008(
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{

  VL53LX_Error  status  = VL53LX_ERROR_NONE;
  uint8_t  i            = 0;
  uint8_t  j            = 0;
  uint8_t  lb            = 0;



  for (lb = palgo->VL53LX_p_044;
       lb < (palgo->VL53LX_p_044 +
             palgo->VL53LX_p_030);
       lb++) {






    i =  lb      % palgo->VL53LX_p_030;
    j = (lb + 1) % palgo->VL53LX_p_030;






    if (i < palgo->VL53LX_p_021 && j < palgo->VL53LX_p_021) {

      if (palgo->VL53LX_p_041[i] == 0 &&
          palgo->VL53LX_p_041[j] == 1) {
        palgo->VL53LX_p_046++;
      }

      if (palgo->VL53LX_p_041[i] > 0) {
        palgo->VL53LX_p_042[i] = palgo->VL53LX_p_046;
      } else {
        palgo->VL53LX_p_042[i] = 0;
      }
    }

  }



  if (palgo->VL53LX_p_046 > palgo->VL53LX_p_045) {
    palgo->VL53LX_p_046 = palgo->VL53LX_p_045;
  }

  return status;

}

VL53LX_Error VL53LX::VL53LX_f_009(
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  uint8_t  i            = 0;
  uint8_t  j            = 0;
  uint8_t  blb            = 0;
  uint8_t  pulse_no     = 0;

  uint8_t  max_filter_half_width = 0;

  VL53LX_hist_pulse_data_t *pdata;



  max_filter_half_width = palgo->VL53LX_p_030 - 1;
  max_filter_half_width = max_filter_half_width >> 1;

  for (blb = palgo->VL53LX_p_044;
       blb < (palgo->VL53LX_p_044 +
              palgo->VL53LX_p_030);
       blb++) {






    i =  blb      % palgo->VL53LX_p_030;
    j = (blb + 1) % palgo->VL53LX_p_030;






    if (i < palgo->VL53LX_p_021 &&
        j < palgo->VL53LX_p_021) {




      if (palgo->VL53LX_p_042[i] == 0 &&
          palgo->VL53LX_p_042[j] > 0) {

        pulse_no = palgo->VL53LX_p_042[j] - 1;
        pdata   = &(palgo->VL53LX_p_003[pulse_no]);

        if (pulse_no < palgo->VL53LX_p_045) {
          pdata->VL53LX_p_012 = blb;
          pdata->VL53LX_p_019    = blb + 1;
          pdata->VL53LX_p_023   = 0xFF;

          pdata->VL53LX_p_024     = 0;
          pdata->VL53LX_p_013   = 0;
        }
      }




      if (palgo->VL53LX_p_042[i] > 0
          && palgo->VL53LX_p_042[j] == 0) {

        pulse_no = palgo->VL53LX_p_042[i] - 1;
        pdata   = &(palgo->VL53LX_p_003[pulse_no]);

        if (pulse_no < palgo->VL53LX_p_045) {

          pdata->VL53LX_p_024    = blb;
          pdata->VL53LX_p_013  = blb + 1;

          pdata->VL53LX_p_025 =
            (pdata->VL53LX_p_024 + 1) -
            pdata->VL53LX_p_019;
          pdata->VL53LX_p_051 =
            (pdata->VL53LX_p_013 + 1) -
            pdata->VL53LX_p_012;

          if (pdata->VL53LX_p_051 >
              max_filter_half_width)
            pdata->VL53LX_p_051 =
              max_filter_half_width;
        }

      }
    }
  }
  return status;

}


VL53LX_Error VL53LX::VL53LX_f_016(
  VL53LX_HistTargetOrder                target_order,
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t  tmp;
  VL53LX_hist_pulse_data_t *ptmp = &tmp;
  VL53LX_hist_pulse_data_t *p0;
  VL53LX_hist_pulse_data_t *p1;

  uint8_t i       = 0;
  uint8_t swapped = 1;

  if (!(palgo->VL53LX_p_046 > 1)) {
    goto ENDFUNC;
  }

  while (swapped > 0) {

    swapped = 0;

    for (i = 1; i < palgo->VL53LX_p_046; i++) {

      p0 = &(palgo->VL53LX_p_003[i - 1]);
      p1 = &(palgo->VL53LX_p_003[i]);




      if (target_order
          == VL53LX_HIST_TARGET_ORDER__STRONGEST_FIRST) {

        if (p0->VL53LX_p_010 <
            p1->VL53LX_p_010) {




          memcpy(ptmp,
                 p1, sizeof(VL53LX_hist_pulse_data_t));
          memcpy(p1,
                 p0, sizeof(VL53LX_hist_pulse_data_t));
          memcpy(p0,
                 ptmp, sizeof(VL53LX_hist_pulse_data_t));

          swapped = 1;
        }

      } else {

        if (p0->VL53LX_p_011 > p1->VL53LX_p_011) {




          memcpy(ptmp,
                 p1, sizeof(VL53LX_hist_pulse_data_t));
          memcpy(p1,
                 p0,   sizeof(VL53LX_hist_pulse_data_t));
          memcpy(p0,
                 ptmp, sizeof(VL53LX_hist_pulse_data_t));

          swapped = 1;
        }

      }
    }
  }
ENDFUNC:
  return status;

}


VL53LX_Error VL53LX::VL53LX_f_010(
  uint8_t                                pulse_no,
  VL53LX_histogram_bin_data_t           *pbins,
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  uint8_t  i            = 0;
  uint8_t  lb            = 0;

  VL53LX_hist_pulse_data_t *pdata = &(palgo->VL53LX_p_003[pulse_no]);


  pdata->VL53LX_p_017  = 0;
  pdata->VL53LX_p_016 = 0;

  for (lb = pdata->VL53LX_p_012; lb <= pdata->VL53LX_p_013; lb++) {
    i =  lb % palgo->VL53LX_p_030;
    pdata->VL53LX_p_017  += pbins->bin_data[i];
    pdata->VL53LX_p_016 += palgo->VL53LX_p_028;
  }


  pdata->VL53LX_p_010 =
    pdata->VL53LX_p_017 - pdata->VL53LX_p_016;

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_015(
  uint8_t                                pulse_no,
  uint8_t                                clip_events,
  VL53LX_histogram_bin_data_t           *pbins,
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  uint8_t   i            = 0;
  int16_t   VL53LX_p_012 = 0;
  int16_t   VL53LX_p_013   = 0;
  int16_t   window_width = 0;
  uint32_t  tmp_phase    = 0;

  VL53LX_hist_pulse_data_t *pdata = &(palgo->VL53LX_p_003[pulse_no]);

  i = pdata->VL53LX_p_023 % palgo->VL53LX_p_030;

  VL53LX_p_012  = (int16_t)i;
  VL53LX_p_012 += (int16_t)pdata->VL53LX_p_012;
  VL53LX_p_012 -= (int16_t)pdata->VL53LX_p_023;

  VL53LX_p_013    = (int16_t)i;
  VL53LX_p_013   += (int16_t)pdata->VL53LX_p_013;
  VL53LX_p_013   -= (int16_t)pdata->VL53LX_p_023;




  window_width = VL53LX_p_013 - VL53LX_p_012;
  if (window_width > 3) {
    window_width = 3;
  }

  status =
    VL53LX_f_020(
      VL53LX_p_012,
      VL53LX_p_012 + window_width,
      palgo->VL53LX_p_030,
      clip_events,
      pbins,
      &(pdata->VL53LX_p_026));


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_020(
        VL53LX_p_013 - window_width,
        VL53LX_p_013,
        palgo->VL53LX_p_030,
        clip_events,
        pbins,
        &(pdata->VL53LX_p_027));





  if (pdata->VL53LX_p_026 > pdata->VL53LX_p_027) {
    tmp_phase        = pdata->VL53LX_p_026;
    pdata->VL53LX_p_026 = pdata->VL53LX_p_027;
    pdata->VL53LX_p_027 = tmp_phase;
  }


  if (pdata->VL53LX_p_011 < pdata->VL53LX_p_026) {
    pdata->VL53LX_p_026 = pdata->VL53LX_p_011;
  }

  if (pdata->VL53LX_p_011 > pdata->VL53LX_p_027) {
    pdata->VL53LX_p_027 = pdata->VL53LX_p_011;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_020(
  int16_t                            VL53LX_p_019,
  int16_t                            VL53LX_p_024,
  uint8_t                            VL53LX_p_030,
  uint8_t                            clip_events,
  VL53LX_histogram_bin_data_t       *pbins,
  uint32_t                          *pphase)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  int16_t  i            = 0;
  int16_t  lb            = 0;

  int64_t VL53LX_p_018        = 0;
  int64_t event_sum     = 0;
  int64_t weighted_sum  = 0;


  *pphase = VL53LX_MAX_ALLOWED_PHASE;

  for (lb = VL53LX_p_019; lb <= VL53LX_p_024; lb++) {



    if (lb < 0) {
      i =  lb + (int16_t)VL53LX_p_030;
    } else {
      i =  lb % (int16_t)VL53LX_p_030;
    }

    VL53LX_p_018 =
      (int64_t)pbins->bin_data[i] -
      (int64_t)pbins->VL53LX_p_028;






    if (clip_events > 0 && VL53LX_p_018 < 0) {
      VL53LX_p_018 = 0;
    }

    event_sum += VL53LX_p_018;

    weighted_sum +=
      (VL53LX_p_018 * (1024 + (2048 * (int64_t)lb)));


  }

  if (event_sum  > 0) {

    weighted_sum += (event_sum / 2);
    weighted_sum /= event_sum;

    if (weighted_sum < 0) {
      weighted_sum = 0;
    }

    *pphase = (uint32_t)weighted_sum;
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_011(
  uint8_t                                pulse_no,
  VL53LX_histogram_bin_data_t           *pbins,
  VL53LX_hist_gen3_algo_private_data_t  *palgo,
  int32_t                                pad_value,
  VL53LX_histogram_bin_data_t           *ppulse)
{


  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  uint8_t  i            = 0;
  uint8_t  lb            = 0;

  VL53LX_hist_pulse_data_t *pdata = &(palgo->VL53LX_p_003[pulse_no]);

  memcpy(ppulse, pbins, sizeof(VL53LX_histogram_bin_data_t));

  for (lb = palgo->VL53LX_p_044;
       lb < (palgo->VL53LX_p_044 +
             palgo->VL53LX_p_030);
       lb++) {

    if (lb < pdata->VL53LX_p_012 || lb > pdata->VL53LX_p_013) {
      i =  lb % palgo->VL53LX_p_030;
      if (i < ppulse->VL53LX_p_021) {
        ppulse->bin_data[i] = pad_value;
      }
    }
  }


  return status;
}


VL53LX_Error VL53LX::VL53LX_f_012(
  uint8_t                                pulse_no,
  VL53LX_histogram_bin_data_t           *ppulse,
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{
  VL53LX_Error  status       = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t *pdata = &(palgo->VL53LX_p_003[pulse_no]);

  uint8_t  lb            = 0;
  uint8_t  i            = 0;
  uint8_t  j            = 0;
  uint8_t  w            = 0;



  for (lb = pdata->VL53LX_p_012; lb <= pdata->VL53LX_p_013; lb++) {

    i =  lb  % palgo->VL53LX_p_030;



    palgo->VL53LX_p_043[i]  = 0;
    palgo->VL53LX_p_018[i] = 0;

    for (w = 0; w < (pdata->VL53LX_p_051 << 1); w++) {





      j = lb + w + palgo->VL53LX_p_030;
      j = j - pdata->VL53LX_p_051;
      j = j % palgo->VL53LX_p_030;






      if (i < ppulse->VL53LX_p_021 && j <
          ppulse->VL53LX_p_021) {
        if (w < pdata->VL53LX_p_051)
          palgo->VL53LX_p_043[i] +=
            ppulse->bin_data[j];
        else
          palgo->VL53LX_p_043[i] -=
            ppulse->bin_data[j];
      }
    }
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_013(
  uint8_t                                pulse_no,
  uint16_t                               noise_threshold,
  VL53LX_hist_gen3_algo_private_data_t  *palgo)
{

  VL53LX_Error  status       = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t *pdata = &(palgo->VL53LX_p_003[pulse_no]);

  uint8_t  lb            = 0;
  uint8_t  i            = 0;
  uint8_t  j            = 0;
  int32_t  bin_x_delta  = 0;

  for (lb = pdata->VL53LX_p_012; lb < pdata->VL53LX_p_013; lb++) {

    i =  lb    % palgo->VL53LX_p_030;
    j = (lb + 1) % palgo->VL53LX_p_030;

    if (i < palgo->VL53LX_p_021 &&
        j < palgo->VL53LX_p_021) {

      if (palgo->VL53LX_p_043[i] <= 0 &&
          palgo->VL53LX_p_043[j] > 0) {





        bin_x_delta = palgo->VL53LX_p_043[j] -
                      palgo->VL53LX_p_043[i];



        if (bin_x_delta >
            (int32_t)noise_threshold) {

          pdata->VL53LX_p_023 = lb;

          VL53LX_f_021(
            lb,
            palgo->VL53LX_p_043[i],
            palgo->VL53LX_p_043[j],
            palgo->VL53LX_p_030,
            &(pdata->VL53LX_p_011));
        }
      }
    }
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_021(
  uint8_t   bin,
  int32_t   filta0,
  int32_t   filta1,
  uint8_t   VL53LX_p_030,
  uint32_t *pmean_phase)
{

  VL53LX_Error  status = VL53LX_ERROR_NONE;
  int32_t  mean_phase  = VL53LX_MAX_ALLOWED_PHASE;
  int32_t  bin_x_phase  = abs(filta0);
  int32_t  bin_x_delta  = filta1 - filta0;


  if (bin_x_delta == 0) {
    mean_phase = 1024;
  } else
    mean_phase  = ((bin_x_phase  * 2048) +
                   (bin_x_delta >> 1)) / bin_x_delta;

  mean_phase += (2048 * (int64_t)bin);



  if (mean_phase  < 0) {
    mean_phase = 0;
  }

  if (mean_phase > VL53LX_MAX_ALLOWED_PHASE) {
    mean_phase = VL53LX_MAX_ALLOWED_PHASE;
  }



  mean_phase = mean_phase % ((int32_t)VL53LX_p_030 * 2048);

  *pmean_phase = (uint32_t)mean_phase;


  return status;
}


VL53LX_Error VL53LX::VL53LX_f_014(
  uint8_t                       bin,
  uint8_t                       sigma_estimator__sigma_ref_mm,
  uint8_t                       VL53LX_p_030,
  uint8_t                       VL53LX_p_051,
  uint8_t                       crosstalk_compensation_enable,
  VL53LX_histogram_bin_data_t  *phist_data_ap,
  VL53LX_histogram_bin_data_t  *phist_data_zp,
  VL53LX_histogram_bin_data_t  *pxtalk_hist,
  uint16_t                     *psigma_est)
{

  VL53LX_Error status      = VL53LX_ERROR_NONE;
  VL53LX_Error func_status = VL53LX_ERROR_NONE;

  uint8_t  i    = 0;
  int32_t  VL53LX_p_007    = 0;
  int32_t  VL53LX_p_032    = 0;
  int32_t  VL53LX_p_001    = 0;
  int32_t  a_zp = 0;
  int32_t  c_zp = 0;
  int32_t  ax   = 0;
  int32_t  bx   = 0;
  int32_t  cx   = 0;




  i = bin % VL53LX_p_030;




  VL53LX_f_022(
    i,
    VL53LX_p_051,
    phist_data_zp,
    &a_zp,
    &VL53LX_p_032,
    &c_zp);




  VL53LX_f_022(
    i,
    VL53LX_p_051,
    phist_data_ap,
    &VL53LX_p_007,
    &VL53LX_p_032,
    &VL53LX_p_001);

  if (crosstalk_compensation_enable > 0)
    VL53LX_f_022(
      i,
      VL53LX_p_051,
      pxtalk_hist,
      &ax,
      &bx,
      &cx);


  func_status =
    VL53LX_f_023(
      sigma_estimator__sigma_ref_mm,
      (uint32_t)VL53LX_p_007,

      (uint32_t)VL53LX_p_032,

      (uint32_t)VL53LX_p_001,

      (uint32_t)a_zp,

      (uint32_t)c_zp,

      (uint32_t)bx,
      (uint32_t)ax,
      (uint32_t)cx,
      (uint32_t)phist_data_ap->VL53LX_p_028,
      phist_data_ap->VL53LX_p_015,
      psigma_est);


  if (func_status == VL53LX_ERROR_DIVISION_BY_ZERO) {
    *psigma_est = 0xFFFF;
  }


  return status;
}


void VL53LX::VL53LX_f_017(
  uint8_t                      range_id,
  uint8_t                      valid_phase_low,
  uint8_t                      valid_phase_high,
  uint16_t                     sigma_thres,
  VL53LX_histogram_bin_data_t *pbins,
  VL53LX_hist_pulse_data_t    *ppulse,
  VL53LX_range_data_t         *pdata)
{

  uint16_t  lower_phase_limit = 0;
  uint16_t  upper_phase_limit = 0;




  pdata->range_id              = range_id;
  pdata->time_stamp            = 0;

  pdata->VL53LX_p_012          = ppulse->VL53LX_p_012;
  pdata->VL53LX_p_019             = ppulse->VL53LX_p_019;
  pdata->VL53LX_p_023            = ppulse->VL53LX_p_023;
  pdata->VL53LX_p_024              = ppulse->VL53LX_p_024;
  pdata->VL53LX_p_013            = ppulse->VL53LX_p_013;
  pdata->VL53LX_p_025             = ppulse->VL53LX_p_025;




  pdata->VL53LX_p_029  =
    (ppulse->VL53LX_p_013 + 1) - ppulse->VL53LX_p_012;




  pdata->zero_distance_phase   = pbins->zero_distance_phase;
  pdata->VL53LX_p_002              = ppulse->VL53LX_p_002;
  pdata->VL53LX_p_026             = (uint16_t)ppulse->VL53LX_p_026;
  pdata->VL53LX_p_011          = (uint16_t)ppulse->VL53LX_p_011;
  pdata->VL53LX_p_027             = (uint16_t)ppulse->VL53LX_p_027;
  pdata->VL53LX_p_017  = (uint32_t)ppulse->VL53LX_p_017;
  pdata->VL53LX_p_010   = ppulse->VL53LX_p_010;
  pdata->VL53LX_p_016 = (uint32_t)ppulse->VL53LX_p_016;
  pdata->total_periods_elapsed = pbins->total_periods_elapsed;




  pdata->range_status = VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK;



  if (sigma_thres > 0 &&
      (uint32_t)ppulse->VL53LX_p_002 > ((uint32_t)sigma_thres << 5)) {
    pdata->range_status = VL53LX_DEVICEERROR_SIGMATHRESHOLDCHECK;
  }




  lower_phase_limit  = (uint8_t)valid_phase_low << 8;
  if (lower_phase_limit < pdata->zero_distance_phase)
    lower_phase_limit =
      pdata->zero_distance_phase -
      lower_phase_limit;
  else {
    lower_phase_limit  = 0;
  }

  upper_phase_limit  = (uint8_t)valid_phase_high << 8;
  upper_phase_limit += pbins->zero_distance_phase;

  if (pdata->VL53LX_p_011 < lower_phase_limit ||
      pdata->VL53LX_p_011 > upper_phase_limit) {
    pdata->range_status = VL53LX_DEVICEERROR_RANGEPHASECHECK;
  }

}



/* vl53lx_hist_algos_gen4.c */

void VL53LX::VL53LX_f_024(
  VL53LX_hist_gen4_algo_filtered_data_t   *palgo)
{

  uint8_t  lb                 = 0;

  palgo->VL53LX_p_020              = VL53LX_HISTOGRAM_BUFFER_SIZE;
  palgo->VL53LX_p_019                = 0;
  palgo->VL53LX_p_021           = 0;

  for (lb = palgo->VL53LX_p_019; lb < palgo->VL53LX_p_020; lb++) {
    palgo->VL53LX_p_007[lb]      = 0;
    palgo->VL53LX_p_032[lb]      = 0;
    palgo->VL53LX_p_001[lb]      = 0;
    palgo->VL53LX_p_053[lb] = 0;
    palgo->VL53LX_p_054[lb] = 0;
    palgo->VL53LX_p_040[lb]  = 0;
  }
}
VL53LX_Error VL53LX::VL53LX_f_025(
  VL53LX_dmax_calibration_data_t         *pdmax_cal,
  VL53LX_hist_gen3_dmax_config_t         *pdmax_cfg,
  VL53LX_hist_post_process_config_t      *ppost_cfg,
  VL53LX_histogram_bin_data_t            *pbins_input,
  VL53LX_histogram_bin_data_t            *pxtalk,
  VL53LX_hist_gen3_algo_private_data_t   *palgo3,
  VL53LX_hist_gen4_algo_filtered_data_t  *pfiltered,
  VL53LX_hist_gen3_dmax_private_data_t   *pdmax_algo,
  VL53LX_range_results_t                 *presults)
{
  VL53LX_Error  status  = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t     *ppulse_data;
  VL53LX_range_data_t          *prange_data;

  uint8_t                       p = 0;
  VL53LX_histogram_bin_data_t *pB = &(palgo3->VL53LX_p_006);

  VL53LX_f_003(palgo3);

  memcpy(
    &(palgo3->VL53LX_p_006),
    pbins_input,
    sizeof(VL53LX_histogram_bin_data_t));

  presults->cfg_device_state = pbins_input->cfg_device_state;
  presults->rd_device_state  = pbins_input->rd_device_state;
  presults->zone_id          = pbins_input->zone_id;
  presults->stream_count     = pbins_input->result__stream_count;
  presults->wrap_dmax_mm     = 0;
  presults->max_results      = VL53LX_MAX_RANGE_RESULTS;
  presults->active_results   = 0;

  for (p = 0; p < VL53LX_MAX_AMBIENT_DMAX_VALUES; p++) {
    presults->VL53LX_p_022[p] = 0;
  }




  VL53LX_hist_calc_zero_distance_phase(&(palgo3->VL53LX_p_006));






  if (ppost_cfg->hist_amb_est_method ==
      VL53LX_HIST_AMB_EST_METHOD__THRESHOLDED_BINS)
    VL53LX_hist_estimate_ambient_from_thresholded_bins(
      (int32_t)ppost_cfg->ambient_thresh_sigma0,
      &(palgo3->VL53LX_p_006));
  else
    VL53LX_hist_estimate_ambient_from_ambient_bins(
      &(palgo3->VL53LX_p_006));





  VL53LX_hist_remove_ambient_bins(&(palgo3->VL53LX_p_006));





  if (ppost_cfg->algo__crosstalk_compensation_enable > 0)
    VL53LX_f_005(
      pxtalk,
      &(palgo3->VL53LX_p_006),
      &(palgo3->VL53LX_p_047));






  pdmax_cfg->ambient_thresh_sigma =
    ppost_cfg->ambient_thresh_sigma1;

  for (p = 0; p < VL53LX_MAX_AMBIENT_DMAX_VALUES; p++) {
    if (status == VL53LX_ERROR_NONE) {

      status =
        VL53LX_f_001(
          pdmax_cfg->target_reflectance_for_dmax_calc[p],
          pdmax_cal,
          pdmax_cfg,
          &(palgo3->VL53LX_p_006),
          pdmax_algo,
          &(presults->VL53LX_p_022[p]));
    }
  }









  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_006(
        ppost_cfg->ambient_thresh_events_scaler,
        (int32_t)ppost_cfg->ambient_thresh_sigma1,
        (int32_t)ppost_cfg->min_ambient_thresh_events,
        ppost_cfg->algo__crosstalk_compensation_enable,
        &(palgo3->VL53LX_p_006),
        &(palgo3->VL53LX_p_047),
        palgo3);









  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_007(palgo3);






  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_008(palgo3);






  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_009(palgo3);






  for (p = 0; p < palgo3->VL53LX_p_046; p++) {

    ppulse_data = &(palgo3->VL53LX_p_003[p]);




    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_010(
          p,
          &(palgo3->VL53LX_p_006),
          palgo3);




    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_011(
          p,
          &(palgo3->VL53LX_p_006),
          palgo3,
          pB->VL53LX_p_028,
          &(palgo3->VL53LX_p_048));




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_011(
          p,
          &(palgo3->VL53LX_p_006),
          palgo3,
          0,
          &(palgo3->VL53LX_p_049));
    }




    if (status == VL53LX_ERROR_NONE) {
      status =
        VL53LX_f_011(
          p,
          &(palgo3->VL53LX_p_047),
          palgo3,
          0,
          &(palgo3->VL53LX_p_050));
    }




    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_026(
          p,
          &(palgo3->VL53LX_p_048),
          palgo3,
          pfiltered);




    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_027(
          p,
          ppost_cfg->noise_threshold,
          pfiltered,
          palgo3);

    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_014(
          ppulse_data->VL53LX_p_023,
          ppost_cfg->sigma_estimator__sigma_ref_mm,
          palgo3->VL53LX_p_030,
          ppulse_data->VL53LX_p_051,
          ppost_cfg->algo__crosstalk_compensation_enable,
          &(palgo3->VL53LX_p_048),
          &(palgo3->VL53LX_p_049),
          &(palgo3->VL53LX_p_050),
          &(ppulse_data->VL53LX_p_002));









    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_f_015(
          p,
          1,
          &(palgo3->VL53LX_p_006),
          palgo3);

  }






  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_f_016(
        ppost_cfg->hist_target_order,
        palgo3);






  for (p = 0; p < palgo3->VL53LX_p_046; p++) {

    ppulse_data = &(palgo3->VL53LX_p_003[p]);



    if (!(presults->active_results < presults->max_results)) {
      continue;
    }








    if (ppulse_data->VL53LX_p_010 >
        ppost_cfg->signal_total_events_limit &&
        ppulse_data->VL53LX_p_023 < 0xFF) {

      prange_data =
        &(presults->VL53LX_p_003[presults->active_results]);

      if (status == VL53LX_ERROR_NONE)
        VL53LX_f_017(
          presults->active_results,
          ppost_cfg->valid_phase_low,
          ppost_cfg->valid_phase_high,
          ppost_cfg->sigma_thresh,
          &(palgo3->VL53LX_p_006),
          ppulse_data,
          prange_data);

      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_f_018(
            pB->vcsel_width,
            pB->VL53LX_p_015,
            pB->total_periods_elapsed,
            pB->result__dss_actual_effective_spads,
            prange_data);

      if (status == VL53LX_ERROR_NONE)
        VL53LX_f_019(
          ppost_cfg->gain_factor,
          ppost_cfg->range_offset_mm,
          prange_data);

      presults->active_results++;
    }

  }



  return status;
}



VL53LX_Error VL53LX::VL53LX_f_026(
  uint8_t                                pulse_no,
  VL53LX_histogram_bin_data_t           *ppulse,
  VL53LX_hist_gen3_algo_private_data_t  *palgo3,
  VL53LX_hist_gen4_algo_filtered_data_t *pfiltered)
{

  VL53LX_Error  status       = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t *pdata = &(palgo3->VL53LX_p_003[pulse_no]);

  uint8_t  lb     = 0;
  uint8_t  i     = 0;
  int32_t  suma  = 0;
  int32_t  sumb  = 0;
  int32_t  sumc  = 0;


  pfiltered->VL53LX_p_020    = palgo3->VL53LX_p_020;
  pfiltered->VL53LX_p_019      = palgo3->VL53LX_p_019;
  pfiltered->VL53LX_p_021 = palgo3->VL53LX_p_021;

  for (lb = pdata->VL53LX_p_012; lb <= pdata->VL53LX_p_013; lb++) {

    i =  lb  % palgo3->VL53LX_p_030;



    VL53LX_f_022(
      i,
      pdata->VL53LX_p_051,
      ppulse,
      &suma,
      &sumb,
      &sumc);



    pfiltered->VL53LX_p_007[i] = suma;
    pfiltered->VL53LX_p_032[i] = sumb;
    pfiltered->VL53LX_p_001[i] = sumc;




    pfiltered->VL53LX_p_053[i] =
      (suma + sumb) -
      (sumc + palgo3->VL53LX_p_028);




    pfiltered->VL53LX_p_054[i] =
      (sumb + sumc) -
      (suma + palgo3->VL53LX_p_028);
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_027(
  uint8_t                                pulse_no,
  uint16_t                               noise_threshold,
  VL53LX_hist_gen4_algo_filtered_data_t *pfiltered,
  VL53LX_hist_gen3_algo_private_data_t  *palgo3)
{

  VL53LX_Error  status       = VL53LX_ERROR_NONE;
  VL53LX_Error  func_status  = VL53LX_ERROR_NONE;

  VL53LX_hist_pulse_data_t *pdata = &(palgo3->VL53LX_p_003[pulse_no]);

  uint8_t  lb            = 0;
  uint8_t  i            = 0;
  uint8_t  j            = 0;

  SUPPRESS_UNUSED_WARNING(noise_threshold);

  for (lb = pdata->VL53LX_p_012; lb < pdata->VL53LX_p_013; lb++) {

    i =  lb    % palgo3->VL53LX_p_030;
    j = (lb + 1) % palgo3->VL53LX_p_030;

    if (i < palgo3->VL53LX_p_021 &&
        j < palgo3->VL53LX_p_021) {

      if (pfiltered->VL53LX_p_053[i] == 0 &&
          pfiltered->VL53LX_p_054[i] == 0)


      {
        pfiltered->VL53LX_p_040[i] = 0;
      }

      else if (pfiltered->VL53LX_p_053[i] >= 0 &&
               pfiltered->VL53LX_p_054[i] >= 0) {
        pfiltered->VL53LX_p_040[i] = 1;
      }

      else if (pfiltered->VL53LX_p_053[i] <  0 &&
               pfiltered->VL53LX_p_054[i] >= 0 &&
               pfiltered->VL53LX_p_053[j] >= 0 &&
               pfiltered->VL53LX_p_054[j] <  0) {
        pfiltered->VL53LX_p_040[i] = 1;
      }

      else {
        pfiltered->VL53LX_p_040[i] = 0;
      }



      if (pfiltered->VL53LX_p_040[i] > 0) {

        pdata->VL53LX_p_023 = lb;

        func_status =
          VL53LX_f_028(
            lb,
            pfiltered->VL53LX_p_007[i],
            pfiltered->VL53LX_p_032[i],
            pfiltered->VL53LX_p_001[i],
            0,

            0,

            0,

            palgo3->VL53LX_p_028,
            palgo3->VL53LX_p_030,
            &(pdata->VL53LX_p_011));

        if (func_status ==
            VL53LX_ERROR_DIVISION_BY_ZERO) {
          pfiltered->VL53LX_p_040[i] = 0;
        }

      }
    }
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_f_028(
  uint8_t   bin,
  int32_t   VL53LX_p_007,
  int32_t   VL53LX_p_032,
  int32_t   VL53LX_p_001,
  int32_t   ax,
  int32_t   bx,
  int32_t   cx,
  int32_t   VL53LX_p_028,
  uint8_t   VL53LX_p_030,
  uint32_t *pmean_phase)
{





  VL53LX_Error  status = VL53LX_ERROR_DIVISION_BY_ZERO;

  int64_t  mean_phase  = VL53LX_MAX_ALLOWED_PHASE;
  int64_t  VL53LX_p_055   = 0;
  int64_t  b_minus_amb = 0;








  VL53LX_p_055    =     4096 * ((int64_t)VL53LX_p_001 -
                                (int64_t)cx - (int64_t)VL53LX_p_007 - (int64_t)ax);
  b_minus_amb  = 2 * 4096 * ((int64_t)VL53LX_p_032 -
                             (int64_t)bx - (int64_t)VL53LX_p_028);

  if (b_minus_amb != 0) {

    mean_phase   = ((4096 * VL53LX_p_055) +
                    (b_minus_amb / 2)) / b_minus_amb;
    mean_phase  +=  2048;
    mean_phase  += (4096 * (int64_t)bin);



    mean_phase  = (mean_phase + 1) / 2;



    if (mean_phase  < 0) {
      mean_phase = 0;
    }
    if (mean_phase > VL53LX_MAX_ALLOWED_PHASE) {
      mean_phase = VL53LX_MAX_ALLOWED_PHASE;
    }



    mean_phase = mean_phase %
                 ((int64_t)VL53LX_p_030 * 2048);

    status = VL53LX_ERROR_NONE;

  }

  *pmean_phase = (uint32_t)mean_phase;

  return status;
}

/* vl53lx_dmax.c */
VL53LX_Error VL53LX::VL53LX_f_001(
  uint16_t                              target_reflectance,
  VL53LX_dmax_calibration_data_t       *pcal,
  VL53LX_hist_gen3_dmax_config_t       *pcfg,
  VL53LX_histogram_bin_data_t          *pbins,
  VL53LX_hist_gen3_dmax_private_data_t *pdata,
  int16_t                              *pambient_dmax_mm)
{

  VL53LX_Error status  = VL53LX_ERROR_NONE;

  uint32_t    pll_period_us       = 0;
  uint32_t    periods_elapsed     = 0;

  uint32_t    tmp32               = 0;
  uint64_t    tmp64               = 0;

  uint32_t    amb_thres_delta     = 0;

  pdata->VL53LX_p_004     = 0x0000;
  pdata->VL53LX_p_033 = 0x0000;
  pdata->VL53LX_p_034          = 0x0000;
  pdata->VL53LX_p_009    = 0x0000;
  pdata->VL53LX_p_028     = 0x0000;
  pdata->VL53LX_p_035 = 0x0000;
  pdata->VL53LX_p_036             = 0;
  pdata->VL53LX_p_022            = 0;

  *pambient_dmax_mm  = 0;





  if ((pbins->VL53LX_p_015        != 0) &&
      (pbins->total_periods_elapsed      != 0)) {




    pll_period_us   =
      VL53LX_calc_pll_period_us(pbins->VL53LX_p_015);




    periods_elapsed = pbins->total_periods_elapsed + 1;






    pdata->VL53LX_p_037  =
      VL53LX_duration_maths(
        pll_period_us,
        1 << 4,
        VL53LX_RANGING_WINDOW_VCSEL_PERIODS,
        periods_elapsed);



    pdata->VL53LX_p_034 =
      VL53LX_rate_maths(
        pbins->VL53LX_p_028,
        pdata->VL53LX_p_037);




    pdata->VL53LX_p_033   =
      VL53LX_events_per_spad_maths(
        pbins->VL53LX_p_028,
        pbins->result__dss_actual_effective_spads,
        pdata->VL53LX_p_037);















    pdata->VL53LX_p_038 = pcfg->max_effective_spads;
    pdata->VL53LX_p_004  = pcfg->max_effective_spads;

    if (pdata->VL53LX_p_033 > 0) {
      tmp64   =
        (uint64_t)pcfg->dss_config__target_total_rate_mcps;
      tmp64  *= 1000;
      tmp64 <<= (11 + 1);
      tmp64  +=
        ((uint64_t)pdata->VL53LX_p_033 / 2);
      tmp64  /= (uint64_t)pdata->VL53LX_p_033;

      if (tmp64 < (uint64_t)pcfg->max_effective_spads) {
        pdata->VL53LX_p_004 = (uint16_t)tmp64;
      }
    }
  }




  if ((pcal->ref__actual_effective_spads != 0) &&
      (pbins->VL53LX_p_015        != 0) &&
      (pcal->ref_reflectance_pc          != 0) &&
      (pbins->total_periods_elapsed      != 0)) {













    tmp64  = (uint64_t)pcal->ref__peak_signal_count_rate_mcps;
    tmp64 *= (1000 * 256);
    tmp64 += ((uint64_t)pcal->ref__actual_effective_spads / 2);
    tmp64 /= (uint64_t)pcal->ref__actual_effective_spads;

    pdata->VL53LX_p_009   = (uint32_t)tmp64;
    pdata->VL53LX_p_009 <<= 4;






    tmp64   = (uint64_t)pdata->VL53LX_p_037;
    tmp64  *= (uint64_t)pdata->VL53LX_p_033;
    tmp64  *= (uint64_t)pdata->VL53LX_p_004;
    tmp64  += (1 << (11 + 7));
    tmp64 >>= (11 + 8);
    tmp64  +=  500;
    tmp64  /= 1000;



    if (tmp64 > 0x00FFFFFF) {
      tmp64 = 0x00FFFFFF;
    }

    pdata->VL53LX_p_028     = (uint32_t)tmp64;






    tmp64   = (uint64_t)pdata->VL53LX_p_037;
    tmp64  *= (uint64_t)pdata->VL53LX_p_009;
    tmp64  *= (uint64_t)pdata->VL53LX_p_004;
    tmp64  += (1 << (11 + 7));
    tmp64 >>= (11 + 8);







    tmp64  *= ((uint64_t)target_reflectance *
               (uint64_t)pcal->coverglass_transmission);

    tmp64  += (((uint64_t)pcal->ref_reflectance_pc * 256) / 2);
    tmp64  /= ((uint64_t)pcal->ref_reflectance_pc * 256);

    tmp64  +=  500;
    tmp64  /= 1000;



    if (tmp64 > 0x00FFFFFF) {
      tmp64 = 0x00FFFFFF;
    }

    pdata->VL53LX_p_035 = (uint32_t)tmp64;















    tmp32  = VL53LX_isqrt(pdata->VL53LX_p_028 << 8);
    tmp32 *= (uint32_t)pcfg->ambient_thresh_sigma;







    if (pdata->VL53LX_p_028 <
        (uint32_t)pcfg->min_ambient_thresh_events) {

      amb_thres_delta =
        pcfg->min_ambient_thresh_events -
        (uint32_t)pdata->VL53LX_p_028;



      amb_thres_delta <<= 8;

      if (tmp32 < amb_thres_delta) {
        tmp32 = amb_thres_delta;
      }
    }




    pdata->VL53LX_p_022 =
      (int16_t)VL53LX_f_002(
        tmp32,

        pdata->VL53LX_p_035,
        (uint32_t)pcal->ref__distance_mm,
        (uint32_t)pcfg->signal_thresh_sigma);








    tmp32  = (uint32_t)pdata->VL53LX_p_035;
    tmp32 *= (uint32_t)pbins->vcsel_width;
    tmp32 += (1 << 3);
    tmp32 /= (1 << 4);

    pdata->VL53LX_p_036 =
      (int16_t)VL53LX_f_002(
        256 * (uint32_t)pcfg->signal_total_events_limit,
        tmp32,
        (uint32_t)pcal->ref__distance_mm,
        (uint32_t)pcfg->signal_thresh_sigma);







    if (pdata->VL53LX_p_036 < pdata->VL53LX_p_022) {
      *pambient_dmax_mm = pdata->VL53LX_p_036;
    } else {
      *pambient_dmax_mm = pdata->VL53LX_p_022;
    }

  }

  return status;

}


uint32_t VL53LX::VL53LX_f_002(
  uint32_t     events_threshold,
  uint32_t     ref_signal_events,
  uint32_t   ref_distance_mm,
  uint32_t     signal_thresh_sigma)
{

  uint32_t    tmp32               = 0;
  uint32_t    range_mm            = 0;

  tmp32 = 4 * events_threshold;






  tmp32 += ((uint32_t)signal_thresh_sigma *
            (uint32_t)signal_thresh_sigma);






  tmp32  = VL53LX_isqrt(tmp32);
  tmp32 += (uint32_t)signal_thresh_sigma;







  range_mm =
    (uint32_t)VL53LX_isqrt(ref_signal_events << 4);
  range_mm *= ref_distance_mm;

  range_mm += (tmp32);
  range_mm /= (2 * tmp32);

  return range_mm;

}

/* vl53lx_api_calibration.c */


VL53LX_Error VL53LX::VL53LX_run_ref_spad_char(
  VL53LX_Error     *pcal_status)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t comms_buffer[6];

  VL53LX_refspadchar_config_t *prefspadchar  = &(pdev->refspadchar);


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_powerforce();
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_ref_spad_char_config(
        prefspadchar->VL53LX_p_005,
        prefspadchar->timeout_us,
        prefspadchar->target_count_rate_mcps,
        prefspadchar->max_count_rate_limit_mcps,
        prefspadchar->min_count_rate_limit_mcps,
        pdev->stat_nvm.osc_measured__fast_osc__frequency);



  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_run_device_test(
               prefspadchar->device_test_mode);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_ReadMulti(
        Dev,
        VL53LX_REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS,
        comms_buffer,
        2);

  if (status == VL53LX_ERROR_NONE) {
    pdev->dbg_results.ref_spad_char_result__num_actual_ref_spads =
      comms_buffer[0];
    pdev->dbg_results.ref_spad_char_result__ref_location =
      comms_buffer[1];
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WriteMulti(
        Dev,
        VL53LX_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS,
        comms_buffer,
        2);

  if (status == VL53LX_ERROR_NONE) {
    pdev->customer.ref_spad_man__num_requested_ref_spads =
      comms_buffer[0];
    pdev->customer.ref_spad_man__ref_location =
      comms_buffer[1];
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_ReadMulti(
        Dev,
        VL53LX_RESULT__SPARE_0_SD1,
        comms_buffer,
        6);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_WriteMulti(
        Dev,
        VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
        comms_buffer,
        6);

  if (status == VL53LX_ERROR_NONE) {
    pdev->customer.global_config__spad_enables_ref_0 =
      comms_buffer[0];
    pdev->customer.global_config__spad_enables_ref_1 =
      comms_buffer[1];
    pdev->customer.global_config__spad_enables_ref_2 =
      comms_buffer[2];
    pdev->customer.global_config__spad_enables_ref_3 =
      comms_buffer[3];
    pdev->customer.global_config__spad_enables_ref_4 =
      comms_buffer[4];
    pdev->customer.global_config__spad_enables_ref_5 =
      comms_buffer[5];
  }
  /*
      if (status == VL53LX_ERROR_NONE)
      VL53LX_print_customer_nvm_managed(
        &(pdev->customer),
        "run_ref_spad_char():pdev->lldata.customer.",
        VL53LX_TRACE_MODULE_REF_SPAD_CHAR);
  */
  if (status == VL53LX_ERROR_NONE) {

    switch (pdev->sys_results.result__range_status) {

      case VL53LX_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS:
        status = VL53LX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS;
        break;

      case VL53LX_DEVICEERROR_REFSPADCHARMORETHANTARGET:
        status = VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH;
        break;

      case VL53LX_DEVICEERROR_REFSPADCHARLESSTHANTARGET:
        status = VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW;
        break;
    }
  }



  *pcal_status = status;


  /*
    IGNORE_STATUS(
      IGNORE_REF_SPAD_CHAR_NOT_ENOUGH_SPADS,
      VL53LX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS,
      status);

    IGNORE_STATUS(
      IGNORE_REF_SPAD_CHAR_RATE_TOO_HIGH,
      VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH,
      status);

    IGNORE_STATUS(
      IGNORE_REF_SPAD_CHAR_RATE_TOO_LOW,
      VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW,
      status);

  */

  return status;
}


VL53LX_Error VL53LX::VL53LX_run_xtalk_extraction(
  VL53LX_Error                       *pcal_status)
{


  VL53LX_Error status        = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);



  VL53LX_xtalkextract_config_t *pX = &(pdev->xtalk_extract_cfg);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
  VL53LX_xtalk_calibration_results_t *pXC = &(pdev->xtalk_cal);

  uint8_t results_invalid  = 0;

  uint8_t i                = 0;
  uint16_t tmp16 = 0;

  uint8_t measurement_mode = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;

  VL53LX_init_histogram_bin_data_struct(
    0,
    (uint16_t)VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(pdev->xtalk_results.central_histogram_avg));

  VL53LX_init_histogram_bin_data_struct(
    0,
    (uint16_t)VL53LX_HISTOGRAM_BUFFER_SIZE,
    &(pdev->xtalk_results.central_histogram_sum));



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_preset_mode(
        VL53LX_DEVICEPRESETMODE_HISTOGRAM_XTALK_PLANAR,

        pX->dss_config__target_total_rate_mcps,
        pX->phasecal_config_timeout_us,
        pX->mm_config_timeout_us,
        pX->range_config_timeout_us,

        100);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_xtalk_compensation();
  }



  pdev->xtalk_results.max_results    = VL53LX_MAX_XTALK_RANGE_RESULTS;
  pdev->xtalk_results.active_results = pdev->zone_cfg.active_zones + 1;



  pdev->xtalk_results.central_histogram__window_start = 0xFF;
  pdev->xtalk_results.central_histogram__window_end   = 0x00;

  pdev->xtalk_results.num_of_samples_status = 0x00;
  pdev->xtalk_results.zero_samples_status   = 0x00;
  pdev->xtalk_results.max_sigma_status      = 0x00;

  for (i = 0; i < pdev->xtalk_results.max_results; i++) {
    pdev->xtalk_results.VL53LX_p_003[i].no_of_samples           = 0;
    pdev->xtalk_results.VL53LX_p_003[i].signal_total_events_avg = 0;
    pdev->xtalk_results.VL53LX_p_003[i].signal_total_events_sum = 0;
    pdev->xtalk_results.VL53LX_p_003[i].rate_per_spad_kcps_sum  = 0;
    pdev->xtalk_results.VL53LX_p_003[i].rate_per_spad_kcps_avg  = 0;
    pdev->xtalk_results.VL53LX_p_003[i].sigma_mm_sum            = 0;
    pdev->xtalk_results.VL53LX_p_003[i].sigma_mm_avg            = 0;

    pdev->xtalk_results.VL53LX_p_003[i].median_phase_sum        = 0;
    pdev->xtalk_results.VL53LX_p_003[i].median_phase_avg        = 0;

  }


  if (status == VL53LX_ERROR_NONE) {

    status =
      VL53LX_get_and_avg_xtalk_samples(
        pX->num_of_samples,

        measurement_mode,

        pX->algo__crosstalk_extract_max_valid_range_mm,
        pX->algo__crosstalk_extract_min_valid_range_mm,
        pX->algo__crosstalk_extract_max_valid_rate_kcps,

        0x0,
        0x4,
        &(pdev->xtalk_results),
        &(pdev->xtalk_results.central_histogram_sum),
        &(pdev->xtalk_results.central_histogram_avg));
  }







  if (status == VL53LX_ERROR_NONE)
    if ((pdev->xtalk_results.VL53LX_p_003[4].no_of_samples == 0) ||
        (pdev->xtalk_results.VL53LX_p_003[4].sigma_mm_avg >
         ((uint32_t)pX->algo__crosstalk_extract_max_sigma_mm
          << 5))) {
      results_invalid = 0x01;
    }

  /*
      if (status == VL53LX_ERROR_NONE)
      VL53LX_print_xtalk_range_results(
        &(pdev->xtalk_results),
        "pdev->xtalk_results",
        VL53LX_TRACE_MODULE_CORE);
  */
  if ((status == VL53LX_ERROR_NONE) && (results_invalid == 0)) {
    status =
      VL53LX_ipp_xtalk_calibration_process_data(
        &(pdev->xtalk_results),
        &(pdev->xtalk_shapes),
        &(pdev->xtalk_cal));
  }
  if ((status == VL53LX_ERROR_NONE) && (results_invalid == 0)) {
    for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
      pXC->algo__xtalk_cpo_HistoMerge_kcps[i] =
        pXC->algo__crosstalk_compensation_plane_offset_kcps;
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
      pXC->algo__crosstalk_compensation_x_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
      pXC->algo__crosstalk_compensation_y_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_plane_offset_kcps =
      pXC->algo__crosstalk_compensation_plane_offset_kcps;
  }

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_xtalk_compensation();
  }

  if (status == VL53LX_ERROR_NONE) {
    for (i = 0; i < pdev->xtalk_results.max_results; i++) {

      if (pdev->xtalk_results.VL53LX_p_003[i].no_of_samples !=

          pX->num_of_samples) {

        pdev->xtalk_results.num_of_samples_status =
          pdev->xtalk_results.num_of_samples_status |
          (1 << i);
      }

      if (pdev->xtalk_results.VL53LX_p_003[i].no_of_samples ==
          0x00) {
        pdev->xtalk_results.zero_samples_status =
          pdev->xtalk_results.zero_samples_status |
          (1 << i);
      }




      tmp16 = pX->algo__crosstalk_extract_max_sigma_mm;
      if (pdev->xtalk_results.VL53LX_p_003[i].sigma_mm_avg >
          ((uint32_t)tmp16 << 5)) {
        pdev->xtalk_results.max_sigma_status =
          pdev->xtalk_results.max_sigma_status |
          (1 << i);
      }

    }
  }


  if (results_invalid > 0) {

    if (pdev->xtalk_results.VL53LX_p_003[4].no_of_samples == 0) {
      status = VL53LX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL;
    } else {


      if (pdev->xtalk_results.VL53LX_p_003[4].sigma_mm_avg >
          (((uint32_t)pX->algo__crosstalk_extract_max_sigma_mm)
           << 5)) {
        status =
          VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL;
      }

    }
  } else {

    if (pdev->xtalk_results.zero_samples_status != 0x00) {
      status = VL53LX_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT;
    } else {
      if (pdev->xtalk_results.max_sigma_status != 0x00) {
        status =
          VL53LX_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT;
      } else {
        if (pdev->xtalk_results.num_of_samples_status !=
            0x00)
          status =
            VL53LX_WARNING_XTALK_MISSING_SAMPLES;
      }
    }
  }



  pdev->xtalk_results.cal_status = status;
  *pcal_status = pdev->xtalk_results.cal_status;



  /*IGNORE_STATUS(
    IGNORE_XTALK_EXTRACTION_NO_SAMPLE_FAIL,
    VL53LX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL,
    status);

  IGNORE_STATUS(
    IGNORE_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL,
    VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL,
    status);

  IGNORE_STATUS(
    IGNORE_XTALK_EXTRACTION_NO_SAMPLE_FOR_GRADIENT_WARN,
    VL53LX_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT,
    status);

  IGNORE_STATUS(
    IGNORE_XTALK_EXTRACTION_SIGMA_LIMIT_FOR_GRADIENT_WARN,
    VL53LX_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT,
    status);

  IGNORE_STATUS(
    IGNORE_XTALK_EXTRACTION_MISSING_SAMPLES_WARN,
    VL53LX_WARNING_XTALK_MISSING_SAMPLES,
    status);
    */
  /*

    VL53LX_print_customer_nvm_managed(
      &(pdev->customer),
      "run_xtalk_extraction():pdev->lldata.customer.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_xtalk_config(
      &(pdev->xtalk_cfg),
      "run_xtalk_extraction():pdev->lldata.xtalk_cfg.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_xtalk_extract_config(
      &(pdev->xtalk_extract_cfg),
      "run_xtalk_extraction():pdev->lldata.xtalk_extract_cfg.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_histogram_bin_data(
      &(pdev->hist_data),
      "run_xtalk_extraction():pdev->lldata.hist_data.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_xtalk_histogram_data(
      &(pdev->xtalk_shapes),
      "pdev->lldata.xtalk_shapes.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_xtalk_range_results(
      &(pdev->xtalk_results),
      "run_xtalk_extraction():pdev->lldata.xtalk_results.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

  #endif
  */

  return status;

}


VL53LX_Error VL53LX::VL53LX_get_and_avg_xtalk_samples(
  uint8_t                       num_of_samples,
  uint8_t                       measurement_mode,
  int16_t                       xtalk_filter_thresh_max_mm,
  int16_t                       xtalk_filter_thresh_min_mm,
  uint16_t                      xtalk_max_valid_rate_kcps,
  uint8_t                       xtalk_result_id,
  uint8_t                       xtalk_histo_id,
  VL53LX_xtalk_range_results_t *pXR,
  VL53LX_histogram_bin_data_t  *psum_histo,
  VL53LX_histogram_bin_data_t  *pavg_histo)
{



  VL53LX_Error status        = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);


  VL53LX_range_results_t      *prs =
    (VL53LX_range_results_t *) pdev->wArea1;

  VL53LX_range_data_t         *prange_data;
  VL53LX_xtalk_range_data_t   *pxtalk_range_data;

  uint8_t i                = 0;
  uint8_t j                = 0;
  uint8_t zone_id          = 0;
  uint8_t final_zone       = pdev->zone_cfg.active_zones + 1;
  uint8_t valid_result;

  uint8_t smudge_corr_en   = 0;




  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;

  status = VL53LX_dynamic_xtalk_correction_disable();


  VL53LX_load_patch();



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_init_and_start_range(
        measurement_mode,
        VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);


  for (i = 0; i <= (final_zone * num_of_samples); i++) {



    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_wait_for_range_completion();
    }



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_get_device_results(
          VL53LX_DEVICERESULTSLEVEL_FULL,
          prs);



    if (status == VL53LX_ERROR_NONE &&
        pdev->ll_state.rd_device_state !=
        VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {

      zone_id = pdev->ll_state.rd_zone_id + xtalk_result_id;
      prange_data       = &(prs->VL53LX_p_003[0]);


      if (prs->active_results > 1) {
        for (j = 1;
             j < prs->active_results; j++) {
          if (prs->VL53LX_p_003[j].median_range_mm
              <
              prange_data->median_range_mm)
            prange_data =
              &(prs->VL53LX_p_003[j]);

        }
      }

      pxtalk_range_data = &(pXR->VL53LX_p_003[zone_id]);



      if ((prs->active_results > 0) &&
          (prange_data->median_range_mm <
           xtalk_filter_thresh_max_mm) &&
          (prange_data->median_range_mm >
           xtalk_filter_thresh_min_mm) &&
          (prange_data->VL53LX_p_009 <
           (uint32_t)(xtalk_max_valid_rate_kcps * 16))) {
        valid_result = 1;
      } else {
        valid_result = 0;
      }

      if (valid_result == 1) {

        pxtalk_range_data->no_of_samples++;

        pxtalk_range_data->rate_per_spad_kcps_sum +=
          prange_data->VL53LX_p_009;

        pxtalk_range_data->signal_total_events_sum +=
          prange_data->VL53LX_p_010;

        pxtalk_range_data->sigma_mm_sum +=
          (uint32_t)prange_data->VL53LX_p_002;



        pxtalk_range_data->median_phase_sum +=
          (uint32_t)prange_data->VL53LX_p_011;




      }

      if ((valid_result == 1) && (zone_id >= 4)) {
        status = VL53LX_sum_histogram_data(
                   &(pdev->hist_data),
                   psum_histo);



        if (prange_data->VL53LX_p_012 <
            pXR->central_histogram__window_start)
          pXR->central_histogram__window_start =
            prange_data->VL53LX_p_012;


        if (prange_data->VL53LX_p_013 >
            pXR->central_histogram__window_end)
          pXR->central_histogram__window_end =
            prange_data->VL53LX_p_013;

      }

    }


    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_wait_for_firmware_ready();
    }



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_clear_interrupt_and_enable_next_range(
          measurement_mode);


  }




  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_stop_range();
  }

  VL53LX_unload_patch();



  for (i = 0; i < (pdev->zone_cfg.active_zones + 1); i++) {

    pxtalk_range_data = &(pXR->VL53LX_p_003[i + xtalk_result_id]);

    if (pxtalk_range_data->no_of_samples > 0) {
      pxtalk_range_data->rate_per_spad_kcps_avg =
        pxtalk_range_data->rate_per_spad_kcps_sum /
        (uint32_t)pxtalk_range_data->no_of_samples;

      pxtalk_range_data->signal_total_events_avg =
        pxtalk_range_data->signal_total_events_sum /
        (int32_t)pxtalk_range_data->no_of_samples;

      pxtalk_range_data->sigma_mm_avg =
        pxtalk_range_data->sigma_mm_sum /
        (uint32_t)pxtalk_range_data->no_of_samples;



      pxtalk_range_data->median_phase_avg =
        pxtalk_range_data->median_phase_sum /
        (uint32_t)pxtalk_range_data->no_of_samples;



    } else {
      pxtalk_range_data->rate_per_spad_kcps_avg =
        pxtalk_range_data->rate_per_spad_kcps_sum;
      pxtalk_range_data->signal_total_events_avg =
        pxtalk_range_data->signal_total_events_sum;
      pxtalk_range_data->sigma_mm_avg =
        pxtalk_range_data->sigma_mm_sum;



      pxtalk_range_data->median_phase_avg =
        pxtalk_range_data->median_phase_sum;


    }
  }



  memcpy(pavg_histo, &(pdev->hist_data),
         sizeof(VL53LX_histogram_bin_data_t));



  if (status == VL53LX_ERROR_NONE) {

    pxtalk_range_data = &(pXR->VL53LX_p_003[xtalk_histo_id]);

    status = VL53LX_avg_histogram_data(
               pxtalk_range_data->no_of_samples,
               psum_histo,
               pavg_histo);
  }




  if (status == VL53LX_ERROR_NONE) {
    if (smudge_corr_en == 1) {
      status = VL53LX_dynamic_xtalk_correction_enable();
    }
  }


  return status;

}


VL53LX_Error VL53LX::VL53LX_run_offset_calibration(
  int16_t                       cal_distance_mm,
  uint16_t                      cal_reflectance_pc,
  VL53LX_Error                 *pcal_status)
{


  VL53LX_Error status        = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_DevicePresetModes device_preset_modes[
   VL53LX_MAX_OFFSET_RANGE_RESULTS];

  VL53LX_range_results_t     *prange_results =
    (VL53LX_range_results_t *) pdev->wArea1;

  VL53LX_range_data_t        *pRData = NULL;
  VL53LX_offset_range_data_t *pfs     = NULL;
  VL53LX_general_config_t *pG = &(pdev->gen_cfg);
  VL53LX_additional_offset_cal_data_t *pAO = &(pdev->add_off_cal_data);

  uint8_t  i                      = 0;
  uint8_t  m                      = 0;
  uint8_t  measurement_mode       =
    VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  uint16_t manual_effective_spads =
    pG->dss_config__manual_effective_spads_select;

  uint8_t num_of_samples[VL53LX_MAX_OFFSET_RANGE_RESULTS];

  uint8_t smudge_corr_en   = 0;




  switch (pdev->offset_calibration_mode) {

    case VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__HISTOGRAM:
    case VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__HISTOGRAM_PRE_RANGE_ONLY:
      device_preset_modes[0] =
        VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING;
      device_preset_modes[1] =
        VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_MM1_CAL;
      device_preset_modes[2] =
        VL53LX_DEVICEPRESETMODE_HISTOGRAM_RANGING_MM2_CAL;
      break;

    default:
      device_preset_modes[0] =
        VL53LX_DEVICEPRESETMODE_STANDARD_RANGING;
      device_preset_modes[1] =
        VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL;
      device_preset_modes[2] =
        VL53LX_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL;
      break;
  }



  num_of_samples[0] = pdev->offsetcal_cfg.pre_num_of_samples;
  num_of_samples[1] = pdev->offsetcal_cfg.mm1_num_of_samples;
  num_of_samples[2] = pdev->offsetcal_cfg.mm2_num_of_samples;




  switch (pdev->offset_calibration_mode) {

    case VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY:
    case VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__HISTOGRAM_PRE_RANGE_ONLY:

      pdev->offset_results.active_results  = 1;

      break;

    default:

      pdev->customer.mm_config__inner_offset_mm  = 0;
      pdev->customer.mm_config__outer_offset_mm  = 0;
      pdev->offset_results.active_results  =
        VL53LX_MAX_OFFSET_RANGE_RESULTS;

      break;
  }

  pdev->customer.algo__part_to_part_range_offset_mm = 0;



  pdev->offset_results.max_results   = VL53LX_MAX_OFFSET_RANGE_RESULTS;
  pdev->offset_results.cal_distance_mm       = cal_distance_mm;
  pdev->offset_results.cal_reflectance_pc    = cal_reflectance_pc;

  for (m = 0; m <  VL53LX_MAX_OFFSET_RANGE_RESULTS; m++) {

    pfs = &(pdev->offset_results.VL53LX_p_003[m]);
    pfs->preset_mode         = 0;
    pfs->no_of_samples       = 0;
    pfs->effective_spads     = 0;
    pfs->peak_rate_mcps      = 0;
    pfs->VL53LX_p_002            = 0;
    pfs->median_range_mm     = 0;
  }




  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;

  status = VL53LX_dynamic_xtalk_correction_disable();



  for (m = 0; m < pdev->offset_results.active_results; m++) {

    pfs = &(pdev->offset_results.VL53LX_p_003[m]);

    pfs->preset_mode         = device_preset_modes[m];



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_set_preset_mode(
          device_preset_modes[m],

          pdev->offsetcal_cfg.dss_config__target_total_rate_mcps,
          pdev->offsetcal_cfg.phasecal_config_timeout_us,
          pdev->offsetcal_cfg.mm_config_timeout_us,
          pdev->offsetcal_cfg.range_config_timeout_us,

          100);

    pG->dss_config__manual_effective_spads_select =
      manual_effective_spads;


    VL53LX_load_patch();

    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_init_and_start_range(
          measurement_mode,
          VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);

    for (i = 0; i <= (num_of_samples[m] + 2); i++) {



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_wait_for_range_completion();



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_get_device_results(
            VL53LX_DEVICERESULTSLEVEL_FULL,
            prange_results);



      pRData  = &(prange_results->VL53LX_p_003[0]);

      if ((prange_results->active_results > 0 &&
           prange_results->stream_count   > 1) &&
          (pRData->range_status ==
           VL53LX_DEVICEERROR_RANGECOMPLETE)) {

        pfs->no_of_samples++;
        pfs->effective_spads +=
          (uint32_t)pRData->VL53LX_p_004;
        pfs->peak_rate_mcps  +=
          (uint32_t)pRData->peak_signal_count_rate_mcps;
        pfs->VL53LX_p_002        +=
          (uint32_t)pRData->VL53LX_p_002;
        pfs->median_range_mm +=
          (int32_t)pRData->median_range_mm;

        pfs->dss_config__roi_mode_control =
          pG->dss_config__roi_mode_control;
        pfs->dss_config__manual_effective_spads_select =
          pG->dss_config__manual_effective_spads_select;

      }



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_wait_for_firmware_ready();



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_clear_interrupt_and_enable_next_range(
            measurement_mode);
    }



    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_stop_range();
    }



    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_WaitUs(Dev, 1000);
    }
    VL53LX_unload_patch();


    if (pfs->no_of_samples > 0) {

      pfs->effective_spads += (pfs->no_of_samples / 2);
      pfs->effective_spads /= pfs->no_of_samples;

      pfs->peak_rate_mcps  += (pfs->no_of_samples / 2);
      pfs->peak_rate_mcps  /= pfs->no_of_samples;

      pfs->VL53LX_p_002        += (pfs->no_of_samples / 2);
      pfs->VL53LX_p_002        /= pfs->no_of_samples;

      pfs->median_range_mm += (pfs->no_of_samples / 2);
      pfs->median_range_mm /= pfs->no_of_samples;

      pfs->range_mm_offset  = (int32_t)cal_distance_mm;
      pfs->range_mm_offset -= pfs->median_range_mm;


      if (pfs->preset_mode ==
          VL53LX_DEVICEPRESETMODE_STANDARD_RANGING)
        manual_effective_spads =
          (uint16_t)pfs->effective_spads;
    }
  }



  switch (pdev->offset_calibration_mode) {

    case VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY:
    case VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__HISTOGRAM_PRE_RANGE_ONLY:


      pdev->customer.mm_config__inner_offset_mm +=
        (int16_t)pdev->offset_results.VL53LX_p_003[0].range_mm_offset;
      pdev->customer.mm_config__outer_offset_mm +=
        (int16_t)pdev->offset_results.VL53LX_p_003[0].range_mm_offset;
      break;

    default:

      pdev->customer.mm_config__inner_offset_mm =
        (int16_t)pdev->offset_results.VL53LX_p_003[1].range_mm_offset;
      pdev->customer.mm_config__outer_offset_mm =
        (int16_t)pdev->offset_results.VL53LX_p_003[2].range_mm_offset;
      pdev->customer.algo__part_to_part_range_offset_mm = 0;



      pAO->result__mm_inner_actual_effective_spads =
        (uint16_t)pdev->offset_results.VL53LX_p_003[1].effective_spads;
      pAO->result__mm_outer_actual_effective_spads =
        (uint16_t)pdev->offset_results.VL53LX_p_003[2].effective_spads;

      pAO->result__mm_inner_peak_signal_count_rtn_mcps =
        (uint16_t)pdev->offset_results.VL53LX_p_003[1].peak_rate_mcps;
      pAO->result__mm_outer_peak_signal_count_rtn_mcps =
        (uint16_t)pdev->offset_results.VL53LX_p_003[2].peak_rate_mcps;

      break;
  }



  pdev->cust_dmax_cal.ref__actual_effective_spads =
    (uint16_t)pdev->offset_results.VL53LX_p_003[0].effective_spads;
  pdev->cust_dmax_cal.ref__peak_signal_count_rate_mcps =
    (uint16_t)pdev->offset_results.VL53LX_p_003[0].peak_rate_mcps;


  pdev->cust_dmax_cal.ref__distance_mm = cal_distance_mm * 16;

  pdev->cust_dmax_cal.ref_reflectance_pc = cal_reflectance_pc;
  pdev->cust_dmax_cal.coverglass_transmission = 0x0100;



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_customer_nvm_managed(
        &(pdev->customer));




  if (status == VL53LX_ERROR_NONE) {
    if (smudge_corr_en == 1) {
      status = VL53LX_dynamic_xtalk_correction_enable();
    }
  }




  for (m = 0; m < pdev->offset_results.active_results; m++) {

    pfs = &(pdev->offset_results.VL53LX_p_003[m]);

    if (status == VL53LX_ERROR_NONE) {

      pdev->offset_results.cal_report = m;

      if (pfs->no_of_samples < num_of_samples[m])
        status =
          VL53LX_WARNING_OFFSET_CAL_MISSING_SAMPLES;


      if (m == 0 && pfs->VL53LX_p_002 >
          ((uint32_t)VL53LX_OFFSET_CAL_MAX_SIGMA_MM << 5))
        status =
          VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;

      if (pfs->peak_rate_mcps >
          VL53LX_OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS)
        status =
          VL53LX_WARNING_OFFSET_CAL_RATE_TOO_HIGH;

      if (pfs->dss_config__manual_effective_spads_select <
          VL53LX_OFFSET_CAL_MIN_EFFECTIVE_SPADS)
        status =
          VL53LX_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW;

      if (pfs->dss_config__manual_effective_spads_select == 0)
        status =
          VL53LX_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL;

      if (pfs->no_of_samples == 0) {
        status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
      }
    }
  }



  pdev->offset_results.cal_status = status;
  *pcal_status = pdev->offset_results.cal_status;


  /*
    IGNORE_STATUS(
      IGNORE_OFFSET_CAL_MISSING_SAMPLES,
      VL53LX_WARNING_OFFSET_CAL_MISSING_SAMPLES,
      status);

    IGNORE_STATUS(
      IGNORE_OFFSET_CAL_SIGMA_TOO_HIGH,
      VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH,
      status);

    IGNORE_STATUS(
      IGNORE_OFFSET_CAL_RATE_TOO_HIGH,
      VL53LX_WARNING_OFFSET_CAL_RATE_TOO_HIGH,
      status);

    IGNORE_STATUS(
      IGNORE_OFFSET_CAL_SPAD_COUNT_TOO_LOW,
      VL53LX_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW,
      status);
  */

  /*
  #ifdef VL53LX_LOG_ENABLE



    VL53LX_print_customer_nvm_managed(
      &(pdev->customer),
      "run_offset_calibration():pdev->lldata.customer.",
      VL53LX_TRACE_MODULE_OFFSET_DATA);

    VL53LX_print_dmax_calibration_data(
      &(pdev->fmt_dmax_cal),
      "run_offset_calibration():pdev->lldata.fmt_dmax_cal.",
      VL53LX_TRACE_MODULE_OFFSET_DATA);

    VL53LX_print_dmax_calibration_data(
      &(pdev->cust_dmax_cal),
      "run_offset_calibration():pdev->lldata.cust_dmax_cal.",
      VL53LX_TRACE_MODULE_OFFSET_DATA);

    VL53LX_print_additional_offset_cal_data(
      &(pdev->add_off_cal_data),
      "run_offset_calibration():pdev->lldata.add_off_cal_data.",
      VL53LX_TRACE_MODULE_OFFSET_DATA);

    VL53LX_print_offset_range_results(
      &(pdev->offset_results),
      "run_offset_calibration():pdev->lldata.offset_results.",
      VL53LX_TRACE_MODULE_OFFSET_DATA);
  #endif
  */

  return status;
}


VL53LX_Error VL53LX::VL53LX_run_phasecal_average(
  uint8_t                 measurement_mode,
  uint8_t                 phasecal_result__vcsel_start,
  uint16_t                phasecal_num_of_samples,
  VL53LX_range_results_t *prange_results,
  uint16_t               *pphasecal_result__reference_phase,
  uint16_t               *pzero_distance_phase)
{


  VL53LX_Error status        = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  uint16_t  i                                = 0;
  uint16_t  m                                = 0;
  uint32_t  samples                          = 0;

  uint32_t  period                           = 0;
  uint32_t  VL53LX_p_014                            = 0;
  uint32_t  phasecal_result__reference_phase = 0;
  uint32_t  zero_distance_phase              = 0;


  VL53LX_load_patch();

  for (m = 0; m < phasecal_num_of_samples; m++) {



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_init_and_start_range(
          measurement_mode,
          VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);

    for (i = 0; i <= 1; i++) {



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_wait_for_range_completion();



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_get_device_results(
            VL53LX_DEVICERESULTSLEVEL_FULL,
            prange_results);



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_wait_for_firmware_ready();



      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_clear_interrupt_and_enable_next_range(
            measurement_mode);
    }



    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_stop_range();
    }



    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_WaitUs(Dev, 1000);
    }



    if (status == VL53LX_ERROR_NONE) {

      samples++;


      period = 2048 *
               (uint32_t)VL53LX_decode_vcsel_period(
                 pdev->hist_data.VL53LX_p_005);

      VL53LX_p_014  = period;
      VL53LX_p_014 += (uint32_t)(
                        pdev->hist_data.phasecal_result__reference_phase);
      VL53LX_p_014 +=
        (2048 *
         (uint32_t)phasecal_result__vcsel_start);
      VL53LX_p_014 -= (2048 *
                       (uint32_t)pdev->hist_data.cal_config__vcsel_start);

      VL53LX_p_014  = VL53LX_p_014 % period;

      phasecal_result__reference_phase += (uint32_t)(
                                            pdev->hist_data.phasecal_result__reference_phase);

      zero_distance_phase += (uint32_t)VL53LX_p_014;
    }
  }
  VL53LX_unload_patch();



  if (status == VL53LX_ERROR_NONE && samples > 0) {

    phasecal_result__reference_phase += (samples >> 1);
    phasecal_result__reference_phase /= samples;

    zero_distance_phase += (samples >> 1);
    zero_distance_phase /= samples;

    *pphasecal_result__reference_phase =
      (uint16_t)phasecal_result__reference_phase;
    *pzero_distance_phase =
      (uint16_t)zero_distance_phase;
  }

  return status;
}

VL53LX_Error VL53LX::VL53LX_run_zone_calibration(
  VL53LX_DevicePresetModes      device_preset_mode,
  VL53LX_DeviceZonePreset       zone_preset,
  VL53LX_zone_config_t         *pzone_cfg,
  int16_t                       cal_distance_mm,
  uint16_t                      cal_reflectance_pc,
  VL53LX_Error                 *pcal_status)
{


  VL53LX_Error status        = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  VL53LX_LLDriverResults_t *pres =
    VL53LXDevStructGetLLResultsHandle(Dev);

  VL53LX_range_results_t         *pRR =
    (VL53LX_range_results_t *) pdev->wArea1;
  VL53LX_range_data_t            *prange_data = NULL;
  VL53LX_zone_calibration_data_t *pzone_data  = NULL;

  uint16_t  i                      = 0;
  uint16_t  m                      = 0;

  uint8_t   z                      = 0;
  uint8_t   measurement_mode       =
    VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;

  VL53LX_OffsetCorrectionMode  offset_cor_mode =
    VL53LX_OFFSETCORRECTIONMODE__NONE;

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_preset_mode(
        device_preset_mode,

        pdev->zonecal_cfg.dss_config__target_total_rate_mcps,
        pdev->zonecal_cfg.phasecal_config_timeout_us,
        pdev->zonecal_cfg.mm_config_timeout_us,
        pdev->zonecal_cfg.range_config_timeout_us,

        100);



  if (zone_preset == VL53LX_DEVICEZONEPRESET_CUSTOM) {

    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_set_zone_config(
          pzone_cfg);

  } else if (zone_preset != VL53LX_DEVICEZONEPRESET_NONE) {

    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_set_zone_preset(
          zone_preset);
  }



  pres->zone_cal.preset_mode        = device_preset_mode;
  pres->zone_cal.zone_preset        = zone_preset;

  pres->zone_cal.cal_distance_mm    = cal_distance_mm * 16;
  pres->zone_cal.cal_reflectance_pc = cal_reflectance_pc;
  pres->zone_cal.max_zones          = VL53LX_MAX_USER_ZONES;
  pres->zone_cal.active_zones       = pdev->zone_cfg.active_zones + 1;

  for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
    pres->zone_cal.VL53LX_p_003[i].no_of_samples   = 0;
    pres->zone_cal.VL53LX_p_003[i].effective_spads = 0;
    pres->zone_cal.VL53LX_p_003[i].peak_rate_mcps  = 0;
    pres->zone_cal.VL53LX_p_003[i].VL53LX_p_011    = 0;
    pres->zone_cal.VL53LX_p_003[i].VL53LX_p_002        = 0;
    pres->zone_cal.VL53LX_p_003[i].median_range_mm = 0;
    pres->zone_cal.VL53LX_p_003[i].range_mm_offset = 0;
  }

  pres->zone_cal.phasecal_result__reference_phase = 0;
  pres->zone_cal.zero_distance_phase              = 0;



  status =
    VL53LX_get_offset_correction_mode(
      &offset_cor_mode);

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_offset_correction_mode(
        VL53LX_OFFSETCORRECTIONMODE__NONE);


  VL53LX_load_patch();

  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_init_and_start_range(
        measurement_mode,
        VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);




  m = (pdev->zonecal_cfg.zone_num_of_samples + 2) *
      (uint16_t)pres->zone_cal.active_zones;


  for (i = 0; i <= m; i++) {



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_wait_for_range_completion();



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_get_device_results(
          VL53LX_DEVICERESULTSLEVEL_FULL,
          pRR);



    prange_data  = &(pRR->VL53LX_p_003[0]);

    if (pRR->active_results > 0 &&
        i > (uint16_t)pres->zone_cal.active_zones) {

      if (prange_data->range_status ==
          VL53LX_DEVICEERROR_RANGECOMPLETE) {

        pres->zone_cal.phasecal_result__reference_phase
          =
            pdev->hist_data.phasecal_result__reference_phase
            ;
        pres->zone_cal.zero_distance_phase =
          pdev->hist_data.zero_distance_phase;

        pzone_data =
          &(pres->zone_cal.VL53LX_p_003[pRR->zone_id]);
        pzone_data->no_of_samples++;
        pzone_data->effective_spads +=
          (uint32_t)prange_data->VL53LX_p_004;
        pzone_data->peak_rate_mcps  += (uint32_t)(
                                         prange_data->peak_signal_count_rate_mcps);
        pzone_data->VL53LX_p_011  +=
          (uint32_t)prange_data->VL53LX_p_011;
        pzone_data->VL53LX_p_002        +=
          (uint32_t)prange_data->VL53LX_p_002;
        pzone_data->median_range_mm +=
          (int32_t)prange_data->median_range_mm;

      }
    }



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_wait_for_firmware_ready();



    if (status == VL53LX_ERROR_NONE)
      status =
        VL53LX_clear_interrupt_and_enable_next_range(
          measurement_mode);
  }



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_stop_range();
  }


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_WaitUs(Dev, 1000);
  }
  VL53LX_unload_patch();


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_run_phasecal_average(
        measurement_mode,
        pdev->hist_data.phasecal_result__vcsel_start,

        pdev->zonecal_cfg.phasecal_num_of_samples,

        pRR,
        &(pres->zone_cal.phasecal_result__reference_phase),
        &(pres->zone_cal.zero_distance_phase));



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_offset_correction_mode(
        offset_cor_mode);



  if (status == VL53LX_ERROR_NONE) {

    for (z = 0; z < pres->zone_cal.active_zones; z++) {

      pzone_data = &(pres->zone_cal.VL53LX_p_003[z]);


      if (pzone_data->no_of_samples > 0) {

        pzone_data->effective_spads +=
          (pzone_data->no_of_samples / 2);
        pzone_data->effective_spads /=
          pzone_data->no_of_samples;

        pzone_data->peak_rate_mcps  +=
          (pzone_data->no_of_samples / 2);
        pzone_data->peak_rate_mcps  /=
          pzone_data->no_of_samples;

        pzone_data->VL53LX_p_011    +=
          (pzone_data->no_of_samples / 2);
        pzone_data->VL53LX_p_011    /=
          pzone_data->no_of_samples;

        pzone_data->VL53LX_p_002        +=
          (pzone_data->no_of_samples / 2);
        pzone_data->VL53LX_p_002        /=
          pzone_data->no_of_samples;



        pzone_data->median_range_mm =
          VL53LX_range_maths(
            pdev->stat_nvm.osc_measured__fast_osc__frequency
            , (uint16_t)pzone_data->VL53LX_p_011,
            pres->zone_cal.zero_distance_phase,
            2,
            0x0800,
            0);

        pzone_data->range_mm_offset  =
          ((int32_t)cal_distance_mm) * 4;
        pzone_data->range_mm_offset -=
          pzone_data->median_range_mm;


        if (pzone_data->no_of_samples <
            pdev->zonecal_cfg.zone_num_of_samples)
          status =
            VL53LX_WARNING_ZONE_CAL_MISSING_SAMPLES;


        if (pzone_data->VL53LX_p_002 >
            ((uint32_t)VL53LX_ZONE_CAL_MAX_SIGMA_MM
             << 5))
          status =
            VL53LX_WARNING_ZONE_CAL_SIGMA_TOO_HIGH;

        if (pzone_data->peak_rate_mcps >
            VL53LX_ZONE_CAL_MAX_PRE_PEAK_RATE_MCPS)
          status =
            VL53LX_WARNING_ZONE_CAL_RATE_TOO_HIGH;

      } else {
        status = VL53LX_ERROR_ZONE_CAL_NO_SAMPLE_FAIL;
      }
    }
  }



  pres->zone_cal.cal_status = status;
  *pcal_status = pres->zone_cal.cal_status;

  /*

    IGNORE_STATUS(
      IGNORE_ZONE_CAL_MISSING_SAMPLES,
      VL53LX_WARNING_ZONE_CAL_MISSING_SAMPLES,
      status);

    IGNORE_STATUS(
      IGNORE_ZONE_CAL_SIGMA_TOO_HIGH,
      VL53LX_WARNING_ZONE_CAL_SIGMA_TOO_HIGH,
      status);

    IGNORE_STATUS(
      IGNORE_ZONE_CAL_RATE_TOO_HIGH,
      VL53LX_WARNING_ZONE_CAL_RATE_TOO_HIGH,
      status);
  */
  /*
  #ifdef VL53LX_LOG_ENABLE



    VL53LX_print_zone_calibration_results(
      &(pres->zone_cal),
      "run_zone_calibration():pdev->llresults.zone_cal.",
      VL53LX_TRACE_MODULE_OFFSET_DATA);

  #endif
  */


  return status;
}


VL53LX_Error VL53LX::VL53LX_run_spad_rate_map(
  VL53LX_DeviceTestMode      device_test_mode,
  VL53LX_DeviceSscArray      array_select,
  uint32_t                   ssc_config_timeout_us,
  VL53LX_spad_rate_data_t   *pspad_rate_data)
{



  VL53LX_Error status = VL53LX_ERROR_NONE;

  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_enable_powerforce();
  }



  if (status == VL53LX_ERROR_NONE) {
    pdev->ssc_cfg.array_select = array_select;
    pdev->ssc_cfg.timeout_us   = ssc_config_timeout_us;
    status =
      VL53LX_set_ssc_config(
        &(pdev->ssc_cfg),
        pdev->stat_nvm.osc_measured__fast_osc__frequency);
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_run_device_test(
        device_test_mode);



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_get_spad_rate_data(
        pspad_rate_data);

  if (device_test_mode == VL53LX_DEVICETESTMODE_LCR_VCSEL_ON) {
    pspad_rate_data->fractional_bits =  7;
  } else {
    pspad_rate_data->fractional_bits = 15;
  }



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_powerforce();
  }
  /*
  #ifdef VL53LX_LOG_ENABLE


    if (status == VL53LX_ERROR_NONE) {
      VL53LX_print_spad_rate_data(
        pspad_rate_data,
        "run_spad_rate_map():",
        VL53LX_TRACE_MODULE_SPAD_RATE_MAP);
      VL53LX_print_spad_rate_map(
        pspad_rate_data,
        "run_spad_rate_map():",
        VL53LX_TRACE_MODULE_SPAD_RATE_MAP);
    }
  #endif
  */

  return status;
}


VL53LX_Error VL53LX::VL53LX_run_device_test(
  VL53LX_DeviceTestMode  device_test_mode)
{


  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  uint8_t      comms_buffer[2];
  uint8_t      gpio_hv_mux__ctrl = 0;


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_RdByte(
        Dev,
        VL53LX_GPIO_HV_MUX__CTRL,
        &gpio_hv_mux__ctrl);

  if (status == VL53LX_ERROR_NONE) {
    pdev->stat_cfg.gpio_hv_mux__ctrl = gpio_hv_mux__ctrl;
  }


  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_start_test(
               device_test_mode);


  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_wait_for_test_completion();
  }


  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_ReadMulti(
        Dev,
        VL53LX_RESULT__RANGE_STATUS,
        comms_buffer,
        2);

  if (status == VL53LX_ERROR_NONE) {
    pdev->sys_results.result__range_status  = comms_buffer[0];
    pdev->sys_results.result__report_status = comms_buffer[1];
  }



  pdev->sys_results.result__range_status &=
    VL53LX_RANGE_STATUS__RANGE_STATUS_MASK;

  if (status == VL53LX_ERROR_NONE) {

    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_clear_interrupt();
    }
  }



  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_start_test(0x00);


  return status;
}

void VL53LX::VL53LX_hist_xtalk_extract_data_init(
  VL53LX_hist_xtalk_extract_data_t *pxtalk_data)
{


  int32_t lb = 0;

  pxtalk_data->sample_count             = 0U;
  pxtalk_data->pll_period_mm            = 0U;
  pxtalk_data->peak_duration_us_sum     = 0U;
  pxtalk_data->effective_spad_count_sum = 0U;
  pxtalk_data->zero_distance_phase_sum  = 0U;
  pxtalk_data->zero_distance_phase_avg  = 0U;
  pxtalk_data->event_scaler_sum         = 0U;
  pxtalk_data->event_scaler_avg         = 4096U;
  pxtalk_data->signal_events_sum        = 0;
  pxtalk_data->xtalk_rate_kcps_per_spad = 0U;
  pxtalk_data->VL53LX_p_012             = 0U;
  pxtalk_data->VL53LX_p_013               = 0U;
  pxtalk_data->target_start             = 0U;

  for (lb = 0; lb < VL53LX_XTALK_HISTO_BINS; lb++) {
    pxtalk_data->bin_data_sums[lb] = 0;
  }

}

VL53LX_Error VL53LX::VL53LX_hist_xtalk_extract_update(
  int16_t                             target_distance_mm,
  uint16_t                            target_width_oversize,
  VL53LX_histogram_bin_data_t        *phist_bins,
  VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{
  VL53LX_Error  status = VL53LX_ERROR_NONE;

  status =
    VL53LX_hist_xtalk_extract_calc_window(
      target_distance_mm,
      target_width_oversize,
      phist_bins,
      pxtalk_data);

  if (status == VL53LX_ERROR_NONE) {
    status =
      VL53LX_hist_xtalk_extract_calc_event_sums(
        phist_bins,
        pxtalk_data);
  }

  return status;
}


VL53LX_Error VL53LX::VL53LX_hist_xtalk_extract_fini(
  VL53LX_histogram_bin_data_t        *phist_bins,
  VL53LX_hist_xtalk_extract_data_t   *pxtalk_data,
  VL53LX_xtalk_calibration_results_t *pxtalk_cal,
  VL53LX_xtalk_histogram_shape_t     *pxtalk_shape)
{


  VL53LX_Error  status = VL53LX_ERROR_NONE;
  VL53LX_xtalk_calibration_results_t *pX = pxtalk_cal;


  if (pxtalk_data->sample_count > 0) {


    pxtalk_data->event_scaler_avg  = pxtalk_data->event_scaler_sum;
    pxtalk_data->event_scaler_avg +=
      (pxtalk_data->sample_count >> 1);
    pxtalk_data->event_scaler_avg /=  pxtalk_data->sample_count;



    status =
      VL53LX_hist_xtalk_extract_calc_rate_per_spad(
        pxtalk_data);



    if (status == VL53LX_ERROR_NONE) {


      pxtalk_data->zero_distance_phase_avg =
        pxtalk_data->zero_distance_phase_sum;
      pxtalk_data->zero_distance_phase_avg +=
        (pxtalk_data->sample_count >> 1);
      pxtalk_data->zero_distance_phase_avg /=
        pxtalk_data->sample_count;


      status =
        VL53LX_hist_xtalk_extract_calc_shape(
          pxtalk_data,
          pxtalk_shape);




      pxtalk_shape->phasecal_result__vcsel_start =
        phist_bins->phasecal_result__vcsel_start;
      pxtalk_shape->cal_config__vcsel_start =
        phist_bins->cal_config__vcsel_start;
      pxtalk_shape->vcsel_width =
        phist_bins->vcsel_width;
      pxtalk_shape->VL53LX_p_015 =
        phist_bins->VL53LX_p_015;
    }


    if (status == VL53LX_ERROR_NONE) {


      pX->algo__crosstalk_compensation_plane_offset_kcps =
        pxtalk_data->xtalk_rate_kcps_per_spad;
      pX->algo__crosstalk_compensation_x_plane_gradient_kcps
        = 0U;
      pX->algo__crosstalk_compensation_y_plane_gradient_kcps
        = 0U;

    }
  }


  return status;
}


VL53LX_Error   VL53LX::VL53LX_run_hist_xtalk_extraction(
  int16_t                             cal_distance_mm,
  VL53LX_Error                       *pcal_status)
{


#define OVERSIZE 4
  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_xtalkextract_config_t *pX = &(pdev->xtalk_extract_cfg);
  VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
  VL53LX_xtalk_calibration_results_t *pXC = &(pdev->xtalk_cal);



  uint8_t smudge_corr_en   = 0;
  uint8_t i                = 0;
  int8_t k = 0;
  uint8_t nbloops;
  int32_t initMergeSize = 0;
  int32_t MergeEnabled = 0;
  uint32_t deltaXtalk;
  uint32_t stepXtalk;
  uint32_t XtalkMin;
  uint32_t XtalkMax;
  uint8_t measurement_mode = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  int8_t MaxId;
  uint8_t histo_merge_nb;
  uint8_t wait_for_accumulation;
  VL53LX_range_results_t     *prange_results =
    (VL53LX_range_results_t *) pdev->wArea1;
  uint8_t Very1stRange = 0;




  if (status == VL53LX_ERROR_NONE)
    status =
      VL53LX_set_preset_mode(
        VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE,
        pX->dss_config__target_total_rate_mcps,
        pX->phasecal_config_timeout_us,
        pX->mm_config_timeout_us,
        pX->range_config_timeout_us,
        100);



  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_disable_xtalk_compensation();
  }



  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;

  if (status == VL53LX_ERROR_NONE) {
    status = VL53LX_dynamic_xtalk_correction_disable();
  }


  VL53LX_load_patch();

  VL53LX_get_tuning_parm(VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
                         &initMergeSize);
  VL53LX_get_tuning_parm(VL53LX_TUNINGPARM_HIST_MERGE,
                         &MergeEnabled);
  memset(&pdev->xtalk_cal, 0, sizeof(pdev->xtalk_cal));

  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_init_and_start_range(measurement_mode,
                                         VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);

  MaxId = pdev->tuning_parms.tp_hist_merge_max_size - 1;
  nbloops = (MergeEnabled == 0 ? 1 : 2);
  for (k = 0; k < nbloops; k++) {

    VL53LX_hist_xtalk_extract_data_init(
      &(pdev->xtalk_extract));
    VL53LX_set_tuning_parm(
      VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
      k * MaxId + 1);

    for (i = 0; i <= pX->num_of_samples; i++) {
      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_wait_for_range_completion();
      }
      if (status == VL53LX_ERROR_NONE)
        status = VL53LX_get_device_results(
                   VL53LX_DEVICERESULTSLEVEL_FULL,
                   prange_results);
      Very1stRange =
        (pdev->ll_state.rd_device_state ==
         VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC);

      VL53LX_compute_histo_merge_nb(&histo_merge_nb);
      wait_for_accumulation = ((k != 0) &&
                               (MergeEnabled) &&
                               (status == VL53LX_ERROR_NONE) &&
                               (histo_merge_nb <
                                pdev->tuning_parms.tp_hist_merge_max_size));
      if (wait_for_accumulation) {
        i = 0;
      } else {
        if ((status == VL53LX_ERROR_NONE) &&
            (!Very1stRange)) {
          status =
            VL53LX_hist_xtalk_extract_update(
              cal_distance_mm,
              OVERSIZE,
              &(pdev->hist_data),
              &(pdev->xtalk_extract));
        }
      }

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_wait_for_firmware_ready();
      }
      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_clear_interrupt_and_enable_next_range(measurement_mode);


      if (status == VL53LX_ERROR_NONE)
        status =
          VL53LX_hist_xtalk_extract_fini(
            &(pdev->hist_data),
            &(pdev->xtalk_extract),
            &(pdev->xtalk_cal),
            &(pdev->xtalk_shapes.xtalk_shape));
      if (status != VL53LX_ERROR_NONE) {
        goto LOOPOUT;
      }
      pXC->algo__xtalk_cpo_HistoMerge_kcps[k * MaxId] =
        pXC->algo__crosstalk_compensation_plane_offset_kcps;
    }
  }

LOOPOUT:

  VL53LX_stop_range();

  VL53LX_set_tuning_parm(VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
                         initMergeSize);
  VL53LX_unload_patch();

  if (status != VL53LX_ERROR_NONE) {
    status = VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL;
  } else if ((MergeEnabled == 1) && (MaxId > 0)) {
    XtalkMin = pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[0];
    XtalkMax =
      pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[MaxId];
    pdev->xtalk_cal.
    algo__crosstalk_compensation_plane_offset_kcps = XtalkMin;
    if (XtalkMax > XtalkMin) {
      deltaXtalk =  XtalkMax - XtalkMin;
      stepXtalk = deltaXtalk / MaxId;
      for (k = 1; k < MaxId; k++)
        pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[k] =
          XtalkMin + stepXtalk * k;
    } else
      status =
        VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL;
  }

  if (status == VL53LX_ERROR_NONE) {
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
      pXC->algo__crosstalk_compensation_x_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
      pXC->algo__crosstalk_compensation_y_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_plane_offset_kcps =
      pXC->algo__crosstalk_compensation_plane_offset_kcps;
  }


  pdev->xtalk_results.cal_status = status;
  *pcal_status = pdev->xtalk_results.cal_status;


  status = VL53LX_enable_xtalk_compensation();
  if (smudge_corr_en == 1) {
    status = VL53LX_dynamic_xtalk_correction_enable();
  }
  /*
  #ifdef VL53LX_LOG_ENABLE



    VL53LX_print_customer_nvm_managed(
      &(pdev->customer),
      "run_xtalk_extraction():pdev->lldata.customer.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_xtalk_config(
      &(pdev->xtalk_cfg),
      "run_xtalk_extraction():pdev->lldata.xtalk_cfg.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

    VL53LX_print_xtalk_histogram_data(
      &(pdev->xtalk_shapes),
      "pdev->lldata.xtalk_shapes.",
      VL53LX_TRACE_MODULE_XTALK_DATA);

  #endif
  */

  return status;
}

/* vl53lx_api.c */

VL53LX_Error VL53LX::VL53LX_GetVersion(VL53LX_Version_t *pVersion)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  pVersion->major = VL53LX_IMPLEMENTATION_VER_MAJOR;
  pVersion->minor = VL53LX_IMPLEMENTATION_VER_MINOR;
  pVersion->build = VL53LX_IMPLEMENTATION_VER_SUB;

  pVersion->revision = VL53LX_IMPLEMENTATION_VER_REVISION;

  return Status;
}


VL53LX_Error VL53LX::VL53LX_GetProductRevision(
  uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t revision_id;
  VL53LX_LLDriverData_t   *pLLData;


  pLLData =  VL53LXDevStructGetLLDriverHandle(Dev);
  revision_id = pLLData->nvm_copy_data.identification__revision_id;
  *pProductRevisionMajor = 1;
  *pProductRevisionMinor = (revision_id & 0xF0) >> 4;

  return Status;

}

VL53LX_Error VL53LX::VL53LX_GetDeviceInfo(
  VL53LX_DeviceInfo_t *pVL53LX_DeviceInfo)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t revision_id;
  VL53LX_LLDriverData_t   *pLLData;

  pLLData =  VL53LXDevStructGetLLDriverHandle(Dev);

  pVL53LX_DeviceInfo->ProductType =
    pLLData->nvm_copy_data.identification__module_type;

  revision_id = pLLData->nvm_copy_data.identification__revision_id;
  pVL53LX_DeviceInfo->ProductRevisionMajor = 1;
  pVL53LX_DeviceInfo->ProductRevisionMinor = (revision_id & 0xF0) >> 4;

  return Status;
}

VL53LX_Error VL53LX::VL53LX_GetUID(uint64_t *pUid)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t fmtdata[8];

  Status = VL53LX_read_nvm_raw_data(
             (uint8_t)(0x1F8 >> 2),
             (uint8_t)(8 >> 2),
             fmtdata);
  memcpy(pUid, fmtdata, sizeof(uint64_t));

  return Status;
}

VL53LX_Error VL53LX::VL53LX_SetDeviceAddress(uint8_t DeviceAddress)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  Status = VL53LX_WrByte(Dev, VL53LX_I2C_SLAVE__DEVICE_ADDRESS,
                         DeviceAddress / 2);

  if (Status == VL53LX_ERROR_NONE) {
    Dev->I2cDevAddr = DeviceAddress;
  }

  return Status;
}


VL53LX_Error VL53LX::VL53LX_DataInit()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev;
  uint8_t  measurement_mode;
  uint8_t i = 0;

  Status = VL53LX_RdByte(Dev, VL53LX_PAD_I2C_HV__EXTSUP_CONFIG, &i);
  if (Status == VL53LX_ERROR_NONE) {
    i = (i & 0xfe) | 0x01;
    Status = VL53LX_WrByte(Dev, VL53LX_PAD_I2C_HV__EXTSUP_CONFIG,
                           i);
  }

  if (Status == VL53LX_ERROR_NONE) {
    Status = VL53LX_data_init(1);
  }

  Status = SetPresetModeL3CX(
             VL53LX_DISTANCEMODE_LONG,
             1000);


  if (Status == VL53LX_ERROR_NONE)
    Status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(
               33333);

  if (Status == VL53LX_ERROR_NONE) {
    Status = SetInterMeasurementPeriodMilliSeconds(1000);
  }

  if (Status == VL53LX_ERROR_NONE) {
    pdev = VL53LXDevStructGetLLDriverHandle(Dev);
    memset(&pdev->per_vcsel_cal_data, 0,
           sizeof(pdev->per_vcsel_cal_data));
  }

  if (Status == VL53LX_ERROR_NONE) {
    Status = VL53LX_set_dmax_mode(
               VL53LX_DEVICEDMAXMODE__CUST_CAL_DATA);
  }


  measurement_mode  = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
  VL53LXDevDataSet(Dev, LLData.measurement_mode, measurement_mode);

  VL53LXDevDataSet(Dev, CurrentParameters.DistanceMode,
                   VL53LX_DISTANCEMODE_LONG);


  return Status;
}

VL53LX_Error VL53LX::VL53LX_WaitDeviceBooted()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  Status = VL53LX_poll_for_boot_completion(
             VL53LX_BOOT_COMPLETION_POLLING_TIMEOUT_MS);

  return Status;
}

VL53LX_Error VL53LX::ComputeDevicePresetMode(
  VL53LX_DistanceModes DistanceMode,
  VL53LX_DevicePresetModes *pDevicePresetMode)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  uint8_t DistIdx;
  VL53LX_DevicePresetModes RangingModes[3] = {
    VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE,
    VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE,
    VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE
  };

  switch (DistanceMode) {
    case VL53LX_DISTANCEMODE_SHORT:
      DistIdx = 0;
      break;
    case VL53LX_DISTANCEMODE_MEDIUM:
      DistIdx = 1;
      break;
    default:
      DistIdx = 2;
  }

  *pDevicePresetMode = RangingModes[DistIdx];

  return Status;
}

VL53LX_Error VL53LX::SetPresetModeL3CX(
  VL53LX_DistanceModes DistanceMode,
  uint32_t inter_measurement_period_ms)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_DevicePresetModes   device_preset_mode;
  uint8_t measurement_mode;
  uint16_t dss_config__target_total_rate_mcps;
  uint32_t phasecal_config_timeout_us;
  uint32_t mm_config_timeout_us;
  uint32_t lld_range_config_timeout_us;


  measurement_mode  = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;

  Status = ComputeDevicePresetMode(DistanceMode,
                                   &device_preset_mode);

  if (Status == VL53LX_ERROR_NONE)
    Status =  VL53LX_get_preset_mode_timing_cfg(
                device_preset_mode,
                &dss_config__target_total_rate_mcps,
                &phasecal_config_timeout_us,
                &mm_config_timeout_us,
                &lld_range_config_timeout_us);

  if (Status == VL53LX_ERROR_NONE)
    Status = VL53LX_set_preset_mode(
               device_preset_mode,
               dss_config__target_total_rate_mcps,
               phasecal_config_timeout_us,
               mm_config_timeout_us,
               lld_range_config_timeout_us,
               inter_measurement_period_ms);

  if (Status == VL53LX_ERROR_NONE)
    VL53LXDevDataSet(Dev, LLData.measurement_mode,
                     measurement_mode);

  return Status;
}

VL53LX_Error VL53LX::VL53LX_SetDistanceMode(
  VL53LX_DistanceModes DistanceMode)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint32_t inter_measurement_period_ms;
  uint32_t TimingBudget;
  uint32_t MmTimeoutUs;
  uint32_t PhaseCalTimeoutUs;

  if ((DistanceMode != VL53LX_DISTANCEMODE_SHORT) &&
      (DistanceMode != VL53LX_DISTANCEMODE_MEDIUM) &&
      (DistanceMode != VL53LX_DISTANCEMODE_LONG)) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  inter_measurement_period_ms =  VL53LXDevDataGet(Dev,
                                                  LLData.inter_measurement_period_ms);

  if (Status == VL53LX_ERROR_NONE)
    Status = VL53LX_get_timeouts_us(&PhaseCalTimeoutUs,
                                    &MmTimeoutUs, &TimingBudget);

  if (Status == VL53LX_ERROR_NONE)
    Status = SetPresetModeL3CX(
               DistanceMode,
               inter_measurement_period_ms);

  if (Status == VL53LX_ERROR_NONE) {
    VL53LXDevDataSet(Dev, CurrentParameters.DistanceMode,
                     DistanceMode);
  }

  if (Status == VL53LX_ERROR_NONE) {
    Status = VL53LX_set_timeouts_us(PhaseCalTimeoutUs,
                                    MmTimeoutUs, TimingBudget);

    if (Status == VL53LX_ERROR_NONE)
      VL53LXDevDataSet(Dev, LLData.range_config_timeout_us,
                       TimingBudget);
  }

  return Status;
}

VL53LX_Error VL53LX::VL53LX_GetDistanceMode(
  VL53LX_DistanceModes *pDistanceMode)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;


  *pDistanceMode = VL53LXDevDataGet(Dev, CurrentParameters.DistanceMode);

  return Status;
}


VL53LX_Error VL53LX::VL53LX_SetMeasurementTimingBudgetMicroSeconds(
  uint32_t MeasurementTimingBudgetMicroSeconds)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint32_t TimingGuard;
  uint32_t divisor;
  uint32_t TimingBudget;
  uint32_t MmTimeoutUs;
  uint32_t PhaseCalTimeoutUs;
  uint32_t FDAMaxTimingBudgetUs = FDA_MAX_TIMING_BUDGET_US;

  if (MeasurementTimingBudgetMicroSeconds > 10000000) {
    Status = VL53LX_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53LX_ERROR_NONE)
    Status = VL53LX_get_timeouts_us(
               &PhaseCalTimeoutUs,
               &MmTimeoutUs,
               &TimingBudget);

  TimingGuard = 1700;
  divisor = 6;

  if (MeasurementTimingBudgetMicroSeconds <= TimingGuard) {
    Status = VL53LX_ERROR_INVALID_PARAMS;
  } else {
    TimingBudget = (MeasurementTimingBudgetMicroSeconds
                    - TimingGuard);
  }

  if (Status == VL53LX_ERROR_NONE) {
    if (TimingBudget > FDAMaxTimingBudgetUs) {
      Status = VL53LX_ERROR_INVALID_PARAMS;
    } else {
      TimingBudget /= divisor;
      Status = VL53LX_set_timeouts_us(
                 PhaseCalTimeoutUs,
                 MmTimeoutUs,
                 TimingBudget);
    }

    if (Status == VL53LX_ERROR_NONE)
      VL53LXDevDataSet(Dev,
                       LLData.range_config_timeout_us,
                       TimingBudget);
  }

  if (Status == VL53LX_ERROR_NONE) {
    VL53LXDevDataSet(Dev,
                     CurrentParameters.MeasurementTimingBudgetMicroSeconds,
                     MeasurementTimingBudgetMicroSeconds);
  }

  return Status;
}


VL53LX_Error VL53LX::VL53LX_GetMeasurementTimingBudgetMicroSeconds(
  uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint32_t MmTimeoutUs = 0;
  uint32_t RangeTimeoutUs = 0;
  uint32_t PhaseCalTimeoutUs = 0;

  *pMeasurementTimingBudgetMicroSeconds = 0;

  if (Status == VL53LX_ERROR_NONE)
    Status = VL53LX_get_timeouts_us(
               &PhaseCalTimeoutUs,
               &MmTimeoutUs,
               &RangeTimeoutUs);

  if (Status == VL53LX_ERROR_NONE)
    *pMeasurementTimingBudgetMicroSeconds = (6 * RangeTimeoutUs) +
                                            1700;

  return Status;
}

VL53LX_Error VL53LX::SetInterMeasurementPeriodMilliSeconds(
  uint32_t InterMeasurementPeriodMilliSeconds)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint32_t adjustedIMP;

  adjustedIMP = InterMeasurementPeriodMilliSeconds;
  adjustedIMP += (adjustedIMP * 64) / 1000;

  Status = VL53LX_set_inter_measurement_period_ms(
             adjustedIMP);

  return Status;
}

VL53LX_Error VL53LX::GetInterMeasurementPeriodMilliSeconds(
  uint32_t *pInterMeasurementPeriodMilliSeconds)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint32_t adjustedIMP;


  Status = VL53LX_get_inter_measurement_period_ms(&adjustedIMP);

  if (Status == VL53LX_ERROR_NONE) {
    adjustedIMP -= (adjustedIMP * 64) / 1000;
    *pInterMeasurementPeriodMilliSeconds = adjustedIMP;
  }

  return Status;
}

VL53LX_Error VL53LX::VL53LX_StartMeasurement()
{
#define TIMED_MODE_TIMING_GUARD_MILLISECONDS 4
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t DeviceMeasurementMode;
  uint32_t MTBus, IMPms;


  VL53LX_load_patch();

  DeviceMeasurementMode = VL53LXDevDataGet(Dev, LLData.measurement_mode);


  if ((Status == VL53LX_ERROR_NONE) &&
      (DeviceMeasurementMode == VL53LX_DEVICEMEASUREMENTMODE_TIMED)) {
    Status = VL53LX_GetMeasurementTimingBudgetMicroSeconds(&MTBus);

    if (Status == VL53LX_ERROR_NONE) {
      MTBus /= 1000;
      Status = GetInterMeasurementPeriodMilliSeconds(&IMPms);
    }

    if ((Status == VL53LX_ERROR_NONE) &&
        (IMPms < MTBus + TIMED_MODE_TIMING_GUARD_MILLISECONDS)) {
      Status = VL53LX_ERROR_INVALID_PARAMS;
    }
  }

  if (Status == VL53LX_ERROR_NONE)
    Status = VL53LX_init_and_start_range(
               DeviceMeasurementMode,
               VL53LX_DEVICECONFIGLEVEL_FULL);

  return Status;
}

VL53LX_Error VL53LX::VL53LX_StopMeasurement()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  Status = VL53LX_stop_range();

  VL53LX_unload_patch();

  return Status;
}

VL53LX_Error VL53LX::VL53LX_ClearInterruptAndStartMeasurement()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t DeviceMeasurementMode;

  DeviceMeasurementMode = VL53LXDevDataGet(Dev, LLData.measurement_mode);

  Status = VL53LX_clear_interrupt_and_enable_next_range(
             DeviceMeasurementMode);

  return Status;
}

VL53LX_Error VL53LX::VL53LX_GetMeasurementDataReady(uint8_t *pMeasurementDataReady)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  Status = VL53LX_is_new_data_ready(pMeasurementDataReady);

  return Status;
}

VL53LX_Error VL53LX::VL53LX_WaitMeasurementDataReady()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  Status = VL53LX_poll_for_range_completion(
             VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS);

  return Status;
}

uint8_t VL53LX::ConvertStatusHisto(uint8_t FilteredRangeStatus)
{
  uint8_t RangeStatus;

  switch (FilteredRangeStatus) {
    case VL53LX_DEVICEERROR_RANGEPHASECHECK:
      RangeStatus = VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;
    case VL53LX_DEVICEERROR_SIGMATHRESHOLDCHECK:
      RangeStatus = VL53LX_RANGESTATUS_SIGMA_FAIL;
      break;
    case VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      RangeStatus =
        VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      break;
    case VL53LX_DEVICEERROR_PHASECONSISTENCY:
      RangeStatus = VL53LX_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53LX_DEVICEERROR_PREV_RANGE_NO_TARGETS:
      RangeStatus = VL53LX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL;
      break;
    case VL53LX_DEVICEERROR_EVENTCONSISTENCY:
      RangeStatus = VL53LX_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53LX_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE:
      RangeStatus = VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE;
      break;
    case VL53LX_DEVICEERROR_RANGECOMPLETE:
      RangeStatus = VL53LX_RANGESTATUS_RANGE_VALID;
      break;
    default:
      RangeStatus = VL53LX_RANGESTATUS_NONE;
  }

  return RangeStatus;
}

VL53LX_Error VL53LX::SetTargetData(
  uint8_t active_results, uint8_t device_status,
  VL53LX_range_data_t *presults_data,
  VL53LX_TargetRangeData_t *pRangeData)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t FilteredRangeStatus;
  FixPoint1616_t AmbientRate;
  FixPoint1616_t SignalRate;
  FixPoint1616_t TempFix1616;
  int16_t Range;

  SUPPRESS_UNUSED_WARNING(Dev);

  FilteredRangeStatus = presults_data->range_status & 0x1F;

  SignalRate = VL53LX_FIXPOINT97TOFIXPOINT1616(
                 presults_data->peak_signal_count_rate_mcps);
  pRangeData->SignalRateRtnMegaCps
    = SignalRate;

  AmbientRate = VL53LX_FIXPOINT97TOFIXPOINT1616(
                  presults_data->ambient_count_rate_mcps);
  pRangeData->AmbientRateRtnMegaCps = AmbientRate;

  TempFix1616 = VL53LX_FIXPOINT97TOFIXPOINT1616(
                  presults_data->VL53LX_p_002);

  pRangeData->SigmaMilliMeter = TempFix1616;

  pRangeData->RangeMilliMeter = presults_data->median_range_mm;
  pRangeData->RangeMaxMilliMeter = presults_data->max_range_mm;
  pRangeData->RangeMinMilliMeter = presults_data->min_range_mm;


  switch (device_status) {
    case VL53LX_DEVICEERROR_MULTCLIPFAIL:
    case VL53LX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53LX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53LX_DEVICEERROR_NOVHVVALUEFOUND:
      pRangeData->RangeStatus =  VL53LX_RANGESTATUS_HARDWARE_FAIL;
      break;
    case VL53LX_DEVICEERROR_USERROICLIP:
      pRangeData->RangeStatus =  VL53LX_RANGESTATUS_MIN_RANGE_FAIL;
      break;
    default:
      pRangeData->RangeStatus =  VL53LX_RANGESTATUS_RANGE_VALID;
  }


  if ((pRangeData->RangeStatus ==  VL53LX_RANGESTATUS_RANGE_VALID) &&
      (active_results == 0)) {
    pRangeData->RangeStatus =  VL53LX_RANGESTATUS_NONE;
    pRangeData->SignalRateRtnMegaCps = 0;
    pRangeData->SigmaMilliMeter = 0;
    pRangeData->RangeMilliMeter = 8191;
    pRangeData->RangeMaxMilliMeter = 8191;
    pRangeData->RangeMinMilliMeter = 8191;
  }


  if (pRangeData->RangeStatus ==  VL53LX_RANGESTATUS_RANGE_VALID)
    pRangeData->RangeStatus =
      ConvertStatusHisto(FilteredRangeStatus);

  Range = pRangeData->RangeMilliMeter;
  if ((pRangeData->RangeStatus ==  VL53LX_RANGESTATUS_RANGE_VALID) &&
      (Range < 0)) {
    if (Range < BDTable[VL53LX_TUNING_PROXY_MIN])
      pRangeData->RangeStatus =
        VL53LX_RANGESTATUS_RANGE_INVALID;
    else {
      pRangeData->RangeMilliMeter = 0;
    }
  }

  return Status;
}

VL53LX_Error VL53LX::SetMeasurementData(
  VL53LX_range_results_t *presults,
  VL53LX_MultiRangingData_t *pMultiRangingData)
{
  uint8_t i;
  uint8_t iteration;
  VL53LX_TargetRangeData_t *pRangeData;
  VL53LX_range_data_t *presults_data;
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  uint8_t ActiveResults;

  pMultiRangingData->NumberOfObjectsFound = presults->active_results;
  pMultiRangingData->HasXtalkValueChanged =
    presults->smudge_corrector_data.new_xtalk_applied_flag;


  pMultiRangingData->TimeStamp = 0;

  pMultiRangingData->StreamCount = presults->stream_count;

  ActiveResults = presults->active_results;
  if (ActiveResults < 1)

  {
    iteration = 1;
  } else {
    iteration = ActiveResults;
  }
  for (i = 0; i < iteration; i++) {
    pRangeData = &(pMultiRangingData->RangeData[i]);

    presults_data = &(presults->VL53LX_p_003[i]);
    if (Status == VL53LX_ERROR_NONE)
      Status = SetTargetData(ActiveResults,
                             presults->device_status,
                             presults_data,
                             pRangeData);

    pMultiRangingData->EffectiveSpadRtnCount =
      presults_data->VL53LX_p_004;

  }
  return Status;
}

VL53LX_Error VL53LX::VL53LX_GetMultiRangingData(
  VL53LX_MultiRangingData_t *pMultiRangingData)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_LLDriverData_t *pdev =
    VL53LXDevStructGetLLDriverHandle(Dev);
  VL53LX_range_results_t *presults =
    (VL53LX_range_results_t *) pdev->wArea1;

  memset(pMultiRangingData, 0xFF,
         sizeof(VL53LX_MultiRangingData_t));


  Status = VL53LX_get_device_results(
             VL53LX_DEVICERESULTSLEVEL_FULL,
             presults);

  Status = SetMeasurementData(
             presults,
             pMultiRangingData);

  return Status;
}
/*
VL53LX_Error VL53LX::VL53LX_GetAdditionalData()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;


  return Status;
}
*/
VL53LX_Error VL53LX::VL53LX_SetTuningParameter(
  uint16_t TuningParameterId, int32_t TuningParameterValue)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  if (TuningParameterId ==
      VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  if (TuningParameterId >= 32768)
    Status = VL53LX_set_tuning_parm(
               TuningParameterId,
               TuningParameterValue);
  else {
    if (TuningParameterId < VL53LX_TUNING_MAX_TUNABLE_KEY) {
      BDTable[TuningParameterId] = TuningParameterValue;
    } else {
      Status = VL53LX_ERROR_INVALID_PARAMS;
    }
  }

  return Status;
}
VL53LX_Error VL53LX::VL53LX_GetTuningParameter(
  uint16_t TuningParameterId, int32_t *pTuningParameterValue)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;


  if (TuningParameterId >= 32768)
    Status = VL53LX_get_tuning_parm(
               TuningParameterId,
               pTuningParameterValue);
  else {
    if (TuningParameterId < VL53LX_TUNING_MAX_TUNABLE_KEY) {
      *pTuningParameterValue = BDTable[TuningParameterId];
    } else {
      Status = VL53LX_ERROR_INVALID_PARAMS;
    }
  }


  return Status;
}

VL53LX_Error VL53LX::VL53LX_PerformRefSpadManagement()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_Error RawStatus;
  uint8_t dcrbuffer[24];
  uint8_t *commbuf;
  uint8_t numloc[2] = {5, 3};
  VL53LX_LLDriverData_t *pdev;
  VL53LX_customer_nvm_managed_t *pc;
  VL53LX_DistanceModes DistanceMode;

  pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  pc = &pdev->customer;

  if (Status == VL53LX_ERROR_NONE) {
    DistanceMode = VL53LXDevDataGet(Dev,
                                    CurrentParameters.DistanceMode);
    Status = VL53LX_run_ref_spad_char(&RawStatus);

    if (Status == VL53LX_ERROR_NONE) {
      Status = VL53LX_SetDistanceMode(DistanceMode);
    }
  }

  if (Status == VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH) {

    Status = VL53LX_read_nvm_raw_data(
               (uint8_t)(0xA0 >> 2),
               (uint8_t)(24 >> 2),
               dcrbuffer);

    if (Status == VL53LX_ERROR_NONE)
      Status = VL53LX_WriteMulti(Dev,
                                 VL53LX_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS,
                                 numloc, 2);

    if (Status == VL53LX_ERROR_NONE) {
      pc->ref_spad_man__num_requested_ref_spads = numloc[0];
      pc->ref_spad_man__ref_location = numloc[1];
    }

    if (Status == VL53LX_ERROR_NONE) {
      commbuf = &dcrbuffer[16];
    }



    if (Status == VL53LX_ERROR_NONE)
      Status = VL53LX_WriteMulti(Dev,
                                 VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
                                 commbuf, 6);

    if (Status == VL53LX_ERROR_NONE) {
      pc->global_config__spad_enables_ref_0 = commbuf[0];
      pc->global_config__spad_enables_ref_1 = commbuf[1];
      pc->global_config__spad_enables_ref_2 = commbuf[2];
      pc->global_config__spad_enables_ref_3 = commbuf[3];
      pc->global_config__spad_enables_ref_4 = commbuf[4];
      pc->global_config__spad_enables_ref_5 = commbuf[5];
    }

  }

  return Status;
}


VL53LX_Error VL53LX::VL53LX_SmudgeCorrectionEnable(
  VL53LX_SmudgeCorrectionModes Mode)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_Error s1 = VL53LX_ERROR_NONE;
  VL53LX_Error s2 = VL53LX_ERROR_NONE;
  VL53LX_Error s3 = VL53LX_ERROR_NONE;

  switch (Mode) {
    case VL53LX_SMUDGE_CORRECTION_NONE:
      s1 = VL53LX_dynamic_xtalk_correction_disable();
      s2 = VL53LX_dynamic_xtalk_correction_apply_disable();
      s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable();
      break;
    case VL53LX_SMUDGE_CORRECTION_CONTINUOUS:
      s1 = VL53LX_dynamic_xtalk_correction_enable();
      s2 = VL53LX_dynamic_xtalk_correction_apply_enable();
      s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable();
      break;
    case VL53LX_SMUDGE_CORRECTION_SINGLE:
      s1 = VL53LX_dynamic_xtalk_correction_enable();
      s2 = VL53LX_dynamic_xtalk_correction_apply_enable();
      s3 = VL53LX_dynamic_xtalk_correction_single_apply_enable();
      break;
    case VL53LX_SMUDGE_CORRECTION_DEBUG:
      s1 = VL53LX_dynamic_xtalk_correction_enable();
      s2 = VL53LX_dynamic_xtalk_correction_apply_disable();
      s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable();
      break;
    default:
      Status = VL53LX_ERROR_INVALID_PARAMS;
      break;
  }

  if (Status == VL53LX_ERROR_NONE) {
    Status = s1;
    if (Status == VL53LX_ERROR_NONE) {
      Status = s2;
    }
    if (Status == VL53LX_ERROR_NONE) {
      Status = s3;
    }
  }

  return Status;
}

VL53LX_Error VL53LX::VL53LX_SetXTalkCompensationEnable(
  uint8_t XTalkCompensationEnable)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  if (XTalkCompensationEnable == 0) {
    Status = VL53LX_disable_xtalk_compensation();
  } else {
    Status = VL53LX_enable_xtalk_compensation();
  }

  return Status;
}

VL53LX_Error VL53LX::VL53LX_GetXTalkCompensationEnable(
  uint8_t *pXTalkCompensationEnable)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;

  VL53LX_get_xtalk_compensation_enable(
    pXTalkCompensationEnable);

  return Status;
}


VL53LX_Error VL53LX::VL53LX_PerformXTalkCalibration()
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_Error UStatus;
  int16_t CalDistanceMm;
  VL53LX_xtalk_calibration_results_t xtalk;

  VL53LX_CalibrationData_t caldata;
  VL53LX_LLDriverData_t *pLLData;
  int i;
  uint32_t *pPlaneOffsetKcps;
  uint32_t Margin =
    BDTable[VL53LX_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN];
  uint32_t DefaultOffset =
    BDTable[VL53LX_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET];
  uint32_t *pLLDataPlaneOffsetKcps;
  uint32_t sum = 0;
  uint8_t binok = 0;

  pPlaneOffsetKcps =
    &caldata.customer.algo__crosstalk_compensation_plane_offset_kcps;
  pLLData = VL53LXDevStructGetLLDriverHandle(Dev);
  pLLDataPlaneOffsetKcps =
    &pLLData->xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps;

  CalDistanceMm = (int16_t)
                  BDTable[VL53LX_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM];
  Status = VL53LX_run_hist_xtalk_extraction(CalDistanceMm,
                                            &UStatus);

  VL53LX_GetCalibrationData(&caldata);
  for (i = 0; i < VL53LX_XTALK_HISTO_BINS; i++) {
    sum += caldata.xtalkhisto.xtalk_shape.bin_data[i];
    if (caldata.xtalkhisto.xtalk_shape.bin_data[i] > 0) {
      binok++;
    }
  }
  if ((UStatus ==
       VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL) ||
      (sum > (1024 + Margin)) || (sum < (1024 - Margin)) ||
      (binok < 3)) {
    *pPlaneOffsetKcps = DefaultOffset;
    *pLLDataPlaneOffsetKcps = DefaultOffset;
    caldata.xtalkhisto.xtalk_shape.bin_data[0] = 307;
    caldata.xtalkhisto.xtalk_shape.bin_data[1] = 410;
    caldata.xtalkhisto.xtalk_shape.bin_data[2] = 410;
    caldata.xtalkhisto.xtalk_shape.bin_data[3] = 307;
    for (i = 4; i < VL53LX_XTALK_HISTO_BINS; i++) {
      caldata.xtalkhisto.xtalk_shape.bin_data[i] = 0;
    }
    for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
      caldata.algo__xtalk_cpo_HistoMerge_kcps[i] =
        DefaultOffset + DefaultOffset * i;
    VL53LX_SetCalibrationData(&caldata);
  }

  if (Status == VL53LX_ERROR_NONE) {
    Status = VL53LX_get_current_xtalk_settings(&xtalk);
    Status = VL53LX_set_tuning_parm(
               VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS,
               xtalk.algo__crosstalk_compensation_plane_offset_kcps);
  }

  return Status;
}
VL53LX_Error VL53LX::VL53LX_SetOffsetCorrectionMode(
  VL53LX_OffsetCorrectionModes OffsetCorrectionMode)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_OffsetCorrectionMode   offset_cor_mode;

  if (OffsetCorrectionMode == VL53LX_OFFSETCORRECTIONMODE_STANDARD) {
    offset_cor_mode =
      VL53LX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
  } else if (OffsetCorrectionMode ==
             VL53LX_OFFSETCORRECTIONMODE_PERVCSEL) {
    offset_cor_mode =
      VL53LX_OFFSETCORRECTIONMODE__PER_VCSEL_OFFSETS;
  } else {
    Status = VL53LX_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53LX_ERROR_NONE)
    Status =  VL53LX_set_offset_correction_mode(
                offset_cor_mode);

  return Status;
}

VL53LX_Error VL53LX::VL53LX_PerformOffsetSimpleCalibration(
  int32_t CalDistanceMilliMeter)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  int32_t sum_ranging;
  uint8_t offset_meas;
  int16_t Max, UnderMax, OverMax, Repeat;
  int32_t total_count, inloopcount;
  int32_t IncRounding;
  int16_t meanDistance_mm;
  int16_t offset;
  VL53LX_MultiRangingData_t RangingMeasurementData;
  VL53LX_LLDriverData_t *pdev;
  uint8_t goodmeas;
  VL53LX_Error SmudgeStatus = VL53LX_ERROR_NONE;
  uint8_t smudge_corr_en;
  VL53LX_TargetRangeData_t *pRange;

  pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;
  SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable();

  pdev->customer.algo__part_to_part_range_offset_mm = 0;
  pdev->customer.mm_config__inner_offset_mm = 0;
  pdev->customer.mm_config__outer_offset_mm = 0;
  memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));
  Repeat = BDTable[VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT];
  Max = BDTable[
         VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
  UnderMax = 1 + (Max / 2);
  OverMax = Max + (Max / 2);
  sum_ranging = 0;
  total_count = 0;

  while ((Repeat > 0) && (Status == VL53LX_ERROR_NONE)) {
    Status = VL53LX_StartMeasurement();

    if (Status == VL53LX_ERROR_NONE) {
      VL53LX_WaitMeasurementDataReady();
      VL53LX_GetMultiRangingData(
        &RangingMeasurementData);
      VL53LX_ClearInterruptAndStartMeasurement();
    }

    inloopcount = 0;
    offset_meas = 0;
    while ((Status == VL53LX_ERROR_NONE) && (inloopcount < Max) &&
           (offset_meas < OverMax)) {
      Status = VL53LX_WaitMeasurementDataReady();
      if (Status == VL53LX_ERROR_NONE)
        Status = VL53LX_GetMultiRangingData(
                   &RangingMeasurementData);
      pRange = &(RangingMeasurementData.RangeData[0]);
      goodmeas = (pRange->RangeStatus ==
                  VL53LX_RANGESTATUS_RANGE_VALID);
      if ((Status == VL53LX_ERROR_NONE) && goodmeas) {
        sum_ranging += pRange->RangeMilliMeter;
        inloopcount++;
      }
      Status = VL53LX_ClearInterruptAndStartMeasurement();
      offset_meas++;
    }
    total_count += inloopcount;


    if (inloopcount < UnderMax) {
      Status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
    }

    VL53LX_StopMeasurement();

    Repeat--;

  }

  if ((SmudgeStatus == VL53LX_ERROR_NONE) && (smudge_corr_en == 1)) {
    SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable();
  }

  if ((sum_ranging < 0) ||
      (sum_ranging > ((int32_t) total_count * 0xffff))) {
    Status = VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
  }

  if ((Status == VL53LX_ERROR_NONE) && (total_count > 0)) {
    IncRounding = total_count / 2;
    meanDistance_mm = (int16_t)((sum_ranging + IncRounding)
                                / total_count);
    offset = (int16_t)CalDistanceMilliMeter - meanDistance_mm;
    pdev->customer.algo__part_to_part_range_offset_mm = 0;
    pdev->customer.mm_config__inner_offset_mm = offset;
    pdev->customer.mm_config__outer_offset_mm = offset;

    Status = VL53LX_set_customer_nvm_managed(
               &(pdev->customer));
  }

  return Status;
}


VL53LX_Error VL53LX::VL53LX_PerformOffsetZeroDistanceCalibration()
{
#define START_OFFSET 50
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  int32_t sum_ranging;
  uint8_t offset_meas;
  int16_t Max, UnderMax, OverMax, Repeat;
  int32_t total_count, inloopcount;
  int32_t IncRounding;
  int16_t meanDistance_mm;
  int16_t offset, ZeroDistanceOffset;
  VL53LX_MultiRangingData_t RangingMeasurementData;
  VL53LX_LLDriverData_t *pdev;
  uint8_t goodmeas;
  VL53LX_Error SmudgeStatus = VL53LX_ERROR_NONE;
  uint8_t smudge_corr_en;
  VL53LX_TargetRangeData_t *pRange;

  pdev = VL53LXDevStructGetLLDriverHandle(Dev);
  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;
  SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable();
  pdev->customer.algo__part_to_part_range_offset_mm = 0;
  pdev->customer.mm_config__inner_offset_mm = START_OFFSET;
  pdev->customer.mm_config__outer_offset_mm = START_OFFSET;
  memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));
  ZeroDistanceOffset = BDTable[
                        VL53LX_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR];
  Repeat = BDTable[VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT];
  Max =
    BDTable[VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
  UnderMax = 1 + (Max / 2);
  OverMax = Max + (Max / 2);
  sum_ranging = 0;
  total_count = 0;

  while ((Repeat > 0) && (Status == VL53LX_ERROR_NONE)) {
    Status = VL53LX_StartMeasurement();
    if (Status == VL53LX_ERROR_NONE) {
      VL53LX_WaitMeasurementDataReady();
      VL53LX_GetMultiRangingData(
        &RangingMeasurementData);
      VL53LX_ClearInterruptAndStartMeasurement();
    }
    inloopcount = 0;
    offset_meas = 0;
    while ((Status == VL53LX_ERROR_NONE) && (inloopcount < Max) &&
           (offset_meas < OverMax)) {
      Status = VL53LX_WaitMeasurementDataReady();
      if (Status == VL53LX_ERROR_NONE)
        Status = VL53LX_GetMultiRangingData(
                   &RangingMeasurementData);
      pRange = &(RangingMeasurementData.RangeData[0]);
      goodmeas = (pRange->RangeStatus ==
                  VL53LX_RANGESTATUS_RANGE_VALID);
      if ((Status == VL53LX_ERROR_NONE) && goodmeas) {
        sum_ranging = sum_ranging +
                      pRange->RangeMilliMeter;
        inloopcount++;
      }
      Status = VL53LX_ClearInterruptAndStartMeasurement();
      offset_meas++;
    }
    total_count += inloopcount;
    if (inloopcount < UnderMax) {
      Status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
    }
    VL53LX_StopMeasurement();
    Repeat--;
  }
  if ((SmudgeStatus == VL53LX_ERROR_NONE) && (smudge_corr_en == 1)) {
    SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable();
  }
  if ((sum_ranging < 0) ||
      (sum_ranging > ((int32_t) total_count * 0xffff))) {
    Status = VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
  }

  if ((Status == VL53LX_ERROR_NONE) && (total_count > 0)) {
    IncRounding = total_count / 2;
    meanDistance_mm = (int16_t)
                      ((sum_ranging + IncRounding) / total_count);
    offset = START_OFFSET - meanDistance_mm + ZeroDistanceOffset;
    pdev->customer.algo__part_to_part_range_offset_mm = 0;
    pdev->customer.mm_config__inner_offset_mm = offset;
    pdev->customer.mm_config__outer_offset_mm = offset;
    Status = VL53LX_set_customer_nvm_managed(
               &(pdev->customer));
  }

  return Status;
}

VL53LX_Error VL53LX::VL53LX_SetCalibrationData(
  VL53LX_CalibrationData_t *pCalibrationData)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_CustomerNvmManaged_t          *pC;
  VL53LX_calibration_data_t            cal_data;
  uint32_t x;
  VL53LX_xtalk_calibration_results_t xtalk;

  cal_data.struct_version = pCalibrationData->struct_version -
                            VL53LX_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;


  memcpy(
    &(cal_data.add_off_cal_data),
    &(pCalibrationData->add_off_cal_data),
    sizeof(VL53LX_additional_offset_cal_data_t));


  memcpy(
    &(cal_data.optical_centre),
    &(pCalibrationData->optical_centre),
    sizeof(VL53LX_optical_centre_t));


  memcpy(
    &(cal_data.xtalkhisto),
    &(pCalibrationData->xtalkhisto),
    sizeof(VL53LX_xtalk_histogram_data_t));


  memcpy(
    &(cal_data.gain_cal),
    &(pCalibrationData->gain_cal),
    sizeof(VL53LX_gain_calibration_data_t));


  memcpy(
    &(cal_data.cal_peak_rate_map),
    &(pCalibrationData->cal_peak_rate_map),
    sizeof(VL53LX_cal_peak_rate_map_t));


  memcpy(
    &(cal_data.per_vcsel_cal_data),
    &(pCalibrationData->per_vcsel_cal_data),
    sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));

  pC = &pCalibrationData->customer;
  x = pC->algo__crosstalk_compensation_plane_offset_kcps;
  cal_data.customer.algo__crosstalk_compensation_plane_offset_kcps =
    (uint16_t)(x & 0x0000FFFF);

  cal_data.customer.global_config__spad_enables_ref_0 =
    pC->global_config__spad_enables_ref_0;
  cal_data.customer.global_config__spad_enables_ref_1 =
    pC->global_config__spad_enables_ref_1;
  cal_data.customer.global_config__spad_enables_ref_2 =
    pC->global_config__spad_enables_ref_2;
  cal_data.customer.global_config__spad_enables_ref_3 =
    pC->global_config__spad_enables_ref_3;
  cal_data.customer.global_config__spad_enables_ref_4 =
    pC->global_config__spad_enables_ref_4;
  cal_data.customer.global_config__spad_enables_ref_5 =
    pC->global_config__spad_enables_ref_5;
  cal_data.customer.global_config__ref_en_start_select =
    pC->global_config__ref_en_start_select;
  cal_data.customer.ref_spad_man__num_requested_ref_spads =
    pC->ref_spad_man__num_requested_ref_spads;
  cal_data.customer.ref_spad_man__ref_location =
    pC->ref_spad_man__ref_location;
  cal_data.customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
  cal_data.customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps;
  cal_data.customer.ref_spad_char__total_rate_target_mcps =
    pC->ref_spad_char__total_rate_target_mcps;
  cal_data.customer.algo__part_to_part_range_offset_mm =
    pC->algo__part_to_part_range_offset_mm;
  cal_data.customer.mm_config__inner_offset_mm =
    pC->mm_config__inner_offset_mm;
  cal_data.customer.mm_config__outer_offset_mm =
    pC->mm_config__outer_offset_mm;

  Status = VL53LX_set_part_to_part_data(&cal_data);

  if (Status != VL53LX_ERROR_NONE) {
    goto ENDFUNC;
  }

  Status = VL53LX_get_current_xtalk_settings(&xtalk);

  if (Status != VL53LX_ERROR_NONE) {
    goto ENDFUNC;
  }

  xtalk.algo__crosstalk_compensation_plane_offset_kcps = x;

  Status = VL53LX_set_tuning_parm(
             VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS,
             x);


  memcpy(
    &(xtalk.algo__xtalk_cpo_HistoMerge_kcps[0]),
    &(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps[0]),
    sizeof(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps));

  Status = VL53LX_set_current_xtalk_settings(&xtalk);
ENDFUNC:
  return Status;

}

VL53LX_Error VL53LX::VL53LX_GetCalibrationData(
  VL53LX_CalibrationData_t  *pCalibrationData)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  VL53LX_calibration_data_t      cal_data;
  VL53LX_CustomerNvmManaged_t         *pC;
  VL53LX_customer_nvm_managed_t       *pC2;
  VL53LX_xtalk_calibration_results_t xtalk;
  uint32_t                          tmp;

  Status = VL53LX_get_part_to_part_data(&cal_data);

  pCalibrationData->struct_version = cal_data.struct_version +
                                     VL53LX_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;


  memcpy(
    &(pCalibrationData->add_off_cal_data),
    &(cal_data.add_off_cal_data),
    sizeof(VL53LX_additional_offset_cal_data_t));


  memcpy(
    &(pCalibrationData->optical_centre),
    &(cal_data.optical_centre),
    sizeof(VL53LX_optical_centre_t));


  memcpy(
    &(pCalibrationData->xtalkhisto),
    &(cal_data.xtalkhisto),
    sizeof(VL53LX_xtalk_histogram_data_t));

  memcpy(
    &(pCalibrationData->gain_cal),
    &(cal_data.gain_cal),
    sizeof(VL53LX_gain_calibration_data_t));


  memcpy(
    &(pCalibrationData->cal_peak_rate_map),
    &(cal_data.cal_peak_rate_map),
    sizeof(VL53LX_cal_peak_rate_map_t));


  memcpy(
    &(pCalibrationData->per_vcsel_cal_data),
    &(cal_data.per_vcsel_cal_data),
    sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));

  pC = &pCalibrationData->customer;
  pC2 = &cal_data.customer;
  pC->global_config__spad_enables_ref_0 =
    pC2->global_config__spad_enables_ref_0;
  pC->global_config__spad_enables_ref_1 =
    pC2->global_config__spad_enables_ref_1;
  pC->global_config__spad_enables_ref_2 =
    pC2->global_config__spad_enables_ref_2;
  pC->global_config__spad_enables_ref_3 =
    pC2->global_config__spad_enables_ref_3;
  pC->global_config__spad_enables_ref_4 =
    pC2->global_config__spad_enables_ref_4;
  pC->global_config__spad_enables_ref_5 =
    pC2->global_config__spad_enables_ref_5;
  pC->global_config__ref_en_start_select =
    pC2->global_config__ref_en_start_select;
  pC->ref_spad_man__num_requested_ref_spads =
    pC2->ref_spad_man__num_requested_ref_spads;
  pC->ref_spad_man__ref_location =
    pC2->ref_spad_man__ref_location;
  pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC2->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC2->algo__crosstalk_compensation_y_plane_gradient_kcps;
  pC->ref_spad_char__total_rate_target_mcps =
    pC2->ref_spad_char__total_rate_target_mcps;
  pC->algo__part_to_part_range_offset_mm =
    pC2->algo__part_to_part_range_offset_mm;
  pC->mm_config__inner_offset_mm =
    pC2->mm_config__inner_offset_mm;
  pC->mm_config__outer_offset_mm =
    pC2->mm_config__outer_offset_mm;

  pC->algo__crosstalk_compensation_plane_offset_kcps =
    (uint32_t)(
      pC2->algo__crosstalk_compensation_plane_offset_kcps);

  Status = VL53LX_get_current_xtalk_settings(&xtalk);

  if (Status != VL53LX_ERROR_NONE) {
    goto ENDFUNC;
  }

  tmp = xtalk.algo__crosstalk_compensation_plane_offset_kcps;
  pC->algo__crosstalk_compensation_plane_offset_kcps = tmp;
  tmp = xtalk.algo__crosstalk_compensation_x_plane_gradient_kcps;
  pC->algo__crosstalk_compensation_x_plane_gradient_kcps = tmp;
  tmp = xtalk.algo__crosstalk_compensation_y_plane_gradient_kcps;
  pC->algo__crosstalk_compensation_y_plane_gradient_kcps = tmp;

  memcpy(&(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps[0]),
         &(xtalk.algo__xtalk_cpo_HistoMerge_kcps[0]),
         sizeof(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps));
ENDFUNC:
  return Status;
}


VL53LX_Error VL53LX::VL53LX_PerformOffsetPerVcselCalibration(
  int32_t CalDistanceMilliMeter)
{
  VL53LX_Error Status = VL53LX_ERROR_NONE;
  int32_t sum_ranging_range_A, sum_ranging_range_B;
  uint8_t offset_meas_range_A, offset_meas_range_B;
  int16_t Max, UnderMax, OverMax, Repeat;
  int32_t inloopcount;
  int32_t IncRounding;
  int16_t meanDistance_mm;
  VL53LX_MultiRangingData_t RangingMeasurementData;
  VL53LX_LLDriverData_t *pdev;
  uint8_t goodmeas;
  VL53LX_DistanceModes currentDist;
  VL53LX_DistanceModes DistMode[3] = {VL53LX_DISTANCEMODE_SHORT,
                                      VL53LX_DISTANCEMODE_MEDIUM, VL53LX_DISTANCEMODE_LONG
                                     };
  int16_t offsetA[3];
  int16_t offsetB[3];

  VL53LX_Error SmudgeStatus = VL53LX_ERROR_NONE;
  uint8_t smudge_corr_en, ics;
  VL53LX_TargetRangeData_t *pRange;

  pdev = VL53LXDevStructGetLLDriverHandle(Dev);

  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;
  SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable();

  pdev->customer.algo__part_to_part_range_offset_mm = 0;
  pdev->customer.mm_config__inner_offset_mm = 0;
  pdev->customer.mm_config__outer_offset_mm = 0;
  pdev->customer.mm_config__outer_offset_mm = 0;
  memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));

  Repeat = 0;
  Max = 2 * BDTable[
         VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
  UnderMax = 1 + (Max / 2);
  OverMax = Max + (Max / 2);

  Status = VL53LX_GetDistanceMode(&currentDist);

  while ((Repeat < 3) && (Status == VL53LX_ERROR_NONE)) {
    Status = VL53LX_SetDistanceMode(DistMode[Repeat]);
    Status = VL53LX_StartMeasurement();

    if (Status == VL53LX_ERROR_NONE) {
      VL53LX_WaitMeasurementDataReady();
      VL53LX_GetMultiRangingData(
        &RangingMeasurementData);
      VL53LX_ClearInterruptAndStartMeasurement();
    }

    inloopcount = 0;
    offset_meas_range_A = 0;
    sum_ranging_range_A = 0;
    offset_meas_range_B = 0;
    sum_ranging_range_B = 0;
    while ((Status == VL53LX_ERROR_NONE) && (inloopcount < Max) &&
           (inloopcount < OverMax)) {
      Status = VL53LX_WaitMeasurementDataReady();
      if (Status == VL53LX_ERROR_NONE)
        Status = VL53LX_GetMultiRangingData(
                   &RangingMeasurementData);
      pRange = &(RangingMeasurementData.RangeData[0]);
      goodmeas = (pRange->RangeStatus ==
                  VL53LX_RANGESTATUS_RANGE_VALID);
      ics = pdev->ll_state.cfg_internal_stream_count;
      if ((Status == VL53LX_ERROR_NONE) && goodmeas) {
        if (ics & 0x01) {
          sum_ranging_range_A +=
            pRange->RangeMilliMeter;
          offset_meas_range_A++;
        } else {
          sum_ranging_range_B +=
            pRange->RangeMilliMeter;
          offset_meas_range_B++;
        }
        inloopcount = offset_meas_range_A +
                      offset_meas_range_B;
      }
      Status = VL53LX_ClearInterruptAndStartMeasurement();
    }


    if (inloopcount < UnderMax) {
      Status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
    }

    VL53LX_StopMeasurement();


    if ((sum_ranging_range_A < 0) ||
        (sum_ranging_range_B < 0) ||
        (sum_ranging_range_A >
         ((int32_t) offset_meas_range_A * 0xffff)) ||
        (sum_ranging_range_B >
         ((int32_t) offset_meas_range_B * 0xffff))) {
      Status = VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
    }

    if ((Status == VL53LX_ERROR_NONE) &&
        (offset_meas_range_A > 0)) {
      IncRounding = offset_meas_range_A / 2;
      meanDistance_mm = (int16_t)
                        ((sum_ranging_range_A + IncRounding)
                         / offset_meas_range_A);
      offsetA[Repeat] = (int16_t)
                        CalDistanceMilliMeter - meanDistance_mm;
    }

    if ((Status == VL53LX_ERROR_NONE) &&
        (offset_meas_range_B > 0)) {
      IncRounding = offset_meas_range_B / 2;
      meanDistance_mm = (int16_t)
                        ((sum_ranging_range_B + IncRounding)
                         / offset_meas_range_B);
      offsetB[Repeat] = (int16_t)
                        CalDistanceMilliMeter - meanDistance_mm;
    }
    Repeat++;
  }

  if ((SmudgeStatus == VL53LX_ERROR_NONE) && (smudge_corr_en == 1)) {
    SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable();
  }

  if (Status == VL53LX_ERROR_NONE) {
    pdev->per_vcsel_cal_data.short_a_offset_mm  = offsetA[0];
    pdev->per_vcsel_cal_data.short_b_offset_mm  = offsetB[0];
    pdev->per_vcsel_cal_data.medium_a_offset_mm = offsetA[1];
    pdev->per_vcsel_cal_data.medium_b_offset_mm = offsetB[1];
    pdev->per_vcsel_cal_data.long_a_offset_mm   = offsetA[2];
    pdev->per_vcsel_cal_data.long_b_offset_mm   = offsetB[2];
  }

  VL53LX_SetDistanceMode(currentDist);

  return Status;
}
