/*******************************************************************************
 Copyright Ã‚Â© 2018, STMicroelectronics International N.V.
 All rights reserved.
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __VL53LX_CLASS_H
#define __VL53LX_CLASS_H

#ifdef _MSC_VER
  #ifdef VL53LX_API_EXPORTS
    #define VL53LX_API  __declspec(dllexport)
  #else
    #define VL53LX_API
  #endif
#else
  #define VL53LX_API
#endif

/* Includes ------------------------------------------------------------------*/
#include "vl53lx_def.h"

namespace AP_HAL {
class I2CDevice;
}

#define VL53LX_DEFAULT_DEVICE_ADDRESS           0x52


/* vl53lx_api_core.c */
#define VL53LX_MAX_I2C_XFER_SIZE 256
/* vl53lx_core.c */
// VL53LX_Error VL53LX_dynamic_xtalk_correction_corrector()
#define CONT_CONTINUE 0
#define CONT_NEXT_LOOP  1
#define CONT_RESET  2


/* vl53lx_api.c */

#ifndef MIN
  #define MIN(v1, v2) ((v1) < (v2) ? (v1) : (v2))
#endif
#ifndef MAX
  #define MAX(v1, v2) ((v1) < (v2) ? (v2) : (v1))
#endif

#define DMAX_REFLECTANCE_IDX 2



#define LOWPOWER_AUTO_VHV_LOOP_DURATION_US 245
#define LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING 1448
#define LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING 2100

#define FDA_MAX_TIMING_BUDGET_US 550000


#define VL53LX_NVM_POWER_UP_DELAY_US             50
#define VL53LX_NVM_READ_TRIGGER_DELAY_US          5


#define VL53LX_D_002  0xFFFF
#define VL53LX_D_008  0xFFFF
#define VL53LX_D_003  0xFFFFFF
#define VL53LX_D_007  0xFFFFFFFF
#define VL53LX_D_005  0x7FFFFFFFFF
#define VL53LX_D_009  0xFFFFFFFFFF
#define VL53LX_D_010  0xFFFFFFFFFFFF
#define VL53LX_D_004  0xFFFFFFFFFFFFFF
#define VL53LX_D_006  0x7FFFFFFFFFFFFFFF
#define VL53LX_D_011  0xFFFFFFFFFFFFFFFF



/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53LX sensor component
 */

class VL53LX {
  public:
    /** Constructor
     * @param[in] i2c device I2C to be used for communication
     * @param[in] pin shutdown pin to be used as component GPIO0
     */
    VL53LX(AP_HAL::I2CDevice *i2c) : dev_i2c(i2c)
    {
      Dev = &MyDevice;
      memset((void *)Dev, 0x0, sizeof(VL53LX_Dev_t));
      MyDevice.I2cHandle = i2c;
      MyDevice.I2cDevAddr = VL53LX_DEFAULT_DEVICE_ADDRESS ;
    }


    /** Destructor
     */
    virtual ~VL53LX() {}
    /* warning: VL53LX class inherits from GenericSensor, RangeSensor and LightSensor, that haven`t a destructor.
       The warning should request to introduce a virtual destructor to make sure to delete the object */

    virtual int begin()
    {
      return 0;
    }

    virtual int end()
    {
      return 0;
    }

    /*** Interface Methods ***/
    /*** High level API ***/
    /**
     * @brief       PowerOn the sensor
     * @return      void
     */
    /* turns on the sensor */
    virtual void VL53LX_On(void)
    {
    }

    /**
     * @brief       PowerOff the sensor
     * @return      void
     */
    /* turns off the sensor */
    virtual void VL53LX_Off(void)
    {
    }

    /**
     * @brief       Initialize the sensor with default values
     * @return      0 on Success
     */

    VL53LX_Error InitSensor(uint8_t address)
    {
      VL53LX_Error status = VL53LX_ERROR_NONE;
      VL53LX_Off();
      VL53LX_On();
      status = VL53LX_SetDeviceAddress(address);

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_WaitDeviceBooted();
      }

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_DataInit();
      }

      return status;
    }

    VL53LX_Error InitSensorAtAddress(uint8_t address)
    {
      VL53LX_Error status = VL53LX_ERROR_NONE;
      Dev->I2cDevAddr = address;

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_WaitDeviceBooted();
      }

      if (status == VL53LX_ERROR_NONE) {
        status = VL53LX_DataInit();
      }

      return status;
    }



    /**
     *
     * @brief One time device initialization
     * @param void
     * @return     0 on success
     */
    virtual int Init()
    {
      return VL53LX_DataInit();
    }



    /* Read function of the ID device */
    virtual int ReadID()
    {
      uint64_t Uid;
      VL53LX_GetUID(&Uid);
      if (Uid == 0x00FF000000FF) {
        return 0;
      }
      return -1;
    }



    /**
     * @brief Get ranging result and only that
     * @param pRange_mm  Pointer to range distance
     * @return           0 on success
     */
    virtual int GetDistance(uint32_t *piData)
    {
      (void)piData;
      return 0;
    }


    /* vl53lx_api.h */

    /** @defgroup VL53LX_group VL53LX Function Definition
     *  @brief    VL53LX Function Definition
     *  @{
     */

    /** @defgroup VL53LX_general_group VL53LX General Functions
     *  @brief    General functions and definitions
     *  @{
     */

    /**
     * @brief Return the VL53LX driver Version
     *
     * @note This function doesn't access to the device
     *
     * @param   pVersion              Rer to current driver Version
     * @return  VL53LX_Error_NONE     Success
     * @return  "Other error code"    See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetVersion(VL53LX_Version_t *pVersion);

    /**
     * @brief Reads the Product Revision for a for given Device
     * This function can be used to distinguish cut1.0 from cut1.1.
     *
     * @param   Dev                 Device Handle
     * @param   pProductRevisionMajor  Pointer to Product Revision Major
     * for a given Device
     * @param   pProductRevisionMinor  Pointer to Product Revision Minor
     * for a given Device
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"    See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetProductRevision(uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor);

    /**
     * @brief Reads the Device information for given Device
     *
     * @note This function Access to the device
     *
     * @param   Dev                 Device Handle
     * @param   pVL53LX_DeviceInfo  Pointer to current device info for a given
     *  Device
     * @return  VL53LX_Error_NONE   Success
     * @return  "Other error code"  See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetDeviceInfo(
      VL53LX_DeviceInfo_t *pVL53LX_DeviceInfo);

    /**
     * @brief Reads the Device unique identifier
     *
     * @note This function Access to the device
     *
     * @param   Dev                 Device Handle
     * @param   pUid                Pointer to current device unique ID
     * @return  VL53LX_Error_NONE   Success
     * @return  "Other error code"  See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetUID(uint64_t *pUid);


    /** @} VL53LX_general_group */

    /** @defgroup VL53LX_init_group VL53LX Init Functions
     *  @brief    VL53LX Init Functions
     *  @{
     */

    /**
     * @brief Set new device address
     *
     * After completion the device will answer to the new address programmed.
     * This function should be called when several devices are used in parallel
     * before start programming the sensor.
     * When a single device us used, there is no need to call this function.
     *
     * When it is requested for multi devices system this function MUST be called
     * prior to VL53LX_DataInit()
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   DeviceAddress         The new Device address
     * @return  VL53LX_Error_NONE     Success
     * @return  "Other error code"    See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetDeviceAddress(uint8_t DeviceAddress);

    /**
     *
     * @brief One time device initialization
     *
     * To be called after device has been powered on and booted
     * see @a VL53LX_WaitDeviceBooted()
     *
     * @par Function Description
     * When not used after a fresh device "power up", it may return
     * @a #VL53LX_Error_CALIBRATION_WARNING meaning wrong calibration data
     * may have been fetched from device that can result in ranging offset error\n
     * If VL53LX_DataInit is called several times then the application must restore
     * calibration calling @a VL53LX_SetOffsetCalibrationData()
     * It implies application has gathered calibration data thanks to
     * @a VL53LX_GetOffsetCalibrationData() after an initial calibration stage.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @return  VL53LX_Error_NONE     Success
     * @return  "Other error code"    See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_DataInit();

    /**
     * @brief Wait for device booted after chip enable (hardware standby)
     *
     * @param   Dev                   Device Handle
     * @return  VL53LX_Error_NONE     Success
     * @return  "Other error code"    See ::VL53LX_Error
     *
     */
    VL53LX_Error VL53LX_WaitDeviceBooted();


    /** @} VL53LX_init_group */

    /** @defgroup VL53LX_parameters_group VL53LX Parameters Functions
     *  @brief    Functions used to prepare and setup the device
     *  @{
     */

    /**
     * @brief  Set the distance mode
     * @par Function Description
     * Set the distance mode to be used for the next ranging.<br>
     * The modes Short, Medium and Long are used to optimize the ranging accuracy
     * in a specific range of distance.<br> The user select one of these modes to
     * select the distance range.
     * @note This function doesn't Access to the device
     *
     * @warning This function should be called after @a VL53LX_DataInit().

     * @param   Dev                   Device Handle
     * @param   DistanceMode          Distance mode to apply, valid values are:
     * @li VL53LX_DISTANCEMODE_SHORT
     * @li VL53LX_DISTANCEMODE_MEDIUM
     * @li VL53LX_DISTANCEMODE_LONG
     * @return  VL53LX_Error_NONE               Success
     * @return  VL53LX_Error_MODE_NOT_SUPPORTED This error occurs when DistanceMode
     *                                          is not in the supported list
     * @return  "Other error code"              See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetDistanceMode(
      VL53LX_DistanceModes DistanceMode);

    /**
     * @brief  Get the distance mode
     * @par Function Description
     * Get the distance mode used for the next ranging.
     *
     * @param   Dev                   Device Handle
     * @param   *pDistanceMode        Pointer to Distance mode
     * @return  VL53LX_Error_NONE            Success
     * @return  "Other error code"           See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetDistanceMode(
      VL53LX_DistanceModes *pDistanceMode);


    /**
     * @brief Set Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Defines the maximum time allowed by the user to the device to run a
     * full ranging sequence for the current mode (ranging, histogram, ASL ...)
     *
     * @param   Dev                                Device Handle
     * @param MeasurementTimingBudgetMicroSeconds  Max measurement time in
     * microseconds.
     * @return  VL53LX_Error_NONE            Success
     * @return  VL53LX_Error_INVALID_PARAMS  Error timing parameter not
     *                                       supported.
     *                                       The maximum accepted value for the
     *                                       computed timing budget is 10 seconds
     *                                       the minimum value depends on the preset
     *                                       mode selected.
     * @return  "Other error code"           See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds);

    /**
     * @brief Get Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Returns the programmed the maximum time allowed by the user to the
     * device to run a full ranging sequence for the current mode
     * (ranging, histogram, ...)
     *
     * @param   Dev                                    Device Handle
     * @param   pMeasurementTimingBudgetMicroSeconds   Max measurement time in
     * microseconds.
     * @return  VL53LX_Error_NONE            Success
     * @return  "Other error code"           See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetMeasurementTimingBudgetMicroSeconds(uint32_t *pMeasurementTimingBudgetMicroSeconds);
    /** @} VL53LX_parameters_group */


    /** @defgroup VL53LX_measurement_group VL53LX Measurement Functions
     *  @brief    Functions used for the measurements
     *  @{
     */

    /**
     * @brief Start device measurement
     *
     * @details Started measurement will depend on distance parameter set through
     * @a VL53LX_SetDistanceMode()
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @return  VL53LX_Error_NONE                  Success
     * @return  VL53LX_Error_TIME_OUT    Time out on start measurement
     * @return  VL53LX_Error_INVALID_PARAMS This error might occur in timed mode
     * when inter measurement period is smaller or too close to the timing budget.
     * In such case measurements are not started and user must correct the timings
     * passed to @a VL53LX_SetMeasurementTimingBudgetMicroSeconds() and
     * @a VL53LX_SetInterMeasurementPeriodMilliSeconds() functions.
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_StartMeasurement();

    /**
     * @brief Stop device measurement
     *
     * @details Will set the device in standby mode at end of current measurement\n
     *          Not necessary in single mode as device shall return automatically
     *          in standby mode at end of measurement.
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @return  VL53LX_Error_NONE    Success
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_StopMeasurement();

    /**
     * @brief Clear the Interrupt flag and start new measurement
     * *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @return  VL53LX_Error_NONE    Success
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement();

    /**
     * @brief Return Measurement Data Ready
     *
     * @par Function Description
     * This function indicate that a measurement data is ready.
     * This function is used for non-blocking capture.
     *
     * @note This function Access to the device
     *
     * @param   Dev                    Device Handle
     * @param   pMeasurementDataReady  Pointer to Measurement Data Ready.
     * 0 = data not ready, 1 = data ready
     * @return  VL53LX_Error_NONE      Success
     * @return  "Other error code"     See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetMeasurementDataReady(
      uint8_t *pMeasurementDataReady);

    /**
     * @brief Wait for measurement data ready.
     * Blocking function.
     * Note that the timeout is given by:
     * VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS defined in def.h
     *
     *
     * @note This function Access to the device
     *
     * @param   Dev      Device Handle
     * @return  VL53LX_Error_NONE        Success
     * @return  VL53LX_Error_TIME_OUT In case of timeout
     */
    VL53LX_Error VL53LX_WaitMeasurementDataReady();


    /**
     * @brief Retrieve all measurements from device with the current setup
     *
     * @par Function Description
     * Get data from last successful Ranging measurement
     *
     * @warning USER must call @a VL53LX_ClearInterruptAndStartMeasurement() prior
     * to call again this function
     *
     * @note This function Access to the device
     *
     * @note The first valid value returned by this function will have a range
     * status equal to VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK which means that
     * the data is valid but no wrap around check have been done. User should take
     * care about that.
     *
     * @param   Dev                      Device Handle
     * @param   pMultiRangingData        Pointer to the data structure to fill up.
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetMultiRangingData(VL53LX_MultiRangingData_t *pMultiRangingData);

    /**
     * @brief Get Additional Data
     *
     * @par Function Description
     * This function is used to get lld debugging data on the last histogram
     * measurement. shall be called when a new measurement is ready (interrupt or
     * positive VL53LX_GetMeasurementDataReady() polling) and before a call to
     * VL53LX_ClearInterruptAndStartMeasurement().
     *
     * @param   Dev                      Device Handle
     * @param   pAdditionalData          Pointer to Additional data
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    //VL53LX_Error VL53LX_GetAdditionalData( VL53LX_AdditionalData_t *pAdditionalData);


    /** @} VL53LX_measurement_group */

    /** @defgroup VL53LX_Calibration_group VL53LX Calibration Functions
     *  @brief    Functions used for Calibration
     *  @{
     */


    /**
     * @brief Set Tuning Parameter value for a given parameter ID
     *
     * @par Function Description
     * This function is used to improve the performance of the device. It permit to
     * change a particular value used for a timeout or a threshold or a constant
     * in an algorithm. The function will change the value of the parameter
     * identified by an unique ID.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                          Device Handle
     * @param   TuningParameterId            Tuning Parameter ID
     * @param   TuningParameterValue         Tuning Parameter Value
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetTuningParameter(
      uint16_t TuningParameterId, int32_t TuningParameterValue);

    /**
     * @brief Get Tuning Parameter value for a given parameter ID
     *
     * @par Function Description
     * This function is used to get the value of the parameter
     * identified by an unique ID.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                          Device Handle
     * @param   TuningParameterId            Tuning Parameter ID
     * @param   pTuningParameterValue        Pointer to Tuning Parameter Value
     * for a given TuningParameterId.
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetTuningParameter(
      uint16_t TuningParameterId, int32_t *pTuningParameterValue);

    /**
     * @brief Performs Reference Spad Management
     *
     * @par Function Description
     * The reference SPAD initialization procedure determines the minimum amount
     * of reference spads to be enables to achieve a target reference signal rate
     * and should be performed once during initialization.
     *
     * @note This function Access to the device
     *
     * @param   Dev                          Device Handle
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_PerformRefSpadManagement();

    /**
     * @brief Enable/Disable dynamic Xtalk compensation feature
     *
     * Enable/Disable dynamic Xtalk compensation (aka smudge correction).
     *
     * @param   Dev    Device Handle
     * @param   Mode   Set the smudge correction mode
     * See ::VL53LX_SmudgeCorrectionModes
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SmudgeCorrectionEnable(
      VL53LX_SmudgeCorrectionModes Mode);


    /**
     * @brief Enable/Disable Cross talk compensation feature
     *
     * Enable/Disable Cross Talk correction.
     *
     * @param   Dev                       Device Handle
     * @param   XTalkCompensationEnable   Cross talk compensation
     *  to be set 0 = disabled or 1 = enabled.
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetXTalkCompensationEnable(
      uint8_t XTalkCompensationEnable);

    /**
     * @brief Get Cross talk compensation rate enable
     *
     * Get if the Cross Talk is Enabled or Disabled.
     *
     * @note This function doesn't access to the device
     *
     * @param   Dev                        Device Handle
     * @param   pXTalkCompensationEnable   Pointer to the Cross talk compensation
     *  state 0=disabled or 1 = enabled
     * @return  VL53LX_Error_NONE        Success
     * @return  "Other error code"       See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetXTalkCompensationEnable(
      uint8_t *pXTalkCompensationEnable);

    /**
     * @brief Perform XTalk Calibration
     *
     * @details Perform a XTalk calibration of the Device.
     * This function will launch a  measurement, if interrupts
     * are enabled an interrupt will be done.
     * This function will clear the interrupt generated automatically.
     * This function will program a new value for the XTalk compensation
     * and it will enable the cross talk before exit.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * the calibration sets appropriate
     * distance mode and thus override existing one<br>
     * The calibration uses a target which should be located at least @60cm from the
     * device. The actual location of the target shall be passed
     * through the bare driver tuning parameters table
     *
     * @return  VL53LX_Error_NONE    Success
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_PerformXTalkCalibration();


    /**
     * @brief Define the mode to be used for the offset correction
     *
     * Define the mode to be used for the offset correction.
     *
     * @param   Dev                       Device Handle
     * @param   OffsetCorrectionMode      Offset Correction Mode valid values are:
     * @li                                VL53LX_OFFSETCORRECTIONMODE_STANDARD
     * @li                                VL53LX_OFFSETCORRECTIONMODE_PERVCSEL
     *
     * @return  VL53LX_Error_NONE         Success
     * @return  "Other error code"        See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetOffsetCorrectionMode(
      VL53LX_OffsetCorrectionModes OffsetCorrectionMode);


    /**
     * @brief Perform Offset simple Calibration
     *
     * @details Perform a very simple offset calibration of the Device.
     * This function will launch few ranging measurements and computes offset
     * calibration. The preset mode and the distance mode MUST be set by the
     * application before to call this function.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @param   CalDistanceMilliMeter     Calibration distance value used for the
     * offset compensation.
     *
     * @return  VL53LX_Error_NONE
     * @return  VL53LX_Error_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
     * lack of valid measurements
     * @return  VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
     * distance combined to the number of loops performed in the calibration lead to
     * an internal overflow. Try to reduce the distance of the target (140 mm)
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_PerformOffsetSimpleCalibration(
      int32_t CalDistanceMilliMeter);

    /**
     * @brief Perform Offset simple Calibration with a "zero distance" target
     *
     * @details Perform a simple offset calibration of the Device.
     * This function will launch few ranging measurements and computes offset
     * calibration. The preset mode and the distance mode MUST be set by the
     * application before to call this function.
     * A target must be place very close to the device.
     * Ideally the target shall be touching the coverglass.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     *
     * @return  VL53LX_Error_NONE
     * @return  VL53LX_Error_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
     * lack of valid measurements
     * @return  VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
     * distance is too large, try to put the target closer to the device
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_PerformOffsetZeroDistanceCalibration();


    /**
     * @brief Perform Offset per Vcsel Calibration. i.e. per distance mode
     *
     * @details Perform offset calibration of the Device depending on the
     * three distance mode settings: short, medium and long.
     * This function will launch few ranging measurements and computes offset
     * calibration in each of the three distance modes.
     * The preset mode MUST be set by the application before to call this function.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @param   CalDistanceMilliMeter     Distance of the target used for the
     * offset compensation calibration.
     *
     * @return  VL53LX_Error_NONE
     * @return  VL53LX_Error_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
     * lack of valid measurements
     * @return  VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
     * distance combined to the number of loops performed in the calibration lead to
     * an internal overflow. Try to reduce the distance of the target (140 mm)
     * @return  "Other error code"   See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_PerformOffsetPerVcselCalibration(
      int32_t CalDistanceMilliMeter);


    /**
     * @brief Sets the Calibration Data.
     *
     * @par Function Description
     * This function set all the Calibration Data issued from the functions
     * @a VL53LX_PerformRefSpadManagement(), @a VL53LX_PerformXTalkCalibration,
     * @a VL53LX_PerformOffsetCalibration()
     *
     * @note This function doesn't Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   *pCalibrationData            Pointer to Calibration data to be set.
     * @return  VL53LX_Error_NONE            Success
     * @return  VL53LX_Error_INVALID_PARAMS  pCalibrationData points to an older
     * version of the inner structure. Need for support to convert its content.
     * @return  "Other error code"           See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_SetCalibrationData(
      VL53LX_CalibrationData_t *pCalibrationData);

    /**
     * @brief Gets the Calibration Data.
     *
     * @par Function Description
     * This function get all the Calibration Data issued from the functions
     * @a VL53LX_PerformRefSpadManagement(), @a VL53LX_PerformXTalkCalibration,
     * @a VL53LX_PerformOffsetCalibration()
     *
     * @note This function doesn't Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   *pCalibrationData            pointer where to store Calibration
     *  data.
     * @return  VL53LX_Error_NONE            Success
     * @return  "Other error code"           See ::VL53LX_Error
     */
    VL53LX_Error VL53LX_GetCalibrationData(
      VL53LX_CalibrationData_t  *pCalibrationData);

    /** @} VL53LX_Calibration_group */

    /** @} VL53LX_group */


  protected:

    /* vl53lx_api_core.h functions */

    VL53LX_Error VL53LX_get_version(VL53LX_ll_version_t  *pversion);
    VL53LX_Error VL53LX_get_device_firmware_version(uint16_t *pfw_version);
    VL53LX_Error VL53LX_data_init(uint8_t read_p2p_data);
    VL53LX_Error VL53LX_read_p2p_data();
    VL53LX_Error VL53LX_software_reset();
    VL53LX_Error VL53LX_set_part_to_part_data(VL53LX_calibration_data_t *pcal_data);
    VL53LX_Error VL53LX_get_part_to_part_data(VL53LX_calibration_data_t *pcal_data);
    VL53LX_Error VL53LX_get_tuning_debug_data(VL53LX_tuning_parameters_t *ptun_data);
    VL53LX_Error VL53LX_set_inter_measurement_period_ms(uint32_t inter_measurement_period_ms);
    VL53LX_Error VL53LX_get_inter_measurement_period_ms(uint32_t *pinter_measurement_period_ms);
    VL53LX_Error VL53LX_set_timeouts_us(
      uint32_t            phasecal_config_timeout_us,
      uint32_t            mm_config_timeout_us,
      uint32_t            range_config_timeout_us);

    VL53LX_Error VL53LX_get_timeouts_us(
      uint32_t           *pphasecal_config_timeout_us,
      uint32_t           *pmm_config_timeout_us,
      uint32_t           *prange_config_timeout_us);

    VL53LX_Error VL53LX_set_calibration_repeat_period(
      uint16_t            cal_config__repeat_period);

    VL53LX_Error VL53LX_get_calibration_repeat_period(
      uint16_t           *pcal_config__repeat_period);

    VL53LX_Error VL53LX_set_sequence_config_bit(
      VL53LX_DeviceSequenceConfig  bit_id,
      uint8_t                      value);

    VL53LX_Error VL53LX_get_sequence_config_bit(
      VL53LX_DeviceSequenceConfig  bit_id,
      uint8_t                     *pvalue);

    VL53LX_Error VL53LX_set_interrupt_polarity(
      VL53LX_DeviceInterruptPolarity  interrupt_polarity);

    VL53LX_Error VL53LX_get_interrupt_polarity(
      VL53LX_DeviceInterruptPolarity  *pinterrupt_polarity);

    VL53LX_Error VL53LX_get_refspadchar_config_struct(
      VL53LX_refspadchar_config_t   *pdata);

    VL53LX_Error VL53LX_set_refspadchar_config_struct(
      VL53LX_refspadchar_config_t   *pdata);

    VL53LX_Error VL53LX_set_range_ignore_threshold(
      uint8_t                 range_ignore_thresh_mult,
      uint16_t                range_ignore_threshold_mcps);

    VL53LX_Error VL53LX_get_range_ignore_threshold(
      uint8_t                *prange_ignore_thresh_mult,
      uint16_t               *prange_ignore_threshold_mcps_internal,
      uint16_t               *prange_ignore_threshold_mcps_current);

    VL53LX_Error VL53LX_set_user_zone(
      VL53LX_user_zone_t *puser_zone);

    VL53LX_Error VL53LX_get_user_zone(
      VL53LX_user_zone_t *puser_zone);


    VL53LX_Error VL53LX_get_mode_mitigation_roi(
      VL53LX_user_zone_t *pmm_roi);


    VL53LX_Error VL53LX_set_zone_config(
      VL53LX_zone_config_t  *pzone_cfg);

    VL53LX_Error VL53LX_get_zone_config(
      VL53LX_zone_config_t  *pzone_cfg);


    VL53LX_Error VL53LX_set_preset_mode(
      VL53LX_DevicePresetModes     device_preset_mode,
      uint16_t                     dss_config__target_total_rate_mcps,
      uint32_t                     phasecal_config_timeout_us,
      uint32_t                     mm_config_timeout_us,
      uint32_t                     range_config_timeout_us,
      uint32_t                     inter_measurement_period_ms);

    VL53LX_Error VL53LX_get_preset_mode_timing_cfg(
      VL53LX_DevicePresetModes     device_preset_mode,
      uint16_t                    *pdss_config__target_total_rate_mcps,
      uint32_t                    *pphasecal_config_timeout_us,
      uint32_t                    *pmm_config_timeout_us,
      uint32_t                    *prange_config_timeout_us);


    VL53LX_Error VL53LX_set_zone_preset(
      VL53LX_DeviceZonePreset  zone_preset);


    VL53LX_Error VL53LX_enable_xtalk_compensation();

    VL53LX_Error VL53LX_disable_xtalk_compensation();


    void VL53LX_get_xtalk_compensation_enable(
      uint8_t       *pcrosstalk_compensation_enable);


    VL53LX_Error VL53LX_init_and_start_range(
      uint8_t                         measurement_mode,
      VL53LX_DeviceConfigLevel        device_config_level);


    VL53LX_Error VL53LX_stop_range();

    VL53LX_Error VL53LX_get_measurement_results(
      VL53LX_DeviceResultsLevel   device_result_level);


    VL53LX_Error VL53LX_get_device_results(
      VL53LX_DeviceResultsLevel  device_result_level,
      VL53LX_range_results_t    *prange_results);


    VL53LX_Error VL53LX_clear_interrupt_and_enable_next_range(
      uint8_t          measurement_mode);


    VL53LX_Error VL53LX_get_histogram_bin_data(
      VL53LX_histogram_bin_data_t *phist_data);

    void VL53LX_copy_sys_and_core_results_to_range_results(
      int32_t                           gain_factor,
      VL53LX_system_results_t          *psys,
      VL53LX_core_results_t            *pcore,
      VL53LX_range_results_t           *presults);



    VL53LX_Error VL53LX_set_zone_dss_config(
      VL53LX_zone_private_dyn_cfg_t  *pzone_dyn_cfg);


    VL53LX_Error VL53LX_calc_ambient_dmax(
      uint16_t      target_reflectance,
      int16_t      *pambient_dmax_mm);


    VL53LX_Error VL53LX_set_GPIO_interrupt_config(
      VL53LX_GPIO_Interrupt_Mode  intr_mode_distance,
      VL53LX_GPIO_Interrupt_Mode  intr_mode_rate,
      uint8_t       intr_new_measure_ready,
      uint8_t       intr_no_target,
      uint8_t       intr_combined_mode,
      uint16_t      thresh_distance_high,
      uint16_t      thresh_distance_low,
      uint16_t      thresh_rate_high,
      uint16_t      thresh_rate_low
    );



    VL53LX_Error VL53LX_set_GPIO_interrupt_config_struct(
      VL53LX_GPIO_interrupt_config_t  intconf);


    VL53LX_Error VL53LX_get_GPIO_interrupt_config(
      VL53LX_GPIO_interrupt_config_t  *pintconf);


    VL53LX_Error VL53LX_set_dmax_mode(
      VL53LX_DeviceDmaxMode   dmax_mode);


    VL53LX_Error VL53LX_get_dmax_mode(
      VL53LX_DeviceDmaxMode   *pdmax_mode);


    VL53LX_Error VL53LX_get_dmax_calibration_data(
      VL53LX_DeviceDmaxMode           dmax_mode,
      VL53LX_dmax_calibration_data_t *pdmax_cal);


    VL53LX_Error VL53LX_set_hist_dmax_config(
      VL53LX_hist_gen3_dmax_config_t *pdmax_cfg);

    VL53LX_Error VL53LX_get_hist_dmax_config(
      VL53LX_hist_gen3_dmax_config_t *pdmax_cfg);


    VL53LX_Error VL53LX_set_offset_calibration_mode(
      VL53LX_OffsetCalibrationMode   offset_cal_mode);


    VL53LX_Error VL53LX_get_offset_calibration_mode(
      VL53LX_OffsetCalibrationMode  *poffset_cal_mode);


    VL53LX_Error VL53LX_set_offset_correction_mode(
      VL53LX_OffsetCalibrationMode   offset_cor_mode);


    VL53LX_Error VL53LX_get_offset_correction_mode(
      VL53LX_OffsetCorrectionMode  *poffset_cor_mode);


    VL53LX_Error VL53LX_set_zone_calibration_data(
      VL53LX_zone_calibration_results_t *pzone_cal);


    VL53LX_Error VL53LX_get_zone_calibration_data(
      VL53LX_zone_calibration_results_t *pzone_cal);


    VL53LX_Error VL53LX_get_lite_xtalk_margin_kcps(
      int16_t                           *pxtalk_margin);


    VL53LX_Error VL53LX_set_lite_xtalk_margin_kcps(
      int16_t                             xtalk_margin);


    VL53LX_Error VL53LX_get_histogram_xtalk_margin_kcps(
      int16_t                           *pxtalk_margin);


    VL53LX_Error VL53LX_set_histogram_xtalk_margin_kcps(
      int16_t                             xtalk_margin);

    VL53LX_Error VL53LX_get_histogram_phase_consistency(
      uint8_t                            *pphase_consistency);



    VL53LX_Error VL53LX_set_histogram_phase_consistency(
      uint8_t                             phase_consistency);



    VL53LX_Error VL53LX_get_histogram_event_consistency(
      uint8_t                            *pevent_consistency);



    VL53LX_Error VL53LX_set_histogram_event_consistency(
      uint8_t                             event_consistency);



    VL53LX_Error VL53LX_get_histogram_ambient_threshold_sigma(
      uint8_t                            *pamb_thresh_sigma);



    VL53LX_Error VL53LX_set_histogram_ambient_threshold_sigma(

      uint8_t                             amb_thresh_sigma);



    VL53LX_Error VL53LX_get_lite_min_count_rate(

      uint16_t                           *plite_mincountrate);




    VL53LX_Error VL53LX_set_lite_min_count_rate(

      uint16_t                            lite_mincountrate);





    VL53LX_Error VL53LX_get_lite_sigma_threshold(

      uint16_t                           *plite_sigma);




    VL53LX_Error VL53LX_set_lite_sigma_threshold(

      uint16_t                            lite_sigma);




    VL53LX_Error VL53LX_restore_xtalk_nvm_default();



    VL53LX_Error VL53LX_get_xtalk_detect_config(

      int16_t                            *pmax_valid_range_mm,
      int16_t                            *pmin_valid_range_mm,
      uint16_t                           *pmax_valid_rate_kcps,
      uint16_t                           *pmax_sigma_mm);



    VL53LX_Error VL53LX_set_xtalk_detect_config(

      int16_t                             max_valid_range_mm,
      int16_t                             min_valid_range_mm,
      uint16_t                            max_valid_rate_kcps,
      uint16_t                            max_sigma_mm);



    VL53LX_Error VL53LX_get_target_order_mode(

      VL53LX_HistTargetOrder             *phist_target_order);



    VL53LX_Error VL53LX_set_target_order_mode(

      VL53LX_HistTargetOrder              hist_target_order);




    VL53LX_Error VL53LX_set_dmax_reflectance_values(

      VL53LX_dmax_reflectance_array_t    *pdmax_reflectances);



    VL53LX_Error VL53LX_get_dmax_reflectance_values(

      VL53LX_dmax_reflectance_array_t    *pdmax_reflectances);



    VL53LX_Error VL53LX_set_vhv_config(
      uint8_t                      vhv_init_en,
      uint8_t                      vhv_init_value);



    VL53LX_Error VL53LX_get_vhv_config(
      uint8_t                     *pvhv_init_en,
      uint8_t                     *pvhv_init_value);



    VL53LX_Error VL53LX_set_vhv_loopbound(

      uint8_t                      vhv_loopbound);



    VL53LX_Error VL53LX_get_vhv_loopbound(

      uint8_t                     *pvhv_loopbound);



    VL53LX_Error VL53LX_get_tuning_parm(
      VL53LX_TuningParms             tuning_parm_key,
      int32_t                       *ptuning_parm_value);



    VL53LX_Error VL53LX_set_tuning_parm(
      VL53LX_TuningParms             tuning_parm_key,
      int32_t                        tuning_parm_value);



    VL53LX_Error VL53LX_dynamic_xtalk_correction_enable();



    VL53LX_Error VL53LX_dynamic_xtalk_correction_disable();




    VL53LX_Error VL53LX_dynamic_xtalk_correction_apply_enable();



    VL53LX_Error VL53LX_dynamic_xtalk_correction_apply_disable();



    VL53LX_Error VL53LX_dynamic_xtalk_correction_single_apply_enable();



    VL53LX_Error VL53LX_dynamic_xtalk_correction_single_apply_disable();



    VL53LX_Error VL53LX_dynamic_xtalk_correction_set_scalers(
      int16_t   x_scaler_in,
      int16_t   y_scaler_in,
      uint8_t   user_scaler_set_in
    );



    VL53LX_Error VL53LX_get_current_xtalk_settings(

      VL53LX_xtalk_calibration_results_t *pxtalk
    );



    VL53LX_Error VL53LX_set_current_xtalk_settings(

      VL53LX_xtalk_calibration_results_t *pxtalk
    );

    VL53LX_Error VL53LX_load_patch();

    VL53LX_Error VL53LX_unload_patch();


    /* vl53lx_register_funcs.h */


    VL53LX_Error VL53LX_i2c_encode_static_nvm_managed(
      VL53LX_static_nvm_managed_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_static_nvm_managed(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_static_nvm_managed_t  *pdata);




    VL53LX_Error VL53LX_set_static_nvm_managed(
      VL53LX_static_nvm_managed_t  *pdata);




    VL53LX_Error VL53LX_get_static_nvm_managed(
      VL53LX_static_nvm_managed_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_customer_nvm_managed(
      VL53LX_customer_nvm_managed_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_customer_nvm_managed(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_customer_nvm_managed_t  *pdata);




    VL53LX_Error VL53LX_set_customer_nvm_managed(
      VL53LX_customer_nvm_managed_t  *pdata);




    VL53LX_Error VL53LX_get_customer_nvm_managed(
      VL53LX_customer_nvm_managed_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_static_config(
      VL53LX_static_config_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_static_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_static_config_t    *pdata);




    VL53LX_Error VL53LX_set_static_config(
      VL53LX_static_config_t    *pdata);




    VL53LX_Error VL53LX_get_static_config(
      VL53LX_static_config_t    *pdata);




    VL53LX_Error VL53LX_i2c_encode_general_config(
      VL53LX_general_config_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_general_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_general_config_t   *pdata);




    VL53LX_Error VL53LX_set_general_config(
      VL53LX_general_config_t   *pdata);




    VL53LX_Error VL53LX_get_general_config(
      VL53LX_general_config_t   *pdata);




    VL53LX_Error VL53LX_i2c_encode_timing_config(
      VL53LX_timing_config_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_timing_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_timing_config_t    *pdata);




    VL53LX_Error VL53LX_set_timing_config(
      VL53LX_timing_config_t    *pdata);




    VL53LX_Error VL53LX_get_timing_config(
      VL53LX_timing_config_t    *pdata);




    VL53LX_Error VL53LX_i2c_encode_dynamic_config(
      VL53LX_dynamic_config_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_dynamic_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_dynamic_config_t   *pdata);




    VL53LX_Error VL53LX_set_dynamic_config(
      VL53LX_dynamic_config_t   *pdata);




    VL53LX_Error VL53LX_get_dynamic_config(
      VL53LX_dynamic_config_t   *pdata);




    VL53LX_Error VL53LX_i2c_encode_system_control(
      VL53LX_system_control_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_system_control(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_system_control_t   *pdata);




    VL53LX_Error VL53LX_set_system_control(
      VL53LX_system_control_t   *pdata);




    VL53LX_Error VL53LX_get_system_control(
      VL53LX_system_control_t   *pdata);




    VL53LX_Error VL53LX_i2c_encode_system_results(
      VL53LX_system_results_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_system_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_system_results_t   *pdata);




    VL53LX_Error VL53LX_set_system_results(
      VL53LX_system_results_t   *pdata);




    VL53LX_Error VL53LX_get_system_results(
      VL53LX_system_results_t   *pdata);




    VL53LX_Error VL53LX_i2c_encode_core_results(
      VL53LX_core_results_t     *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_core_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_core_results_t     *pdata);




    VL53LX_Error VL53LX_set_core_results(
      VL53LX_core_results_t     *pdata);




    VL53LX_Error VL53LX_get_core_results(
      VL53LX_core_results_t     *pdata);




    VL53LX_Error VL53LX_i2c_encode_debug_results(
      VL53LX_debug_results_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_debug_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_debug_results_t    *pdata);




    VL53LX_Error VL53LX_set_debug_results(
      VL53LX_debug_results_t    *pdata);




    VL53LX_Error VL53LX_get_debug_results(
      VL53LX_debug_results_t    *pdata);




    VL53LX_Error VL53LX_i2c_encode_nvm_copy_data(
      VL53LX_nvm_copy_data_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_nvm_copy_data(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_nvm_copy_data_t    *pdata);




    VL53LX_Error VL53LX_set_nvm_copy_data(
      VL53LX_nvm_copy_data_t    *pdata);




    VL53LX_Error VL53LX_get_nvm_copy_data(
      VL53LX_nvm_copy_data_t    *pdata);




    VL53LX_Error VL53LX_i2c_encode_prev_shadow_system_results(
      VL53LX_prev_shadow_system_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_prev_shadow_system_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_prev_shadow_system_results_t  *pdata);




    VL53LX_Error VL53LX_set_prev_shadow_system_results(
      VL53LX_prev_shadow_system_results_t  *pdata);




    VL53LX_Error VL53LX_get_prev_shadow_system_results(
      VL53LX_prev_shadow_system_results_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_prev_shadow_core_results(
      VL53LX_prev_shadow_core_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_prev_shadow_core_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_prev_shadow_core_results_t  *pdata);




    VL53LX_Error VL53LX_set_prev_shadow_core_results(
      VL53LX_prev_shadow_core_results_t  *pdata);




    VL53LX_Error VL53LX_get_prev_shadow_core_results(
      VL53LX_prev_shadow_core_results_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_patch_debug(
      VL53LX_patch_debug_t      *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_patch_debug(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_patch_debug_t      *pdata);




    VL53LX_Error VL53LX_set_patch_debug(
      VL53LX_patch_debug_t      *pdata);




    VL53LX_Error VL53LX_get_patch_debug(
      VL53LX_patch_debug_t      *pdata);




    VL53LX_Error VL53LX_i2c_encode_gph_general_config(
      VL53LX_gph_general_config_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_gph_general_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_gph_general_config_t  *pdata);




    VL53LX_Error VL53LX_set_gph_general_config(
      VL53LX_gph_general_config_t  *pdata);




    VL53LX_Error VL53LX_get_gph_general_config(
      VL53LX_gph_general_config_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_gph_static_config(
      VL53LX_gph_static_config_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_gph_static_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_gph_static_config_t  *pdata);




    VL53LX_Error VL53LX_set_gph_static_config(
      VL53LX_gph_static_config_t  *pdata);




    VL53LX_Error VL53LX_get_gph_static_config(
      VL53LX_gph_static_config_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_gph_timing_config(
      VL53LX_gph_timing_config_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_gph_timing_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_gph_timing_config_t  *pdata);




    VL53LX_Error VL53LX_set_gph_timing_config(
      VL53LX_gph_timing_config_t  *pdata);




    VL53LX_Error VL53LX_get_gph_timing_config(
      VL53LX_gph_timing_config_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_fw_internal(
      VL53LX_fw_internal_t      *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_fw_internal(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_fw_internal_t      *pdata);




    VL53LX_Error VL53LX_set_fw_internal(
      VL53LX_fw_internal_t      *pdata);




    VL53LX_Error VL53LX_get_fw_internal(
      VL53LX_fw_internal_t      *pdata);




    VL53LX_Error VL53LX_i2c_encode_patch_results(
      VL53LX_patch_results_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_patch_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_patch_results_t    *pdata);




    VL53LX_Error VL53LX_set_patch_results(
      VL53LX_patch_results_t    *pdata);




    VL53LX_Error VL53LX_get_patch_results(
      VL53LX_patch_results_t    *pdata);




    VL53LX_Error VL53LX_i2c_encode_shadow_system_results(
      VL53LX_shadow_system_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_shadow_system_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_shadow_system_results_t  *pdata);




    VL53LX_Error VL53LX_set_shadow_system_results(
      VL53LX_shadow_system_results_t  *pdata);




    VL53LX_Error VL53LX_get_shadow_system_results(
      VL53LX_shadow_system_results_t  *pdata);




    VL53LX_Error VL53LX_i2c_encode_shadow_core_results(
      VL53LX_shadow_core_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53LX_Error VL53LX_i2c_decode_shadow_core_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_shadow_core_results_t  *pdata);




    VL53LX_Error VL53LX_set_shadow_core_results(
      VL53LX_shadow_core_results_t  *pdata);




    VL53LX_Error VL53LX_get_shadow_core_results(
      VL53LX_shadow_core_results_t  *pdata);



    /* vl53lx_nvm.h */


    VL53LX_Error VL53LX_nvm_enable(
      uint16_t       nvm_ctrl_pulse_width,
      int32_t        nvm_power_up_delay_us);




    VL53LX_Error VL53LX_nvm_read(
      uint8_t        start_address,
      uint8_t        count,
      uint8_t       *pdata);




    VL53LX_Error VL53LX_nvm_disable();




    VL53LX_Error VL53LX_nvm_format_decode(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53LX_decoded_nvm_data_t *pdata);




    VL53LX_Error VL53LX_nvm_decode_optical_centre(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53LX_optical_centre_t             *pdata);




    VL53LX_Error VL53LX_nvm_decode_cal_peak_rate_map(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53LX_cal_peak_rate_map_t          *pdata);




    VL53LX_Error VL53LX_nvm_decode_additional_offset_cal_data(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53LX_additional_offset_cal_data_t *pdata);




    VL53LX_Error VL53LX_nvm_decode_fmt_range_results_data(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53LX_decoded_nvm_fmt_range_data_t *pdata);




    VL53LX_Error VL53LX_nvm_decode_fmt_info(
      uint16_t                       buf_size,
      uint8_t                       *pbuffer,
      VL53LX_decoded_nvm_fmt_info_t *pdata);




    VL53LX_Error VL53LX_nvm_decode_ews_info(
      uint16_t                       buf_size,
      uint8_t                       *pbuffer,
      VL53LX_decoded_nvm_ews_info_t *pdata);




    void VL53LX_nvm_format_encode(
      VL53LX_decoded_nvm_data_t *pnvm_info,
      uint8_t                   *pnvm_data);




    VL53LX_Error VL53LX_read_nvm_raw_data(
      uint8_t        start_address,
      uint8_t        count,
      uint8_t       *pnvm_raw_data);




    VL53LX_Error VL53LX_read_nvm(
      uint8_t                    nvm_format,
      VL53LX_decoded_nvm_data_t *pnvm_info);




    VL53LX_Error VL53LX_read_nvm_optical_centre(
      VL53LX_optical_centre_t             *pcentre);




    VL53LX_Error VL53LX_read_nvm_cal_peak_rate_map(
      VL53LX_cal_peak_rate_map_t          *pcal_data);




    VL53LX_Error VL53LX_read_nvm_additional_offset_cal_data(
      VL53LX_additional_offset_cal_data_t *pcal_data);




    VL53LX_Error VL53LX_read_nvm_fmt_range_results_data(
      uint16_t                             range_results_select,
      VL53LX_decoded_nvm_fmt_range_data_t *prange_data);



    /* vl53lx_platform_ipp.h */

    VL53LX_Error VL53LX_ipp_hist_process_data(
      VL53LX_dmax_calibration_data_t    *pdmax_cal,
      VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
      VL53LX_hist_post_process_config_t *ppost_cfg,
      VL53LX_histogram_bin_data_t       *pbins,
      VL53LX_xtalk_histogram_data_t     *pxtalk,
      uint8_t                           *pArea1,
      uint8_t                           *pArea2,
      uint8_t                           *phisto_merge_nb,
      VL53LX_range_results_t            *presults);

    VL53LX_Error VL53LX_ipp_hist_ambient_dmax(
      uint16_t                           target_reflectance,
      VL53LX_dmax_calibration_data_t    *pdmax_cal,
      VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
      VL53LX_histogram_bin_data_t       *pbins,
      int16_t                           *pambient_dmax_mm);

    VL53LX_Error VL53LX_ipp_xtalk_calibration_process_data(
      VL53LX_xtalk_range_results_t       *pxtalk_ranges,
      VL53LX_xtalk_histogram_data_t      *pxtalk_shape,
      VL53LX_xtalk_calibration_results_t *pxtalk_cal);

    VL53LX_Error VL53LX_ipp_hist_xtalk_correction(
      VL53LX_customer_nvm_managed_t *pcustomer,
      VL53LX_dynamic_config_t       *pdyn_cfg,
      VL53LX_xtalk_histogram_data_t *pxtalk_shape,
      VL53LX_histogram_bin_data_t   *pip_hist_data,
      VL53LX_histogram_bin_data_t   *pop_hist_data,
      VL53LX_histogram_bin_data_t   *pxtalk_count_data);

    VL53LX_Error VL53LX_ipp_generate_dual_reflectance_xtalk_samples(
      VL53LX_xtalk_range_results_t  *pxtalk_results,
      uint16_t                       expected_target_distance_mm,
      uint8_t                        higher_reflectance,
      VL53LX_histogram_bin_data_t   *pxtalk_avg_samples);


    /* vl53lx_hist_funcs.h */


    VL53LX_Error VL53LX_hist_process_data(
      VL53LX_dmax_calibration_data_t    *pdmax_cal,
      VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
      VL53LX_hist_post_process_config_t *ppost_cfg,
      VL53LX_histogram_bin_data_t       *pbins,
      VL53LX_xtalk_histogram_data_t     *pxtalk,
      uint8_t                           *pArea1,
      uint8_t                           *pArea2,
      VL53LX_range_results_t            *presults,
      uint8_t                           *HistMergeNumber);


    VL53LX_Error VL53LX_hist_ambient_dmax(
      uint16_t                            target_reflectance,
      VL53LX_dmax_calibration_data_t     *pdmax_cal,
      VL53LX_hist_gen3_dmax_config_t     *pdmax_cfg,
      VL53LX_histogram_bin_data_t        *pbins,
      int16_t                            *pambient_dmax_mm);


    /* vl53lx_core_support.h */

    uint32_t VL53LX_calc_pll_period_us(
      uint16_t fast_osc_frequency);

    uint32_t VL53LX_duration_maths(
      uint32_t  pll_period_us,
      uint32_t  vcsel_parm_pclks,
      uint32_t  window_vclks,
      uint32_t  periods_elapsed_mclks);

    uint32_t VL53LX_events_per_spad_maths(
      int32_t   VL53LX_p_010,
      uint16_t  num_spads,
      uint32_t  duration);

    uint32_t VL53LX_isqrt(
      uint32_t  num);

    void VL53LX_hist_calc_zero_distance_phase(
      VL53LX_histogram_bin_data_t    *pdata);

    void VL53LX_hist_estimate_ambient_from_thresholded_bins(
      int32_t                      ambient_threshold_sigma,
      VL53LX_histogram_bin_data_t *pdata);

    void VL53LX_hist_remove_ambient_bins(
      VL53LX_histogram_bin_data_t    *pdata);

    uint32_t VL53LX_calc_pll_period_mm(
      uint16_t fast_osc_frequency);

    uint16_t VL53LX_rate_maths(
      int32_t   VL53LX_p_018,
      uint32_t  time_us);

    uint16_t VL53LX_rate_per_spad_maths(
      uint32_t  frac_bits,
      uint32_t  peak_count_rate,
      uint16_t  num_spads,
      uint32_t  max_output_value);

    int32_t VL53LX_range_maths(
      uint16_t  fast_osc_frequency,
      uint16_t  VL53LX_p_014,
      uint16_t  zero_distance_phase,
      uint8_t   fractional_bits,
      int32_t   gain_factor,
      int32_t   range_offset_mm);

    uint8_t VL53LX_decode_vcsel_period(
      uint8_t vcsel_period_reg);

    void VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
      VL53LX_xtalk_histogram_shape_t *pxtalk,
      VL53LX_histogram_bin_data_t    *phist);

    void VL53LX_init_histogram_bin_data_struct(
      int32_t                      bin_value,
      uint16_t                     VL53LX_p_021,
      VL53LX_histogram_bin_data_t *pdata);

    void VL53LX_decode_row_col(
      uint8_t   spad_number,
      uint8_t  *prow,
      uint8_t  *pcol);

    void VL53LX_hist_find_min_max_bin_values(
      VL53LX_histogram_bin_data_t   *pdata);

    void VL53LX_hist_estimate_ambient_from_ambient_bins(
      VL53LX_histogram_bin_data_t    *pdata);



    /* vl53lx_core.h */

    void VL53LX_init_version();

    void VL53LX_init_ll_driver_state(
      VL53LX_DeviceState ll_state);

    VL53LX_Error VL53LX_update_ll_driver_rd_state();

    VL53LX_Error VL53LX_check_ll_driver_rd_state();

    VL53LX_Error VL53LX_update_ll_driver_cfg_state();

    void VL53LX_copy_rtn_good_spads_to_buffer(
      VL53LX_nvm_copy_data_t  *pdata,
      uint8_t                 *pbuffer);

    void VL53LX_init_system_results(
      VL53LX_system_results_t      *pdata);

    void V53L1_init_zone_results_structure(
      uint8_t                 active_zones,
      VL53LX_zone_results_t  *pdata);

    void V53L1_init_zone_dss_configs();

    void VL53LX_init_histogram_config_structure(
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
      VL53LX_histogram_config_t  *pdata);

    void VL53LX_init_histogram_multizone_config_structure(
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
      VL53LX_histogram_config_t  *pdata);

    void VL53LX_init_xtalk_bin_data_struct(
      uint32_t                        bin_value,
      uint16_t                        VL53LX_p_021,
      VL53LX_xtalk_histogram_shape_t *pdata);

    void VL53LX_i2c_encode_uint16_t(
      uint16_t    ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);

    uint16_t VL53LX_i2c_decode_uint16_t(
      uint16_t    count,
      uint8_t    *pbuffer);

    void VL53LX_i2c_encode_int16_t(
      int16_t     ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);

    int16_t VL53LX_i2c_decode_int16_t(
      uint16_t    count,
      uint8_t    *pbuffer);

    void VL53LX_i2c_encode_uint32_t(
      uint32_t    ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);

    uint32_t VL53LX_i2c_decode_uint32_t(
      uint16_t    count,
      uint8_t    *pbuffer);

    uint32_t VL53LX_i2c_decode_with_mask(
      uint16_t    count,
      uint8_t    *pbuffer,
      uint32_t    bit_mask,
      uint32_t    down_shift,
      uint32_t    offset);

    void VL53LX_i2c_encode_int32_t(
      int32_t     ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);

    int32_t VL53LX_i2c_decode_int32_t(
      uint16_t    count,
      uint8_t    *pbuffer);

    VL53LX_Error VL53LX_start_test(
      uint8_t        test_mode__ctrl);

    VL53LX_Error VL53LX_set_firmware_enable_register(uint8_t value);

    VL53LX_Error VL53LX_enable_firmware();

    VL53LX_Error VL53LX_disable_firmware();

    VL53LX_Error VL53LX_set_powerforce_register(
      uint8_t            value);

    VL53LX_Error VL53LX_enable_powerforce();

    VL53LX_Error VL53LX_disable_powerforce();

    VL53LX_Error VL53LX_clear_interrupt();

    VL53LX_Error VL53LX_force_shadow_stream_count_to_zero();

    uint32_t VL53LX_calc_macro_period_us(
      uint16_t fast_osc_frequency,
      uint8_t  VL53LX_p_005);

    uint16_t VL53LX_calc_range_ignore_threshold(
      uint32_t central_rate,
      int16_t  x_gradient,
      int16_t  y_gradient,
      uint8_t  rate_mult);

    uint32_t VL53LX_calc_timeout_mclks(
      uint32_t  timeout_us,
      uint32_t  macro_period_us);

    uint16_t VL53LX_calc_encoded_timeout(
      uint32_t  timeout_us,
      uint32_t  macro_period_us);

    uint32_t VL53LX_calc_timeout_us(
      uint32_t  timeout_mclks,
      uint32_t  macro_period_us);

    uint32_t VL53LX_calc_decoded_timeout_us(
      uint16_t  timeout_encoded,
      uint32_t  macro_period_us);

    uint16_t VL53LX_encode_timeout(
      uint32_t timeout_mclks);

    uint32_t VL53LX_decode_timeout(
      uint16_t encoded_timeout);

    VL53LX_Error  VL53LX_calc_timeout_register_values(
      uint32_t                 phasecal_config_timeout_us,
      uint32_t                 mm_config_timeout_us,
      uint32_t                 range_config_timeout_us,
      uint16_t                 fast_osc_frequency,
      VL53LX_general_config_t *pgeneral,
      VL53LX_timing_config_t  *ptiming);

    uint8_t VL53LX_encode_vcsel_period(
      uint8_t VL53LX_p_030);

    uint32_t VL53LX_decode_unsigned_integer(
      uint8_t  *pbuffer,
      uint8_t   no_of_bytes);

    void   VL53LX_encode_unsigned_integer(
      uint32_t  ip_value,
      uint8_t   no_of_bytes,
      uint8_t  *pbuffer);

    VL53LX_Error VL53LX_hist_copy_and_scale_ambient_info(
      VL53LX_zone_hist_info_t        *pidata,
      VL53LX_histogram_bin_data_t    *podata);

    void  VL53LX_hist_get_bin_sequence_config(
      VL53LX_histogram_bin_data_t   *pdata);

    VL53LX_Error  VL53LX_hist_phase_consistency_check(
      VL53LX_zone_hist_info_t     *phist_prev,
      VL53LX_zone_objects_t       *prange_prev,
      VL53LX_range_results_t      *prange_curr);

    VL53LX_Error  VL53LX_hist_events_consistency_check(
      uint8_t                      event_sigma,
      uint16_t                     min_effective_spad_count,
      VL53LX_zone_hist_info_t     *phist_prev,
      VL53LX_object_data_t        *prange_prev,
      VL53LX_range_data_t         *prange_curr,
      int32_t                     *pevents_tolerance,
      int32_t                     *pevents_delta,
      VL53LX_DeviceError          *prange_status);

    VL53LX_Error  VL53LX_hist_merged_pulse_check(
      int16_t                      min_max_tolerance_mm,
      VL53LX_range_data_t         *pdata,
      VL53LX_DeviceError          *prange_status);

    VL53LX_Error  VL53LX_hist_xmonitor_consistency_check(
      VL53LX_zone_hist_info_t     *phist_prev,
      VL53LX_zone_objects_t       *prange_prev,
      VL53LX_range_data_t         *prange_curr);

    VL53LX_Error  VL53LX_hist_wrap_dmax(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_histogram_bin_data_t       *pcurrent,
      int16_t                           *pwrap_dmax_mm);

    void VL53LX_hist_combine_mm1_mm2_offsets(
      int16_t                              mm1_offset_mm,
      int16_t                              mm2_offset_mm,
      uint8_t                              encoded_mm_roi_centre,
      uint8_t                              encoded_mm_roi_size,
      uint8_t                              encoded_zone_centre,
      uint8_t                              encoded_zone_size,
      VL53LX_additional_offset_cal_data_t *pcal_data,
      uint8_t                             *pgood_spads,
      uint16_t                             aperture_attenuation,
      int16_t                             *prange_offset_mm);

    VL53LX_Error VL53LX_hist_xtalk_extract_calc_window(
      int16_t                             target_distance_mm,
      uint16_t                            target_width_oversize,
      VL53LX_histogram_bin_data_t        *phist_bins,
      VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);

    VL53LX_Error VL53LX_hist_xtalk_extract_calc_event_sums(
      VL53LX_histogram_bin_data_t        *phist_bins,
      VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);

    VL53LX_Error VL53LX_hist_xtalk_extract_calc_rate_per_spad(
      VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);

    VL53LX_Error VL53LX_hist_xtalk_extract_calc_shape(
      VL53LX_hist_xtalk_extract_data_t  *pxtalk_data,
      VL53LX_xtalk_histogram_shape_t    *pxtalk_shape);

    VL53LX_Error VL53LX_hist_xtalk_shape_model(
      uint16_t                         events_per_bin,
      uint16_t                         pulse_centre,
      uint16_t                         pulse_width,
      VL53LX_xtalk_histogram_shape_t  *pxtalk_shape);

    uint16_t VL53LX_hist_xtalk_shape_model_interp(
      uint16_t      events_per_bin,
      uint32_t      phase_delta);

    void VL53LX_spad_number_to_byte_bit_index(
      uint8_t  spad_number,
      uint8_t *pbyte_index,
      uint8_t *pbit_index,
      uint8_t *pbit_mask);

    void VL53LX_encode_row_col(
      uint8_t  row,
      uint8_t  col,
      uint8_t *pspad_number);

    void VL53LX_decode_zone_size(
      uint8_t   encoded_xy_size,
      uint8_t  *pwidth,
      uint8_t  *pheight);

    void VL53LX_encode_zone_size(
      uint8_t  width,
      uint8_t  height,
      uint8_t *pencoded_xy_size);

    void VL53LX_decode_zone_limits(
      uint8_t   encoded_xy_centre,
      uint8_t   encoded_xy_size,
      int16_t  *px_ll,
      int16_t  *py_ll,
      int16_t  *px_ur,
      int16_t  *py_ur);

    uint8_t VL53LX_is_aperture_location(
      uint8_t   row,
      uint8_t   col);

    void VL53LX_calc_max_effective_spads(
      uint8_t     encoded_zone_centre,
      uint8_t     encoded_zone_size,
      uint8_t    *pgood_spads,
      uint16_t    aperture_attenuation,
      uint16_t   *pmax_effective_spads);

    void VL53LX_calc_mm_effective_spads(
      uint8_t     encoded_mm_roi_centre,
      uint8_t     encoded_mm_roi_size,
      uint8_t     encoded_zone_centre,
      uint8_t     encoded_zone_size,
      uint8_t    *pgood_spads,
      uint16_t    aperture_attenuation,
      uint16_t   *pmm_inner_effective_spads,
      uint16_t   *pmm_outer_effective_spads);

    void VL53LX_hist_copy_results_to_sys_and_core(
      VL53LX_histogram_bin_data_t      *pbins,
      VL53LX_range_results_t           *phist,
      VL53LX_system_results_t          *psys,
      VL53LX_core_results_t            *pcore);

    VL53LX_Error VL53LX_sum_histogram_data(
      VL53LX_histogram_bin_data_t *phist_input,
      VL53LX_histogram_bin_data_t *phist_output);

    VL53LX_Error VL53LX_avg_histogram_data(
      uint8_t no_of_samples,
      VL53LX_histogram_bin_data_t *phist_sum,
      VL53LX_histogram_bin_data_t *phist_avg);

    VL53LX_Error VL53LX_save_cfg_data();

    VL53LX_Error VL53LX_dynamic_zone_update(
      VL53LX_range_results_t *presults);

    VL53LX_Error VL53LX_update_internal_stream_counters(
      uint8_t     external_stream_count,
      uint8_t     *pinternal_stream_count,
      uint8_t     *pinternal_stream_count_val
    );

    VL53LX_Error VL53LX_multizone_hist_bins_update();

    VL53LX_Error VL53LX_set_histogram_multizone_initial_bin_config(
      VL53LX_zone_config_t           *pzone_cfg,
      VL53LX_histogram_config_t      *phist_cfg,
      VL53LX_histogram_config_t      *pmulti_hist
    );

    uint8_t VL53LX_encode_GPIO_interrupt_config(
      VL53LX_GPIO_interrupt_config_t  *pintconf);

    VL53LX_GPIO_interrupt_config_t VL53LX_decode_GPIO_interrupt_config(
      uint8_t   system__interrupt_config);

    VL53LX_Error VL53LX_set_GPIO_distance_threshold(
      uint16_t      threshold_high,
      uint16_t      threshold_low);

    VL53LX_Error VL53LX_set_GPIO_rate_threshold(
      uint16_t      threshold_high,
      uint16_t      threshold_low);

    VL53LX_Error VL53LX_set_GPIO_thresholds_from_struct(
      VL53LX_GPIO_interrupt_config_t *pintconf);

    VL53LX_Error VL53LX_set_ref_spad_char_config(
      uint8_t       vcsel_period_a,
      uint32_t      phasecal_timeout_us,
      uint16_t      total_rate_target_mcps,
      uint16_t      max_count_rate_rtn_limit_mcps,
      uint16_t      min_count_rate_rtn_limit_mcps,
      uint16_t      fast_osc_frequency);

    VL53LX_Error VL53LX_set_ssc_config(
      VL53LX_ssc_config_t *pssc_cfg,
      uint16_t             fast_osc_frequency);

    VL53LX_Error VL53LX_get_spad_rate_data(
      VL53LX_spad_rate_data_t  *pspad_rates);

    uint32_t VL53LX_calc_crosstalk_plane_offset_with_margin(
      uint32_t     plane_offset_kcps,
      int16_t      margin_offset_kcps);

    VL53LX_Error VL53LX_low_power_auto_data_init();

    VL53LX_Error VL53LX_low_power_auto_data_stop_range();

    VL53LX_Error VL53LX_dynamic_xtalk_correction_calc_required_samples();

    VL53LX_Error VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
      uint32_t        xtalk_offset_out,
      VL53LX_smudge_corrector_config_t  *pconfig,
      VL53LX_smudge_corrector_data_t    *pout,
      uint8_t         add_smudge,
      uint8_t         soft_update
    );

    VL53LX_Error VL53LX_dynamic_xtalk_correction_corrector();

    VL53LX_Error VL53LX_dynamic_xtalk_correction_data_init();

    VL53LX_Error VL53LX_dynamic_xtalk_correction_output_init(
      VL53LX_LLDriverResults_t *pres
    );

    VL53LX_Error VL53LX_xtalk_cal_data_init();

    VL53LX_Error VL53LX_config_low_power_auto_mode(
      VL53LX_general_config_t   *pgeneral,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_low_power_auto_data_t *plpadata
    );

    VL53LX_Error VL53LX_low_power_auto_setup_manual_calibration();

    VL53LX_Error VL53LX_low_power_auto_update_DSS();

    VL53LX_Error VL53LX_compute_histo_merge_nb(uint8_t *histo_merge_nb);



    /* vl53lx_wait.h */

    VL53LX_Error VL53LX_wait_for_boot_completion();

    VL53LX_Error VL53LX_wait_for_firmware_ready();

    VL53LX_Error VL53LX_wait_for_range_completion();

    VL53LX_Error VL53LX_wait_for_test_completion();

    VL53LX_Error VL53LX_is_boot_complete(uint8_t        *pready);

    VL53LX_Error VL53LX_is_firmware_ready(uint8_t        *pready);

    VL53LX_Error VL53LX_is_new_data_ready(uint8_t        *pready);

    VL53LX_Error VL53LX_poll_for_boot_completion(uint32_t        timeout_ms);

    VL53LX_Error VL53LX_poll_for_firmware_ready(uint32_t        timeout_ms);

    VL53LX_Error VL53LX_poll_for_range_completion(uint32_t     timeout_ms);


    /* vl53lx_zone_presets.h */

    VL53LX_Error VL53LX_init_zone_config_structure(
      uint8_t x_off,
      uint8_t x_inc,
      uint8_t x_zones,
      uint8_t y_off,
      uint8_t y_inc,
      uint8_t y_zones,
      uint8_t width,
      uint8_t height,
      VL53LX_zone_config_t   *pdata);


    VL53LX_Error VL53LX_zone_preset_xtalk_planar(
      VL53LX_general_config_t *pgeneral,
      VL53LX_zone_config_t    *pzone_cfg);



    VL53LX_Error VL53LX_init_zone_config_histogram_bins(
      VL53LX_zone_config_t   *pdata);


    /* vl53lx_api_preset_modes.h */

    VL53LX_Error VL53LX_init_refspadchar_config_struct(
      VL53LX_refspadchar_config_t     *pdata);

    VL53LX_Error VL53LX_init_ssc_config_struct(
      VL53LX_ssc_config_t     *pdata);

    VL53LX_Error VL53LX_init_xtalk_config_struct(
      VL53LX_customer_nvm_managed_t *pnvm,
      VL53LX_xtalk_config_t   *pdata);

    VL53LX_Error VL53LX_init_xtalk_extract_config_struct(
      VL53LX_xtalkextract_config_t   *pdata);

    VL53LX_Error VL53LX_init_offset_cal_config_struct(
      VL53LX_offsetcal_config_t   *pdata);

    VL53LX_Error VL53LX_init_zone_cal_config_struct(
      VL53LX_zonecal_config_t   *pdata);

    VL53LX_Error VL53LX_init_hist_post_process_config_struct(
      uint8_t                              xtalk_compensation_enable,
      VL53LX_hist_post_process_config_t   *pdata);

    VL53LX_Error VL53LX_init_dmax_calibration_data_struct(
      VL53LX_dmax_calibration_data_t   *pdata);

    VL53LX_Error VL53LX_init_tuning_parm_storage_struct(
      VL53LX_tuning_parm_storage_t   *pdata);

    VL53LX_Error VL53LX_init_hist_gen3_dmax_config_struct(
      VL53LX_hist_gen3_dmax_config_t   *pdata);

    VL53LX_Error VL53LX_preset_mode_standard_ranging(
      VL53LX_static_config_t     *pstatic,
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic,
      VL53LX_system_control_t    *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t       *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_standard_ranging_short_range(
      VL53LX_static_config_t     *pstatic,
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic,
      VL53LX_system_control_t    *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t       *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_standard_ranging_long_range(
      VL53LX_static_config_t     *pstatic,
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic,
      VL53LX_system_control_t    *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t       *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_standard_ranging_mm1_cal(
      VL53LX_static_config_t     *pstatic,
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic,
      VL53LX_system_control_t    *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t       *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_standard_ranging_mm2_cal(
      VL53LX_static_config_t     *pstatic,
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic,
      VL53LX_system_control_t    *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t       *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_timed_ranging(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_timed_ranging_short_range(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_timed_ranging_long_range(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_low_power_auto_ranging(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg,
      VL53LX_low_power_auto_data_t *plpadata);

    VL53LX_Error VL53LX_preset_mode_low_power_auto_short_ranging(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg,
      VL53LX_low_power_auto_data_t *plpadata);

    VL53LX_Error VL53LX_preset_mode_low_power_auto_long_ranging(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg,
      VL53LX_low_power_auto_data_t *plpadata);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging_with_mm1(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging_with_mm2(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging_mm1_cal(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging_mm2_cal(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging_ref(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_characterisation(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_xtalk_planar(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_xtalk_mm1(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_xtalk_mm2(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_multizone(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_multizone_short_range(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_multizone_long_range(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_ranging_short_timing(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_long_range(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_medium_range(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_short_range(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_special_histogram_short_range(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_long_range_mm1(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_long_range_mm2(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_medium_range_mm1(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_medium_range_mm2(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_short_range_mm1(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_histogram_short_range_mm2(
      VL53LX_hist_post_process_config_t *phistpostprocess,
      VL53LX_static_config_t            *pstatic,
      VL53LX_histogram_config_t         *phistogram,
      VL53LX_general_config_t           *pgeneral,
      VL53LX_timing_config_t            *ptiming,
      VL53LX_dynamic_config_t           *pdynamic,
      VL53LX_system_control_t           *psystem,
      VL53LX_tuning_parm_storage_t      *ptuning_parms,
      VL53LX_zone_config_t              *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_olt(
      VL53LX_static_config_t     *pstatic,
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic,
      VL53LX_system_control_t    *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t       *pzone_cfg);

    VL53LX_Error VL53LX_preset_mode_singleshot_ranging(

      VL53LX_static_config_t    *pstatic,
      VL53LX_histogram_config_t *phistogram,
      VL53LX_general_config_t   *pgeneral,
      VL53LX_timing_config_t    *ptiming,
      VL53LX_dynamic_config_t   *pdynamic,
      VL53LX_system_control_t   *psystem,
      VL53LX_tuning_parm_storage_t *ptuning_parms,
      VL53LX_zone_config_t      *pzone_cfg);

    void VL53LX_copy_hist_cfg_to_static_cfg(
      VL53LX_histogram_config_t  *phistogram,
      VL53LX_static_config_t     *pstatic,
      VL53LX_general_config_t    *pgeneral,
      VL53LX_timing_config_t     *ptiming,
      VL53LX_dynamic_config_t    *pdynamic);

    void VL53LX_copy_hist_bins_to_static_cfg(
      VL53LX_histogram_config_t *phistogram,
      VL53LX_static_config_t    *pstatic,
      VL53LX_timing_config_t    *ptiming);


    /* vl53lx_silicon_core.h */
    VL53LX_Error VL53LX_is_firmware_ready_silicon(
      uint8_t        *pready);


    /* vl53lx_hist_core.h */

    void  VL53LX_f_022(
      uint8_t                         VL53LX_p_032,
      uint8_t                         filter_woi,
      VL53LX_histogram_bin_data_t    *pbins,
      int32_t                        *pa,
      int32_t                        *pb,
      int32_t                        *pc);

    VL53LX_Error VL53LX_f_018(
      uint16_t                        vcsel_width,
      uint16_t                        fast_osc_frequency,
      uint32_t                        total_periods_elapsed,
      uint16_t                        VL53LX_p_004,
      VL53LX_range_data_t            *pdata);

    void VL53LX_f_019(
      uint16_t             gain_factor,
      int16_t              range_offset_mm,
      VL53LX_range_data_t *pdata);

    void  VL53LX_f_029(
      VL53LX_histogram_bin_data_t   *pdata,
      int32_t                        ambient_estimate_counts_per_bin);

    void  VL53LX_f_005(
      VL53LX_histogram_bin_data_t   *pxtalk,
      VL53LX_histogram_bin_data_t   *pbins,
      VL53LX_histogram_bin_data_t   *pxtalk_realigned);

    int8_t  VL53LX_f_030(
      VL53LX_histogram_bin_data_t   *pdata1,
      VL53LX_histogram_bin_data_t   *pdata2);

    VL53LX_Error  VL53LX_f_031(
      VL53LX_histogram_bin_data_t   *pidata,
      VL53LX_histogram_bin_data_t   *podata);


    /* vl53lx_xtalk.h */

    VL53LX_Error VL53LX_xtalk_calibration_process_data(
      VL53LX_xtalk_range_results_t        *pxtalk_ranges,
      VL53LX_xtalk_histogram_data_t       *pxtalk_shape,
      VL53LX_xtalk_calibration_results_t  *pxtalk_cal);

    VL53LX_Error VL53LX_f_041(
      VL53LX_histogram_bin_data_t        *pavg_bins,
      VL53LX_xtalk_algo_data_t           *pdebug,
      VL53LX_xtalk_range_data_t          *pxtalk_data,
      uint8_t                             histogram__window_start,
      uint8_t                             histogram__window_end,
      VL53LX_xtalk_histogram_shape_t     *pxtalk_shape);

    VL53LX_Error VL53LX_f_039(
      VL53LX_xtalk_range_results_t  *pxtalk_results,
      VL53LX_xtalk_algo_data_t      *pdebug,
      int16_t                       *xgradient,
      int16_t                       *ygradient);

    VL53LX_Error VL53LX_f_040(
      VL53LX_xtalk_range_data_t *pxtalk_data,
      VL53LX_xtalk_algo_data_t  *pdebug,
      uint32_t                  *xtalk_mean_offset_kcps);

    VL53LX_Error VL53LX_f_045(
      VL53LX_histogram_bin_data_t    *phist_data,
      VL53LX_xtalk_range_data_t      *pxtalk_data,
      VL53LX_xtalk_algo_data_t       *pdebug,
      VL53LX_xtalk_histogram_shape_t *pxtalk_histo);

    VL53LX_Error VL53LX_f_032(
      uint32_t                       mean_offset,
      int16_t                        xgradient,
      int16_t                        ygradient,
      int8_t                         centre_offset_x,
      int8_t                         centre_offset_y,
      uint16_t                       roi_effective_spads,
      uint8_t                        roi_centre_spad,
      uint8_t                        roi_xy_size,
      uint32_t                      *xtalk_rate_kcps);

    VL53LX_Error VL53LX_f_033(
      VL53LX_histogram_bin_data_t    *phist_data,
      VL53LX_xtalk_histogram_shape_t *pxtalk_data,
      uint32_t                        xtalk_rate_kcps,
      VL53LX_histogram_bin_data_t    *pxtalkcount_data);

    VL53LX_Error VL53LX_f_047(
      VL53LX_histogram_bin_data_t   *phist_data,
      VL53LX_histogram_bin_data_t   *pxtalk_data,
      uint8_t                        xtalk_bin_offset);

    VL53LX_Error VL53LX_f_044(
      VL53LX_histogram_bin_data_t       *pxtalk_data,
      uint32_t                           amb_threshold,
      uint8_t                            VL53LX_p_019,
      uint8_t                            VL53LX_p_024);

    VL53LX_Error VL53LX_f_046(
      VL53LX_customer_nvm_managed_t *pcustomer,
      VL53LX_dynamic_config_t       *pdyn_cfg,
      VL53LX_xtalk_histogram_data_t *pxtalk_shape,
      VL53LX_histogram_bin_data_t   *pip_hist_data,
      VL53LX_histogram_bin_data_t   *pop_hist_data,
      VL53LX_histogram_bin_data_t   *pxtalk_count_data);

    VL53LX_Error VL53LX_f_043(
      uint8_t                      sigma_mult,
      int32_t                      VL53LX_p_028,
      uint32_t                    *ambient_noise);


    VL53LX_Error VL53LX_generate_dual_reflectance_xtalk_samples(
      VL53LX_xtalk_range_results_t *pxtalk_results,
      uint16_t                      expected_target_distance_mm,
      uint8_t                       higher_reflectance,
      VL53LX_histogram_bin_data_t  *pxtalk_avg_samples
    );

    VL53LX_Error VL53LX_f_042(
      VL53LX_histogram_bin_data_t   *pzone_avg_1,
      VL53LX_histogram_bin_data_t   *pzone_avg_2,
      uint16_t                             expected_target_distance,
      uint8_t                              subtract_amb,
      uint8_t                              higher_reflectance,
      VL53LX_histogram_bin_data_t     *pxtalk_output
    );


    /* vl53lx_sigma_estimate.h */

    uint16_t VL53LX_f_034(
      uint8_t  sigma_estimator__effective_pulse_width_ns,
      uint8_t  sigma_estimator__effective_ambient_width_ns,
      uint8_t  sigma_estimator__sigma_ref_mm,
      VL53LX_range_data_t  *pdata);

    uint16_t VL53LX_f_036(
      uint8_t  sigma_estimator__effective_pulse_width_ns,
      uint8_t  sigma_estimator__effective_ambient_width_ns,
      uint8_t  sigma_estimator__sigma_ref_mm,
      VL53LX_range_data_t *pdata);

    VL53LX_Error  VL53LX_f_037(
      uint8_t       sigma_estimator__sigma_ref_mm,
      uint32_t      VL53LX_p_007,
      uint32_t      VL53LX_p_032,
      uint32_t      VL53LX_p_001,
      uint32_t      a_zp,
      uint32_t      c_zp,
      uint32_t      bx,
      uint32_t      ax_zp,
      uint32_t      cx_zp,
      uint32_t      VL53LX_p_028,
      uint16_t      fast_osc_frequency,
      uint16_t      *psigma_est);

    VL53LX_Error  VL53LX_f_023(
      uint8_t       sigma_estimator__sigma_ref_mm,
      uint32_t      VL53LX_p_007,
      uint32_t      VL53LX_p_032,
      uint32_t      VL53LX_p_001,
      uint32_t      a_zp,
      uint32_t      c_zp,
      uint32_t      bx,
      uint32_t      ax_zp,
      uint32_t      cx_zp,
      uint32_t      VL53LX_p_028,
      uint16_t      fast_osc_frequency,
      uint16_t      *psigma_est);

    uint32_t VL53LX_f_038(
      uint64_t VL53LX_p_007,
      uint32_t size
    );

    uint32_t VL53LX_f_035(
      uint32_t  VL53LX_p_007,
      uint32_t  VL53LX_p_032);


    /* vl53lx_hist_algos_gen3.h */

    void VL53LX_f_003(
      VL53LX_hist_gen3_algo_private_data_t   *palgo);

    VL53LX_Error VL53LX_f_004(
      VL53LX_dmax_calibration_data_t         *pdmax_cal,
      VL53LX_hist_gen3_dmax_config_t         *pdmax_cfg,
      VL53LX_hist_post_process_config_t      *ppost_cfg,
      VL53LX_histogram_bin_data_t            *pbins,
      VL53LX_histogram_bin_data_t            *pxtalk,
      VL53LX_hist_gen3_algo_private_data_t   *palgo,
      VL53LX_hist_gen3_dmax_private_data_t   *pdmax_algo,
      VL53LX_range_results_t                 *presults);

    VL53LX_Error VL53LX_f_006(
      uint16_t                               ambient_threshold_events_scaler,
      int32_t                                ambient_threshold_sigma,
      int32_t                                min_ambient_threshold_events,
      uint8_t                            algo__crosstalk_compensation_enable,
      VL53LX_histogram_bin_data_t           *pbins,
      VL53LX_histogram_bin_data_t           *pxtalk,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);


    VL53LX_Error VL53LX_f_007(
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_008(
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_009(
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_016(
      VL53LX_HistTargetOrder                target_order,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_010(
      uint8_t                                pulse_no,
      VL53LX_histogram_bin_data_t           *pbins,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_015(
      uint8_t                                pulse_no,
      uint8_t                             clip_events,
      VL53LX_histogram_bin_data_t           *pbins,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_020(
      int16_t                            VL53LX_p_019,
      int16_t                            VL53LX_p_024,
      uint8_t                            VL53LX_p_030,
      uint8_t                            clip_events,
      VL53LX_histogram_bin_data_t       *pbins,
      uint32_t                          *pphase);


    VL53LX_Error VL53LX_f_011(
      uint8_t                                pulse_no,
      VL53LX_histogram_bin_data_t           *pbins,
      VL53LX_hist_gen3_algo_private_data_t  *palgo,
      int32_t                                pad_value,
      VL53LX_histogram_bin_data_t           *ppulse);

    VL53LX_Error VL53LX_f_012(
      uint8_t                                pulse_no,
      VL53LX_histogram_bin_data_t           *ppulse,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_013(
      uint8_t                                pulse_no,
      uint16_t                               noise_threshold,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_021(
      uint8_t   bin,
      int32_t   filta0,
      int32_t   filta1,
      uint8_t   VL53LX_p_030,
      uint32_t *pmedian_phase);


    VL53LX_Error VL53LX_f_014(
      uint8_t                       bin,
      uint8_t                       sigma_estimator__sigma_ref_mm,
      uint8_t                       VL53LX_p_030,
      uint8_t                       VL53LX_p_051,
      uint8_t                       crosstalk_compensation_enable,
      VL53LX_histogram_bin_data_t  *phist_data_ap,
      VL53LX_histogram_bin_data_t  *phist_data_zp,
      VL53LX_histogram_bin_data_t  *pxtalk_hist,
      uint16_t                     *psigma_est);


    void VL53LX_f_017(
      uint8_t                      range_id,
      uint8_t                      valid_phase_low,
      uint8_t                      valid_phase_high,
      uint16_t                     sigma_thres,
      VL53LX_histogram_bin_data_t *pbins,
      VL53LX_hist_pulse_data_t    *ppulse,
      VL53LX_range_data_t         *pdata);


    /* vl53lx_hist_algos_gen4.h */

    void VL53LX_f_024(
      VL53LX_hist_gen4_algo_filtered_data_t  *palgo);

    VL53LX_Error VL53LX_f_025(
      VL53LX_dmax_calibration_data_t         *pdmax_cal,
      VL53LX_hist_gen3_dmax_config_t         *pdmax_cfg,
      VL53LX_hist_post_process_config_t      *ppost_cfg,
      VL53LX_histogram_bin_data_t            *pbins,
      VL53LX_histogram_bin_data_t            *pxtalk,
      VL53LX_hist_gen3_algo_private_data_t   *palgo,
      VL53LX_hist_gen4_algo_filtered_data_t  *pfiltered,
      VL53LX_hist_gen3_dmax_private_data_t   *pdmax_algo,
      VL53LX_range_results_t                 *presults);

    VL53LX_Error VL53LX_f_026(
      uint8_t                                pulse_no,
      VL53LX_histogram_bin_data_t           *ppulse,
      VL53LX_hist_gen3_algo_private_data_t  *palgo,
      VL53LX_hist_gen4_algo_filtered_data_t *pfiltered);

    VL53LX_Error VL53LX_f_027(
      uint8_t                                pulse_no,
      uint16_t                               noise_threshold,
      VL53LX_hist_gen4_algo_filtered_data_t *pfiltered,
      VL53LX_hist_gen3_algo_private_data_t  *palgo);

    VL53LX_Error VL53LX_f_028(
      uint8_t   bin,
      int32_t   VL53LX_p_007,
      int32_t   VL53LX_p_032,
      int32_t   VL53LX_p_001,
      int32_t   ax,
      int32_t   bx,
      int32_t   cx,
      int32_t   VL53LX_p_028,
      uint8_t   VL53LX_p_030,
      uint32_t *pmedian_phase);



    /* vl53lx_dmax.h */

    VL53LX_Error VL53LX_f_001(
      uint16_t                              target_reflectance,
      VL53LX_dmax_calibration_data_t       *pcal,
      VL53LX_hist_gen3_dmax_config_t       *pcfg,
      VL53LX_histogram_bin_data_t          *pbins,
      VL53LX_hist_gen3_dmax_private_data_t *pdata,
      int16_t                              *pambient_dmax_mm);

    uint32_t VL53LX_f_002(
      uint32_t     events_threshold,
      uint32_t     ref_signal_events,
      uint32_t   ref_distance_mm,
      uint32_t     signal_thresh_sigma);



    /* vl53lx_api_calibration.h */

    VL53LX_Error VL53LX_run_ref_spad_char(
      VL53LX_Error            *pcal_status);




    VL53LX_Error VL53LX_run_device_test(
      VL53LX_DeviceTestMode      device_test_mode);




    VL53LX_Error VL53LX_run_spad_rate_map(
      VL53LX_DeviceTestMode      device_test_mode,
      VL53LX_DeviceSscArray      array_select,
      uint32_t                   ssc_config_timeout_us,
      VL53LX_spad_rate_data_t   *pspad_rate_data);




    VL53LX_Error   VL53LX_run_xtalk_extraction(
      VL53LX_Error                       *pcal_status);



    VL53LX_Error VL53LX_get_and_avg_xtalk_samples(
      uint8_t                       num_of_samples,
      uint8_t                       measurement_mode,
      int16_t                       xtalk_filter_thresh_max_mm,
      int16_t                       xtalk_filter_thresh_min_mm,
      uint16_t                      xtalk_max_valid_rate_kcps,
      uint8_t                       xtalk_result_id,
      uint8_t                       xtalk_histo_id,
      VL53LX_xtalk_range_results_t *pxtalk_results,
      VL53LX_histogram_bin_data_t  *psum_histo,
      VL53LX_histogram_bin_data_t  *pavg_histo);



    VL53LX_Error   VL53LX_run_offset_calibration(
      int16_t                       cal_distance_mm,
      uint16_t                      cal_reflectance_pc,
      VL53LX_Error                 *pcal_status);




    VL53LX_Error   VL53LX_run_phasecal_average(
      uint8_t                 measurement_mode,
      uint8_t                 phasecal_result__vcsel_start,
      uint16_t                phasecal_num_of_samples,
      VL53LX_range_results_t *prange_results,
      uint16_t               *pphasecal_result__reference_phase,
      uint16_t               *pzero_distance_phase);




    VL53LX_Error VL53LX_run_zone_calibration(
      VL53LX_DevicePresetModes      device_preset_mode,
      VL53LX_DeviceZonePreset       zone_preset,
      VL53LX_zone_config_t         *pzone_cfg,
      int16_t                       cal_distance_mm,
      uint16_t                      cal_reflectance_pc,
      VL53LX_Error                 *pcal_status);




    void VL53LX_hist_xtalk_extract_data_init(
      VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);



    VL53LX_Error VL53LX_hist_xtalk_extract_update(
      int16_t                             target_distance_mm,
      uint16_t                            target_width_oversize,
      VL53LX_histogram_bin_data_t        *phist_bins,
      VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);



    VL53LX_Error VL53LX_hist_xtalk_extract_fini(
      VL53LX_histogram_bin_data_t        *phist_bins,
      VL53LX_hist_xtalk_extract_data_t   *pxtalk_data,
      VL53LX_xtalk_calibration_results_t *pxtalk_cal,
      VL53LX_xtalk_histogram_shape_t     *pxtalk_shape);




    VL53LX_Error   VL53LX_run_hist_xtalk_extraction(
      int16_t                       cal_distance_mm,
      VL53LX_Error                 *pcal_status);

    /* vl53lx_api_core.c static functions */
    VL53LX_Error select_offset_per_vcsel(VL53LX_LLDriverData_t *pdev, int16_t *poffset);
    void vl53lx_diff_histo_stddev(VL53LX_LLDriverData_t *pdev, VL53LX_histogram_bin_data_t *pdata, uint8_t timing, uint8_t HighIndex, uint8_t prev_pos, int32_t *pdiff_histo_stddev) ;
    void vl53lx_histo_merge(VL53LX_histogram_bin_data_t *pdata);

    /* vl53lx_api.c static functions */

    int32_t BDTable[VL53LX_TUNING_MAX_TUNABLE_KEY] = {
      TUNING_VERSION,
      TUNING_PROXY_MIN,
      TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
      TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
      TUNING_MIN_AMBIENT_DMAX_VALID,
      TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
      TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
      TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
      TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
      TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
      TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT
    };

    VL53LX_Error ComputeDevicePresetMode(
      VL53LX_DistanceModes DistanceMode,
      VL53LX_DevicePresetModes *pDevicePresetMode);

    VL53LX_Error SetPresetModeL3CX(
      VL53LX_DistanceModes DistanceMode,
      uint32_t inter_measurement_period_ms);

    VL53LX_Error SetInterMeasurementPeriodMilliSeconds(uint32_t InterMeasurementPeriodMilliSeconds);

    VL53LX_Error GetInterMeasurementPeriodMilliSeconds(uint32_t *pInterMeasurementPeriodMilliSeconds);

    uint8_t ConvertStatusHisto(uint8_t FilteredRangeStatus);

    VL53LX_Error SetTargetData(
      uint8_t active_results, uint8_t device_status,
      VL53LX_range_data_t *presults_data,
      VL53LX_TargetRangeData_t *pRangeData);

    VL53LX_Error SetMeasurementData(
      VL53LX_range_results_t *presults,
      VL53LX_MultiRangingData_t *pMultiRangingData);


    /* Write and read functions from I2C */

    VL53LX_Error VL53LX_WrByte(VL53LX_DEV dev_handle, uint16_t index, uint8_t data);
    VL53LX_Error VL53LX_WrWord(VL53LX_DEV dev_handle, uint16_t index, uint16_t data);
    VL53LX_Error VL53LX_WrDWord(VL53LX_DEV dev_handle, uint16_t index, uint32_t data);
    VL53LX_Error VL53LX_RdByte(VL53LX_DEV dev_handle, uint16_t index, uint8_t *data);
    VL53LX_Error VL53LX_RdWord(VL53LX_DEV dev_handle, uint16_t index, uint16_t *data);
    VL53LX_Error VL53LX_RdDWord(VL53LX_DEV dev_handle, uint16_t index, uint32_t *data);
    VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV dev_handle, uint16_t index, uint8_t AndData, uint8_t OrData);

    VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV dev_handle, uint16_t index, uint8_t *pdata, uint32_t count);
    VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV dev_handle, uint16_t index, uint8_t *pdata, uint32_t count);

    VL53LX_Error VL53LX_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);
    VL53LX_Error VL53LX_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToRead);
    VL53LX_Error VL53LX_GetTickCount(uint32_t *ptick_count_ms);
    VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us);
    VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms);

    VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_Dev_t *pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms);


  protected:

    /* IO Device */
    AP_HAL::I2CDevice *dev_i2c;
    /* Device data */
    VL53LX_Dev_t MyDevice;
    VL53LX_DEV Dev;
};

#endif /* _VL53LX_CLASS_H_ */
