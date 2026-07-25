/*
    ChibiOS - Copyright (C) 2016..2023 Rocco Marco Guglielmi

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
*/

/**
 * @file    l3gd20.c
 * @brief   L3GD20 MEMS interface module code.
 *
 * @addtogroup L3GD20
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "l3gd20.h"

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

#if (L3GD20_USE_SPI) || defined(__DOXYGEN__)
/**
 * @brief   Reads a generic register value using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of consecutive registers to read
 * @param[in] b         pointer to an output buffer.
 */
static void l3gd20SPIReadRegister(SPIDriver *spip, uint8_t reg,  size_t n,
                                     uint8_t* b) {
  uint8_t cmd;
  (n == 1) ? (cmd = reg | L3GD20_RW) : (cmd = reg | L3GD20_RW | L3GD20_MS);
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiReceive(spip, n, b);
  spiUnselect(spip);
}

/**
 * @brief   Writes a value into a generic register using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer of values.
 */
static void l3gd20SPIWriteRegister(SPIDriver *spip, uint8_t reg, size_t n,
                                   uint8_t* b) {
  uint8_t cmd;
  (n == 1) ? (cmd = reg) : (cmd = reg | L3GD20_MS);
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiSend(spip, n, b);
  spiUnselect(spip);
}
#endif /* L3GD20_USE_SPI */

/**
 * @brief   Return the number of axes of the BaseGyroscope.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              the number of axes.
 */
static size_t gyro_get_axes_number(void *ip) {
  (void)ip;
  
  return L3GD20_GYRO_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseGyroscope.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_read_raw(void *ip, int32_t axes[L3GD20_GYRO_NUMBER_OF_AXES]) {
  L3GD20Driver* devp;
  int16_t tmp;
  uint8_t i, buff [2 * L3GD20_GYRO_NUMBER_OF_AXES];
  msg_t msg = MSG_OK;
  
  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);
  
  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_read_raw(), invalid state");
#if L3GD20_USE_SPI
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "gyro_read_raw(), channel not ready");
                
#if	L3GD20_SHARED_SPI
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,
           devp->config->spicfg);
#endif /* L3GD20_SHARED_SPI */

  l3gd20SPIReadRegister(devp->config->spip, L3GD20_AD_OUT_X_L,
                        L3GD20_GYRO_NUMBER_OF_AXES * 2, buff);

#if	L3GD20_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* L3GD20_SHARED_SPI */
#endif /* L3GD20_USE_SPI */

    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
      axes[i] = (int32_t)tmp;
    }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseGyroscope.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as DPS.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_read_cooked(void *ip, float axes[]) {
  L3GD20Driver* devp;
  uint32_t i;
  int32_t raw[L3GD20_GYRO_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);
  
  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_read_cooked(), invalid state");

  msg = gyro_read_raw(ip, raw);
  for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++){
    axes[i] = (raw[i] * devp->gyrosensitivity[i]) - devp->gyrobias[i];
  }
  return msg;
}

/**
 * @brief   Samples bias values for the BaseGyroscope.
 * @note    The L3GD20 shall not be moved during the whole procedure.
 * @note    After this function internal bias is automatically updated.
 * @note    The behavior of this function depends on @p L3GD20_BIAS_ACQ_TIMES
 *          and @p L3GD20_BIAS_SETTLING_US.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_sample_bias(void *ip) {
  L3GD20Driver* devp;
  uint32_t i, j;
  int32_t raw[L3GD20_GYRO_NUMBER_OF_AXES];
  int32_t buff[L3GD20_GYRO_NUMBER_OF_AXES] = {0, 0, 0};
  msg_t msg;
	
  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);
  
  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_sample_bias(), invalid state");
#if L3GD20_USE_SPI
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "gyro_sample_bias(), channel not ready");
#endif

  for(i = 0; i < L3GD20_BIAS_ACQ_TIMES; i++){
    msg = gyro_read_raw(ip, raw);
		if(msg != MSG_OK)
			return msg;
    for(j = 0; j < L3GD20_GYRO_NUMBER_OF_AXES; j++){
      buff[j] += raw[j];
    }
    osalThreadSleepMicroseconds(L3GD20_BIAS_SETTLING_US);
  }

  for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++){
    devp->gyrobias[i] = (buff[i] / L3GD20_BIAS_ACQ_TIMES);
    devp->gyrobias[i] *= devp->gyrosensitivity[i];
  }
  return msg;
}

/**
 * @brief   Set bias values for the BaseGyroscope.
 * @note    Bias must be expressed as DPS.
 * @note    The bias buffer must be at least the same size of the BaseGyroscope
 *          axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_set_bias(void *ip, float *bp) {
  L3GD20Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;
  
  osalDbgCheck((ip != NULL) && (bp != NULL));
  
  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);
  
  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_set_bias(), invalid state");
  
  for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
    devp->gyrobias[i] = bp[i];
  }
  return msg;
}

/**
 * @brief   Reset bias values for the BaseGyroscope.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_reset_bias(void *ip) {
  L3GD20Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);
  
  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);
  
  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_reset_bias(), invalid state");

  for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++)
    devp->gyrobias[i] = L3GD20_GYRO_BIAS;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseGyroscope.
 * @note    Sensitivity must be expressed as DPS/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_set_sensivity(void *ip, float *sp) {
  L3GD20Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;
  
  osalDbgCheck((ip != NULL) && (sp !=NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_set_sensivity(), invalid state");
  
  for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
    devp->gyrosensitivity[i] = sp[i];
  }
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseGyroscope.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t gyro_reset_sensivity(void *ip) {
  L3GD20Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;
  
  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(L3GD20Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_reset_sensivity(), invalid state");

  if(devp->config->gyrofullscale == L3GD20_FS_250DPS)
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = L3GD20_GYRO_SENS_250DPS;
  else if(devp->config->gyrofullscale == L3GD20_FS_500DPS)
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++)
        devp->gyrosensitivity[i] = L3GD20_GYRO_SENS_500DPS;
  else if(devp->config->gyrofullscale == L3GD20_FS_2000DPS)
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++)
        devp->gyrosensitivity[i] = L3GD20_GYRO_SENS_2000DPS;
  else {
    osalDbgAssert(FALSE, "gyro_reset_sensivity(), full scale issue");
    return MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the L3GD20Driver gyroscope fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p BaseGyroscope interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t gyro_set_full_scale(L3GD20Driver *devp, l3gd20_fs_t fs) {
  float newfs, scale;
  uint8_t i, cr;
  msg_t msg = MSG_OK;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == L3GD20_READY),
                "gyro_set_full_scale(), invalid state");
#if L3GD20_USE_SPI
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "gyro_set_full_scale(), channel not ready");
#endif

  if(fs == L3GD20_FS_250DPS) {
    newfs = L3GD20_250DPS;
  }
  else if(fs == L3GD20_FS_500DPS) {
    newfs = L3GD20_500DPS;
  }
  else if(fs == L3GD20_FS_2000DPS) {
    newfs = L3GD20_2000DPS;
  }
  else {
    return MSG_RESET;
  }

  if(newfs != devp->gyrofullscale) {
    scale = newfs / devp->gyrofullscale;
    devp->gyrofullscale = newfs;

#if L3GD20_USE_SPI
#if	L3GD20_SHARED_SPI
		spiAcquireBus(devp->config->spip);
		spiStart(devp->config->spip,
						 devp->config->spicfg);
#endif /* L3GD20_SHARED_SPI */

    /* Updating register.*/
    l3gd20SPIReadRegister(devp->config->spip,
                          L3GD20_AD_CTRL_REG4, 1, &cr);

#if	L3GD20_SHARED_SPI
		spiReleaseBus(devp->config->spip);
#endif /* L3GD20_SHARED_SPI */
#endif /* L3GD20_USE_SPI */
    cr &= ~(L3GD20_CTRL_REG4_FS_MASK);
    cr |= fs;

#if L3GD20_USE_SPI
#if	L3GD20_SHARED_SPI
		spiAcquireBus(devp->config->spip);
		spiStart(devp->config->spip,
						 devp->config->spicfg);
#endif /* L3GD20_SHARED_SPI */

    l3gd20SPIWriteRegister(devp->config->spip,
                           L3GD20_AD_CTRL_REG4, 1, &cr);
#if	L3GD20_SHARED_SPI
		spiReleaseBus(devp->config->spip);
#endif /* L3GD20_SHARED_SPI */
#endif /* L3GD20_USE_SPI */

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway. */
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      devp->gyrosensitivity[i] *= scale;
      devp->gyrobias[i] *= scale;
    }
  }
  return msg;
}

static const struct L3GD20VMT vmt_device = {
  (size_t)0,
  gyro_set_full_scale
};

static const struct BaseGyroscopeVMT vmt_gyroscope = {
  sizeof(struct L3GD20VMT*),
  gyro_get_axes_number, gyro_read_raw, gyro_read_cooked,
  gyro_sample_bias, gyro_set_bias, gyro_reset_bias,
  gyro_set_sensivity, gyro_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p L3GD20Driver object
 *
 * @init
 */
void l3gd20ObjectInit(L3GD20Driver *devp) {
  devp->vmt = &vmt_device;
  devp->gyro_if.vmt = &vmt_gyroscope;
  
  devp->config = NULL;

  devp->state  = L3GD20_STOP;
}

/**
 * @brief   Configures and activates L3GD20 Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p L3GD20Driver object
 * @param[in] config    pointer to the @p L3GD20Config object
 *
 * @api
 */
void l3gd20Start(L3GD20Driver *devp, const L3GD20Config *config) {
  uint32_t i;
  uint8_t cr[5] = {0, 0, 0, 0, 0};
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == L3GD20_STOP) || (devp->state == L3GD20_READY),
              "l3gd20Start(), invalid state");

  devp->config = config;
             
  /* Control register 1 configuration block.*/
  {
    cr[0] = L3GD20_CTRL_REG1_XEN | L3GD20_CTRL_REG1_YEN | 
          L3GD20_CTRL_REG1_ZEN | L3GD20_CTRL_REG1_PD |
          devp->config->gyroodr;
#if L3GD20_USE_ADVANCED || defined(__DOXYGEN__)
    cr[0] |= devp->config->gyrobandwidth;
#endif
  }
  
  /* Control register 2 configuration block.*/
  {
#if L3GD20_USE_ADVANCED || defined(__DOXYGEN__)
  if(devp->config->gyrohpmode != L3GD20_HPM_BYPASSED)
    cr[1] = devp->config->gyrohpmode | devp->config->gyrohpcfg;
#endif
  }
  
  /* Control register 4 configuration block.*/
  {
    cr[3] = devp->config->gyrofullscale;
#if L3GD20_USE_ADVANCED || defined(__DOXYGEN__)
    cr[3] |= devp->config->gyrobdu |
             devp->config->gyroendianness;
#endif
  }
  
  /* Control register 5 configuration block.*/
  {    
#if L3GD20_USE_ADVANCED || defined(__DOXYGEN__)
  if((devp->config->gyrohpmode != L3GD20_HPM_BYPASSED)) {
    cr[4] = L3GD20_CTRL_REG5_HPEN;
    if(devp->config->gyrolp2mode != L3GD20_LP2M_BYPASSED) {
      cr[4] |= L3GD20_CTRL_REG5_INT1_SEL1 |
               L3GD20_CTRL_REG5_OUT_SEL1;
    }
    else {
      cr[4] |= L3GD20_CTRL_REG5_INT1_SEL0 |
               L3GD20_CTRL_REG5_OUT_SEL0; 
    }
  }
#endif
  }

#if L3GD20_USE_SPI
#if	L3GD20_SHARED_SPI
  spiAcquireBus(devp->config->spip);
#endif /* L3GD20_SHARED_SPI */
  spiStart(devp->config->spip,
           devp->config->spicfg);
           
  l3gd20SPIWriteRegister(devp->config->spip, L3GD20_AD_CTRL_REG1, 
                         5, cr);
#if	L3GD20_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* L3GD20_SHARED_SPI */
#endif /* L3GD20_USE_SPI */
  
  /* Storing sensitivity information according to full scale.*/
  if(devp->config->gyrofullscale == L3GD20_FS_250DPS) {
    devp->gyrofullscale = L3GD20_250DPS;
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      if (devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = L3GD20_GYRO_SENS_250DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
  }
  else if(devp->config->gyrofullscale == L3GD20_FS_500DPS) {
    devp->gyrofullscale = L3GD20_500DPS;
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      if (devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = L3GD20_GYRO_SENS_500DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
  }
  else if(devp->config->gyrofullscale == L3GD20_FS_2000DPS) {
    devp->gyrofullscale = L3GD20_2000DPS;
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      if (devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = L3GD20_GYRO_SENS_2000DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
  }
  else
    osalDbgAssert(FALSE, "l3gd20Start(), full scale issue");

  /* Storing bias information.*/
  if(devp->config->gyrobias != NULL) {
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      devp->gyrobias[i] = devp->config->gyrobias[i];
    }
  }
  else {
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrobias[i] = L3GD20_GYRO_BIAS;
  }
  
  /* This is the Gyroscope transient recovery time.*/
  osalThreadSleepMilliseconds(10);

  devp->state = L3GD20_READY;
} 

/**
 * @brief   Deactivates the L3GD20 Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p L3GD20Driver object
 *
 * @api
 */
void l3gd20Stop(L3GD20Driver *devp) {
  uint8_t cr1;
  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == L3GD20_STOP) || (devp->state == L3GD20_READY),
                "l3gd20Stop(), invalid state");

  if (devp->state == L3GD20_READY) {
    /* Disabling all axes and enabling power down mode.*/
    cr1 = 0;
    
#if L3GD20_USE_SPI
#if	L3GD20_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* L3GD20_SHARED_SPI */

    l3gd20SPIWriteRegister(devp->config->spip, L3GD20_AD_CTRL_REG1, 
                           1, &cr1);
    spiStop(devp->config->spip);
    
#if	L3GD20_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* L3GD20_SHARED_SPI */ 
#endif /* L3GD20_USE_SPI */
  }			 
  devp->state = L3GD20_STOP;
}
/** @} */
