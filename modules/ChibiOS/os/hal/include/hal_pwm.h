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
 * @file    hal_pwm.h
 * @brief   PWM Driver macros and structures.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_H
#define HAL_PWM_H

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    PWM output mode macros
 * @{
 */
/**
 * @brief   Standard output modes mask.
 */
#define PWM_OUTPUT_MASK                         0x0FU

/**
 * @brief   Output not driven, callback only.
 */
#define PWM_OUTPUT_DISABLED                     0x00U

/**
 * @brief   Positive PWM logic, active is logic level one.
 */
#define PWM_OUTPUT_ACTIVE_HIGH                  0x01U

/**
 * @brief   Inverse PWM logic, active is logic level zero.
 */
#define PWM_OUTPUT_ACTIVE_LOW                   0x02U
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  PWM_UNINIT = 0,                   /**< Not initialized.                   */
  PWM_STOP = 1,                     /**< Stopped.                           */
  PWM_READY = 2                     /**< Ready.                             */
} pwmstate_t;

/**
 * @brief   Type of a structure representing a PWM driver.
 */
typedef struct PWMDriver PWMDriver;

/**
 * @brief   Type of a PWM notification callback.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 */
typedef void (*pwmcallback_t)(PWMDriver *pwmp);

#include "hal_pwm_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    PWM duty cycle conversion
 * @{
 */
/**
 * @brief   Converts from fraction to pulse width.
 * @note    Be careful with rounding errors, this is integer math not magic.
 *          You can specify tenths of thousandth but make sure you have the
 *          proper hardware resolution by carefully choosing the clock source
 *          and prescaler settings, see @p PWM_COMPUTE_PSC.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] denominator denominator of the fraction
 * @param[in] numerator numerator of the fraction
 * @return              The pulse width to be passed to @p pwmEnableChannel().
 *
 * @api
 */
#define PWM_FRACTION_TO_WIDTH(pwmp, denominator, numerator)                 \
  ((pwmcnt_t)((((pwmcnt_t)(pwmp)->period) *                                 \
               (pwmcnt_t)(numerator)) / (pwmcnt_t)(denominator)))

/**
 * @brief   Converts from degrees to pulse width.
 * @note    Be careful with rounding errors, this is integer math not magic.
 *          You can specify hundredths of degrees but make sure you have the
 *          proper hardware resolution by carefully choosing the clock source
 *          and prescaler settings, see @p PWM_COMPUTE_PSC.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] degrees   degrees as an integer between 0 and 36000
 * @return              The pulse width to be passed to @p pwmEnableChannel().
 *
 * @api
 */
#define PWM_DEGREES_TO_WIDTH(pwmp, degrees)                                 \
  PWM_FRACTION_TO_WIDTH(pwmp, 36000, degrees)

/**
 * @brief   Converts from percentage to pulse width.
 * @note    Be careful with rounding errors, this is integer math not magic.
 *          You can specify tenths of thousandth but make sure you have the
 *          proper hardware resolution by carefully choosing the clock source
 *          and prescaler settings, see @p PWM_COMPUTE_PSC.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] percentage percentage as an integer between 0 and 10000
 * @return              The pulse width to be passed to @p pwmEnableChannel().
 *
 * @api
 */
#define PWM_PERCENTAGE_TO_WIDTH(pwmp, percentage)                           \
  PWM_FRACTION_TO_WIDTH(pwmp, 10000, percentage)
/** @} */

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] value     new cycle time in ticks
 *
 * @iclass
 */
#define pwmChangePeriodI(pwmp, value) {                                     \
  (pwmp)->period = (value);                                                 \
  pwm_lld_change_period(pwmp, value);                                       \
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    Depending on the hardware implementation this function has
 *          effect starting on the next cycle (recommended implementation)
 *          or immediately (fallback implementation).
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @iclass
 */
#define pwmEnableChannelI(pwmp, channel, width) do {                        \
  (pwmp)->enabled |= ((pwmchnmsk_t)1U << (pwmchnmsk_t)(channel));           \
  pwm_lld_enable_channel(pwmp, channel, width);                             \
} while (false)

/**
 * @brief   Disables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    Depending on the hardware implementation this function has
 *          effect starting on the next cycle (recommended implementation)
 *          or immediately (fallback implementation).
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @iclass
 */
#define pwmDisableChannelI(pwmp, channel) do {                              \
  (pwmp)->enabled &= ~((pwmchnmsk_t)1U << (pwmchnmsk_t)(channel));          \
  pwm_lld_disable_channel(pwmp, channel);                                   \
} while (false)

/**
 * @brief   Returns a PWM channel status.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @iclass
 */
#define pwmIsChannelEnabledI(pwmp, channel)                                 \
  (((pwmp)->enabled & ((pwmchnmsk_t)1U << (pwmchnmsk_t)(channel))) != 0U)

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @iclass
 */
#define pwmEnablePeriodicNotificationI(pwmp)                                \
  pwm_lld_enable_periodic_notification(pwmp)

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @iclass
 */
#define pwmDisablePeriodicNotificationI(pwmp)                               \
  pwm_lld_disable_periodic_notification(pwmp)

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @iclass
 */
#define pwmEnableChannelNotificationI(pwmp, channel)                        \
  pwm_lld_enable_channel_notification(pwmp, channel)

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @iclass
 */
#define pwmDisableChannelNotificationI(pwmp, channel)                       \
  pwm_lld_disable_channel_notification(pwmp, channel)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void pwmInit(void);
  void pwmObjectInit(PWMDriver *pwmp);
  msg_t pwmStart(PWMDriver *pwmp, const PWMConfig *config);
  void pwmStop(PWMDriver *pwmp);
  void pwmChangePeriod(PWMDriver *pwmp, pwmcnt_t period);
  void pwmEnableChannel(PWMDriver *pwmp,
                        pwmchannel_t channel,
                        pwmcnt_t width);
  void pwmDisableChannel(PWMDriver *pwmp, pwmchannel_t channel);
  void pwmEnablePeriodicNotification(PWMDriver *pwmp);
  void pwmDisablePeriodicNotification(PWMDriver *pwmp);
  void pwmEnableChannelNotification(PWMDriver *pwmp, pwmchannel_t channel);
  void pwmDisableChannelNotification(PWMDriver *pwmp, pwmchannel_t channel);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM == TRUE */

#endif /* HAL_PWM_H */

/** @} */
