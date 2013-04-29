
#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include "RCInput.h"

#include <hwf4/timer.h>

using namespace SMACCM;

/* Constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p)
{
  if (p > RC_INPUT_MAX_PULSEWIDTH)
    return RC_INPUT_MAX_PULSEWIDTH;
  if (p < RC_INPUT_MIN_PULSEWIDTH)
    return RC_INPUT_MIN_PULSEWIDTH;

  return p;
}

SMACCMRCInput::SMACCMRCInput()
{
}

void SMACCMRCInput::init(void *unused)
{
  clear_overrides();
}

uint8_t SMACCMRCInput::valid_channels()
{
  // If any of the overrides are positive, we have valid data.
  for (int i = 0; i < SMACCM_RCINPUT_CHANNELS; ++i)
    if (_override[i] > 0)
      return true;

  return timer_is_ppm_valid();
}

// It looks like the APM2 driver clears the PPM sample after this
// function is called, so we do the same thing here for compatibility.
uint16_t SMACCMRCInput::read(uint8_t ch)
{
  uint16_t result = 0;

  if (_override[ch] != 0) {
    result = _override[ch];
  } else {
    timer_get_ppm_channel(ch, &result);
    result = constrain_pulse(result);
  }

  timer_clear_ppm();
  return constrain_pulse(result);
}

// It looks like the APM2 driver clears the PPM sample after this
// function is called, so we do the same thing here for compatibility.
uint8_t SMACCMRCInput::read(uint16_t *periods, uint8_t len)
{
  uint8_t result;
  result = timer_get_ppm(periods, len, NULL);

  for (int i = 0; i < result; ++i) {
    periods[i] = constrain_pulse(periods[i]);
    if (_override[i] != 0)
      periods[i] = _override[i];
  }

  timer_clear_ppm();
  return result;
}

bool SMACCMRCInput::set_overrides(int16_t *overrides, uint8_t len)
{
  bool result = false;

  for (int i = 0; i < len; ++i)
    result |= set_override(i, overrides[i]);

  return result;
}

bool SMACCMRCInput::set_override(uint8_t channel, int16_t override)
{
  if (override < 0)
    return false;

  if (channel < SMACCM_RCINPUT_CHANNELS) {
    _override[channel] = override;
    if (override != 0) {
      return true;
    }
  }

  return false;
}

void SMACCMRCInput::clear_overrides()
{
  for (int i = 0; i < SMACCM_RCINPUT_CHANNELS; ++i)
    _override[i] = 0;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
