

#ifndef __AP_HAL_SMACCM_MAIN_H__
#define __AP_HAL_SMACCM_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

namespace SMACCM
{
  void hal_main();
}

#define AP_HAL_MAIN()                                      \
  extern "C" int main (void)                               \
  {                                                        \
    SMACCM::hal_main();                                    \
    return 0;                                              \
  }

#endif // HAL_BOARD_SMACCM */

#endif // __AP_HAL_SMACCM_MAIN_H__
