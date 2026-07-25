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
 * @file    ch.cpp
 * @brief   C++ wrapper code.
 *
 * @addtogroup cpp_library
 * @{
 */

#include "ch.hpp"

namespace chibios_rt {

#if (CH_CFG_NO_IDLE_THREAD == FALSE) || defined(__DOXYGEN__)
  ThreadReference System::getIdleThreadX(void) {

    return ThreadReference(chSysGetIdleThreadX());
  }
#endif /* CH_CFG_NO_IDLE_THREAD == FALSE */

  /*------------------------------------------------------------------------*
   * chibios_rt::BaseStaticThread                                           *
   *------------------------------------------------------------------------*/

  void _thd_start(void *arg) {

    ((BaseThread *)arg)->main();
  }
}

/** @} */
