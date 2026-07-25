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

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * MAX32625 drivers configuration.
 */

#define MAX32625_MCUCONF

/*
 * HAL driver system settings.
 */
#define MAX32_NO_INIT                       FALSE
#define MAX32_SYS_SRC                       MAX32_SYS_SRC_SRO
#define MAX32_CM4_CLKMAN                    MAX32_CM4_DIV1
#define MAX32_GPIO_CLKMAN                   MAX32_GPIO_DIV1

/*
 * ST driver system settings.
 */
#define MAX32_ST_IRQ_PRIORITY               3

#endif /* MCUCONF_H */
