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

#include "hal.h"
#include "cmsis_os.h"

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static void Thread1(void const *arg) {

  (void)arg;

  while (true) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    osDelay(500);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    osDelay(500);
  }
}

/*
 * Thread definition block.
 */
osThreadDef(Thread1, osPriorityAboveNormal, 128, "blinker");

/*
 * Application entry point.
 */
int main(void) {

  /* HAL initialization, this also initializes the configured device drivers
     and performs the board-specific initializations.*/
  halInit();

  /* The kernel is initialized but not started yet, this means that
     main() is executing with absolute priority but interrupts are
     already enabled.*/
  osKernelInitialize();

  /* Activates the serial driver 2 using the driver default configuration.
    PA2(TX) and PA3(RX) are routed to USART2.*/
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /* Creates the example thread, it does not start immediately.*/
  osThreadCreate(osThread(Thread1), NULL);

  /* Kernel started, the main() thread has priority osPriorityNormal
     by default.*/
  osKernelStart();

  /* In the ChibiOS/RT CMSIS RTOS implementation the main() is an
     usable thread, here we just sleep in a loop printing a message.*/
  while (true) {
    sdWrite(&SD2, (uint8_t *)"Hello World!\r\n", 14);
    osDelay(500);
  }
}
