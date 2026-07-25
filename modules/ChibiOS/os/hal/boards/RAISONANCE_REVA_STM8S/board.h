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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for Raisonance REva V3 + STM8S208RB daughter board.
 */

/*
 * Board identifiers.
 */
#define BOARD_REVA_V3_STM8S208RB
#define BOARD_NAME      "Raisonance REva V3 + STM8S208RB"

/*
 * Board frequencies.
 */
#define HSECLK          0

/*
 * MCU model used on the board.
 */
#define STM8S208

/*
 * Pin definitions.
 */
#define PA_OSCIN        1
#define PA_J2_25        2               /* It is also OSCOUT.               */
#define PA_J2_27        3
#define PA_RX           4
#define PA_TX           5

#define PB_LED(n)       (n)
#define PB_LCD_D0       0
#define PB_LCD_D1       1
#define PB_LCD_CSB      2
#define PB_LCD_RESB     3

#define PC_ADC_ETR      0
#define PC_J2_51        1
#define PC_J2_53        2
#define PC_J2_55        3
#define PC_J2_57        4
#define PC_SCK          5
#define PC_MOSI         6
#define PC_MISO         7

#define PD_J2_69        0
#define PD_J2_21        1
#define PD_J2_67        2
#define PD_J2_65        3
#define PD_PWM          4
#define PD_J2_63        5
#define PD_J2_61        6
#define PD_J2_59        7

#define PE_P2_49        0
#define PE_SCL          1
#define PE_SDA          2
#define PE_P2_47        3
#define PE_P2_45        4
#define PE_P2_43        5
#define PE_P2_41        6
#define PE_P2_39        7

#define PF_J2_37        0
#define PF_J2_35        1
#define PF_J2_33        2
#define PF_J2_31        3
#define PF_ANA_IN1      4
#define PF_ANA_IN2      5
#define PF_ANA_TEMP     6
#define PF_ANA_POT      7

#define PG_CAN_TX       0
#define PG_CAN_RX       1
#define PG_BT5          2
#define PG_BT6          3
#define PG_SW4          4
#define PG_SW3          5
#define PG_SW2          6
#define PG_SW1          7

#define PI_J2_71        0

/*
 * Port A initial setup.
 */
#define VAL_GPIOAODR    (1 << PA_TX)    /* PA_TX initially to 1.            */
#define VAL_GPIOADDR    (1 << PA_TX)    /* PA_TX output, others inputs.     */
#define VAL_GPIOACR1    0xFF            /* All pull-up or push-pull.        */
#define VAL_GPIOACR2    0

/*
 * Port B initial setup.
 */
#define VAL_GPIOBODR    0xFF            /* Initially all set to high.       */
#define VAL_GPIOBDDR    0xFF            /* All outputs.                     */
#define VAL_GPIOBCR1    0xFF            /* All push-pull.                   */
#define VAL_GPIOBCR2    0

/*
 * Port C initial setup.
 */
#define VAL_GPIOCODR    0
#define VAL_GPIOCDDR    0               /* All inputs.                      */
#define VAL_GPIOCCR1    0xFF            /* All pull-up.                     */
#define VAL_GPIOCCR2    0

/*
 * Port D initial setup.
 */
#define VAL_GPIODODR    0
#define VAL_GPIODDDR    0               /* All inputs.                      */
#define VAL_GPIODCR1    0xFF            /* All pull-up.                     */
#define VAL_GPIODCR2    0

/*
 * Port E initial setup.
 */
#define VAL_GPIOEODR    0 
#define VAL_GPIOEDDR    0               /* All inputs.                      */
#define VAL_GPIOECR1    0xFF            /* All pull-up.                     */
#define VAL_GPIOECR2    0

/*
 * Port F initial setup.
 */
#define VAL_GPIOFODR    0
#define VAL_GPIOFDDR    0               /* All inputs.                      */
#define VAL_GPIOFCR1    0xFF            /* All pull-up.                     */
#define VAL_GPIOFCR2    0

/*
 * Port G initial setup.
 */
#define VAL_GPIOGODR    (1 << PG_CAN_TX)/* CAN_TX initially to 1.           */
#define VAL_GPIOGDDR    (1 << PG_CAN_TX)/* CAN_TX output, others inputs.    */
#define VAL_GPIOGCR1    0xFF            /* All pull-up or push-pull.        */
#define VAL_GPIOGCR2    0

/*
 * Port H initial setup (dummy, not present).
 */
#define VAL_GPIOHODR    0
#define VAL_GPIOHDDR    0               /* All inputs.                      */
#define VAL_GPIOHCR1    0xFF            /* All pull-up.                     */
#define VAL_GPIOHCR2    0

/*
 * Port I initial setup.
 */
#define VAL_GPIOIODR    0
#define VAL_GPIOIDDR    0               /* All inputs.                      */
#define VAL_GPIOICR1    0xFF            /* All pull-up.                     */
#define VAL_GPIOICR2    0

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
