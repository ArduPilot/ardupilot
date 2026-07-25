[#ftl]
[#--
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  --]
[@pp.dropOutputFile /]
[#import "/@lib/libutils.ftl" as utils /]
[#import "/@lib/liblicense.ftl" as license /]
[@pp.changeOutputFile name="board.h" /]
/*
[@license.EmitLicenseAsText /]
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for ${doc1.board.board_name[0]} board.
 */

/*
 * Board identifier.
 */
#define BOARD_${doc1.board.board_id[0]}
#define BOARD_NAME                  "${doc1.board.board_name[0]}"

/*
 * Board oscillators-related settings.
[#if doc1.board.clocks.@LSEFrequency[0]?number == 0]
 * NOTE: LSE not fitted.
[/#if]
[#if doc1.board.clocks.@HSEFrequency[0]?number == 0]
 * NOTE: HSE not fitted.
[/#if]
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                ${doc1.board.clocks.@LSEFrequency[0]}U
#endif

[#if doc1.board.clocks.@LSEBypass[0]?string == "true"]
#define STM32_LSE_BYPASS

[/#if]
#define STM32_LSEDRV                (${doc1.board.clocks.@LSEDrive[0]?word_list[0]?number}U << 11U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                ${doc1.board.clocks.@HSEFrequency[0]}U
#endif

[#if doc1.board.clocks.@HSEBypass[0]?string == "true"]
#define STM32_HSE_BYPASS

[/#if]
/*
 * MCU type as defined in the ST header.
 */
#undef ${doc1.board.subtype[0]}
#define ${doc1.board.subtype[0]}

/*
 * IO pins assignments.
 */
[#list doc1.board.ports.* as port]
  [#assign port_name = port?node_name?upper_case /]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign pin_name = pin?node_name?upper_case /]
#define ${(port_name + "_" + pin_name)?right_pad(27, " ")} ${pin_index?string}U
    [#else]
      [#list names as name]
#define ${(port_name + "_" + name)?right_pad(27, " ")} ${pin_index?string}U
      [/#list]
    [/#if]
  [/#list]

[/#list]
/*
 * IO lines assignments.
 */
[#list doc1.board.ports.* as port]
  [#assign port_name = port?node_name?upper_case /]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size > 0]
      [#list names as name]
#define LINE_${name?right_pad(22, " ")} PAL_LINE(${port_name}, ${pin_index?string}U)
      [/#list]
    [/#if]
  [/#list]
[/#list]

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

[#list doc1.board.ports.* as port]
  [#assign port_name = port?node_name?upper_case /]
/*
 * ${port_name} setup:
 *
  [#-- Generating pin descriptions inside the comment.--]
  [#list port.* as pin]
    [#assign pin_name = pin?node_name?upper_case /]
    [#assign name = pin.@ID[0]?string?trim /]
    [#if name?length == 0]
      [#assign name = pin_name /]
    [/#if]
    [#assign mode = pin.@Mode[0] /]
    [#assign type = pin.@Type[0] /]
    [#assign resistor = pin.@Resistor[0] /]
    [#assign speed = pin.@Speed[0] /]
    [#assign alternate = pin.@Alternate[0] /]
    [#if mode == "Input"]
      [#assign desc = mode + " " + resistor /]
    [#elseif mode == "Output"]
      [#assign desc = mode + " " + type + " " + speed /]
    [#elseif mode == "Alternate"]
      [#assign desc = mode + " " + alternate /]
    [#else]
      [#assign desc = "Analog" /]
    [/#if]
 * P${(port?node_name[4..] + pin_index?string)?right_pad(3, " ")} - ${name?right_pad(26, " ")}(${desc?lower_case}).
  [/#list]
 */
  [#--
    -- Generating MODER register value.
    --]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign name = pin?node_name?upper_case /]
    [#else]
      [#assign name = names[0] /]
    [/#if]
    [#assign mode = pin.@Mode[0] /]
    [#if mode == "Input"]
      [#assign out = "PIN_MODE_INPUT(" + port_name + "_" + name + ")" /]
    [#elseif mode == "Output"]
      [#assign out = "PIN_MODE_OUTPUT(" + port_name + "_" + name + ")" /]
    [#elseif mode == "Alternate"]
      [#assign out = "PIN_MODE_ALTERNATE(" + port_name + "_" + name + ")" /]
    [#else]
      [#assign out = "PIN_MODE_ANALOG(" + port_name + "_" + name + ")" /]
    [/#if]
    [#if pin_index == 0]
      [#assign line = "#define VAL_" + port_name + "_MODER             (" + out /]
    [#else]
      [#assign line = "                                     " + out /]
    [/#if]
    [#if pin_index < 15]
${(line + " |")?right_pad(76, " ") + "\\"}
    [#else]
${line + ")"}
    [/#if]
  [/#list]
  [#--
    -- Generating OTYPER register value.
    --]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign name = pin?node_name?upper_case /]
    [#else]
      [#assign name = names[0] /]
    [/#if]
    [#assign type = pin.@Type[0] /]
    [#if type == "PushPull"]
      [#assign out = "PIN_OTYPE_PUSHPULL(" + port_name + "_" + name + ")" /]
    [#else]
      [#assign out = "PIN_OTYPE_OPENDRAIN(" + port_name + "_" + name + ")" /]
    [/#if]
    [#if pin_index == 0]
      [#assign line = "#define VAL_" + port_name + "_OTYPER            (" + out /]
    [#else]
      [#assign line = "                                     " + out /]
    [/#if]
    [#if pin_index < 15]
${(line + " |")?right_pad(76, " ") + "\\"}
    [#else]
${line + ")"}
    [/#if]
  [/#list]
  [#--
    -- Generating SPEEDR register value.
    --]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign name = pin?node_name?upper_case /]
    [#else]
      [#assign name = names[0] /]
    [/#if]
    [#assign speed = pin.@Speed[0] /]
    [#if speed == "Minimum"]
      [#assign out = "PIN_OSPEED_VERYLOW(" + port_name + "_" + name + ")" /]
    [#elseif speed == "Low"]
      [#assign out = "PIN_OSPEED_LOW(" + port_name + "_" + name + ")" /]
    [#elseif speed == "High"]
      [#assign out = "PIN_OSPEED_MEDIUM(" + port_name + "_" + name + ")" /]
    [#else]
      [#assign out = "PIN_OSPEED_HIGH(" + port_name + "_" + name + ")" /]
    [/#if]
    [#if pin_index == 0]
      [#assign line = "#define VAL_" + port_name + "_OSPEEDR           (" + out /]
    [#else]
      [#assign line = "                                     " + out /]
    [/#if]
    [#if pin_index < 15]
${(line + " |")?right_pad(76, " ") + "\\"}
    [#else]
${line + ")"}
    [/#if]
  [/#list]
  [#--
    -- Generating PUPDR register value.
    --]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign name = pin?node_name?upper_case /]
    [#else]
      [#assign name = names[0] /]
    [/#if]
    [#assign resistor = pin.@Resistor[0] /]
    [#if resistor == "Floating"]
      [#assign out = "PIN_PUPDR_FLOATING(" + port_name + "_" + name + ")" /]
    [#elseif resistor == "PullUp"]
      [#assign out = "PIN_PUPDR_PULLUP(" + port_name + "_" + name + ")" /]
    [#else]
      [#assign out = "PIN_PUPDR_PULLDOWN(" + port_name + "_" + name + ")" /]
    [/#if]
    [#if pin_index == 0]
      [#assign line = "#define VAL_" + port_name + "_PUPDR             (" + out /]
    [#else]
      [#assign line = "                                     " + out /]
    [/#if]
    [#if pin_index < 15]
${(line + " |")?right_pad(76, " ") + "\\"}
    [#else]
${line + ")"}
    [/#if]
  [/#list]
  [#--
    -- Generating ODR register value.
    --]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign name = pin?node_name?upper_case /]
    [#else]
      [#assign name = names[0] /]
    [/#if]
    [#assign level = pin.@Level[0] /]
    [#if level == "Low"]
      [#assign out = "PIN_ODR_LOW(" + port_name + "_" + name + ")" /]
    [#else]
      [#assign out = "PIN_ODR_HIGH(" + port_name + "_" + name + ")" /]
    [/#if]
    [#if pin_index == 0]
      [#assign line = "#define VAL_" + port_name + "_ODR               (" + out /]
    [#else]
      [#assign line = "                                     " + out /]
    [/#if]
    [#if pin_index < 15]
${(line + " |")?right_pad(76, " ") + "\\"}
    [#else]
${line + ")"}
    [/#if]
  [/#list]
  [#--
    -- Generating AFRx registers values.
    --]
  [#list port.* as pin]
    [#assign names = pin.@ID[0]?string?word_list /]
    [#if names?size == 0]
      [#assign name = pin?node_name?upper_case /]
    [#else]
      [#assign name = names[0] /]
    [/#if]
    [#assign alternate = pin.@Alternate[0]?trim /]
    [#assign out = "PIN_AFIO_AF(" + port_name + "_" + name + ", " + alternate + "U)" /]
    [#if pin_index == 0]
      [#assign line = "#define VAL_" + port_name + "_AFRL              (" + out /]
    [#elseif pin_index == 8]
      [#assign line = "#define VAL_" + port_name + "_AFRH              (" + out /]
    [#else]
      [#assign line = "                                     " + out /]
    [/#if]
    [#if (pin_index == 7) || (pin_index == 15)]
${line + ")"}
    [#else]
${(line + " |")?right_pad(76, " ") + "\\"}
    [/#if]
  [/#list]

[/#list]
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
