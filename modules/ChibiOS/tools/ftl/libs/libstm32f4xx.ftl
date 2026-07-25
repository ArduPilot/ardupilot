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

[#--
  -- Emits the STM32F4xx ADC driver constant configuration structures.
  --]
[#macro EmitADCConfig config]
  [#local cfg_name = config.name[0]?string /]
  [@code.EmitDoxygenDocumentationComment config /]
const ADCConfig ${cfg_name} = {0};

  [#list config.groups.group as group]
    [#local grpcfg_name = group.name[0]?string /]
    [@code.EmitDoxygenDocumentationComment group /]
const ADCGroupConfig ${grpcfg_name} = {
  /* Circular conversion flag.*/
  ${group.circular[0]?string?upper_case},
  /* Number of channels sampled in the conversion group.*/
  ${group.channels_sequence.channel?size},
  /* End of conversion callback or NULL.*/
    [#if group.conv_callback[0]?string?trim == ""]
  NULL,
    [#else]
  ${group.conv_callback[0]?string?trim},
    [/#if]
  /* Error callback or NULL.*/
    [#if group.error_callback[0]?string?trim == ""]
  NULL,
    [#else]
  ${group.error_callback[0]?string?trim},
    [/#if]
  /* CR1 register initialization value.*/
    [#local resolution = group.resolution[0]?word_list[0]?number /]
    [#local cr1 = "ADC_CR1_RESOLUTION_N(" + resolution?string + ")" /]
    [#local disc = group.discontinuous[0]?word_list[0]?number /]
    [#if disc > 0]
      [#local cr1 = cr1 + " | ADC_CR1_DISCEN" /]
      [#local cr1 = cr1 + " | ADC_CR1_DISCNUM_N(" + (disc - 1)?string + ")" /]
    [/#if]
  ${cr1},
  /* CR2 register initialization value.*/
    [#local exten = group.trigger_mode[0]?word_list[0]?number /]
    [#local cr2 = "ADC_CR1_EXTEN_N(" + exten?string + ")" /]
    [#local extsel = group.trigger_source[0]?word_list[0]?number /]
    [#local cr2 = cr2 + " | ADC_CR1_EXSEL_N(" + extsel?string + ")" /]
    [#if group.alignment[0]?word_list[0]?number != 0]
      [#local cr2 = cr2 + " | ADC_CR2_ALIGN" /]
    [/#if]
  ${cr2},
  /* Channels sample time settings.*/
    [#local smpr1 = "" /]
    [#local smpr2 = "" /]
    [#list group.sample_time.* as input]
      [#local sinput = input?node_name]
      [#if input_index < 9]
        [#local smpr2 = smpr2 + "ADC_SMPR2_SMP_" + input?node_name +
                                "(" + input?string + ") | " /]
      [#elseif input_index == 9]
        [#local smpr2 = smpr2 + "ADC_SMPR2_SMP_" + input?node_name +
                                "(" + input?string + ")," /]
      [#elseif input_index < 18]
        [#local smpr1 = smpr1 + "ADC_SMPR1_SMP_" + input?node_name +
                                "(" + input?string + ") | " /]
      [#else]
        [#local smpr1 = smpr1 + "ADC_SMPR1_SMP_" + input?node_name +
                                "(" + input?string + ")," /]
      [/#if]
    [/#list]
    [@utils.FormatStringAsText "  " "  " smpr1 80 /]
    [@utils.FormatStringAsText "  " "  " smpr2 80 /]
  /* Channels sequence.*/
    [#local sqr1 = "ADC_SQR1_NUM_CH(" + group.channels_sequence?size + ")" /]
    [#local sqr2 = "" /]
    [#local sqr3 = "" /]
    [#list group.channels_sequence.channel as channel]
      [#if channel_index <= 5]
        [#local sqr3 = sqr3 + "ADC_SQR3_SQ" + (channel_index + 1) +
                              "_N(" + channel + ")" /]
        [#if channel_has_next && channel_index < 5]
          [#local sqr3 = sqr3 + " | " /]
        [/#if]
      [#elseif channel_index <= 11]
        [#local sqr2 = sqr2 + "ADC_SQR2_SQ" + (channel_index + 1) +
                              "_N(" + channel + ")" /]
        [#if channel_has_next && channel_index < 11]
          [#local sqr2 = sqr2 + " | " /]
        [/#if]
      [#else]
        [#local sqr1 = sqr1 + " | ADC_SQR2_SQ" + (channel_index + 1) +
                              "_N(" + channel + ")" /]
      [/#if]
    [/#list]
    [#-- SQR2 could be empty.--]
    [#if sqr2 == ""]
       [#local sqr2 = "0" /]
    [/#if]      
    [#local sqr1 = sqr1 + "," /]
    [#local sqr2 = sqr2 + "," /]
    [@utils.FormatStringAsText "  " "  " sqr1 80 /]
    [@utils.FormatStringAsText "  " "  " sqr2 80 /]
    [@utils.FormatStringAsText "  " "  " sqr3 80 /]
};
  [/#list]
[/#macro]

[#--
  -- Emits the STM32F4xx ADC driver configuration external declarations.
  --]
[#macro EmitADCConfigExtern config]
  [#local cfg_name = config.name[0]?string /]
  [#list config.groups.group as group]
    [#local grpcfg_name = group.name[0]?string /]
    [#-- Only emits the comment if there is at least a callback defined.--]
  /* ADC configuration "${cfg_name}".*/
  extern const ADCConfig ${cfg_name};
  /* ADC conversion group "${grpcfg_name}".*/
  extern const ADCGroupConfig ${grpcfg_name};
    [#if group.conv_callback[0]?string?trim != ""]
  void ${group.conv_callback[0]?string?trim}(ADCDriver *, adcsample_t *, size_t);
    [/#if]
    [#if group.error_callback[0]?string?trim != ""]
  void ${group.error_callback[0]?string?trim}(ADCDriver *, adcerror_t);
    [/#if]

  [/#list]
[/#macro]
