[#ftl]
[#--
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
  --]
[#import "/@ftllibs/libutils.ftl" as utils /]
[@pp.dropOutputFile /]
[#if xml.instance[0]??]
  [#assign instance = xml.instance /]
[#else]
  [#list xml.*.application.instances.instance as inst]
    [#if inst.@id?string == "org.chibios.spc5.components.portable.chibios_unitary_tests_engine"]
      [#assign instance = inst /]
      [#break]
    [/#if]
  [/#list]
[/#if]
[#assign conf = {"instance":instance} /]
[#assign prefix_lower = conf.instance.global_data_and_code.code_prefix.value[0]?trim?lower_case /]
[#assign prefix_upper =conf. instance.global_data_and_code.code_prefix.value[0]?trim?upper_case /]
[#list conf.instance.sequences.sequence as sequence]
  [@pp.changeOutputFile name=prefix_lower+"test_sequence_" + (sequence_index + 1)?string("000") + ".h" /]
[@utils.EmitIndentedCCode "" 2 conf.instance.description.copyright.value[0] /]

/**
 * @file    ${prefix_lower}test_sequence_${(sequence_index + 1)?string("000")}.h
 * @brief   Test Sequence ${(sequence_index + 1)?string("000")} header.
 */

#ifndef ${prefix_upper}TEST_SEQUENCE_${(sequence_index + 1)?string("000")}_H
#define ${prefix_upper}TEST_SEQUENCE_${(sequence_index + 1)?string("000")}_H

extern const testsequence_t ${prefix_lower}test_sequence_${(sequence_index + 1)?string("000")};

#endif /* ${prefix_upper}TEST_SEQUENCE_${(sequence_index + 1)?string("000")}_H */
[/#list]
