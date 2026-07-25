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

<#--
  -- Coding style global settings.
  -->
[#assign indentation = "  " /]
[#assign fields_align = 24 /]
[#assign define_value_align = 36 /]
[#assign comments_align = 48 /]
[#assign boundary = 80 /]

[#--
  -- This macro generates a brief description in DoxyGen format.
  --]
[#macro EmitDoxygenBrief object=[]]
  [#if object.brief[0]??]
[@utils.FormatStringAsText " * @brief   "
                           " *          "
                           utils.WithDot(object.brief[0]?cap_first)
                           boundary /]
  [/#if]
[/#macro]

[#--
  -- This macro generates a detailed description in DoxyGen format.
  --]
[#macro EmitDoxygenDetails object=[]]
  [#if object.details[0]??]
[@utils.FormatStringAsText " * @details "
                           " *          "
                           utils.WithDot(object.details[0]?cap_first)
                           boundary /]
  [/#if]
[/#macro]

[#--
  -- This macro generates a notes list in DoxyGen format.
  --]
[#macro EmitDoxygenNotes object=[]]
  [#list object.* as note]
    [#if note?node_name == "note"]
      [@utils.FormatStringAsText " * @note    "
                                 " *          "
                                 utils.WithDot(note[0]?cap_first)
                                 boundary /]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a pre-requisites list in DoxyGen format.
  --]
[#macro EmitDoxygenPrerequisites object=[]]
  [#list object.* as pre]
    [#if pre?node_name == "pre"]
      [@utils.FormatStringAsText " * @pre     "
                                 " *          "
                                 utils.WithDot(pre[0]?cap_first)
                                 boundary /]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a post-requisites list in DoxyGen format.
  --]
[#macro EmitDoxygenPostrequisites object=[]]
  [#list object.* as post]
    [#if post?node_name == "post"]
      [@utils.FormatStringAsText " * @post    "
                                 " *          "
                                 utils.WithDot(post[0]?cap_first)
                                 boundary /]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a complete Doxygen documentation comment.
  --]
[#macro EmitDoxygenDocumentationComment object=[]]
/**
  [@code.EmitDoxygenBrief object /]
  [@code.EmitDoxygenDetails object /]
  [@code.EmitDoxygenPrerequisites object /]
  [@code.EmitDoxygenPostrequisites object /]
  [@code.EmitDoxygenNotes object /]
 */
[/#macro]

[#--
  -- This macro generates the parameters description in DoxyGen format.
  --]
[#macro EmitDoxygenParams params=[]]
  [#list params as param]
    [#local name = (param.@name[0]!"no-name")?trim /]
    [#local brief = (param.@brief[0]!"")?trim /]
    [#local dir = (param.@dir[0]!"boh")?trim?lower_case /]
    [#if dir == "in"]
[@utils.FormatStringAsText " * @param[in] "
                           " *            "
                           utils.IntelligentDot(name + " " + brief?uncap_first)
                           boundary /]
    [#elseif dir == "out"]
[@utils.FormatStringAsText " * @param[out] "
                           " *             "
                           utils.IntelligentDot(name + " " + brief?uncap_first)
                           boundary /]
    [#elseif dir == "both"]
[@utils.FormatStringAsText " * @param[in,out] "
                           " *                "
                           utils.IntelligentDot(name + " " + brief?uncap_first)
                           boundary /]
    [#elseif dir == "boh"]
[@utils.FormatStringAsText " * @param "
                           " *        "
                           utils.IntelligentDot(name + " " + brief?uncap_first)
                           boundary /]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a return description followed by a retval list
  -- in DoxyGen format.
  --]
[#macro EmitDoxygenReturn return=[]]
  [#if return[0]?? && ((return[0].@type[0]!"void")?trim != "void")]
    [#local brief = (return[0].@brief[0]!"")?trim /]
    [#if brief != ""]
[@utils.FormatStringAsText " * @return "
                           " *         "
                           utils.WithDot(brief?cap_first)
                           boundary /]
    [/#if]
    [#list return[0].value as value]
      [#local label = (value.@name[0]!"no-val")?trim /]
      [#local brief = (value.@brief[0]!"")?trim /]
[@utils.FormatStringAsText " * @retval "
                           " *         "
                           utils.WithDot(label + " " + brief?uncap_first)
                           boundary /]
    [/#list]
  [/#if]
[/#macro]

[#--
  -- This macro generates the inner function code (if present).
  --]
[#macro EmitCode code=[]]
  [#if function.code[0]?? && (function.code[0]?trim != "")]
${indentation}${function.code[0]?trim}
  [/#if]
[/#macro]

[#--
  -- Returns true if the module exports some functions.
  --]
[#function HasPublicFunctions module=[]]
  [#local flag = false /]
  [#list module.function as function]
    [#if (function.@visibility[0]!"private") == "public"]
      [#local flag = true /]
    [/#if]
  [/#list]
  [#return flag /]
[/#function]

[#--
  -- Returns true if the module has static functions.
  --]
[#function HasPrivateFunctions module=[]]
  [#local flag = false /]
  [#list module.function as function]
    [#if (function.@visibility[0]!"private") == "private"]
      [#local flag = true /]
    [/#if]
  [/#list]
  [#return flag /]
[/#function]

[#--
  -- This macro generates a function prototype from an XML "function"
  -- node passed as parameter.
  -- @note Does not generate the final EOL.
  --]
[#macro GeneratePrototype function={}]
  [#if function.return?? && function.return[0]??]
    [#local rettype = (function.return[0].@type[0]!"void")?trim /]
  [#else]
    [#local rettype = "void" /]
  [/#if]
  [#local name = (function.@name[0]!"no-name")?trim /]
  [#local visibility = (function.@visibility[0]!"private")?trim /]
  [#if function.param?? && function.param[0]??]
    [#-- If the function has parameters then generates the parameters list --] 
    [#local l1 = rettype + " " + name + "(" /]
    [#if visibility == "private"]
      [#local l1 = "static " + l1 /]
    [/#if]
    [#local ln = ""?right_pad(l1?length) /]
    [#list function.param as param]
      [#local type = (param.@type[0]!"no-type")?trim /]
      [#if type?contains("$")]
        [#local pstring = type?replace("$", (param.@name[0]!"no-name")?trim) /]
      [#else]
        [#local pstring = type + " " + (param.@name[0]!"no-name")?trim /]
      [/#if]
      [#local dir = (param.@dir[0]!"boh")?trim?lower_case /]
      [#if dir == "in"]
        [#local pstring = "const " + pstring /]
      [/#if]
      [#if param_index == 0]
        [#local line = l1 + pstring /]
      [#else]
        [#if (line + ", " + pstring + "  ")?length > boundary]
${line + ","}
          [#local line = ln + pstring /]
        [#else]
          [#local line = line + ", " + pstring /]
        [/#if]
      [/#if]
    [/#list]
${line + ")"}[#rt]
  [#else]
${rettype + " " + name}(void)[#rt]
  [/#if]
[/#macro]

[#--
  -- This macro generates a function (and its Doxygen documentation)
  -- from an XML "function" node passed as parameter.
  --]
[#macro GenerateFunction function={}]
/**
[@EmitDoxygenBrief function.@brief /]
[@EmitDoxygenDetails function.details /]
[@EmitDoxygenParams function.param /]
[@EmitDoxygenReturn function.return /]
 *
 * @note --Implementer notes here (or remove the tag)--
 * @bug --Known problems please here (or remove the tag)--
 * @todo --Implement this function (then remove the tag)--
 */
[@GeneratePrototype function /] {
  [#if function.code[0]??]
    [#-- Makes sure to undef the do_code macro --]
    [#assign inline = "[#ftl][#macro do_code function][/#macro]"?interpret /]
[@inline /]
    [#-- Interprets the code within the code element --]
    [#assign inline = function.code[0]?interpret /]
[@inline /]
[@do_code function /]
  [#else]

${indentation}/* ${function.@name[0]!"no-name"}() Implementation here! */
  [/#if]
}
[/#macro]

[#--
  -- Generates the implementations for the private functions in the specified
  -- module.
  --]
[#macro GeneratePrivateFunctionsImplementations module]
  [#list module.function as function]
    [#if (function.@visibility[0]!"private") == "private"]
[@code.GenerateFunction function /]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates the prototypes of the public functions in the specified
  -- module.
  --]
[#macro GeneratePublicFunctionsPrototypes indentation module]
  [#list module.function as function]
    [#if (function.@visibility[0]!"private")?trim == "public"]
${indentation}[@code.GeneratePrototype function /];
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates the implementations for the public functions in the specified
  -- module.
  --]
[#macro GeneratePublicFunctionsImplementations module]
  [#list module.function as function]
    [#if (function.@visibility[0]!"private") == "public"]
[@code.GenerateFunction function /]

    [/#if]
  [/#list]
[/#macro]
