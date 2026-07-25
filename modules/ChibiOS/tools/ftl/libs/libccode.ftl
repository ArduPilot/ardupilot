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
  -- Global flag for generated code.
  -->
[#assign generated = false]

<#--
  -- Coding style global settings.
  -->
[#assign indentation = "  "]
[#assign tab = 2]
[#assign fields_align = 28]
[#assign define_value_align = 44]
[#assign initializers_align = 28]
[#assign backslash_align = 76]
[#assign boundary = 80]

[#--
  -- Resets global variables.
  --]
[#macro ResetState]
  [#assign generated = false]
[/#macro]

[#--
  -- Returns the function/macro name from an XML node.
  --]
[#function GetName node=[] default="no-name"]
  [#local name = (node.@name[0]!default)?trim]
  [#return name]
[/#function]

[#--
  -- Returns the function return C type from an XML node.
  --]
[#function GetCType node=[] default="void"]
  [#local ctype = (node.@ctype[0]!default)?trim]
  [#if ctype?length == 0]
    [#local ctype = default]
  [/#if]
  [#return ctype]
[/#function]

[#--
  -- Returns the function/macro implementation from an XML node.
  --]
[#function GetImplementation node=[] default=""]
  [#local impl = node.implementation[0]!default]
  [#return impl]
[/#function]

[#--
  -- This function generates a variable or field declaration in a string.
  -- @note Processes the $I and $N tokens in the ctype.
  --]
[#function MakeVariableDeclaration indent="" name="no-name" ctype="no-ctype"]
  [#if ctype?contains("$I")]
    [#local s1 = ctype?keep_before("$I")?trim
            s2 = ctype?keep_after("$I")?trim /]
    [#local fstring = (indent + s1 + " ")?right_pad(fields_align) + s2]
    [#if fstring?contains("$N")]
      [#local fstring = fstring?replace("$N", name)]
    [#else]
      [#local fstring = fstring + name]
    [/#if]
  [#else]
    [#local fstring = indent + ctype]
    [#if fstring?contains("$N")]
      [#local fstring = fstring?replace("$N", name)]
    [#else]
      [#local fstring = (fstring + " ")?right_pad(fields_align) + name]
    [/#if]
  [/#if]
  [#return fstring + ";"]
[/#function]

[#--
  -- Creates a sequence containing function parameters taken from an XML node.
  -- @note Processes the $I and $N tokens in the ccode.
  --]
[#function MakeProtoParamsSequence params=[] node=[]]
  [#list node.param as param]
    [#local name  = (param.@name[0]!"no-name")?trim
            ctype = (param.@ctype[0]!"no-type $N")?trim /]
    [#if ctype?contains("$N")]
      [#local pstring = ctype?replace("$N", name)]
    [#else]
      [#if ctype?ends_with("*")]
        [#local pstring = ctype + "" + name]
      [#else]
        [#local pstring = ctype + " " + name]
      [/#if]
    [/#if]
    [#local params = params + [pstring]]
  [/#list]
  [#if params?size == 0]
    [#local params = ["void"]]
  [/#if] 
  [#return params]
[/#function]

[#--
  -- Creates a sequence containing parameters names taken from an XML node.
  --]
[#function MakeCallParamsSequence params=[] node=[]]
  [#list node.param as param]
    [#local name  = (param.@name[0]!"no-name")?trim]
    [#local params = params + [name]]
  [/#list]
  [#return params]
[/#function]

[#--
  -- Emits the specified indentation as spaces.
  --]
[#macro Indent n=0]
  [#list 1..n as i]
${indentation}[#rt]
  [/#list]
[/#macro]

[#--
  -- Emits a C function body code reformatting the indentation using the
  -- specified line prefix.
  --]
[#macro GenerateIndentedCCode indent=indentation ccode=""]
  [#local lines = ccode?string?split("^", "rm")]
  [#list lines as line]
    [#local s = line?chop_linebreak]
    [#if !line?is_first || (s?trim?length > 0)]
      [#if s?trim?length > 0]
        [#if s[0] == "#"]
${s}
        [#else]
${indent + s}
        [/#if]
      [#else]

      [/#if]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a variable or field.
  -- @note Does not generate the final EOL.
  -- @note Processes the $I and $N tokens in the ctype.
  --]
[#macro GenerateVariableDeclaration indent="" name="no-name" ctype="no-ctype"]
  [#local fstring = MakeVariableDeclaration(indent name ctype)]
${fstring}[#rt]
[/#macro]

[#--
  -- This macro generates a function prototype from an XML node.
  -- @note Does not generate the final EOL.
  --]
[#macro GeneratePrototype indent="" name="no-name" ctype="no-ctype"
                          modifiers=[] params=[]]
  [#if ctype?ends_with("*")]
    [#local l1 = ctype + name + "("]
  [#else]
    [#local l1 = ctype + " " + name + "("]
  [/#if]
  [#if modifiers?size > 0]
    [#local l1 = modifiers?join(" ") + " " + l1]
  [/#if]
  [#local l1 = indent + l1]
  [#local ln = ""?right_pad(l1?length)]
  [#list params as param]
    [#if param_index == 0]
      [#local line = l1 + param]
    [#else]
      [#if (line + ", " + param + "  ")?length > boundary]
${line + ","}
        [#local line = ln + param]
      [#else]
        [#local line = line + ", " + param]
      [/#if]
    [/#if]
  [/#list]
${line + ")"}[#rt]
[/#macro]

[#--
  -- This macro generates a function prototype from an XML node.
  -- @note Does not generate the final EOL.
  --]
[#macro GeneratePrototypeFromNode indent="" name="" ctype=""
                                  modifiers=[] params=[] node=[]]
  [#if name?length == 0]
    [#local name = GetName(node)]
  [/#if]
  [#if ctype?length == 0]
    [#local ctype = GetCType(node)]
  [/#if]
  [#local params = MakeProtoParamsSequence(params node)]
[@GeneratePrototype indent    = indent
                    name      = name
                    ctype     = ctype
                    modifiers = modifiers
                    params    = params /]
[/#macro]

[#--
  -- This macro generates a function call.
  --]
[#macro GenerateFunctionCall indent="" destination="" name="" params=[]]
  [#if name?length == 0]
    [#local name = "no-name"]
  [/#if]
  [#if destination?trim?length > 0]
    [#local l1 = indent + destination?trim + " " + name + "("]
  [#else]
    [#local l1 = indent + destination?trim + name + "("]
  [/#if]
  [#local ln = ""?right_pad(l1?length)]
  [#local line = l1]
  [#list params as param]
    [#if param_index == 0]
      [#local line = l1 + param]
    [#else]
      [#if (line + ", " + param + "  ")?length > boundary]
${line + ","}
        [#local line = ln + param]
      [#else]
        [#local line = line + ", " + param]
      [/#if]
    [/#if]
  [/#list]
${line + ");"}
[/#macro]

[#--
  -- Generates inclusions from an XML node.
  --]
[#macro GenerateInclusionsFromNode node=[]]
  [#list node.* as this]
    [#if this?node_name == "include"]
      [#local style = (this.@style[0]!"regular")?trim]
      [#if style == "angular"]
#include <${this[0]}>
      [#else]
#include "${this[0]}"
      [/#if]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateInclusionsFromNode node=this /]
#endif
    [#elseif this?node_name == "elseif"]
      [#local condcheck = (this.@check[0]!"")?trim]
      [#if condcheck?length == 0]
#else
      [#else]
#elif ${condcheck}
      [/#if]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates a single line definition macro from an XML node.
  --]
[#macro GenerateDefineFromNode node=[]]
  [#local define = node]
  [#local name   = (define.@name[0]!"no-name")?trim
          value  = (define.@value[0]!"no-value")?trim]
  [#if define.param[0]??]
    [#local params = MakeCallParamsSequence([], define)]
    [#local s = ("#define " + name +  "(" + params?join(", ") + ")")?right_pad(define_value_align) + value]
  [#else]
    [#local s = ("#define " + name +  " ")?right_pad(define_value_align) + value]
  [/#if]
${s}
[/#macro]

[#--
  -- Generates all single line definitions from an XML node.
  --]
[#macro GenerateDefinesFromNode node=[]]
  [#list node.* as this]
    [#if this?node_name == "define"]
      [#if this.brief[0]??]
[@doxygen.EmitFullCommentFromNode "" this /]
[@GenerateDefineFromNode this /]
        [#if this?has_next || node?node_name?starts_with("definitions")]

        [/#if]
      [#else]
[@GenerateDefineFromNode this /]
        [#if node?node_name?starts_with("definitions")]

        [/#if]
      [/#if]
    [#elseif this?node_name == "verbatim"]
      [#local ccode = (this[0]!"")?trim]
[@GenerateIndentedCCode "" ccode /]
      [#if node?node_name?starts_with("definitions")]

      [/#if]
    [#elseif this?node_name == "group"]
      [#local groupdescription = (this.@description[0]!"no-description")?trim]
/**
 * @name    ${groupdescription}
 * @{
 */
[@GenerateDefinesFromNode this /]
/** @} */
      [#if (node?node_name != "group") && (node?node_name != "condition")]

      [/#if]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateDefinesFromNode this /]
#endif /* ${condcheck} */
      [#if (node?node_name != "group") && (node?node_name != "condition")]

      [/#if]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates a conmfiguration definition from an XML node.
  --]
[#macro GenerateConfigFromNode node=[]]
  [#local config  = node]
  [#local name    = (config.@name[0]!"no-name")?trim]
  [#local default = (config.@default[0]!"no-default")?trim]
  [#local s         = ("#define " + name +  " ")?right_pad(define_value_align) +
                      default]
#if !defined(${name}) || defined(__DOXYGEN__)
${s}
#endif
[/#macro]

[#--
  -- Generates all configurations from an XML node.
  --]
[#macro GenerateConfigsFromNode node=[]]
  [#local configs = node]
  [#if configs.config[0]??]
/**
 * @name    Configuration options
 * @{
 */
    [#list configs.config as config]
[@doxygen.EmitFullCommentFromNode indent="" node=config /]
[@GenerateConfigFromNode node=config /]
      [#if !config?is_last]

      [/#if]
    [/#list]
/** @} */

  [/#if]
[/#macro]

[#--
  -- Generates all configuration checks from an XML node.
  -- @note Processes the $N token in the check expression and message.
  --]
[#macro GenerateConfigAssertsFromNode node=[]]
  [#local configs = node]
  [#list configs.* as this]
    [#if this?node_name == "config"]
      [#local name    = (this.@name[0]!"no-name")?trim]
      [#if this.assert[0]??]
/* Checks on ${name} configuration.*/
        [#list this.assert as assert]
          [#local invalid = (assert.@invalid[0]!"TRUE")?replace("$N", name)
                  message = assert[0]?trim?replace("$N", name) /]
#if ${invalid}
          [#if message?length > 0]
#error "${message}"
          [#else]
#error "invalid ${name} value"
          [/#if]
#endif

        [/#list]
      [/#if]
    [#elseif this?node_name == "verbatim"]
      [#local ccode = (this[0]!"")?trim]
      [#if ccode?length > 0]
[@GenerateIndentedCCode "" ccode /]

      [/#if]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates a multi-line C macro from an XML node.
  --]
[#macro GenerateMacroFromNode indent=indentation node=[]]
  [#local macro  = node]
  [#local name   = (macro.@name[0]!"no-name")?trim]
  [#local params = MakeCallParamsSequence([], macro)]
  [#local macroimpl = GetImplementation(macro)]
  [#if macroimpl?length == 0]
    [#local s      = "#define " + name + "(" + params?join(", ") + ")"]
${s}
  [#else]
    [#local s      = ("#define " + name + "(" + params?join(", ") +
                     ") ")?right_pad(backslash_align) + "\\"]
${s}
    [#local lines     = macroimpl?string?split("^", "rm")]
    [#list lines?filter(line -> line?trim?length > 0) as line]
      [#local s = line?chop_linebreak]
      [#if line?is_last]
${(indent + s + "")}
      [#else]
${(indent + s + "")?right_pad(backslash_align) + "\\"}
      [/#if]
    [/#list]
  [/#if]
[/#macro]

[#--
  -- Generates all macros from an XML node.
  --]
[#macro GenerateMacrosFromNode indent=indentation node=[]]
  [#list node.* as this]
    [#if this?node_name == "macro"]
      [#if this.brief[0]?? && !this?is_first]

      [/#if]
[@doxygen.EmitFullCommentFromNode "" this /]
[@GenerateMacroFromNode node=this /]
    [#elseif this?node_name == "group"]
      [#local groupdescription = (this.@description[0]!"no-description")?trim]
      [#if !this?is_first]

      [/#if]
/**
 * @name    ${groupdescription}
 * @{
 */
[@GenerateMacrosFromNode node=this /]
/** @} */
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
      [#if !this?is_first]

      [/#if]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateMacrosFromNode node=this /]
#endif /* ${condcheck} */
    [#elseif this?node_name == "elseif"]
      [#local condcheck = (this.@check[0]!"")?trim]
      [#if !this?is_first]

      [/#if]
      [#if condcheck?length == 0]
#else
      [#else]
#elif ${condcheck}
      [/#if]
    [/#if]
    [#if this?is_last && (node?node_name?starts_with("macros"))]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a simple type definition from an XML node.
  -- @note Processes the $N token in the ctype.
  --]
[#macro GenerateTypedefFromNode indent="" node=[]]
  [#local typedef = node]
  [#local typename = GetName(typedef)]
  [#if typedef.basetype[0]??]
    [#local basectype = GetCType(typedef.basetype[0])]
    [#if basectype?contains("$N")]
      [#local basectype = basectype?replace("$N", typename)]
${indent}typedef ${basectype};
    [#else]
${indent}typedef ${basectype} ${typename};
    [/#if]
  [#elseif typedef.structtype[0]??]
  [#elseif typedef.enumtype[0]??]
  [#elseif typedef.functype[0]??]
  [#else]
  [/#if]
[/#macro]

[#--
  -- This macro generates a struct definition from an XML node.
  --]
[#macro GenerateStructFromNode indent="" node=[] default="###no-name###"]
  [#local structname = GetName(node, default)]
  [#if structname?length > 0]
    [#local structname = structname + " "]
  [/#if]
${indent}struct ${structname}{
[@GenerateStructureFieldsFromNode indent+indentation node.fields /]
${indent}};
[/#macro]

[#--
  -- This macro generates a union definition from an XML node.
  --]
[#macro GenerateUnionFromNode indent="" node=[] default="###no-name###"]
  [#local unionname = GetName(node, default)]
  [#if unionname?length > 0]
    [#local unionname = unionname + " "]
  [/#if]
${indent}union ${unionname}{
[@GenerateStructureFieldsFromNode indent+indentation node.fields /]
${indent}};
[/#macro]

[#--
  -- Generates type definitions from an XML node.
  --]
[#macro GenerateTypesFromNode indent="" node=[]]
  [#list node.* as this]
    [#if this?node_name == "typedef"]
[@doxygen.EmitFullCommentFromNode indent this /]
[@GenerateTypedefFromNode node=this /]
    [#elseif this?node_name == "struct"]
[@doxygen.EmitFullCommentFromNode indent this /]
[@GenerateStructFromNode indent this /]
    [#elseif this?node_name == "union"]
[@doxygen.EmitFullCommentFromNode indent this /]
[@GenerateUnionFromNode indent this /]
    [#elseif this?node_name == "class"]
[@cclasses.GenerateClass this /]
    [#elseif this?node_name == "interface"]
[@cclasses.GenerateInterface this /]
    [#elseif this?node_name == "verbatim"]
      [#local ccode = (this[0]!"")?trim]
[@GenerateIndentedCCode indent ccode /]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateTypesFromNode "" this /]
#endif /* ${condcheck} */
    [/#if]
    [#if this?has_next || (node?node_name != "condition")]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a variable definition from an XML node.
  -- @note Processes the $N token in the ctype.
  --]
[#macro GenerateVariableFromNode indent="" node=[] static=false]
  [#local varname   = GetName(node)
          varctype  = GetCType(node)
          varstring = MakeVariableDeclaration(indent varname varctype) /]
  [#if static]
    [#local varstring = indent + "static " + varstring]
  [#else]
    [#local varstring = indent + varstring]
  [/#if]
[@doxygen.EmitFullCommentFromNode indent node /]
${varstring}
[/#macro]

[#--
  -- Generates variable definitions from an XML node.
  --]
[#macro GenerateVariablesFromNode indent="" node=[] static=false]
  [#list node.* as this]
    [#if this?node_name == "variable"]
      [#assign generated = true]
[@GenerateVariableFromNode indent this static /]
    [#elseif this?node_name == "verbatim"]
      [#local ccode = (this[0]!"")?trim]
[@GenerateIndentedCCode indent ccode /]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateVariablesFromNode indent this static /]
#endif /* ${condcheck} */
    [/#if]
    [#if this?has_next || (node?node_name != "condition")]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a variable extern definition from an XML node.
  -- @note Processes the $N token in the ctype.
  --]
[#macro GenerateVariableExternFromNode indent="" node=[] ]
  [#local varname   = GetName(node)
          varctype  = GetCType(node)
          varstring = MakeVariableDeclaration(indent+"extern " varname varctype) /]
${varstring}
[/#macro]

[#--
  -- Generates variable extern definitions from an XML node.
  --]
[#macro GenerateVariablesExternFromNode indent="" node=[]]
  [#list node.* as this]
    [#if this?node_name == "variable"]
      [#assign generated = true]
[@GenerateVariableExternFromNode indent this /]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateVariablesExternFromNode indent this /]
#endif /* ${condcheck} */
    [/#if]
    [#if this?is_last && (node?node_name != "condition")]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates structure fields from an XML node.
  --]
[#macro GenerateStructureFieldsFromNode indent="" fields=[]]
  [#list fields.* as node]
    [#if node?node_name == "field"]
      [#local field = node]
      [#local fieldname  = (field.@name[0]!"no-name")?trim
              fieldctype = (field.@ctype[0]!"no-ctype")?trim
              fieldstring = MakeVariableDeclaration(indentation fieldname fieldctype)]
[@doxygen.EmitFullCommentFromNode indent=indentation node=field /]
${fieldstring}
    [#elseif node?node_name == "condition"]
      [#local condition = node]
      [#local condcheck = (condition.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateStructureFieldsFromNode indent condition /]
#endif /* ${condcheck} */
    [#elseif node?node_name == "verbatim"]
      [#local ccode = (node[0]!"")?trim]
[@GenerateIndentedCCode indent ccode /]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates a function from an XML node.
  --]
[#macro GenerateFunctionFromNode modifiers=[] node=[]]
  [#local funcimpl = GetImplementation(node)]
[@GeneratePrototypeFromNode modifiers=modifiers node=node /] {
[@GenerateIndentedCCode indent=indentation ccode=funcimpl /]
}
[/#macro]

[#--
  -- Generates all functions from an XML node.
  --]
[#macro GenerateFunctionsFromNode modifiers=[] node=[]]
  [#list node.* as this]
    [#if this?node_name == "function"]
      [#assign generated = true]
      [#if !this?is_first]

      [/#if]
[@doxygen.EmitFullCommentFromNode "" this /]
[@GenerateFunctionFromNode modifiers=modifiers node=this /]
    [#elseif this?node_name == "group"]
      [#local groupdescription = (this.@description[0]!"no-description")?trim]
      [#if !this?is_first]

      [/#if]
/**
 * @name    ${groupdescription}
 * @{
 */
[@GenerateFunctionsFromNode modifiers=modifiers node=this /]
/** @} */
    [#elseif this?node_name == "verbatim"]
      [#if !this?is_first]

      [/#if]
      [#local ccode = (this[0]!"")?trim]
[@GenerateIndentedCCode "" ccode /]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
      [#if !this?is_first]

      [/#if]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateFunctionsFromNode modifiers=modifiers node=this /]
#endif /* ${condcheck} */
    [#elseif this?node_name == "elseif"]
      [#local condcheck = (this.@check[0]!"")?trim]
      [#if !this?is_first]

      [/#if]
      [#if condcheck?length == 0]
#else
      [#else]
#elif ${condcheck}
      [/#if]
    [/#if]
    [#if this?is_last && (node?node_name?starts_with("functions"))]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- Generates all function prototypes from an XML node.
  -- Prototypes are generated without spacing and without comments.
  --]
[#macro GenerateFunctionPrototypesFromNode indent=indentation modifiers=[] node=[]]
  [#list node.* as this]
    [#if this?node_name == "function"]
[@GeneratePrototypeFromNode indent=indent modifiers=modifiers node=this /];
    [#elseif this?node_name == "group"]
[@GenerateFunctionPrototypesFromNode indent=indent modifiers=modifiers node=this /]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if ${condcheck}
[@GenerateFunctionPrototypesFromNode indent=indent modifiers=modifiers node=this /]
#endif
    [#elseif this?node_name == "elseif"]
      [#local condcheck = (this.@check[0]!"")?trim]
      [#if condcheck?length == 0]
#else
      [#else]
#elif ${condcheck}
      [/#if]
    [/#if]
  [/#list]
[/#macro]
