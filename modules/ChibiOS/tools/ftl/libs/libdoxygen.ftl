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
[#assign doxygen_boundary = 80]
[#assign near_align = 16]
[#assign middle_align = 32]

[#--
  -- Ouputs a formatted text starting from a "rich text" node containing
  -- text and formatting elements.
  --]
[#macro EmitRichTextFromNode p1="" pn="" textnode=[] limit=doxygen_boundary]
  [#list textnode?children as node]
    [#if node?is_first]
      [#local l1 = p1
              ln = pn /]
    [#else]
      [#local l1 = pn
              ln = pn /]
    [/#if]
    [#if node?node_type == "text"]
      [#local text = node?trim?cap_first]
      [#if text?matches("^.*[a-zA-Z0-9]$")]
[@utils.FormatStringAsText l1 ln utils.WithDot(text) limit /]
      [#else]
[@utils.FormatStringAsText l1 ln text doxygen_boundary /]
      [/#if]
    [#elseif node?node_type == "element"]
      [#if node?node_name == "verbatim"]
        [#local lines = node?string?split("^", "rm")]
        [#list lines as line]
          [#local s = line?chop_linebreak]
          [#if (line_index > 0) || (s?trim?length > 0)]
            [#if line?is_first]
${l1 + s}
            [#else]
${ln + s}
            [/#if]
          [/#if]
        [/#list]
      [#elseif node?node_name == "br"]
      [/#if]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a generic tag with unformatted text.
  --]
[#macro EmitTagVerbatim indent="" tag="no-tag" text="" align=near_align]
  [#local s = (indent + " * @" + tag + " ")?right_pad(align) + text]
${s}
[/#macro]

[#--
  -- This macro generates a generic tag with formatted text.
  --]
[#macro EmitTagFormatted indent="" tag="no-tag" text="" align=near_align]
[@utils.FormatStringAsText indent + (" * @" + tag + " ")?right_pad(align)
                           indent + (" *")?right_pad(align)
                           utils.WithDot(text?cap_first)
                           doxygen_boundary /]
[/#macro]

[#--
  -- This macro generates a generic tag with formatted text.
  --]
[#macro EmitTagFormattedNoCap indent="" tag="no-tag" text="" align=near_align]
[@utils.FormatStringAsText indent + (" * @" + tag + " ")?right_pad(align)
                           indent + (" *")?right_pad(align)
                           utils.WithDot(text)
                           doxygen_boundary /]
[/#macro]

[#--
  -- This macro generates a brief tag description.
  --]
[#macro EmitBrief indent="" text=""]
[@EmitTagFormatted indent "brief" text /]
[/#macro]

[#--
  -- This macro generates a details tag description.
  --]
[#macro EmitDetails indent="" text=""]
[@EmitTagFormatted indent "details" text /]
[/#macro]

[#--
  -- This macro generates a pre tag description.
  --]
[#macro EmitPre indent="" text=""]
[@EmitTagFormatted indent "pre" text /]
[/#macro]

[#--
  -- This macro generates a post tag description.
  --]
[#macro EmitPost indent="" text=""]
[@EmitTagFormatted indent "post" text /]
[/#macro]

[#--
  -- This macro generates a note tag description.
  --]
[#macro EmitNote indent="" text=""]
[@EmitTagFormatted indent "note" text /]
[/#macro]

[#--
  -- This macro generates a param tag description.
  --]
[#macro EmitParam indent="" name="no-name" dir="boh" text="no-text"]
  [#if text?trim?length == 0]
    [#local text="missing description"]
  [/#if]
  [#if dir == "in"]
[@utils.FormatStringAsText indent + (" * @param[in]     " + name + " ")?right_pad(middle_align)
                           indent + " *"?right_pad(middle_align)
                           utils.WithDot(text?cap_first)
                           doxygen_boundary /]
  [#elseif dir == "out"]
[@utils.FormatStringAsText indent + (" * @param[out]    " + name + " ")?right_pad(middle_align)
                           indent + " *"?right_pad(middle_align)
                           utils.WithDot(text?cap_first)
                           doxygen_boundary /]
  [#elseif dir == "both"]
[@utils.FormatStringAsText indent + (" * @param[in,out] " + name + " ")?right_pad(middle_align)
                           indent + " *"?right_pad(middle_align)
                           utils.WithDot(text?cap_first)
                           doxygen_boundary /]
  [#else]
[@utils.FormatStringAsText indent + (" * @param         " + name + " ")?right_pad(middle_align)
                           indent + " *"?right_pad(middle_align)
                           utils.WithDot(text?cap_first)
                           doxygen_boundary /]
  [/#if]
[/#macro]

[#--
  -- This macro generates a return tag description.
  --]
[#macro EmitReturn indent="" text=""]
[@utils.FormatStringAsText indent + (" * @return")?right_pad(middle_align)
                           indent + " *"?right_pad(middle_align)
                           utils.WithDot(text?cap_first)
                           doxygen_boundary /]
[/#macro]

[#--
  -- This macro generates a brief description from an XML node.
  --]
[#macro EmitBriefFromNode indent="" node=[]]
  [#if node.brief[0]??]
[@doxygen.EmitBrief indent node.brief[0]!"no description" /]
  [/#if]
[/#macro]

[#--
  -- This macro generates a detailed description from an XML node.
  --]
[#macro EmitDetailsFromNode indent="" node=[]]
  [#if node.details[0]??]
[@EmitRichTextFromNode indent + (" * @details ")?right_pad(near_align)
                       indent + " *"?right_pad(near_align)
                       node.details /]
  [/#if]
[/#macro]

[#--
  -- This macro generates a pre tags list from an XML nodet.
  --]
[#macro EmitPreFromNode indent="" node=[]]
  [#list node.pre as pre]
[@EmitRichTextFromNode indent + " * @pre "?right_pad(near_align)
                       indent + " *"?right_pad(near_align)
                       pre /]
  [/#list]
[/#macro]

[#--
  -- This macro generates a post tags list from an XML node.
  --]
[#macro EmitPostFromNode indent="" node=[]]
  [#list node.post as post]
[@EmitRichTextFromNode indent + " * @post "?right_pad(near_align)
                       indent + " *"?right_pad(near_align)
                       post /]
  [/#list]
[/#macro]

[#--
  -- This macro generates a notes list from an XML node.
  --]
[#macro EmitNoteFromNode indent="" node=[]]
  [#list node.note as note]
[@EmitRichTextFromNode indent + " * @note "?right_pad(near_align)
                       indent + " *"?right_pad(near_align)
                       note /]
  [/#list]
[/#macro]

[#--
  -- This macro generates a params list from an XML node.
  --]
[#macro EmitParamFromNode indent="" node=[]]
  [#list node.param as param]
    [#local name = param.@name[0]!"no-name"
            dir  = param.@dir[0]!"no-dir" /]
    [#if dir == "in"]
      [#local p1 = indent + (" * @param[in]     " + name + " ")?right_pad(middle_align)]
    [#elseif dir == "out"]
      [#local p1 = indent + (" * @param[out]    " + name + " ")?right_pad(middle_align)]
    [#elseif dir == "both"]
      [#local p1 = indent + (" * @param[in,out] " + name + " ")?right_pad(middle_align)]
    [#else]
      [#local p1 = indent + (" * @param         " + name + " ")?right_pad(middle_align)]
    [/#if]
    [#local pn = indent + " *"?right_pad(middle_align)]
[@EmitRichTextFromNode p1 pn param /]
  [/#list]
[/#macro]

[#--
  -- This macro generates a return tag from an XML node.
  --]
[#macro EmitReturnFromNode indent="" node=[]]
  [#if node.return[0]??]
    [#local p1 = (indent + " * @return ")?right_pad(middle_align)
            pn = (indent + " *         ")?right_pad(middle_align) /]
[@EmitRichTextFromNode p1 pn node.return /]
  [/#if]
[/#macro]

[#--
  -- This macro generates retval tags from an XML node.
  --]
[#macro EmitRetvalFromNode indent="" node=[]]
  [#list node.retval as retval]
    [#local value = (retval.@value[0]!"no-value")?trim ]
    [#local p1 = (indent + " * @retval " + value + " ")?right_pad(middle_align)
            pn = (indent + " *         ")?right_pad(middle_align) /]
[@EmitRichTextFromNode p1 pn retval /]
  [/#list]
[/#macro]

[#--
  -- This macro generates a function class tag from an XML node.
  --]
[#macro EmitFunctionClassFromNode indent="" node=[]]
  [#if node.init[0]??]
${indent + " *"}
${indent + " * @init"}
  [#elseif node.objinit[0]??]
${indent + " *"}
${indent + " * @objinit"}
  [#elseif node.dispose[0]??]
${indent + " *"}
${indent + " * @dispose"}
  [#elseif node.api[0]??]
${indent + " *"}
${indent + " * @api"}
  [#elseif node.notapi[0]??]
${indent + " *"}
${indent + " * @notapi"}
  [#elseif node.iclass[0]??]
${indent + " *"}
${indent + " * @iclass"}
  [#elseif node.sclass[0]??]
${indent + " *"}
${indent + " * @sclass"}
  [#elseif node.xclass[0]??]
${indent + " *"}
${indent + " * @xclass"}
  [/#if]
[/#macro]

[#--
  -- This macro generates a complete Doxygen documentation comment.
  --]
[#macro EmitFullCommentFromNode indent="" node=[]
                                extraname="" extradir="" extratext=""
                                memberof=""]
  [#if node.brief[0]?? || (memberof?length > 0)]
${indent}/**
  [#if memberof?length > 0]
[@EmitTagVerbatim indent "memberof" memberof /]
${indent} * @public
${indent} *
  [/#if]
[@EmitBriefFromNode indent node /]
[@EmitDetailsFromNode indent node /]
[@EmitPreFromNode indent node /]
[@EmitPostFromNode indent node /]
[@EmitNoteFromNode indent node /]
    [#if node.param[0]?? || node.return[0]?? || (extraname?length > 0)]
${indent} *
      [#if extraname?length > 0]
[@EmitParam indent extraname extradir extratext /]
      [/#if]
[@EmitParamFromNode indent node /]
[@EmitReturnFromNode indent node /]
[@EmitRetvalFromNode indent node /]
    [/#if]
[@EmitFunctionClassFromNode indent node /]
${indent} */
  [/#if]
[/#macro]
