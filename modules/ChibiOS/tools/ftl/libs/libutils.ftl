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
  -- Returns the trimmed text "s" making sure it is terminated by a dot.
  -- The empty string is always returned as an empty string, the dot is not
  -- added.
  --]
[#function WithDot s]
  [#local s = s?trim]
  [#if s == ""]
    [#return s]
  [/#if]
  [#if s?ends_with(".")]
    [#return s]
  [/#if]
  [#return s + "."]
[/#function]

[#--
  -- Returns the trimmed text "s" making sure it is not terminated by a dot.
  --]
[#function WithoutDot s]
  [#local s = s?trim]
  [#if s?ends_with(".")]
    [#return s?substring(0, s?length - 1)]
  [/#if]
  [#return s]
[/#function]

[#--
  -- Returns the trimmed text "s" making sure it is terminated by a dot if the
  -- text is composed of multiple phrases, if the text is composed of a single
  -- phrase then makes sure it is *not* terminated by a dot.
  -- A phrase is recognized by the pattern ". " into the text.
  -- The empty string is always returned as an empty string, the dot is never
  -- added.
  --]
[#function IntelligentDot s]
  [#local s = s?trim]
  [#if s?contains(". ")]
    [#return WithDot(s)]
  [/#if]
  [#return WithoutDot(s)]
[/#function]

[#--
  -- Formats a text string in a sequence of strings no longer than "len" (first
  -- line) or "lenn" (subsequent lines).
  -- White spaces are normalized between words, sequences of white spaces become
  -- a single space.
  --]
[#function StringToText len1 lenn s]
  [#local words=s?word_list]
  [#local line=""]
  [#local lines=[]]
  [#list words as word]
    [#if lines?size == 0]
      [#local len = len1]
    [#else]
      [#local len = lenn]
    [/#if]
    [#if (line?length + word?length + 1 > len)]
      [#local lines = lines + [line?trim]]
      [#local line = word + " "]
    [#else]
      [#local line = line + word + " "]
    [/#if]
  [/#list]
  [#if line != ""]
    [#local lines = lines + [line?trim]]
  [/#if]
  [#return lines]
[/#function]

[#--
  -- Emits a string "s" as a formatted text, the first line is prefixed by the
  -- "p1" parameter, subsequent lines are prefixed by the "pn" paramenter.
  -- Emitted lines are no longer than the "len" parameter.
  -- White spaces are normalized between words.
  --]
[#macro FormatStringAsText p1 pn s len]
  [#local lines = StringToText(len - p1?length, len - pn?length, s)]
  [#list lines as line]
    [#if line_index == 0]
${p1}${line}
    [#else]
${pn}${line}
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Emits a C function body code reformatting the indentation using the
  -- specified tab size and line prefix.
  --]
[#macro EmitIndentedCCode start tab ccode]
  [#assign lines = ccode?string?split("^", "rm")]
  [#list lines as line]
    [#if (line_index > 0) || (line?trim?length > 0)]
      [#if line?trim?length > 0]
        [#if line[0] == "#"]
${line?chop_linebreak}
        [#else]
${start + line?chop_linebreak}
        [/#if]
      [#else]

      [/#if]
    [/#if]
  [/#list]
[/#macro]
