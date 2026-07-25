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

[#import "/@ftllibs/libutils.ftl" as utils /]
[#import "/@ftllibs/liblicense.ftl" as license /]
[#import "/@ftllibs/libdoxygen.ftl" as doxygen /]
[#import "/@ftllibs/libccode.ftl" as ccode /]
[#import "/@ftllibs/libcclasses.ftl" as cclasses /]
[@pp.dropOutputFile /]
[#assign instance = xml.instance /]
[#-- Scanning all files to be generated.--]
[#list instance.modules.module as module]
  [#-- Generating the source file.--]
  [#assign modulename        = cclasses.GetNodeName(module) /]
  [#assign moduledescription = cclasses.GetNodeDescription(module) /]
  [#assign moduleheadername  = modulename + ".h" /]
  [#assign modulesourcename  = modulename + ".c" /]
  [#assign moduleimplname    = modulename + "_impl.inc" /]
  [#assign modulesourcepath  = (module.@sourcepath[0]!"src")?trim?ensure_ends_with("/") /]
  [#assign moduledocgroup    = modulename?upper_case /]
  [#-- Generating source file.--]
  [@pp.changeOutputFile name=pp.home?trim?ensure_ends_with("/") + modulesourcepath + modulesourcename /]
  [@cclasses.InitModule node=module /]
/*
[@license.EmitLicenseAsText /]
*/

/**
[@doxygen.EmitTagVerbatim indent="" tag="file" text=modulesourcename /]
[@doxygen.EmitBrief "" "Generated " + moduledescription + " source." /]
[@doxygen.EmitNote text="This is a generated file, do not edit directly." /]
 *
[@doxygen.EmitTagVerbatim indent="" tag="addtogroup" text=moduledocgroup /]
 * @{
 */

  [#-- Generating inclusions.--]
  [#if !module.private.includes_always[0]?? && !module.private.includes[0]??]
#include "${moduleheadername}"

  [/#if]
  [#if module.private.includes_always[0]??]
[@ccode.GenerateInclusionsFromNode module.private.includes_always /]

  [/#if]
  [#-- Handling of conditional modules.--]
  [#assign module_condition =  module.@check[0]!""?trim/]
  [#if module_condition?length > 0]
#if (${module_condition}) || defined(__DOXYGEN__)

  [/#if]
  [#if module.private.includes[0]??]
[@ccode.GenerateInclusionsFromNode module.private.includes /]

  [/#if]
/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

[#-- Generating local definitions.--]
[@ccode.GenerateDefinesFromNode node=module.private.definitions /]
/*===========================================================================*/
/* Module local macros.                                                      */
/*===========================================================================*/

[#-- Generating multi-line macros.--]
[@ccode.GenerateMacrosFromNode node=module.private.macros /]
/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

[@ccode.GenerateVariablesFromNode "" module.public.variables false /]
/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

[#-- Generating local types.--]
[@ccode.GenerateTypesFromNode node=module.private.types /]
/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

[@ccode.GenerateVariablesFromNode "" module.private.variables true /]
  [#if (module.@editcode[0]!"no")?trim == "false"]
/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

    [#-- Generating local C functions.--]
[@ccode.GenerateFunctionsFromNode modifiers=["static"] node=module.private.functions /]
/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

[@ccode.GenerateFunctionsFromNode node=module.public.functions /]
    [#-- Generating private classes code.--]
    [#list module.private.types.class as class]
/*===========================================================================*/
/* Module class ${"\"" + (cclasses.GetClassCType(class) + "\"" + " methods.")?right_pad(60)}*/
/*===========================================================================*/

[@cclasses.GenerateClassCode class=class modifiers=["static"] /]
[@cclasses.GenerateClassPrivateConstructor node=class /]
    [/#list]
    [#-- Generating public classes code.--]
    [#list module.public.types.class as class]
/*===========================================================================*/
/* Module class ${"\"" + (cclasses.GetClassCType(class) + "\"" + " methods.")?right_pad(60)}*/
/*===========================================================================*/

[@cclasses.GenerateClassCode class=class /]
    [/#list]
    [#-- Dropping the file if nothing has been generated inside.--]
    [#if (ccode.generated == false) && (cclasses.generated == false)]
      [@pp.dropOutputFile /]
    [/#if]
  [#else]
/* Module code has been generated into an hand-editable file and included
   here.*/
#include "${moduleimplname}"

  [/#if]
  [#if module_condition?length > 0]
#endif /* ${module_condition} */

  [/#if]
/** @} */
[/#list]
