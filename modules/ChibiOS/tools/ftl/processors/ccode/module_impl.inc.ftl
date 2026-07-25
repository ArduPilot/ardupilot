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
  [#-- Checking if an implementation file is required.--]
  [#if (module.@editcode[0]!"no")?trim == "true"]
    [#assign modulename        = cclasses.GetNodeName(module) /]
    [#assign moduledescription = cclasses.GetNodeDescription(module) /]
    [#assign moduleheadername  = modulename + ".h" /]
    [#assign modulesourcename  = modulename + ".c" /]
    [#assign moduleimplname    = modulename + "_impl.inc" /]
    [#assign modulesourcepath  = (module.@sourcepath[0]!"src")?trim?ensure_ends_with("/") /]
    [#assign moduledocgroup    = modulename?upper_case /]
    [#assign filename = pp.home?trim?ensure_ends_with("/") +
                        modulesourcepath + moduleimplname /]
    [#-- Generating the implementation file only if it does not already exists.--]
    [#if pp.outputFileExists(filename) == false]
      [#-- Generating handwritten source file.--]
      [@pp.changeOutputFile name=filename /]
      [@cclasses.InitModule node=module /]
/*
[@license.EmitLicenseAsText /]
*/

/**
[@doxygen.EmitTagVerbatim indent="" tag="file" text=moduleimplname /]
[@doxygen.EmitBrief "" "Template of " + moduledescription + " source." /]
[@doxygen.EmitNote text="This is a template file, can be edited directly." /]
 *
[@doxygen.EmitTagVerbatim indent="" tag="addtogroup" text=moduledocgroup /]
 * @{
 */

/* This is an, automatically generated, implementation file that can be
   manually edited, it is not re-generated if already present.*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

      [#-- Generating local C functions.--]
[@ccode.GenerateFunctionsFromNode modifiers=["static"]
                                  node=module.private.functions /]
/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

      [#-- Generating global C functions.--]
[@ccode.GenerateFunctionsFromNode modifiers=[]
                                  node=module.public.functions /]
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

[@cclasses.GenerateClassCode class=class modifiers=[] /]
      [/#list]
    [/#if]
  [/#if]
/** @} */

[/#list]
