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

[#assign class_suffix = "_c"]
[#assign interface_suffix = "_i"]
[#assign xmlcache = {}]
[#assign classescache={}]
[#assign ifscache={}]

[#--
  -- Resets global variables.
  --]
[#macro ResetState]
  [#assign generated = false]
[/#macro]

[#--
  -- Loads an XML file into the XML cache.
  --]
[#function LoadXML xmlname=""]
  [#attempt]
    [#-- Trying the default path.--]
    [#return pp.loadData("xml", xmlname)]
  [#recover]
    [#list xml.instance.paths.path as path]
      [#attempt]
        [#return pp.loadData("xml", path[0]?ensure_ends_with("/") + xmlname)]
      [#recover]
      [/#attempt]
    [/#list]
  [/#attempt]
  [@pp.dropOutputFile /]
  [#stop ">>>> Importing '" + xmlname + "' failed!"]
[/#function]

[#--
  -- Getting references of all module public classes/interfaces.
  --]
[#macro ImportModulePublicClasses node=[]]
  [#list node.public.types.interface as interface]
    [#assign ifscache = ifscache + {GetNodeName(interface):interface}]
  [/#list]
  [#list node.public.types.class as class]
    [#assign classescache = classescache + {GetNodeName(class):class}]
  [/#list]
  [#list node.imports.import as import]
    [#local xmlname = import?trim]
    [#if xmlcache[xmlname]??]
        [#-- Already in cache, no need to reimport.--]
    [#else]
      [#local xmldoc = LoadXML(xmlname)]
      [@ImportModulePublicClasses node=xmldoc.module /]
      [#assign xmlcache = xmlcache + {xmlname:xmldoc}]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- Getting references of all module private classes/interfaces.
  --]
[#macro ImportModulePrivateClasses node=[]]
  [#list node.public.types.interface as interface]
    [#assign ifscache = ifscache + {GetNodeName(interface):interface}]
  [/#list]
  [#list node.public.types.class as class]
    [#assign classescache = classescache + {GetNodeName(class):class}]
  [/#list]
[/#macro]

[#macro InitModule node=[]]
  [@ccode.ResetState /]
  [@ResetState /]
  [@ImportModulePrivateClasses node=node /]
  [@ImportModulePublicClasses node=node /]
[#-- ${xmlcache?keys?join(", ")} - ${classescache?keys?join(", ")} - ${ifscache?keys?join(", ")} --]
[/#macro]

[#--
  -- Returns the name attribute from an XML node.
  --]
[#function GetNodeName node=[] default="no-name"]
  [#return (node.@name[0]!default)?trim]
[/#function]

[#--
  -- Returns the namespace attribute from an XML node.
  --]
[#function GetNodeNamespace node=[] default="no-namespace"]
  [#return (node.@namespace[0]!default)?trim]
[/#function]

[#--
  -- Returns the ancestorname attribute from an XML node.
  --]
[#function GetNodeAncestorName node=[] default=""]
  [#return (node.@ancestorname[0]!default)?trim]
[/#function]

[#--
  -- Returns the descr attribute from an XML node.
  --]
[#function GetNodeDescription node=[] default="no-descr"]
  [#return (node.@descr[0]!default)?trim]
[/#function]

[#--
  -- Returns the interface C type from an XML node.
  --]
[#function GetInterfaceCType node=[] default=""]
  [#local if = node]
  [#local ifname  = GetNodeName(if default)]
  [#if ifname?length > 0]
    [#local ifctype = ifname + interface_suffix]
  [#else ]
    [#local ifctype = default]
  [/#if]
  [#return ifctype]
[/#function]

[#--
  -- Returns the interface ancestor C type from an XML node.
  --]
[#function GetInterfaceAncestorCType node=[] default=""]
  [#local if = node]
  [#local ancestorname  = GetNodeAncestorName(if default)]
  [#if ancestorname?length > 0]
    [#local ancestorctype = ancestorname + interface_suffix]
  [#else ]
    [#local ancestorctype = default]
  [/#if]
  [#return ancestorctype]
[/#function]

[#--
  -- Returns the class C type from an XML node.
  --]
[#function GetClassCType node=[] default=""]
  [#local class = node]
  [#local classname  = GetNodeName(class default)]
  [#if classname?length > 0]
    [#local classctype = classname + class_suffix]
  [#else ]
    [#local classctype = default]
  [/#if]
  [#return classctype]
[/#function]

[#--
  -- Returns the class ancestor C type from an XML node.
  --]
[#function GetClassAncestorCType node=[] default=""]
  [#local class = node]
  [#local ancestorname  = GetNodeAncestorName(class default)]
  [#if ancestorname?length > 0]
    [#local ancestorctype = ancestorname + class_suffix]
  [#else ]
    [#local ancestorctype = default]
  [/#if]
  [#return ancestorctype]
[/#function]

[#--
  -- Returns the class type from an XML node.
  --]
[#function GetClassType node=[] default="no-type"]
  [#local class = node]
  [#local classtype = (class.@type[0]!default)?trim]
  [#return classtype]
[/#function]

[#--
  -- Returns the method short name from an XML node.
  --]
[#function GetMethodShortName node=[] default=""]
  [#local method = node]
  [#local methodsname = (method.@shortname[0]!default)?trim]
    [#if methodsname?length == 0]
      [#local methodsname = GetNodeName(method default)?lower_case]
    [/#if]
  [#return methodsname]
[/#function]

[#--
  -- Returns the method return C type from an XML node.
  --]
[#function GetMethodCType node=[] default="void"]
  [#local method = node]
  [#local methodctype = (method.@ctype[0]!default)?trim]
  [#return methodctype]
[/#function]

[#--
  -- Returns the method short name from an XML node.
  --]
[#function GetObjinitCallsuper node=[] default="true"]
  [#return (node.@callsuper[0]!default)?trim]
[/#function]

[#--
  -- Get class by name.
  --]
[#function GetClassByName classname=""]
  [#if classescache[classname]??]
    [#return classescache[classname]]
  [/#if]
  [#stop ">>>> Unknown class '" + classname + "'"]
[/#function]

[#--
  -- Get interface by name.
  --]
[#function GetInterfaceByName ifname=""]
  [#if ifscache[ifname]??]
    [#return ifscache[ifname]]
  [/#if]
  [#stop ">>>> Unknown interface '" + ifname + "'"]
[/#function]

[#--
  -- Returns the ancestor of the specified class. 
  --]
[#function GetAncestorClass class=[]]
  [#local ancestorname = GetNodeAncestorName(class)]
  [#if classescache[ancestorname]??]
    [#return classescache[ancestorname]]
  [/#if]
  [#stop ">>>> Unknown class '" + ancestorname + "'"]
[/#function]

[#--
  -- Returns a sequence of class ancestors classes from an XML node.
  --]
[#function GetClassAncestorsSequence class=[]]
  [#local ancestorname = GetNodeAncestorName(class)]
  [#if ancestorname?length == 0]
    [#return []]
  [/#if]
  [#if classescache[ancestorname]??]
    [#local ancestorclass = classescache[ancestorname]]
    [#return GetClassAncestorsSequence(ancestorclass) + [ancestorclass]]
  [/#if]
  [#stop ">>>> Unknown class '" + ancestorname + "'"]
[/#function]

[#-- 
  -- Returns a sequence of class virtual methods, including those from
  -- ancestors.
  --]
[#function GetClassVirtualMethodsSequence node=[]]
  [#local classes = GetClassAncestorsSequence(node) + [node]]
  [#local methods = []]
  [#list classes as class]
    [#list class.methods.virtual.method as method]
      [#local methods = methods + method]
    [/#list]
  [/#list]
  [#return methods]
[/#function]

[#--
  -- Returns a sequence of interface ancestors interfaces from an XML node.
  --]
[#function GetInterfaceAncestorsSequence if=[]]
  [#local ancestorname = GetNodeAncestorName(if)]
  [#if ancestorname?length == 0]
    [#return []]
  [/#if]
  [#if ifscache[ancestorname]??]
    [#local ancestorif = ifscache[ancestorname]]
    [#return GetInterfaceAncestorsSequence(ancestorif) + [ancestorif]]
  [/#if]
  [#stop ">>>> Unknown interface '" + ancestorname + "'"]
[/#function]

[#-- 
  -- Returns a sequence of interface methods, including those from
  -- ancestors.
  --]
[#function GetInterfaceMethodsSequence node=[]]
  [#local ifs = GetInterfaceAncestorsSequence(node) + [node]]
  [#local methods = []]
  [#list ifs as if]
    [#list if.methods.method as method]
      [#local methods = methods + method]
    [/#list]
  [/#list]
  [#return methods]
[/#function]

[#--
  -- Returns a sequence of class ancestors CTypes.
  --]
[#function GetClassAncestorsCTypes ancestors=[]]
  [#local names = []]
  [#list ancestors as ancestor]
    [#local names = names + [GetClassCType(ancestor)]]
  [/#list]
  [#return names]
[/#function]

[#--
  -- Returns a sequence of interface ancestors CTypes.
  --]
[#function GetInterfaceAncestorsCTypes ancestors=[]]
  [#local names = []]
  [#list ancestors as ancestor]
    [#local names = names + [GetInterfaceCType(ancestor)]]
  [/#list]
  [#return names]
[/#function]

[#--
  -- Returns the closest method implementation name searching in the specified
  -- set of classes.
  --]
[#function GetMethodNearestImplementation classes=[] method=[]]
  [#local shortname = GetMethodShortName(method)]
  [#list classes?reverse as class]
    [#local classname      = GetNodeName(class)
            classnamespace = GetNodeNamespace(class)]
    [#-- The dispose method is a special clase, it is always generated for
         each class so picking the current class one. --]
    [#if shortname == "dispose"]
      [#return "__" + classnamespace + "_" + shortname + "_impl"]
    [/#if]
    [#-- First checking among the current class overrides.--]
    [#local m = class["methods/override/method[@shortname='" + shortname + "']"]]
    [#if m[0]??]
      [#return "__" + classnamespace + "_" + shortname + "_impl"]
    [/#if]
    [#-- Checking if we reached the class where the method was declared.--]
    [#local m = class["methods/virtual/method[@shortname='" + shortname + "']"]]
    [#if m[0]??]
      [#-- If the methods has an original implementation then returning it
           else we return NULL but the system has to generate an error
           because the method has no implementation in any ancestor.--]
      [#if m.implementation[0]??]
        [#return "__" + classnamespace + "_" + shortname + "_impl"]
      [#else]
        [#return "NULL /* Method not found.*/"]
      [/#if]
    [/#if]
  [/#list]
  [#return "NULL"]
[/#function]

[#--
  -- Returns the class in the sequence of ancestors owning the specified
  -- virtual method. Fails if the method is not defined anywere in the
  -- sequence.
  --]
[#function GetVirtualMethodOwnerClass ancestors=[] shortname=""]
  [#list ancestors as ancestor]
    [#local m = ancestor["methods/virtual/method[@shortname='" + shortname + "']"]]
    [#if m[0]??]
      [#return [ancestor, m]]
    [/#if]]
  [/#list]
  [#stop ">>>> Undefined method '" + shortname + "'"]
[/#function]

[#--
  -- This macro generates a class wrapper from an XML node.
  --]
[#macro GenerateClass node=[]]
  [#local class = node]
  [#local classname         = GetNodeName(class)
          classnamespace    = GetNodeNamespace(class)
          classctype        = GetClassCType(class)
          classdescr        = GetNodeDescription(class)
          ancestorname      = GetNodeAncestorName(class, "")]
  [#local ancestors = GetClassAncestorsSequence(class)]
/**
  [@doxygen.EmitTagVerbatim indent="" tag="class" text=classctype /]
  [#if ancestorname?length > 0]
    [@doxygen.EmitTagVerbatim indent="" tag="extends" text=GetClassAncestorCType(class "") /]
  [/#if]
  [@GenerateClassImplementsTags class.implements /]
 *
  [@doxygen.EmitBriefFromNode node=class /]
  [@doxygen.EmitDetailsFromNode node=class /]
  [@doxygen.EmitPreFromNode node=class /]
  [@doxygen.EmitPostFromNode node=class /]
  [@doxygen.EmitNoteFromNode node=class /]
 *
  [@doxygen.EmitTagVerbatim indent="" tag="name" text="Class @p " + classctype + " structures"/]
 * @{
 */

/**
  [@doxygen.EmitBrief "" "Type of a " + classdescr + " class." /]
 */
typedef struct ${classname} ${classctype};

/**
  [@doxygen.EmitBrief "" "Class @p " + classctype + " virtual methods table." /]
 */
struct ${classname?lower_case}_vmt {
  [#list ancestors as ancestor]
[@ccode.Indent 1 /]/* From ${GetClassCType(ancestor)}.*/
    [@GenerateVMTPointers ancestor.methods.virtual /]
  [/#list]
[@ccode.Indent 1 /]/* From ${classctype}.*/
  [@GenerateVMTPointers class.methods.virtual /]
};

/**
  [@doxygen.EmitBrief "" "Structure representing a " + classdescr + " class." /]
 */
struct ${classname?lower_case} {
[@ccode.Indent 1 /]/**
[@doxygen.EmitBrief ccode.indentation "Virtual Methods Table." /]
[@ccode.Indent 1 /] */
  [#local vmtctype  = "const struct " + classname?lower_case + "_vmt$I*$N"]
${ccode.MakeVariableDeclaration(ccode.indentation "vmt" vmtctype)}
  [#list ancestors as ancestor]
    [@GenerateClassInterfaceFields node = ancestor.implements /]
    [@ccode.GenerateStructureFieldsFromNode indent = ccode.indentation
                                            fields = ancestor.fields /]
  [/#list]
  [@GenerateClassInterfaceFields node = class.implements /]
  [@ccode.GenerateStructureFieldsFromNode indent = ccode.indentation
                                          fields = class.fields /]
};
/** @} */
[/#macro]

[#--
  -- This macro generates an interface wrapper from an XML node.
  --]
[#macro GenerateInterface node=[]]
  [#local if = node]
  [#local ifname            = GetNodeName(if)
          ifnamespace       = GetNodeNamespace(if)
          ifctype           = GetInterfaceCType(if)
          ifdescr           = GetNodeDescription(if)
          ancestorname      = GetNodeAncestorName(if, "")]
  [#local ancestors = GetInterfaceAncestorsSequence(if)]
/**
 * @interface   ${ifctype}
  [#if ancestorname?length > 0]
    [@doxygen.EmitTagVerbatim indent="" tag="extends" text=GetInterfaceAncestorCType(if "") /]
  [/#if]
 *
[@doxygen.EmitBriefFromNode node=if /]
[@doxygen.EmitDetailsFromNode node=if /]
[@doxygen.EmitPreFromNode node=if /]
[@doxygen.EmitPostFromNode node=if /]
[@doxygen.EmitNoteFromNode node=if /]
 *
[@doxygen.EmitTagVerbatim indent="" tag="name" text="Interface @p " + ifctype + " structures"/]
 * @{
 */

/**
  [@doxygen.EmitBrief "" "Type of a " + ifdescr + " interface." /]
 */
typedef struct ${ifname} ${ifctype};

/**
  [@doxygen.EmitBrief "" "Interface @p " + ifctype + " virtual methods table." /]
 */
struct ${ifname?lower_case}_vmt {
${(ccode.indentation + "/* Memory offset between this interface structure and begin of")}
${(ccode.indentation + "   the implementing class structure.*/")}
${(ccode.indentation + "size_t instance_offset;")}
  [#list ancestors as ancestor]
[@ccode.Indent 1 /]/* From ${GetInterfaceCType(ancestor)}.*/
    [@GenerateVMTPointers ancestor.methods /]
  [/#list]
[@ccode.Indent 1 /]/* From ${ifctype}.*/
  [@GenerateVMTPointers if.methods /]
};

/**
  [@doxygen.EmitBrief "" "Structure representing a " + ifdescr + " interface." /]
 */
struct ${ifname?lower_case} {
[@ccode.Indent 1 /]/**
[@doxygen.EmitBrief ccode.indentation "Virtual Methods Table." /]
[@ccode.Indent 1 /] */
  [#local vmtctype  = "const struct " + ifname?lower_case + "_vmt$I*$N"]
${ccode.MakeVariableDeclaration(ccode.indentation "vmt" vmtctype)}
};
/** @} */
[/#macro]

[#--
  -- Generates implementation functions for constructor, destructor,
  -- overidden virtual methods and virtual methods.
  --]
[#macro GenerateClassImplementations node=[] modifiers=[]]
  [#local class = node]
  [#local classname         = GetNodeName(class)
          classnamespace    = GetNodeNamespace(class)
          classctype        = GetClassCType(class)
          classdescr        = GetNodeDescription(class)
          ancestorname      = GetNodeAncestorName(class, "")]
  [#assign generated = true]
/**
[@doxygen.EmitTagVerbatim "" "name" "Methods implementations of " + classctype /]
 * @{
 */
  [#-- Constructor.--]
/**
[@doxygen.EmitBrief "" "Implementation of object creation." /]
[@doxygen.EmitNote  "" "This function is meant to be used by derived classes." /]
 *
[@doxygen.EmitParam name="ip" dir="out"
                    text="Pointer to a @p " + classctype + " instance to be initialized." /]
[@doxygen.EmitParam name="vmt" dir="in"
                    text="VMT pointer for the new object." /]
[@doxygen.EmitParamFromNode node = class.methods.objinit[0] /]
[@doxygen.EmitReturn text="A new reference to the object." /]
 */
[@ccode.GeneratePrototypeFromNode indent    = ""
                                  name      = "__" + classnamespace + "_objinit_impl"
                                  ctype     = "void *"
                                  modifiers = modifiers
                                  params    = ["void *ip", "const void *vmt"]
                                  node      = class.methods.objinit[0] /] {
[@ccode.Indent 1 /]${classctype} *self = (${classctype} *)ip;

  [#if ancestorname?length == 0]
[@ccode.Indent 1 /]/* This is a root class, initializing the VMT pointer here.*/
[@ccode.Indent 1 /]self->vmt = (struct base_object_vmt *)vmt;

  [#else]
    [#local ancestor = GetAncestorClass(class)]
    [#if GetObjinitCallsuper(class.methods.objinit[0]) == "true"]
[@ccode.Indent 1 /]/* Initialization of the ancestors-defined parts.*/
[@ccode.GenerateFunctionCall indent      = ccode.indentation
                             destination = ""
                             name        = "__" + GetNodeNamespace(ancestor) + "_objinit_impl"
                             params      = ["self", "vmt"] /]

    [/#if]
  [/#if]
  [#if class.implements.*?size > 0]
[@GenerateClassInterfacesInit node           = class.implements
                              classctype     = classctype
                              classnamespace = classnamespace /]
  [/#if]
  [#if (class.methods.objinit[0].implementation[0])?? &&
       (class.methods.objinit[0].implementation[0]?trim?length > 0)]
[@ccode.Indent 1 /]/* Initialization code.*/
[@ccode.GenerateIndentedCCode indent=ccode.indentation
                              ccode=class.methods.objinit[0].implementation[0]?string /]
  [#else]
[@ccode.Indent 1 /]/* No initialization code.*/
  [/#if]

[@ccode.Indent 1 /]return self;
}

  [#-- Destructor.--]
/**
[@doxygen.EmitBrief "" "Implementation of object finalization." /]
[@doxygen.EmitNote  "" "This function is meant to be used by derived classes." /]
 *
[@doxygen.EmitParam name="ip" dir="both"
                    text="Pointer to a @p " + classctype + " instance to be disposed." /]
 */
[@ccode.GeneratePrototype indent    = ""
                          name      = "__" + classnamespace + "_dispose_impl"
                          ctype     = "void"
                          modifiers = modifiers
                          params    = ["void *ip"] /] {
[@ccode.Indent 1 /]${classctype} *self = (${classctype} *)ip;

  [#if (class.methods.dispose[0].implementation[0])?? &&
       (class.methods.dispose[0].implementation[0]?trim?length > 0)]
[@ccode.Indent 1 /]/* Finalization code.*/
[@ccode.GenerateIndentedCCode indent=ccode.indentation
                              ccode=class.methods.dispose[0].implementation[0]?string /]
  [#else]
[@ccode.Indent 1 /]/* No finalization code.*/
[@ccode.Indent 1 /](void)self;
  [/#if]
  [#if ancestorname?length > 0]
    [#local ancestor = GetAncestorClass(class)]

[@ccode.Indent 1 /]/* Finalization of the ancestors-defined parts.*/
[@ccode.Indent 1 /]__${GetNodeNamespace(ancestor)}_dispose_impl(self);
  [/#if]
}
  [#-- Scanning for al method overrides defined in the current class then
       checking in which class the virtual method is defined.--]
  [#local ancestors = GetClassAncestorsSequence(class)]
  [#list class.methods.override.method as method]
    [#local shortname      = GetMethodShortName(method)]
    [#local found          = GetVirtualMethodOwnerClass(ancestors, shortname)]
    [#local ancestorclass  = found[0]]
    [#local ancestormethod = found[1]]
    [#local methodname     = GetNodeName(ancestormethod)
            methodsname    = GetMethodShortName(ancestormethod)
            methodretctype = GetMethodCType(ancestormethod)
            methodimpl     = method.implementation[0]!""]

/**
[@doxygen.EmitBrief "" "Override of method @p " + methodname + "()." /]
 *
[@doxygen.EmitParam name="ip" dir="both"
                    text="Pointer to a @p " + classctype + " instance." /]
[@doxygen.EmitParamFromNode node=ancestormethod /]
[@doxygen.EmitReturnFromNode node=ancestormethod /]
 */
[@ccode.GeneratePrototypeFromNode indent    = ""
                                  name      = "__" + classnamespace + "_" + methodsname + "_impl"
                                  modifiers = modifiers
                                  params    = ["void *ip"]
                                  node      = ancestormethod /] {
[@ccode.Indent 1 /]${classctype} *self = (${classctype} *)ip;
[@ccode.GenerateIndentedCCode indent = ccode.indentation
                              ccode  = methodimpl /]
}
  [/#list]
  [#-- Scanning for all virtual methods defined in the current class.--]
  [#list class.methods.virtual.method as method]
    [#local methodname     = GetNodeName(method)
            methodsname    = GetMethodShortName(method)
            methodretctype = GetMethodCType(method)
            methodimpl     = method.implementation[0]!""]
    [#if methodimpl?length > 0]

/**
[@doxygen.EmitBrief "" "Implementation of method @p " + methodname + "()." /]
[@doxygen.EmitNote  "" "This function is meant to be used by derived classes." /]
 *
[@doxygen.EmitParam name="ip" dir="both"
                    text="Pointer to a @p " + classctype + " instance." /]
[@doxygen.EmitParamFromNode node=method /]
[@doxygen.EmitReturnFromNode node=method /]
 */
[@ccode.GeneratePrototypeFromNode indent    = ""
                                  name      = "__" + classnamespace + "_" + methodsname + "_impl"
                                  modifiers = modifiers
                                  params    = ["void *ip"]
                                  node      = method /] {
[@ccode.Indent 1 /]${classctype} *self = (${classctype} *)ip;
[@ccode.GenerateIndentedCCode indent = ccode.indentation
                              ccode  = methodimpl /]
}
    [/#if]
  [/#list]
/** @} */

[/#macro]

[#--
  -- This macro generates regular methods from an XML node.
  --]
[#macro GenerateMethods node=[] classctype="no-ctype" modifiers=[] nodoc=false]
  [#list node.* as this]
    [#if this?node_name == "method"]
      [#local method = this]
      [#local methodname     = GetNodeName(method)
              methodsname    = GetMethodShortName(method)
              methodretctype = GetMethodCType(method)
              methodimpl     = method.implementation[0]!""]
      [#assign generated = true]
      [#if !nodoc]
[@doxygen.EmitFullCommentFromNode indent="" node=method
                                  extraname="ip" extradir="both"
                                  extratext="Pointer to a @p " + classctype + " instance."
                                  memberof="" /]
      [/#if]
      [#if modifiers?seq_contains("static") && modifiers?seq_contains("inline")]
CC_FORCE_INLINE
      [/#if]
[@ccode.GeneratePrototypeFromNode modifiers = modifiers
                                  params    = ["void *ip"]
                                  node      = method /] {
  ${classctype} *self = (${classctype} *)ip;
[@ccode.GenerateIndentedCCode indent=ccode.indentation
                              ccode=methodimpl /]
}
      [#if this?has_next]

      [/#if]
    [#elseif this?node_name == "condition"]
      [#local condition = this]
      [#local condcheck = (condition.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateMethods node       = condition
                  classctype = classctype
                  modifiers  = modifiers
                  nodoc      = nodoc /]
#endif /* ${condcheck} */
      [#if this?has_next]

      [/#if]
    [#elseif this?node_name == "elseif"]
      [#local nodoc = true]
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
  -- This macro generates regular methods from an XML node.
  --]
[#macro GenerateClassRegularMethods node=[] modifiers=[]]
  [#local class = node]
  [#if class.methods.regular.*?size > 0]
    [#local classctype = GetClassCType(class)]
/**
[@doxygen.EmitTagVerbatim "" "name" "Regular methods of " + classctype /]
 * @{
 */
[@GenerateMethods node       = class.methods.regular
                  classctype = classctype
                  modifiers  = [] /]
/** @} */

  [/#if]
[/#macro]

[#--
  -- This macro generates inline methods from an XML node.
  --]
[#macro GenerateClassInlineMethods node=[]]
  [#local class = node]
  [#if class.methods.inline.*?size > 0]
    [#local classctype = GetClassCType(class)]
/**
[@doxygen.EmitTagVerbatim "" "name" "Inline methods of " + classctype /]
 * @{
 */
[@GenerateMethods node       = class.methods.inline
                  classctype = classctype
                  modifiers  = ["static", "inline"] /]
/** @} */

  [/#if]
[/#macro]

[#--
  -- This macro generates virtual methods as inline functions
  -- from an XML node.
  --]
[#macro GenerateVirtualMethods methods=[] ctype="no-ctype" namespace="no-namespace"]
  [#if methods.method?size > 0]
/**
[@doxygen.EmitTagVerbatim "" "name" "Virtual methods of " + ctype /]
 * @{
 */
    [#list methods.method as method]
      [#local methodsname    = GetMethodShortName(method)
              methodretctype = GetMethodCType(method) /]
[@doxygen.EmitFullCommentFromNode indent="" node=method
                                  extraname="ip" extradir="both"
                                  extratext="Pointer to a @p " + ctype + " instance."
                                  memberof="" /]
CC_FORCE_INLINE
[@ccode.GeneratePrototypeFromNode modifiers = ["static", "inline"]
                                  params    = ["void *ip"]
                                  node=method /] {
[@ccode.Indent 1 /]${ctype} *self = (${ctype} *)ip;

      [#local callname   = "self->vmt->" [#-- + namespace + "."--] + methodsname]
      [#local callparams = ccode.MakeCallParamsSequence(["ip"] method)]
      [#if methodretctype == "void"]
[@ccode.GenerateFunctionCall ccode.indentation "" callname callparams /]
      [#else]
[@ccode.GenerateFunctionCall ccode.indentation "return " callname callparams /]
      [/#if]
}
      [#if method?has_next]

      [/#if]
    [/#list]
/** @} */

  [/#if]
[/#macro]

[#--
  -- This macro generates class virtual methods as inline functions
  -- from an XML node.
  --]
[#macro GenerateClassVirtualMethods node=[]]
  [#local class = node]
  [#local classnamespace = GetNodeNamespace(class)
          classctype     = GetClassCType(class) /]
[@GenerateVirtualMethods methods   = class.methods.virtual
                         ctype     = classctype
                         namespace = classnamespace /]
[/#macro]

[#--
  -- Generates extern prototypes for constructor, destructor, overidden
  -- virtual methods, virtual methods and regular methods.
  --]
[#macro GenerateClassPrototypes node=[]]
  [#local class = node]
  [#local classname        = GetNodeName(class)
          classnamespace   = GetNodeNamespace(class)]
  [#-- Constructor.--]
[@ccode.GeneratePrototypeFromNode indent    = ccode.indentation
                                  name      = "__" + classnamespace + "_objinit_impl"
                                  ctype     = "void *"
                                  modifiers = []
                                  params    = ["void *ip", "const void *vmt"]
                                  node      = class.methods.objinit[0] /];
  [#-- Destructor.--]
[@ccode.GeneratePrototype indent    = ccode.indentation
                          name      = "__" + classnamespace + "_dispose_impl"
                          ctype     = "void"
                          modifiers = []
                          params    = ["void *ip"] /];
  [#-- Scanning for al method overrides defined in the current class then
       checking in which class the virtual method is defined.--]
  [#local ancestors = GetClassAncestorsSequence(class)]
  [#list class.methods.override.method as method]
    [#local shortname      = GetMethodShortName(method)]
    [#local found          = GetVirtualMethodOwnerClass(ancestors, shortname)]
    [#local ancestorclass  = found[0]]
    [#local ancestormethod = found[1]]
    [#local methodname     = GetNodeName(ancestormethod)
            methodsname    = GetMethodShortName(ancestormethod)
            methodretctype = GetMethodCType(ancestormethod)
            methodimpl     = method.implementation[0]!""]
[@ccode.GeneratePrototypeFromNode indent    = ccode.indentation
                                  name      = "__" + classnamespace + "_" + methodsname + "_impl"
                                  modifiers = []
                                  params    = ["void *ip"]
                                  node      = ancestormethod /];
  [/#list]
  [#-- Scanning for all virtual methods defined in the current class.--]
  [#list class.methods.virtual.method as method]
    [#local methodname     = GetNodeName(method)
            methodsname    = GetMethodShortName(method)
            methodretctype = GetMethodCType(method)
            methodimpl     = method.implementation[0]!"" /]
    [#if methodimpl?length > 0]
[@ccode.GeneratePrototypeFromNode indent    = ccode.indentation
                                  name      = "__" + classnamespace + "_" + methodsname + "_impl"
                                  modifiers = []
                                  params    = ["void *ip"]
                                  node      = method /];
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates regular method prototypes from an XML node.
  --]
[#macro GenerateClassRegularMethodsPrototypes node=[]]
  [#list node.* as this]
    [#if this?node_name == "method"]
      [#local method = this]
      [#local methodname     = GetNodeName(method)
              methodsname    = GetMethodShortName(method)
              methodretctype = GetMethodCType(method)]
[@ccode.GeneratePrototypeFromNode indent    = ccode.indentation
                                  modifiers = []
                                  params    = ["void *ip"]
                                  node=method /];
    [#elseif this?node_name == "condition"]
      [#local condition = this]
      [#local condcheck = (condition.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateClassRegularMethodsPrototypes node=condition /]
#endif /* ${condcheck} */
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates regular method prototypes from a class XML node.
  --]
[#macro GenerateClassMethodsPrototypes class=[]]
    [#local classctype = GetClassCType(class)]
[@ccode.Indent 1 /]/* Methods of ${classctype}.*/
[@GenerateClassPrototypes node=class /]
[@GenerateClassRegularMethodsPrototypes node=class.methods.regular /]
[/#macro]

[#--
  -- This macro generates virtual methods pointers from a sequence of XML nodes.
  --]
[#macro GenerateVMTPointers methods=[] ipctype="void *"]
  [#list methods.method as method]
    [#local methodsname    = GetMethodShortName(method)
            methodretctype = GetMethodCType(method) /]
    [#local funcptr = ccode.indentation + methodretctype + " (*" + methodsname + ")(" +
                      ccode.MakeProtoParamsSequence([ipctype + "ip"], method)?join(", ") +
                      ");" /]
${funcptr}
  [/#list]
[/#macro]

[#--
  -- This macro generates a class VMT structure from an XML node.
  --]
[#macro GenerateClassVMT node=[] modifiers=[]]
  [#local classname      = GetNodeName(node)
          classnamespace = GetNodeNamespace(node)
          classtype      = GetClassType(node)
          classctype     = GetClassCType(node)
          classdescr     = GetNodeDescription(node) /]
  [#if classtype == "regular"]
    [#assign generated = true]
/**
[@doxygen.EmitBrief "" "VMT structure of " + classdescr + " class." /]
[@doxygen.EmitNote "" "It is public because accessed by the inlined constructor." /]
 */
  [#if modifiers?size > 0]
${modifiers?join(" ") + " "}[#rt]
  [/#if]
const struct ${classname}_vmt __${classname}_vmt = {
  [#local classes = GetClassAncestorsSequence(node) + [node]]
  [#local methods = GetClassVirtualMethodsSequence(node)]
  [#list methods as method]
    [#local methodimpl = GetMethodNearestImplementation(classes, method)]
    [#local s = (ccode.indentation + "." +
                 GetMethodShortName(method))?right_pad(ccode.initializers_align) +
                 "= " + methodimpl]
    [#if method?has_next]
${s},
    [#else]
${s}
    [/#if]
  [/#list]
};

  [/#if]
[/#macro]

[#--
  -- This macro generates class method implementations from an XML node.
  --]
[#macro GenerateClassInterfacesInit node=[] classctype="no-ctype" classnamespace="no-namespace"]
  [#list node.* as this]
    [#if this?node_name == "if"]
      [#local ifname      = GetNodeName(this)]
      [#local if          = GetInterfaceByName(ifname)]
      [#local ifnamespace = GetNodeNamespace(if)
              ifctype     = GetInterfaceCType(if) /]
[@ccode.Indent 1 /]/* Initialization of interface ${ifctype}.*/
[@ccode.Indent 1 /]{
[@ccode.Indent 2 /]static const struct ${ifname}_vmt ${classnamespace}_${ifnamespace}_vmt = {
      [#local s = (ccode.indentation + ccode.indentation + ccode.indentation +
                   ".instance_offset")?right_pad(ccode.initializers_align) + "= " +
                   "offsetof(" + classctype + ", " + ifnamespace + ")"]
${s},
      [#local methods = GetInterfaceMethodsSequence(if)]
      [#list methods as method]
        [#local methodshortname = GetMethodShortName(method)]
        [#local s = (ccode.indentation + ccode.indentation + ccode.indentation + "." +
                       methodshortname)?right_pad(ccode.initializers_align) + "= "]
        [#local ifmethod = this["method[@shortname='" + methodshortname + "']"]]
        [#if ifmethod[0]??]
          [#local s = s + "__" + classnamespace + "_" + ifnamespace + "_" +
                      methodshortname + "_impl"]
        [#else]
          [#local s = s + "NULL /* Missing implementation.*/"]
        [/#if]
        [#if method?has_next]
${s},
        [#else]
${s}
        [/#if]
      [/#list]
[@ccode.Indent 2 /]};
[@ccode.Indent 2 /]oopIfObjectInit(&self->${ifnamespace}, &${classnamespace}_${ifnamespace}_vmt);
[@ccode.Indent 1 /]}
      [#if node?node_name != "condition"]

      [/#if]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateClassInterfacesInit node=this
                              classctype=classctype
                              classnamespace=classnamespace /]
#endif /* ${condcheck} */

    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates class interface fields from an XML node.
  --]
[#macro GenerateClassInterfaceFields node=[]]
  [#list node.* as this]
    [#if this?node_name == "if"]
      [#local ifname      = GetNodeName(this)
              ifctype     = GetInterfaceCType(this) /]
      [#local if = GetInterfaceByName(ifname)]
[@ccode.Indent 1 /]/**
[@doxygen.EmitBrief ccode.indentation "Implemented interface @p " + ifctype + "." /]
[@ccode.Indent 1 /] */
[@ccode.GenerateVariableDeclaration indent = ccode.indentation
                                    name   = GetNodeNamespace(if)
                                    ctype  = ifctype /]

    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
[@GenerateClassInterfaceFields this /]
#endif /* ${condcheck} */
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a Doxygen list of implemented interfaces from an XML node.
  --]
[#macro GenerateClassImplementsTags node=[]]
  [#list node.* as this]
    [#if this?node_name == "if"]
    [#local refname      = GetNodeName(this)
            refnamespace = GetNodeNamespace(this)
            refctype     = GetInterfaceCType(this) /]
[@doxygen.EmitTagVerbatim indent="" tag="implements" text=refctype /]
    [#elseif this?node_name == "condition"]
[@GenerateClassImplementsTags this /]
    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates private constructor and destructor from an XML node.
  --]
[#macro GenerateClassPrivateConstructor node=[]]
  [#local classname         = GetNodeName(node)
          classnamespace    = GetNodeNamespace(node)
          classtype         = GetClassType(node)
          classctype        = GetClassCType(node)
          classdescr        = GetNodeDescription(node) /]
  [#if classtype == "regular"]
    [#assign generated = true]
/**
[@doxygen.EmitTagVerbatim "" "name" "Default constructor of " + classctype /]
 * @{
 */
/**
[@doxygen.EmitBrief "" "Default initialization function of @p " + classctype + "." /]
 *
[@doxygen.EmitParam name = "self"
                    dir  = "out"
                    text = "Pointer to a @p " + classctype + " instance to be initialized." /]
[@doxygen.EmitParamFromNode node = node.methods.objinit[0] /]
[@doxygen.EmitReturn text = "Pointer to the initialized object." /]
 *
 * @objinit
 */
[@ccode.GeneratePrototypeFromNode indent    = ""
                                  name      = classnamespace + "ObjectInit"
                                  ctype     = classctype + " *"
                                  modifiers = ["static"]
                                  params    = [classctype + " *self"]
                                  node      = node.methods.objinit[0] /] {

    [#local params = ccode.MakeCallParamsSequence(["self", "&__" + classname + "_vmt"], node.methods.objinit[0])]
[@ccode.GenerateFunctionCall indent      = ccode.indentation
                             destination = "return"
                             name        = "__" + classnamespace + "_objinit_impl"
                             params      = params /]
}
/** @} */

  [/#if]
[/#macro]

[#--
  -- This macro generates public constructor and destructor from an XML node.
  --]
[#macro GenerateClassPublicConstructor node=[]]
  [#local classname         = GetNodeName(node)
          classnamespace    = GetNodeNamespace(node)
          classtype         = GetClassType(node)
          classctype        = GetClassCType(node)
          classdescr        = GetNodeDescription(node) /]
  [#if classtype == "regular"]
    [#assign generated = true]
/**
[@doxygen.EmitTagVerbatim "" "name" "Default constructor of " + classctype /]
 * @{
 */
/**
[@doxygen.EmitBrief "" "Default initialization function of @p " + classctype + "." /]
 *
[@doxygen.EmitParam name = "self"
                    dir  = "out"
                    text = "Pointer to a @p " + classctype + " instance to be initialized." /]
[@doxygen.EmitParamFromNode node = node.methods.objinit[0] /]
[@doxygen.EmitReturn text = "Pointer to the initialized object." /]
 *
 * @objinit
 */
CC_FORCE_INLINE
[@ccode.GeneratePrototypeFromNode indent    = ""
                                  name      = classnamespace + "ObjectInit"
                                  ctype     = classctype + " *"
                                  modifiers = ["static", "inline"]
                                  params    = [classctype + " *self"]
                                  node      = node.methods.objinit[0] /] {
[@ccode.Indent 1 /]extern const struct ${classname}_vmt __${classname}_vmt;

    [#local params = ccode.MakeCallParamsSequence(["self", "&__" + classname + "_vmt"], node.methods.objinit[0])]
[@ccode.GenerateFunctionCall indent      = ccode.indentation
                             destination = "return"
                             name        = "__" + classnamespace + "_objinit_impl"
                             params      = params /]
}
/** @} */

  [/#if]
[/#macro]

[#--
  -- This macro generates interface virtual methods as inline functions
  -- from an XML node.
  --]
[#macro GenerateInterfaceVirtualMethods if=[]]
  [#local ifnamespace = GetNodeNamespace(if)
          ifctype     = GetInterfaceCType(if) /]
[@GenerateVirtualMethods methods   = if.methods
                         ctype     = ifctype
                         namespace = ifnamespace /]
[/#macro]

[#--
  -- This macro generates a class wrapper from an XML node.
  --]
[#macro GenerateInterfaceWrapper node=[]]
  [#local if = node]
  [#local ifname            = GetNodeName(if)
          ifnamespace       = GetNodeNamespace(if)
          ifctype           = GetInterfaceCType(if)
          ifdescr           = GetNodeDescription(if)
          ancestorname      = GetNodeAncestorName(if, "")
          ancestorctype     = GetInterfaceAncestorCType(if)]
/**
 * @interface   ${ifctype}
  [#if ancestorctype?length > 0]
 * @extends     ${ancestorctype}
  [/#if]
 *
[@doxygen.EmitBriefFromNode node=if /]
[@doxygen.EmitDetailsFromNode node=if /]
[@doxygen.EmitPreFromNode node=if /]
[@doxygen.EmitPostFromNode node=if /]
[@doxygen.EmitNoteFromNode node=if /]
[@doxygen.EmitNote text="The interface namespace is <tt>" + ifnamespace + "</tt>, access to " +
                        "an implemented interface is done using: " +
                        "<tt>&<objp>-><classnamespace>." + ifnamespace + "</tt>"/]
 *
[@doxygen.EmitTagVerbatim indent="" tag="name" text="Interface @p " + ifctype + " structures"/]
 * @{
 */

/**
[@doxygen.EmitBrief "" "Type of a " + ifdescr + " interface." /]
 */
typedef struct ${ifname} ${ifctype};

  [#local methodsstruct = ifname + "_methods"]
  [#if node.methods.method?size > 0]
/**
[@doxygen.EmitBrief "" "Interface @p " + ifctype + " methods as a structure." /]
 */
struct ${methodsstruct} {
[@GenerateVMTPointers methods=if.methods /]
};

  [/#if]
/**
[@doxygen.EmitBrief "" "Interface @p " + ifctype + " methods." /]
 */
  [#local methodsdefine = "__" + ifname + "_methods"]
${("#define " + methodsdefine)?right_pad(ccode.backslash_align) + "\\"}
  [#if ancestorname?length > 0]
${(ccode.indentation + "__" + ancestorname +
                       "_methods")?right_pad(ccode.backslash_align) + "\\"}
  [/#if]
  [#if if.methods.method?size > 0]
[@ccode.GenerateVariableDeclaration indent=ccode.indentation
                                    name=ifnamespace
                                    ctype="struct " + methodsstruct /]


  [#else]
${(ccode.indentation + "/* Memory offset between this interface structure and begin of")}
${(ccode.indentation + "   the implementing class structure.*/")?right_pad(ccode.backslash_align) + "\\"}
${(ccode.indentation + "size_t instance_offset;")}

  [/#if]
/**
[@doxygen.EmitBrief "" "Interface @p " + ifctype + " VMT initializer." /]
 *
[@doxygen.EmitParam "" "ns" "" "Namespace of the implementing class." /]
[@doxygen.EmitParam "" "off" "in" "VMT offset to be stored." /]
 */
  [#local vmtinitsdefine = "__" + ifname + "_vmt_init(ns, off)"]
#define ${vmtinitsdefine?right_pad(68) + "\\"}
  [#if ancestorname?length > 0]
    [#local s = "  __" + ancestorname + "_vmt_init(ns, off)"]
    [#if node.methods?size > 0]
      [#local s = (s + " ")?right_pad(76) + "\\"]
    [/#if]
${s}
[@GenerateVMTInitializers methods=node.methods namespace=ifnamespace /]
  [#else]
${(ccode.indentation + ".instance_offset")?right_pad(ccode.initializers_align) + "= off,"}
  [/#if]

/**
[@doxygen.EmitBrief "" "Interface @p " + ifctype + " virtual methods table." /]
 */
struct ${ifname?lower_case}_vmt {
  ${methodsdefine}
};

/**
[@doxygen.EmitBrief "" "Structure representing a " + ifdescr + "." /]
 */
struct ${ifname?lower_case} {
[@ccode.Indent 1 /]/**
[@doxygen.EmitBrief ccode.indentation "Virtual Methods Table." /]
[@ccode.Indent 1 /] */
  [#local vmtctype  = "const struct " + ifname?lower_case + "_vmt$I*$N"]
${ccode.MakeVariableDeclaration(ccode.indentation "vmt" vmtctype)}
};
/** @} */
[/#macro]

[#macro GenerateInterfacesImplementations node=[]
                                          classnamespace="no-namespace"
                                          classctype="no-ctype"]
  [#list node.* as this]
    [#if this?node_name == "if"]
      [#local if          = GetInterfaceByName(GetNodeName(this))
              ifctype     = GetInterfaceCType(if)
              ifnamespace = GetNodeNamespace(if)]
      [#list this.method as methodref]
        [#local methodsname  = GetMethodShortName(methodref)]
        [#local ifmethods    = GetInterfaceMethodsSequence(if)]
        [#local foundmethods = ifmethods?filter(m -> GetMethodShortName(m) == methodsname)]
        [#if !foundmethods[0]??]
          [#stop ">>>> Method '" + methodsname + "' not found."]
        [/#if]
        [#local method     = foundmethods[0]]
        [#local methodname = GetNodeName(method)]
/**
[@doxygen.EmitBrief "" "Implementation of interface method @p " + methodname + "()." /]
 *
[@doxygen.EmitParam name="ip" dir="both"
                    text="Pointer to the @p " + ifctype + " class interface." /]
[@doxygen.EmitParamFromNode node=method /]
[@doxygen.EmitReturnFromNode node=method /]
 */
[@ccode.GeneratePrototypeFromNode indent    = ""
                                  name      = "__" + classnamespace + "_" +
                                              ifnamespace + "_" + methodsname + "_impl"
                                  modifiers = ["static"]
                                  params    = ["void *ip"]
                                  node      = method /] {
[@ccode.Indent 1 /]${classctype} *self = oopIfGetOwner(${classctype}, ip);
[@ccode.GenerateIndentedCCode indent = ccode.indentation
                              ccode  = methodref.implementation[0] /]
}
        [#if methodref?has_next]

        [/#if]
      [/#list]
    [#elseif this?node_name == "condition"]
      [#local condcheck = (this.@check[0]!"1")?trim]
#if (${condcheck}) || defined (__DOXYGEN__)
      [@GenerateInterfacesImplementations node           = this
                                          classnamespace = classnamespace
                                          classctype     = classctype /]
#endif /* ${condcheck} */
    [/#if]
    [#if this?has_next]

    [#elseif (this?parent?node_name != "condition") &&
             (this?parent?node_name != "implements")]

    [/#if]
  [/#list]
[/#macro]

[#--
  -- This macro generates a class interfaces implementatios from an XML node.
  --]
[#macro GenerateClassInterfacesImplementations node=[] private=false]
  [#local classnamespace = GetNodeNamespace(node)
          classctype     = GetClassCType(node)]
  [#if node.implements.*?size > 0]
/**
[@doxygen.EmitTagVerbatim "" "name" "Interfaces implementation of " + classctype /]
 * @{
 */
  [@GenerateInterfacesImplementations node           = node.implements
                                      classnamespace = classnamespace
                                      classctype     = classctype /]
/** @} */

  [/#if]
[/#macro]

[#--
  -- This macro generates the class code from an XML node.
  --]
[#macro GenerateClassCode class=[] modifiers=[]]
[@GenerateClassInterfacesImplementations node=class /]
[@GenerateClassImplementations node=class modifiers=modifiers /]
[@GenerateClassVMT node=class modifiers=modifiers /]
[@GenerateClassRegularMethods node=class modifiers=modifiers /]
[/#macro]
