/*
        wadl.h

        WADL schema

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

//gsoap wadl schema documentation:      WADL binding schema
//gsoap wadl schema namespace:          http://wadl.dev.java.net/2009/02
//gsoap wadl schema elementForm:        qualified
//gsoap wadl schema attributeForm:      unqualified

#import "imports.h"

class wadl__resource; // forward declaration
class wadl__resource_USCOREtype; // forward declaration

typedef xsd__string wadl__statusCodeList; // list of unsigned ints

enum wadl__HTTPMethods { GET, POST, PUT, HEAD, DELETE_ };

enum wadl__ParamStyle { plain, query, matrix, header, template_ };

class wadl__doc
{ public:
       @xsd__string                      title;
       @xsd__string                      xml__lang;
       _XML                              __mixed;
};

class wadl__option
{ public:
       @xsd__string                      value;
       @xsd__string                      mediaType;
        std::vector<wadl__doc>           doc;
};

class wadl__link
{ public:
       @xsd__anyURI                      resource_USCOREtype;
       @xsd__token                       rel;
       @xsd__token                       rev;
        std::vector<wadl__doc>           doc;
  private:
        wadl__resource_USCOREtype*       linkRef;
  public:
                                         wadl__link();
        int                              traverse(wsdl__definitions&);
        void                             linkPtr(wadl__resource_USCOREtype*);
        const wadl__resource_USCOREtype* linkPtr() const;
};

class wadl__param
{ public:
       @xsd__anyURI                      href;
       @xsd__NMTOKEN                     name;
       @enum wadl__ParamStyle*           style;
       @xsd__ID                          id;
       @xsd__QName                       type;
       @xsd__string                      default_;
       @bool                             required = false;
       @bool                             repeating = false;
       @xsd__string                      fixed;
       @xsd__string                      path;
        std::vector<wadl__doc>           doc;
        std::vector<wadl__option>        option;
        wadl__link*                      link;
  private:
        wadl__param*                     paramRef;
        xs__simpleType*                  simpleTypeRef;
        xs__complexType*                 complexTypeRef;
  public:
                                         wadl__param();
        int                              traverse(wsdl__definitions&);
        void                             paramPtr(wadl__param*);
        const wadl__param*               paramPtr() const;
        void                             simpleTypePtr(xs__simpleType*);
        xs__simpleType*                  simpleTypePtr() const;
        void                             complexTypePtr(xs__complexType*);
        xs__complexType*                 complexTypePtr() const;
        void                             mark();
};

class wadl__include
{ public:
       @xsd__anyURI                      href;
        std::vector<wadl__doc>           doc;
  public:
        int                              preprocess(wsdl__definitions&);
};

class wadl__grammars
{ public:
        std::vector<wadl__doc>           doc;
        std::vector<wadl__include>       include;
  public:
        int                              preprocess(wsdl__definitions&);
};

class wadl__representation
{ public:
       @xsd__ID                          id;
       @xsd__QName                       element;
       @xsd__string                      mediaType;
       @xsd__anyURI                      href;
       @xsd__anyURI                      profile;
        std::vector<wadl__doc>           doc;
        std::vector<wadl__param>         param;
  private:
        wadl__representation*            representationRef;
        xs__element*                     elementRef;
  public:
                                         wadl__representation();
        int                              traverse(wsdl__definitions&);
        void                             representationPtr(wadl__representation*);
        const wadl__representation*      representationPtr() const;
        void                             elementPtr(xs__element*);
        xs__element*                     elementPtr() const;
        void                             mark();
};

class wadl__request
{ public:
        std::vector<wadl__doc>           doc;
        std::vector<wadl__param>         param;
        std::vector<wadl__representation> representation;
  public:
        int                              traverse(wsdl__definitions&);
        void                             mark();
};

class wadl__response : public wadl__request
{ public:
       @wadl__statusCodeList             status;
};

class wadl__method
{ public:
       @xsd__ID                          id;
       @enum wadl__HTTPMethods           name;
       @xsd__anyURI                      href;
        std::vector<wadl__doc>           doc;
        wadl__request*                   request;
        std::vector<wadl__response>      response;
  private:
        wadl__method*                    methodRef;
  public:
                                         wadl__method();
        int                              traverse(wsdl__definitions&);
        void                             methodPtr(wadl__method*);
        const wadl__method*              methodPtr() const;
        void                             mark();
};

class __wadl__method_resource_choice
{ public:
        wadl__method*                    method;
        wadl__resource*                  resource;
  public:
        int                              traverse(wsdl__definitions&);
};

class wadl__resource_USCOREtype
{ public:
       @xsd__ID                          id;
        std::vector<wadl__doc>           doc;
        std::vector<wadl__param>         param;
        std::vector<__wadl__method_resource_choice> __choice;
  public:
        int                              traverse(wsdl__definitions&);
};

class wadl__resource : public wadl__resource_USCOREtype
{ public:
       @xsd__anyURI                      type;
       @xsd__string                      queryType = "application/x-www-form-urlencoded";
       @xsd__string                      path;
  private:
        std::vector<wadl__resource_USCOREtype*> typeRefs;
  public:
        int                              traverse(wsdl__definitions&);
        void                             typePtr(wadl__resource_USCOREtype*);
        const std::vector<wadl__resource_USCOREtype*>& typePtrs() const;
};

class wadl__resources
{ public:
       @xsd__anyURI                      base;
        std::vector<wadl__doc>           doc;
        std::vector<wadl__resource>      resource;
  public:
        int                              traverse(wsdl__definitions&);
};

class wadl__application
{ public:
        std::vector<wadl__doc>           doc;
        wadl__grammars*                  grammars;
        std::vector<wadl__resources>     resources;
        std::vector<wadl__resource_USCOREtype> resource_USCOREtype;
        std::vector<wadl__method>        method;
        std::vector<wadl__representation> representation;
        std::vector<wadl__param>         param;
        struct soap*                     soap;
  public:
        int                              preprocess(wsdl__definitions&);
        int                              traverse(wsdl__definitions&);
        void                             mark();
};
