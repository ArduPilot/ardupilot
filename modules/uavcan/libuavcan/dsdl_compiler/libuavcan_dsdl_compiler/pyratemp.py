#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Small, simple and powerful template-engine for Python.

A template-engine for Python, which is very simple, easy to use, small,
fast, powerful, modular, extensible, well documented and pythonic.

See documentation for a list of features, template-syntax etc.

:Version:   0.3.0
:Requires:  Python >=2.6 / 3.x

:Usage:
    see class ``Template`` and examples below.

:Example:

    Note that the examples are in Python 2; they also work in
    Python 3 if you replace u"..." by "...", unicode() by str()
    and partly "..." by b"...".

    quickstart::
        >>> t = Template("hello @!name!@")
        >>> print(t(name="marvin"))
        hello marvin

    quickstart with a template-file::
        # >>> t = Template(filename="mytemplate.tmpl")
        # >>> print(t(name="marvin"))
        # hello marvin

    generic usage::
        >>> t = Template(u"output is in Unicode \\xe4\\xf6\\xfc\\u20ac")
        >>> t                                           #doctest: +ELLIPSIS
        <...Template instance at 0x...>
        >>> t()
        u'output is in Unicode \\xe4\\xf6\\xfc\\u20ac'
        >>> unicode(t)
        u'output is in Unicode \\xe4\\xf6\\xfc\\u20ac'

    with data::
        >>> t = Template("hello @!name!@", data={"name":"world"})
        >>> t()
        u'hello world'
        >>> t(name="worlds")
        u'hello worlds'

        # >>> t(note="data must be Unicode or ASCII", name=u"\\xe4")
        # u'hello \\xe4'

    escaping::
        >>> t = Template("hello escaped: @!name!@, unescaped: $!name!$")
        >>> t(name='''<>&'"''')
        u'hello escaped: &lt;&gt;&amp;&#39;&quot;, unescaped: <>&\\'"'

    result-encoding::
        # encode the unicode-object to your encoding with encode()
        >>> t = Template(u"hello \\xe4\\xf6\\xfc\\u20ac")
        >>> result = t()
        >>> result
        u'hello \\xe4\\xf6\\xfc\\u20ac'
        >>> result.encode("utf-8")
        'hello \\xc3\\xa4\\xc3\\xb6\\xc3\\xbc\\xe2\\x82\\xac'
        >>> result.encode("ascii")
        Traceback (most recent call last):
          ...
        UnicodeEncodeError: 'ascii' codec can't encode characters in position 6-9: ordinal not in range(128)
        >>> result.encode("ascii", 'xmlcharrefreplace')
        'hello &#228;&#246;&#252;&#8364;'

    Python-expressions::
        >>> Template('formatted: @! "%8.5f" % value !@')(value=3.141592653)
        u'formatted:  3.14159'
        >>> Template("hello --@!name.upper().center(20)!@--")(name="world")
        u'hello --       WORLD        --'
        >>> Template("calculate @!var*5+7!@")(var=7)
        u'calculate 42'

    blocks (if/for/macros/...)::
        >>> t = Template("<!--(if foo == 1)-->bar<!--(elif foo == 2)-->baz<!--(else)-->unknown(@!foo!@)<!--(end)-->")
        >>> t(foo=2)
        u'baz'
        >>> t(foo=5)
        u'unknown(5)'

        >>> t = Template("<!--(for i in mylist)-->@!i!@ <!--(else)-->(empty)<!--(end)-->")
        >>> t(mylist=[])
        u'(empty)'
        >>> t(mylist=[1,2,3])
        u'1 2 3 '

        >>> t = Template("<!--(for i,elem in enumerate(mylist))--> - @!i!@: @!elem!@<!--(end)-->")
        >>> t(mylist=["a","b","c"])
        u' - 0: a - 1: b - 2: c'

        >>> t = Template('<!--(macro greetings)-->hello <strong>@!name!@</strong><!--(end)-->  @!greetings(name=user)!@')
        >>> t(user="monty")
        u'  hello <strong>monty</strong>'

    exists::
        >>> t = Template('<!--(if exists("foo"))-->YES<!--(else)-->NO<!--(end)-->')
        >>> t()
        u'NO'
        >>> t(foo=1)
        u'YES'
        >>> t(foo=None)       # note this difference to 'default()'
        u'YES'

    default-values::
        # non-existing variables raise an error
        >>> Template('hi @!optional!@')()
        Traceback (most recent call last):
          ...
        TemplateRenderError: Cannot eval expression 'optional'. (NameError: name 'optional' is not defined)

        >>> t = Template('hi @!default("optional","anyone")!@')
        >>> t()
        u'hi anyone'
        >>> t(optional=None)
        u'hi anyone'
        >>> t(optional="there")
        u'hi there'

        # the 1st parameter can be any eval-expression
        >>> t = Template('@!default("5*var1+var2","missing variable")!@')
        >>> t(var1=10)
        u'missing variable'
        >>> t(var1=10, var2=2)
        u'52'

        # also in blocks
        >>> t = Template('<!--(if default("opt1+opt2",0) > 0)-->yes<!--(else)-->no<!--(end)-->')
        >>> t()
        u'no'
        >>> t(opt1=23, opt2=42)
        u'yes'

        >>> t = Template('<!--(for i in default("optional_list",[]))-->@!i!@<!--(end)-->')
        >>> t()
        u''
        >>> t(optional_list=[1,2,3])
        u'123'


        # but make sure to put the expression in quotation marks, otherwise:
        >>> Template('@!default(optional,"fallback")!@')()
        Traceback (most recent call last):
          ...
        TemplateRenderError: Cannot eval expression 'default(optional,"fallback")'. (NameError: name 'optional' is not defined)

    setvar::
        >>> t = Template('$!setvar("i", "i+1")!$@!i!@')
        >>> t(i=6)
        u'7'

        >>> t = Template('''<!--(if isinstance(s, (list,tuple)))-->$!setvar("s", '"\\\\\\\\n".join(s)')!$<!--(end)-->@!s!@''')
        >>> t(isinstance=isinstance, s="123")
        u'123'
        >>> t(isinstance=isinstance, s=["123", "456"])
        u'123\\n456'

:Author:    Roland Koebler (rk at simple-is-better dot org)
:Copyright: Roland Koebler
:License:   MIT/X11-like, see __license__

:RCS:       $Id: pyratemp.py,v 1.12 2013/04/02 20:26:06 rk Exp $
"""
from __future__ import unicode_literals

__version__ = "0.3.0"
__author__   = "Roland Koebler <rk at simple-is-better dot org>"
__license__  = """Copyright (c) Roland Koebler, 2007-2013

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE."""

#=========================================

import os, re, sys
if sys.version_info[0] >= 3:
    import builtins
    unicode = str
    long = int
else:
    import __builtin__ as builtins
    from codecs import open

#=========================================
# some useful functions

#----------------------
# string-position: i <-> row,col

def srow(string, i):
    """Get line numer of ``string[i]`` in `string`.

    :Returns: row, starting at 1
    :Note:    This works for text-strings with ``\\n`` or ``\\r\\n``.
    """
    return string.count('\n', 0, max(0, i)) + 1

def scol(string, i):
    """Get column number of ``string[i]`` in `string`.

    :Returns: column, starting at 1 (but may be <1 if i<0)
    :Note:    This works for text-strings with ``\\n`` or ``\\r\\n``.
    """
    return i - string.rfind('\n', 0, max(0, i))

def sindex(string, row, col):
    """Get index of the character at `row`/`col` in `string`.

    :Parameters:
        - `row`: row number, starting at 1.
        - `col`: column number, starting at 1.
    :Returns:    ``i``, starting at 0 (but may be <1 if row/col<0)
    :Note:       This works for text-strings with '\\n' or '\\r\\n'.
    """
    n = 0
    for _ in range(row-1):
        n = string.find('\n', n) + 1
    return n+col-1

#----------------------

def dictkeyclean(d):
    """Convert all keys of the dict `d` to strings.
    """
    new_d = {}
    for k, v in d.items():
        new_d[str(k)] = v
    return new_d

#----------------------

def dummy(*_, **__):
    """Dummy function, doing nothing.
    """
    pass

def dummy_raise(exception, value):
    """Create an exception-raising dummy function.

    :Returns: dummy function, raising ``exception(value)``
    """
    def mydummy(*_, **__):
        raise exception(value)
    return mydummy

#=========================================
# escaping

(NONE, HTML, LATEX, MAIL_HEADER) = range(0, 4)
ESCAPE_SUPPORTED = {"NONE":None, "HTML":HTML, "LATEX":LATEX, "MAIL_HEADER":MAIL_HEADER}

def escape(s, format=HTML):
    """Replace special characters by their escape sequence.

    :Parameters:
        - `s`: unicode-string to escape
        - `format`:

          - `NONE`:  nothing is replaced
          - `HTML`:  replace &<>'" by &...;
          - `LATEX`: replace \#$%&_{}~^
          - `MAIL_HEADER`: escape non-ASCII mail-header-contents
    :Returns:
        the escaped string in unicode
    :Exceptions:
        - `ValueError`: if `format` is invalid.

    :Uses:
        MAIL_HEADER uses module email
    """
    #Note: If you have to make sure that every character gets replaced
    #      only once (and if you cannot achieve this with the following code),
    #      use something like "".join([replacedict.get(c,c) for c in s])
    #      which is about 2-3 times slower (but maybe needs less memory).
    #Note: This is one of the most time-consuming parts of the template.
    if format is None or format == NONE:
        pass
    elif format == HTML:
        s = s.replace("&", "&amp;") # must be done first!
        s = s.replace("<", "&lt;")
        s = s.replace(">", "&gt;")
        s = s.replace('"', "&quot;")
        s = s.replace("'", "&#39;")
    elif format == LATEX:
        s = s.replace("\\", "\\x")    #must be done first!
        s = s.replace("#",  "\\#")
        s = s.replace("$",  "\\$")
        s = s.replace("%",  "\\%")
        s = s.replace("&",  "\\&")
        s = s.replace("_",  "\\_")
        s = s.replace("{",  "\\{")
        s = s.replace("}",  "\\}")
        s = s.replace("\\x","\\textbackslash{}")
        s = s.replace("~",  "\\textasciitilde{}")
        s = s.replace("^",  "\\textasciicircum{}")
    elif format == MAIL_HEADER:
        import email.header
        try:
            s.encode("ascii")
            return s
        except UnicodeEncodeError:
            return email.header.make_header([(s, "utf-8")]).encode()
    else:
        raise ValueError('Invalid format (only None, HTML, LATEX and MAIL_HEADER are supported).')
    return s

#=========================================

#-----------------------------------------
# Exceptions

class TemplateException(Exception):
    """Base class for template-exceptions."""
    pass

class TemplateParseError(TemplateException):
    """Template parsing failed."""
    def __init__(self, err, errpos):
        """
        :Parameters:
            - `err`:    error-message or exception to wrap
            - `errpos`: ``(filename,row,col)`` where the error occured.
        """
        self.err = err
        self.filename, self.row, self.col = errpos
        TemplateException.__init__(self)
    def __str__(self):
        if not self.filename:
            return "line %d, col %d: %s" % (self.row, self.col, str(self.err))
        else:
            return "file %s, line %d, col %d: %s" % (self.filename, self.row, self.col, str(self.err))

class TemplateSyntaxError(TemplateParseError, SyntaxError):
    """Template syntax-error."""
    pass

class TemplateIncludeError(TemplateParseError):
    """Template 'include' failed."""
    pass

class TemplateRenderError(TemplateException):
    """Template rendering failed."""
    pass

#-----------------------------------------
# Loader

class LoaderString:
    """Load template from a string/unicode.

    Note that 'include' is not possible in such templates.
    """
    def __init__(self, encoding='utf-8'):
        self.encoding = encoding

    def load(self, s):
        """Return template-string as unicode.
        """
        if isinstance(s, unicode):
            u = s
        else:
            u = s.decode(self.encoding)
        return u

class LoaderFile:
    """Load template from a file.

    When loading a template from a file, it's possible to including other
    templates (by using 'include' in the template). But for simplicity
    and security, all included templates have to be in the same directory!
    (see ``allowed_path``)
    """
    def __init__(self, allowed_path=None, encoding='utf-8'):
        """Init the loader.

        :Parameters:
            - `allowed_path`: path of the template-files
            - `encoding`: encoding of the template-files
        :Exceptions:
            - `ValueError`: if `allowed_path` is not a directory
        """
        if allowed_path and not os.path.isdir(allowed_path):
            raise ValueError("'allowed_path' has to be a directory.")
        self.path     = allowed_path
        self.encoding = encoding

    def load(self, filename):
        """Load a template from a file.

        Check if filename is allowed and return its contens in unicode.

        :Parameters:
            - `filename`: filename of the template without path
        :Returns:
            the contents of the template-file in unicode
        :Exceptions:
            - `ValueError`: if `filename` contains a path
        """
        if filename != os.path.basename(filename):
            raise ValueError("No path allowed in filename. (%s)" %(filename))
        filename = os.path.join(self.path, filename)

        f = open(filename, 'r', encoding=self.encoding)
        u = f.read()
        f.close()

        return u

#-----------------------------------------
# Parser

class Parser(object):
    """Parse a template into a parse-tree.

    Includes a syntax-check, an optional expression-check and verbose
    error-messages.

    See documentation for a description of the parse-tree.
    """
    # template-syntax
    _comment_start = "#!"
    _comment_end   = "!#"
    _sub_start     = "$!"
    _sub_end       = "!$"
    _subesc_start  = "@!"
    _subesc_end    = "!@"
    _block_start   = "<!--("
    _block_end     = ")-->"

    # build regexps
    # comment
    #   single-line, until end-tag or end-of-line.
    _strComment = r"""%s(?P<content>.*?)(?P<end>%s|\n|$)""" \
                    % (re.escape(_comment_start), re.escape(_comment_end))
    _reComment = re.compile(_strComment, re.M)

    # escaped or unescaped substitution
    #   single-line ("|$" is needed to be able to generate good error-messges)
    _strSubstitution = r"""
                    (
                    %s\s*(?P<sub>.*?)\s*(?P<end>%s|$)       #substitution
                    |
                    %s\s*(?P<escsub>.*?)\s*(?P<escend>%s|$) #escaped substitution
                    )
                """ % (re.escape(_sub_start),    re.escape(_sub_end),
                       re.escape(_subesc_start), re.escape(_subesc_end))
    _reSubstitution = re.compile(_strSubstitution, re.X|re.M)

    # block
    #   - single-line, no nesting.
    #   or
    #   - multi-line, nested by whitespace indentation:
    #       * start- and end-tag of a block must have exactly the same indentation.
    #       * start- and end-tags of *nested* blocks should have a greater indentation.
    # NOTE: A single-line block must not start at beginning of the line with
    #       the same indentation as the enclosing multi-line blocks!
    #       Note that "       " and "\t" are different, although they may
    #       look the same in an editor!
    _s = re.escape(_block_start)
    _e = re.escape(_block_end)
    _strBlock = r"""
                    ^(?P<mEnd>[ \t]*)%send%s(?P<meIgnored>.*)\r?\n?   # multi-line end  (^   <!--(end)-->IGNORED_TEXT\n)
                    |
                    (?P<sEnd>)%send%s                               # single-line end (<!--(end)-->)
                    |
                    (?P<sSpace>[ \t]*)                              # single-line tag (no nesting)
                    %s(?P<sKeyw>\w+)[ \t]*(?P<sParam>.*?)%s
                    (?P<sContent>.*?)
                    (?=(?:%s.*?%s.*?)??%send%s)                     # (match until end or i.e. <!--(elif/else...)-->)
                    |
                                                                    # multi-line tag, nested by whitespace indentation
                    ^(?P<indent>[ \t]*)                             #   save indentation of start tag
                    %s(?P<mKeyw>\w+)\s*(?P<mParam>.*?)%s(?P<mIgnored>.*)\r?\n
                    (?P<mContent>(?:.*\n)*?)
                    (?=(?P=indent)%s(?:.|\s)*?%s)                   #   match indentation
                """ % (_s, _e,
                       _s, _e,
                       _s, _e, _s, _e, _s, _e,
                       _s, _e, _s, _e)
    _reBlock = re.compile(_strBlock, re.X|re.M)

    # "for"-block parameters: "var(,var)* in ..."
    _strForParam = r"""^(?P<names>\w+(?:\s*,\s*\w+)*)\s+in\s+(?P<iter>.+)$"""
    _reForParam  = re.compile(_strForParam)

    # allowed macro-names
    _reMacroParam = re.compile(r"""^\w+$""")


    def __init__(self, loadfunc=None, testexpr=None, escape=HTML):
        """Init the parser.

        :Parameters:
            - `loadfunc`: function to load included templates
              (i.e. ``LoaderFile(...).load``)
            - `testexpr`: function to test if a template-expressions is valid
              (i.e. ``EvalPseudoSandbox().compile``)
            - `escape`:   default-escaping (may be modified by the template)
        :Exceptions:
            - `ValueError`: if `testexpr` or `escape` is invalid.
        """
        if loadfunc is None:
            self._load = dummy_raise(NotImplementedError, "'include' not supported, since no 'loadfunc' was given.")
        else:
            self._load = loadfunc

        if testexpr is None:
            self._testexprfunc = dummy
        else:
            try:    # test if testexpr() works
                testexpr("i==1")
            except Exception as err:
                raise ValueError("Invalid 'testexpr'. (%s)" %(err))
            self._testexprfunc = testexpr

        if escape not in ESCAPE_SUPPORTED.values():
            raise ValueError("Unsupported 'escape'. (%s)" %(escape))
        self.escape = escape
        self._includestack = []

    def parse(self, template):
        """Parse a template.

        :Parameters:
            - `template`: template-unicode-string
        :Returns:         the resulting parse-tree
        :Exceptions:
            - `TemplateSyntaxError`: for template-syntax-errors
            - `TemplateIncludeError`: if template-inclusion failed
            - `TemplateException`
        """
        self._includestack = [(None, template)]   # for error-messages (_errpos)
        return self._parse(template)

    def _errpos(self, fpos):
        """Convert `fpos` to ``(filename,row,column)`` for error-messages."""
        filename, string = self._includestack[-1]
        return filename, srow(string, fpos), scol(string, fpos)

    def _testexpr(self, expr,  fpos=0):
        """Test a template-expression to detect errors."""
        try:
            self._testexprfunc(expr)
        except SyntaxError as err:
            raise TemplateSyntaxError(err, self._errpos(fpos))

    def _parse_sub(self, parsetree, text, fpos=0):
        """Parse substitutions, and append them to the parse-tree.

        Additionally, remove comments.
        """
        curr = 0
        for match in self._reSubstitution.finditer(text):
            start = match.start()
            if start > curr:
                parsetree.append(("str", self._reComment.sub('', text[curr:start])))

            if match.group("sub") is not None:
                if not match.group("end"):
                    raise TemplateSyntaxError("Missing closing tag '%s' for '%s'."
                            % (self._sub_end, match.group()), self._errpos(fpos+start))
                if len(match.group("sub")) > 0:
                    self._testexpr(match.group("sub"), fpos+start)
                    parsetree.append(("sub", match.group("sub")))
            else:
                assert(match.group("escsub") is not None)
                if not match.group("escend"):
                    raise TemplateSyntaxError("Missing closing tag '%s' for '%s'."
                            % (self._subesc_end, match.group()), self._errpos(fpos+start))
                if len(match.group("escsub")) > 0:
                    self._testexpr(match.group("escsub"), fpos+start)
                    parsetree.append(("esc", self.escape, match.group("escsub")))

            curr = match.end()

        if len(text) > curr:
            parsetree.append(("str", self._reComment.sub('', text[curr:])))

    def _parse(self, template, fpos=0):
        """Recursive part of `parse()`.

        :Parameters:
            - template
            - fpos: position of ``template`` in the complete template (for error-messages)
        """
        # blank out comments
        # (So that its content does not collide with other syntax, and
        #  because removing them completely would falsify the character-
        #  position ("match.start()") of error-messages)
        template = self._reComment.sub(lambda match: self._comment_start+" "*len(match.group(1))+match.group(2), template)

        # init parser
        parsetree = []
        curr = 0            # current position (= end of previous block)
        block_type = None   # block type: if,for,macro,raw,...
        block_indent = None # None: single-line, >=0: multi-line

        # find blocks
        for match in self._reBlock.finditer(template):
            start = match.start()
            # process template-part before this block
            if start > curr:
                self._parse_sub(parsetree, template[curr:start], fpos)

            # analyze block syntax (incl. error-checking and -messages)
            keyword = None
            block = match.groupdict()
            pos__ = fpos + start                # shortcut
            if   block["sKeyw"] is not None:    # single-line block tag
                block_indent = None
                keyword = block["sKeyw"]
                param   = block["sParam"]
                content = block["sContent"]
                if block["sSpace"]:             # restore spaces before start-tag
                    if len(parsetree) > 0 and parsetree[-1][0] == "str":
                        parsetree[-1] = ("str", parsetree[-1][1] + block["sSpace"])
                    else:
                        parsetree.append(("str", block["sSpace"]))
                pos_p = fpos + match.start("sParam")    # shortcuts
                pos_c = fpos + match.start("sContent")
            elif block["mKeyw"] is not None:    # multi-line block tag
                block_indent = len(block["indent"])
                keyword = block["mKeyw"]
                param   = block["mParam"]
                content = block["mContent"]
                pos_p = fpos + match.start("mParam")
                pos_c = fpos + match.start("mContent")
                ignored = block["mIgnored"].strip()
                if ignored  and  ignored != self._comment_start:
                    raise TemplateSyntaxError("No code allowed after block-tag.", self._errpos(fpos+match.start("mIgnored")))
            elif block["mEnd"] is not None:     # multi-line block end
                if block_type is None:
                    raise TemplateSyntaxError("No block to end here/invalid indent.", self._errpos(pos__) )
                if block_indent != len(block["mEnd"]):
                    raise TemplateSyntaxError("Invalid indent for end-tag.", self._errpos(pos__) )
                ignored = block["meIgnored"].strip()
                if ignored  and  ignored != self._comment_start:
                    raise TemplateSyntaxError("No code allowed after end-tag.", self._errpos(fpos+match.start("meIgnored")))
                block_type = None
            elif block["sEnd"] is not None:     # single-line block end
                if block_type is None:
                    raise TemplateSyntaxError("No block to end here/invalid indent.", self._errpos(pos__))
                if block_indent is not None:
                    raise TemplateSyntaxError("Invalid indent for end-tag.", self._errpos(pos__))
                block_type = None
            else:
                raise TemplateException("FATAL: Block regexp error. Please contact the author. (%s)" % match.group())

            # analyze block content (mainly error-checking and -messages)
            if keyword:
                keyword = keyword.lower()
                if   'for'   == keyword:
                    if block_type is not None:
                        raise TemplateSyntaxError("Missing block-end-tag before new block at '%s'." %(match.group()), self._errpos(pos__))
                    block_type = 'for'
                    cond = self._reForParam.match(param)
                    if cond is None:
                        raise TemplateSyntaxError("Invalid 'for ...' at '%s'." %(param), self._errpos(pos_p))
                    names = tuple(n.strip()  for n in cond.group("names").split(","))
                    self._testexpr(cond.group("iter"), pos_p+cond.start("iter"))
                    parsetree.append(("for", names, cond.group("iter"), self._parse(content, pos_c)))
                elif 'if'    == keyword:
                    if block_type is not None:
                        raise TemplateSyntaxError("Missing block-end-tag before new block at '%s'." %(match.group()), self._errpos(pos__))
                    if not param:
                        raise TemplateSyntaxError("Missing condition for 'if' at '%s'." %(match.group()), self._errpos(pos__))
                    block_type = 'if'
                    self._testexpr(param, pos_p)
                    parsetree.append(("if", param, self._parse(content, pos_c)))
                elif 'elif'  == keyword:
                    if block_type != 'if':
                        raise TemplateSyntaxError("'elif' may only appear after 'if' at '%s'." %(match.group()), self._errpos(pos__))
                    if not param:
                        raise TemplateSyntaxError("Missing condition for 'elif' at '%s'." %(match.group()), self._errpos(pos__))
                    self._testexpr(param, pos_p)
                    parsetree.append(("elif", param, self._parse(content, pos_c)))
                elif 'else'  == keyword:
                    if block_type not in ('if', 'for'):
                        raise TemplateSyntaxError("'else' may only appear after 'if' or 'for' at '%s'." %(match.group()), self._errpos(pos__))
                    if param:
                        raise TemplateSyntaxError("'else' may not have parameters at '%s'." %(match.group()), self._errpos(pos__))
                    parsetree.append(("else", self._parse(content, pos_c)))
                elif 'macro' == keyword:
                    if block_type is not None:
                        raise TemplateSyntaxError("Missing block-end-tag before new block '%s'." %(match.group()), self._errpos(pos__))
                    block_type = 'macro'
                    # make sure param is "\w+" (instead of ".+")
                    if not param:
                        raise TemplateSyntaxError("Missing name for 'macro' at '%s'." %(match.group()), self._errpos(pos__))
                    if not self._reMacroParam.match(param):
                        raise TemplateSyntaxError("Invalid name for 'macro' at '%s'." %(match.group()), self._errpos(pos__))
                    #remove last newline
                    if len(content) > 0 and content[-1] == '\n':
                        content = content[:-1]
                    if len(content) > 0 and content[-1] == '\r':
                        content = content[:-1]
                    parsetree.append(("macro", param, self._parse(content, pos_c)))

                # parser-commands
                elif 'raw'   == keyword:
                    if block_type is not None:
                        raise TemplateSyntaxError("Missing block-end-tag before new block '%s'." %(match.group()), self._errpos(pos__))
                    if param:
                        raise TemplateSyntaxError("'raw' may not have parameters at '%s'." %(match.group()), self._errpos(pos__))
                    block_type = 'raw'
                    parsetree.append(("str", content))
                elif 'include' == keyword:
                    if block_type is not None:
                        raise TemplateSyntaxError("Missing block-end-tag before new block '%s'." %(match.group()), self._errpos(pos__))
                    if param:
                        raise TemplateSyntaxError("'include' may not have parameters at '%s'." %(match.group()), self._errpos(pos__))
                    block_type = 'include'
                    try:
                        u = self._load(content.strip())
                    except Exception as err:
                        raise TemplateIncludeError(err, self._errpos(pos__))
                    self._includestack.append((content.strip(), u))  # current filename/template for error-msg.
                    p = self._parse(u)
                    self._includestack.pop()
                    parsetree.extend(p)
                elif 'set_escape' == keyword:
                    if block_type is not None:
                        raise TemplateSyntaxError("Missing block-end-tag before new block '%s'." %(match.group()), self._errpos(pos__))
                    if param:
                        raise TemplateSyntaxError("'set_escape' may not have parameters at '%s'." %(match.group()), self._errpos(pos__))
                    block_type = 'set_escape'
                    esc = content.strip().upper()
                    if esc not in ESCAPE_SUPPORTED:
                        raise TemplateSyntaxError("Unsupported escape '%s'." %(esc), self._errpos(pos__))
                    self.escape = ESCAPE_SUPPORTED[esc]
                else:
                    raise TemplateSyntaxError("Invalid keyword '%s'." %(keyword), self._errpos(pos__))
            curr = match.end()

        if block_type is not None:
            raise TemplateSyntaxError("Missing end-tag.", self._errpos(pos__))

        if len(template) > curr:            # process template-part after last block
            self._parse_sub(parsetree, template[curr:], fpos+curr)

        return parsetree

#-----------------------------------------
# Evaluation

# some checks
assert len(eval("dir()", {'__builtins__':{'dir':dir}})) == 1, \
    "FATAL: 'eval' does not work as expected (%s)."
assert compile("0 .__class__", "<string>", "eval").co_names == ('__class__',), \
    "FATAL: 'compile' does not work as expected."

class EvalPseudoSandbox:
    """An eval-pseudo-sandbox.

    The pseudo-sandbox restricts the available functions/objects, so the
    code can only access:

    - some of the builtin Python-functions, which are considered "safe"
      (see safe_builtins)
    - some additional functions (exists(), default(), setvar(), escape())
    - the passed objects incl. their methods.

    Additionally, names beginning with "_" are forbidden.
    This is to prevent things like '0 .__class__', with which you could
    easily break out of a "sandbox".

    Be careful to only pass "safe" objects/functions to the template,
    because any unsafe function/method could break the sandbox!
    For maximum security, restrict the access to as few objects/functions
    as possible!

    :Warning:
        Note that this is no real sandbox! (And although I don't know any
        way to break out of the sandbox without passing-in an unsafe object,
        I cannot guarantee that there is no such way. So use with care.)

        Take care if you want to use it for untrusted code!!
    """

    safe_builtins = {
        "True"      : True,
        "False"     : False,
        "None"      : None,

        "abs"       : builtins.abs,
        "chr"       : builtins.chr,
        "divmod"    : builtins.divmod,
        "hash"      : builtins.hash,
        "hex"       : builtins.hex,
        "len"       : builtins.len,
        "max"       : builtins.max,
        "min"       : builtins.min,
        "oct"       : builtins.oct,
        "ord"       : builtins.ord,
        "pow"       : builtins.pow,
        "range"     : builtins.range,
        "round"     : builtins.round,
        "sorted"    : builtins.sorted,
        "sum"       : builtins.sum,
        "unichr"    : builtins.chr,
        "zip"       : builtins.zip,

        "bool"      : builtins.bool,
        "bytes"     : builtins.bytes,
        "complex"   : builtins.complex,
        "dict"      : builtins.dict,
        "enumerate" : builtins.enumerate,
        "float"     : builtins.float,
        "int"       : builtins.int,
        "list"      : builtins.list,
        "long"      : long,
        "reversed"  : builtins.reversed,
        "str"       : builtins.str,
        "tuple"     : builtins.tuple,
        "unicode"   : unicode,
    }
    if sys.version_info[0] < 3:
        safe_builtins["unichr"] = builtins.unichr

    def __init__(self):
        self._compile_cache = {}
        self.locals_ptr = None
        self.eval_allowed_globals = self.safe_builtins.copy()
        self.register("__import__", self.f_import)
        self.register("exists",  self.f_exists)
        self.register("default", self.f_default)
        self.register("setvar",  self.f_setvar)
        self.register("escape",  self.f_escape)

    def register(self, name, obj):
        """Add an object to the "allowed eval-globals".

        Mainly useful to add user-defined functions to the pseudo-sandbox.
        """
        self.eval_allowed_globals[name] = obj

    def compile(self, expr):
        """Compile a Python-eval-expression.

        - Use a compile-cache.
        - Raise a `NameError` if `expr` contains a name beginning with ``_``.

        :Returns: the compiled `expr`
        :Exceptions:
            - `SyntaxError`: for compile-errors
            - `NameError`: if expr contains a name beginning with ``_``
        """
        if expr not in self._compile_cache:
            c = compile(expr, "", "eval")
            for i in c.co_names:    #prevent breakout via new-style-classes
                if i[0] == '_':
                    raise NameError("Name '%s' is not allowed." % i)
            self._compile_cache[expr] = c
        return self._compile_cache[expr]

    def eval(self, expr, locals):
        """Eval a Python-eval-expression.

        Sets ``self.locals_ptr`` to ``locales`` and compiles the code
        before evaluating.
        """
        sav = self.locals_ptr
        self.locals_ptr = locals
        x = eval(self.compile(expr), {"__builtins__":self.eval_allowed_globals}, locals)
        self.locals_ptr = sav
        return x

    def f_import(self, name, *_, **__):
        """``import``/``__import__()`` for the sandboxed code.

        Since "import" is insecure, the PseudoSandbox does not allow to
        import other modules. But since some functions need to import
        other modules (e.g. "datetime.datetime.strftime" imports "time"),
        this function replaces the builtin "import" and allows to use
        modules which are already accessible by the sandboxed code.

        :Note:
            - This probably only works for rather simple imports.
            - For security, it may be better to avoid such (complex) modules
              which import other modules. (e.g. use time.localtime and
              time.strftime instead of datetime.datetime.strftime,
              or write a small wrapper.)

        :Example:

            >>> from datetime import datetime
            >>> import pyratemp
            >>> t = pyratemp.Template('@!mytime.strftime("%H:%M:%S")!@')

            # >>> print(t(mytime=datetime.now()))
            # Traceback (most recent call last):
            #   ...
            # ImportError: import not allowed in pseudo-sandbox; try to import 'time' yourself and pass it to the sandbox/template

            >>> import time
            >>> print(t(mytime=datetime.strptime("13:40:54", "%H:%M:%S"), time=time))
            13:40:54

            # >>> print(t(mytime=datetime.now(), time=time))
            # 13:40:54
        """
        import types
        if self.locals_ptr is not None  and  name in self.locals_ptr  and  isinstance(self.locals_ptr[name], types.ModuleType):
            return self.locals_ptr[name]
        else:
            raise ImportError("import not allowed in pseudo-sandbox; try to import '%s' yourself (and maybe pass it to the sandbox/template)" % name)

    def f_exists(self, varname):
        """``exists()`` for the sandboxed code.

        Test if the variable `varname` exists in the current locals-namespace.

        This only works for single variable names. If you want to test
        complicated expressions, use i.e. `default`.
        (i.e. `default("expr",False)`)

        :Note:      the variable-name has to be quoted! (like in eval)
        :Example:   see module-docstring
        """
        return (varname in self.locals_ptr)

    def f_default(self, expr, default=None):
        """``default()`` for the sandboxed code.

        Try to evaluate an expression and return the result or a
        fallback-/default-value; the `default`-value is used
        if `expr` does not exist/is invalid/results in None.

        This is very useful for optional data.

        :Parameter:
            - expr: eval-expression
            - default: fallback-falue if eval(expr) fails or is None.
        :Returns:
            the eval-result or the "fallback"-value.

        :Note:      the eval-expression has to be quoted! (like in eval)
        :Example:   see module-docstring
        """
        try:
            r = self.eval(expr, self.locals_ptr)
            if r is None:
                return default
            return r
        #TODO: which exceptions should be catched here?
        except (NameError, LookupError, TypeError):
            return default

    def f_setvar(self, name, expr):
        """``setvar()`` for the sandboxed code.

        Set a variable.

        :Example:   see module-docstring
        """
        self.locals_ptr[name] = self.eval(expr, self.locals_ptr)
        return ""

    def f_escape(self, s, format="HTML"):
        """``escape()`` for the sandboxed code.
        """
        if isinstance(format, (str, unicode)):
            format = ESCAPE_SUPPORTED[format.upper()]
        return escape(unicode(s), format)

#-----------------------------------------
# basic template / subtemplate

class TemplateBase:
    """Basic template-class.

    Used both for the template itself and for 'macro's ("subtemplates") in
    the template.
    """

    def __init__(self, parsetree, renderfunc, data=None):
        """Create the Template/Subtemplate/Macro.

        :Parameters:
            - `parsetree`: parse-tree of the template/subtemplate/macro
            - `renderfunc`: render-function
            - `data`: data to fill into the template by default (dictionary).
              This data may later be overridden when rendering the template.
        :Exceptions:
            - `TypeError`: if `data` is not a dictionary
        """
        #TODO: parameter-checking?
        self.parsetree = parsetree
        if isinstance(data, dict):
            self.data = data
        elif data is None:
            self.data = {}
        else:
            raise TypeError('"data" must be a dict (or None).')
        self.current_data = data
        self._render = renderfunc

    def __call__(self, **override):
        """Fill out/render the template.

        :Parameters:
            - `override`: objects to add to the data-namespace, overriding
              the "default"-data.
        :Returns:    the filled template (in unicode)
        :Note:       This is also called when invoking macros
                     (i.e. ``$!mymacro()!$``).
        """
        self.current_data = self.data.copy()
        self.current_data.update(override)
        u = "".join(self._render(self.parsetree, self.current_data))
        self.current_data = self.data       # restore current_data
        return _dontescape(u)               # (see class _dontescape)

    def __unicode__(self):
        """Alias for __call__()."""
        return self.__call__()
    def __str__(self):
        """Alias for __call__()."""
        return self.__call__()

#-----------------------------------------
# Renderer

class _dontescape(unicode):
    """Unicode-string which should not be escaped.

    If ``isinstance(object,_dontescape)``, then don't escape the object in
    ``@!...!@``. It's useful for not double-escaping macros, and it's
    automatically used for macros/subtemplates.

    :Note: This only works if the object is used on its own in ``@!...!@``.
           It i.e. does not work in ``@!object*2!@`` or ``@!object + "hi"!@``.
    """
    __slots__ = []


class Renderer(object):
    """Render a template-parse-tree.

    :Uses: `TemplateBase` for macros
    """

    def __init__(self, evalfunc, escapefunc):
        """Init the renderer.

        :Parameters:
            - `evalfunc`: function for template-expression-evaluation
              (i.e. ``EvalPseudoSandbox().eval``)
            - `escapefunc`: function for escaping special characters
              (i.e. `escape`)
        """
        #TODO: test evalfunc
        self.evalfunc = evalfunc
        self.escapefunc = escapefunc

    def _eval(self, expr, data):
        """evalfunc with error-messages"""
        try:
            return self.evalfunc(expr, data)
        #TODO: any other errors to catch here?
        except (TypeError,NameError,LookupError,AttributeError, SyntaxError) as err:
            raise TemplateRenderError("Cannot eval expression '%s'. (%s: %s)" %(expr, err.__class__.__name__, err))

    def render(self, parsetree, data):
        """Render a parse-tree of a template.

        :Parameters:
            - `parsetree`: the parse-tree
            - `data`:      the data to fill into the template (dictionary)
        :Returns:   the rendered output-unicode-string
        :Exceptions:
            - `TemplateRenderError`
        """
        _eval = self._eval  # shortcut
        output = []
        do_else = False     # use else/elif-branch?

        if parsetree is None:
            return ""
        for elem in parsetree:
            if   "str"   == elem[0]:
                output.append(elem[1])
            elif "sub"   == elem[0]:
                output.append(unicode(_eval(elem[1], data)))
            elif "esc"   == elem[0]:
                obj = _eval(elem[2], data)
                #prevent double-escape
                if isinstance(obj, _dontescape) or isinstance(obj, TemplateBase):
                    output.append(unicode(obj))
                else:
                    output.append(self.escapefunc(unicode(obj), elem[1]))
            elif "for"   == elem[0]:
                do_else = True
                (names, iterable) = elem[1:3]
                try:
                    loop_iter = iter(_eval(iterable, data))
                except TypeError:
                    raise TemplateRenderError("Cannot loop over '%s'." % iterable)
                for i in loop_iter:
                    do_else = False
                    if len(names) == 1:
                        data[names[0]] = i
                    else:
                        data.update(zip(names, i))   #"for a,b,.. in list"
                    output.extend(self.render(elem[3], data))
            elif "if"    == elem[0]:
                do_else = True
                if _eval(elem[1], data):
                    do_else = False
                    output.extend(self.render(elem[2], data))
            elif "elif"  == elem[0]:
                if do_else and _eval(elem[1], data):
                    do_else = False
                    output.extend(self.render(elem[2], data))
            elif "else"  == elem[0]:
                if do_else:
                    do_else = False
                    output.extend(self.render(elem[1], data))
            elif "macro" == elem[0]:
                data[elem[1]] = TemplateBase(elem[2], self.render, data)
            else:
                raise TemplateRenderError("Invalid parse-tree (%s)." %(elem))

        return output

#-----------------------------------------
# template user-interface (putting it all together)

class Template(TemplateBase):
    """Template-User-Interface.

    :Usage:
        ::
            t = Template(...)  (<- see __init__)
            output = t(...)    (<- see TemplateBase.__call__)

    :Example:
        see module-docstring
    """

    def __init__(self, string=None,filename=None,parsetree=None, encoding='utf-8', data=None, escape=HTML,
            loader_class=LoaderFile,
            parser_class=Parser,
            renderer_class=Renderer,
            eval_class=EvalPseudoSandbox,
            escape_func=escape):
        """Load (+parse) a template.

        :Parameters:
            - `string,filename,parsetree`: a template-string,
                                           filename of a template to load,
                                           or a template-parsetree.
                                           (only one of these 3 is allowed)
            - `encoding`: encoding of the template-files (only used for "filename")
            - `data`:     data to fill into the template by default (dictionary).
                          This data may later be overridden when rendering the template.
            - `escape`:   default-escaping for the template, may be overwritten by the template!
            - `loader_class`
            - `parser_class`
            - `renderer_class`
            - `eval_class`
            - `escapefunc`
        """
        if [string, filename, parsetree].count(None) != 2:
            raise ValueError('Exactly 1 of string,filename,parsetree is necessary.')

        tmpl = None
        # load template
        if filename is not None:
            incl_load = loader_class(os.path.dirname(filename), encoding).load
            tmpl = incl_load(os.path.basename(filename))
        if string is not None:
            incl_load = dummy_raise(NotImplementedError, "'include' not supported for template-strings.")
            tmpl = LoaderString(encoding).load(string)

        # eval (incl. compile-cache)
        templateeval = eval_class()

        # parse
        if tmpl is not None:
            p = parser_class(loadfunc=incl_load, testexpr=templateeval.compile, escape=escape)
            parsetree = p.parse(tmpl)
            del p

        # renderer
        renderfunc = renderer_class(templateeval.eval, escape_func).render

        #create template
        TemplateBase.__init__(self, parsetree, renderfunc, data)


#=========================================
#doctest

def _doctest():
    """doctest this module."""
    import doctest
    doctest.testmod()

#----------------------
if __name__ == '__main__':
    if sys.version_info[0] <= 2:
        _doctest()

#=========================================

