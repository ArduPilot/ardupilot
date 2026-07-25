#!/usr/bin/env python3

'''parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

General process:
 - each filename passed in:
    - may be validated, based on --validate
    - is parsed using mavparse.MAVXML into an xml document and appended to a list, "xml"

 - expand_includes is called to do a breadth-first search of the xml
    documents structure inferred by the <include> tags in each
    document, expanding the xml list from its base (just the ones on
    the commandline) to the entire structure

 - update_includes works on the xml list created by expand_includes
    - any xml document with no includes is added to the "done" list (there must be at least one of these)
    - it repeatedly calls update_one_iteration
    - each iteration is intended to include the crcs and other information from includes into the xml document doing the include

'''

import os
import re
import sys
from . import mavparse

# XSD schema file
schemaFile = os.path.join(os.path.dirname(os.path.realpath(__file__)), "mavschema.xsd")

# Set defaults for generating MAVLink code
DEFAULT_WIRE_PROTOCOL = mavparse.PROTOCOL_1_0
DEFAULT_LANGUAGE = 'Python'
DEFAULT_VALIDATE = True
DEFAULT_STRICT_UNITS = False

MAXIMUM_INCLUDE_FILE_NESTING = 5

# List the supported languages. This is done globally because it's used by the GUI wrapper too
# Right now, 'JavaScript' ~= 'JavaScript_Stable', in the future it may be made equivalent to 'JavaScript_NextGen'
supportedLanguages = ["Ada", "C", "CS", "JavaScript", "JavaScript_Stable","JavaScript_NextGen", "TypeScript", "Python3", "Python", "Lua", "WLua", "ObjC", "Spin2", "Swift", "Java", "C++11"]


def mavgen(opts, args):
    """Generate mavlink message formatters and parsers (C and Python ) using options
    and args where args are a list of xml files. This function allows python
    scripts under Windows to control mavgen using the same interface as
    shell scripts under Unix"""

    xml = []
    all_files = set()

    # Enable validation by default, disabling it if explicitly requested
    if opts.validate:
        try:
            from lxml import etree
            with open(schemaFile, 'r') as f:
                xmlschema_root = etree.parse(f)
                if not opts.strict_units:
                    # replace the strict "SI_Unit" list of known unit strings with a more generic "xs:string" type
                    for elem in xmlschema_root.iterfind('xs:attribute[@name="units"]', xmlschema_root.getroot().nsmap):
                        elem.set("type", "xs:string")
                xmlschema = etree.XMLSchema(xmlschema_root)
        except ImportError:
            print("WARNING: Failed to import lxml module etree. Are lxml, libxml2 and libxslt installed? XML validation will not be performed", file=sys.stderr)
            opts.validate = False
        except etree.XMLSyntaxError as err:
            print("WARNING: XML Syntax Errors detected in %s XML schema file. XML validation will not be performed" % schemaFile, file=sys.stderr)
            print(str(err.error_log), file=sys.stderr)
            opts.validate = False
        except Exception as e:
            print("Exception:", e)
            print("WARNING: Unable to load XML validator libraries. XML validation will not be performed", file=sys.stderr)
            opts.validate = False

    def expand_includes():
        """Expand includes. Root files already parsed objects in the xml list."""

        def expand_oneiteration():
            '''takes the list of xml files to process and finds includes which
            have not already been turned into xml documents added to
            xml files to process, turns them into xml documents and
            adds them to the xml files list.  Returns false if no more
            documents were added.
            '''
            includeadded = False
            for x in xml[:]:
                for i in x.include:
                    fname = os.path.abspath(os.path.join(os.path.dirname(x.filename), i))
                    # Only parse new include files
                    if fname in all_files:
                        continue
                    # Validate XML file with XSD file if possible.
                    if opts.validate:
                        print("Validating %s" % fname)
                        if not mavgen_validate(fname):
                            print("ERROR Validation of %s failed" % fname)
                            exit(1)
                    else:
                        print("Validation skipped for %s." % fname)
                    # Parsing
                    print("Parsing %s" % fname)
                    xml.append(mavparse.MAVXML(fname, opts.wire_protocol))
                    all_files.add(fname)
                    includeadded = True
            return includeadded

        for i in range(MAXIMUM_INCLUDE_FILE_NESTING):
            if not expand_oneiteration():
                break

        if mavparse.check_duplicates(xml):
            return False
        if opts.validate and mavparse.check_missing_enum(xml):
            return False
        return True

    def update_includes():
        """Update dialects with crcs etc of included files.  Included files
        were already found and parsed into xml list in
        expand_includes().
        """

        # 1: Mark files that don't have includes as "done"
        done = []
        for x in xml:
            #print("\n",x)
            if len(x.include) == 0:
                done.append(x)
                #print("\nFile with no includes found (ENDPOINT): %s" % x.filename )
        if len(done) == 0:
            print("\nERROR in includes tree, no base found!")
            exit(1)

        #print("\n",done)

        # 2: Update all 'not done' files for which all includes have
        # been done.  Returns True if any updates were made
        def update_oneiteration():
            initial_done_length = len(done)
            for x in xml:
                #print("\nCHECK %s" % x.filename)
                if x in done:
                    #print("  already done, skip")
                    continue
                #check if all its includes were already done
                all_includes_done = True
                for i in x.include:
                    fname = os.path.abspath(os.path.join(os.path.dirname(x.filename), i))
                    if fname not in [d.filename for d in done]:
                        all_includes_done = False
                        break
                if not all_includes_done:
                    #print("  not all includes ready, skip")
                    continue
                #Found file where all includes are done
                done.append(x)
                #print("  all includes ready, add" )
                #now update it with the facts from all it's includes
                for i in x.include:
                    fname = os.path.abspath(os.path.join(os.path.dirname(x.filename), i))
                    #print("  include file %s" % i )
                    #Find the corresponding x
                    for ix in xml:
                        if ix.filename != fname:
                            continue
                        #print("    add %s" % ix.filename )
                        x.message_crcs.update(ix.message_crcs)
                        x.message_lengths.update(ix.message_lengths)
                        x.message_min_lengths.update(ix.message_min_lengths)
                        x.message_flags.update(ix.message_flags)
                        x.message_target_system_ofs.update(ix.message_target_system_ofs)
                        x.message_target_component_ofs.update(ix.message_target_component_ofs)
                        x.message_names.update(ix.message_names)
                        x.largest_payload = max(x.largest_payload, ix.largest_payload)
                        break

            if len(done) == len(xml):
                return False  # finished
            if len(done) == initial_done_length:
                # we've made no progress
                print("ERROR include tree can't be resolved, no base found!")
                exit(1)
            return True

        for i in range(MAXIMUM_INCLUDE_FILE_NESTING):
            #print("\nITERATION "+str(i))
            if not update_oneiteration():
                break

    def mavgen_validate(xmlfile):
        """Uses lxml to validate an XML file. We define mavgen_validate
           here because it relies on the XML libs that were loaded in mavgen(), so it can't be called standalone"""
        xmlvalid = True
        try:
            with open(xmlfile, 'r') as f:
                xmldocument = etree.parse(f)
                xmlschema.assertValid(xmldocument)
                forbidden_names_re = re.compile("^(break$|case$|class$|catch$|const$|continue$|debugger$|default$|delete$|do$|else$|\
                                    export$|extends$|finally$|for$|function$|if$|import$|in$|instanceof$|let$|new$|\
                                    return$|super$|switch$|this$|throw$|try$|typeof$|var$|void$|while$|with$|yield$|\
                                    enum$|await$|implements$|package$|protected$|static$|interface$|private$|public$|\
                                    abstract$|boolean$|byte$|char$|double$|final$|float$|goto$|int$|long$|native$|\
                                    short$|synchronized$|transient$|volatile$).*", re.IGNORECASE)
                for element in xmldocument.iter('enum', 'entry', 'message', 'field'):
                    if forbidden_names_re.search(element.get('name')):
                        print("Validation error:", file=sys.stderr)
                        print("Element : %s at line : %s contains forbidden word" % (element.tag, element.sourceline), file=sys.stderr)
                        xmlvalid = False

            return xmlvalid
        except etree.XMLSchemaError:
            return False
        except etree.DocumentInvalid as err:
            sys.exit('ERROR: %s' % str(err.error_log))
        return True

    # Process all XML files, validating them as necessary.
    for fname in args:
        # only add each dialect file argument once.
        if fname in all_files:
            continue
        all_files.add(fname)

        if opts.validate:
            print("Validating %s" % fname)
            if not mavgen_validate(fname):
                return False
        else:
            print("Validation skipped for %s." % fname)

        print("Parsing %s" % fname)
        xml.append(mavparse.MAVXML(fname, opts.wire_protocol))

    # expand includes
    if not expand_includes():
        return False
    update_includes()

    print("Found %u MAVLink message types in %u XML files" % (
        mavparse.total_msgs(xml), len(xml)))

    # convert language option to lowercase and validate
    opts.language = opts.language.lower()
    if opts.language == 'python3' or opts.language == 'python':
        from . import mavgen_python
        mavgen_python.generate(opts.output, xml)
    elif opts.language == 'c':
        from . import mavgen_c
        mavgen_c.generate(opts.output, xml)
    elif opts.language == 'lua':
        from . import mavgen_lua
        mavgen_lua.generate(opts.output, xml)
    elif opts.language == 'wlua':
        from . import mavgen_wlua
        mavgen_wlua.generate(opts.output, xml)
    elif opts.language == 'cs':
        from . import mavgen_cs
        mavgen_cs.generate(opts.output, xml)
    elif (opts.language == 'javascript' ) or ( opts.language == 'javascript_stable' ):
        from . import mavgen_javascript_stable as mavgen_javascript
        mavgen_javascript.generate(opts.output, xml)
    elif opts.language == 'javascript_nextgen':
        from . import mavgen_javascript
        mavgen_javascript.generate(opts.output, xml)
    elif opts.language == 'typescript':
        from . import mavgen_typescript
        mavgen_typescript.generate(opts.output, xml)
    elif opts.language == 'objc':
        from . import mavgen_objc
        mavgen_objc.generate(opts.output, xml)
    elif opts.language == 'swift':
        from . import mavgen_swift
        mavgen_swift.generate(opts.output, xml)
    elif opts.language == 'java':
        from . import mavgen_java
        mavgen_java.generate(opts.output, xml)
    elif opts.language == 'c++11':
        from . import mavgen_cpp11
        mavgen_cpp11.generate(opts.output, xml)
    elif opts.language == 'spin2':
        from . import mavgen_spin2
        mavgen_spin2.generate(opts.output, xml)
    elif opts.language == 'ada':
        from . import mavgen_ada
        if opts.wire_protocol == mavparse.PROTOCOL_1_0:
            mavgen_ada.generate(opts.output, xml)
        elif opts.wire_protocol == mavparse.PROTOCOL_2_0:
            mavgen_ada.generate_v2(opts.output, xml)
        else:
            raise DeprecationWarning("Error! Mavgen_Ada does not supports protocol version")
    else:
        print("Unsupported language %s" % opts.language)

    return True


# build all the dialects in the dialects subpackage
class Opts(object):
    def __init__(self, output, wire_protocol=DEFAULT_WIRE_PROTOCOL, language=DEFAULT_LANGUAGE, validate=DEFAULT_VALIDATE, strict_units=DEFAULT_STRICT_UNITS):
        self.wire_protocol = wire_protocol
        self.language = language
        self.output = output
        self.validate = validate
        self.strict_units = strict_units


def mavgen_python_dialect(dialect, wire_protocol):
    '''generate the python code on the fly for a MAVLink dialect'''
    dialects = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'dialects')
    mdef = os.getenv("MDEF", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'message_definitions'))
    if wire_protocol == mavparse.PROTOCOL_0_9:
        py = os.path.join(dialects, 'v09', dialect + '.py')
        xml = os.path.join(dialects, 'v09', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v0.9', dialect + '.xml')
    elif wire_protocol == mavparse.PROTOCOL_1_0:
        py = os.path.join(dialects, 'v10', dialect + '.py')
        xml = os.path.join(dialects, 'v10', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v1.0', dialect + '.xml')
    else:
        py = os.path.join(dialects, 'v20', dialect + '.py')
        xml = os.path.join(dialects, 'v20', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v1.0', dialect + '.xml')

    opts = Opts(py, wire_protocol, language="Python3")

    import io

    # throw away stdout while generating
    stdout_saved = sys.stdout
    sys.stdout = io.StringIO()
    try:
        xml = os.path.relpath(xml)
        if not mavgen(opts, [xml]):
            sys.stdout.seek(0)
            stdout_saved.write(sys.stdout.getvalue())
            sys.stdout = stdout_saved
            return False
    except Exception:
        sys.stdout = stdout_saved
        raise
    sys.stdout = stdout_saved
    return True

if __name__ == "__main__":
    raise DeprecationWarning("Executable was moved to pymavlink.tools.mavgen")
