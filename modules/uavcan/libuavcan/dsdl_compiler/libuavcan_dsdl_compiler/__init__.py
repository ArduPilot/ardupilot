#!/usr/bin/env python
#
# UAVCAN DSDL compiler for libuavcan
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

'''
This module implements the core functionality of the UAVCAN DSDL compiler for libuavcan.
Supported Python versions: 3.2+, 2.7.
It accepts a list of root namespaces and produces the set of C++ header files for libuavcan.
It is based on the DSDL parsing package from pyuavcan.
'''

from __future__ import division, absolute_import, print_function, unicode_literals
import sys, os, logging, errno, re
from .pyratemp import Template
from dronecan import dsdl

# Python 2.7 compatibility
try:
    str = unicode
except NameError:
    pass

OUTPUT_FILE_EXTENSION = 'hpp'
OUTPUT_FILE_PERMISSIONS = 0o444  # Read only for all
TEMPLATE_FILENAME = os.path.join(os.path.dirname(__file__), 'data_type_template.tmpl')

__all__ = ['run', 'logger', 'DsdlCompilerException']

class DsdlCompilerException(Exception):
    pass

logger = logging.getLogger(__name__)

def run(source_dirs, include_dirs, output_dir):
    '''
    This function takes a list of root namespace directories (containing DSDL definition files to parse), a
    possibly empty list of search directories (containing DSDL definition files that can be referenced from the types
    that are going to be parsed), and the output directory path (possibly nonexistent) where the generated C++
    header files will be stored.

    Note that this module features lazy write, i.e. if an output file does already exist and its content is not going
    to change, it will not be overwritten. This feature allows to avoid unnecessary recompilation of dependent object
    files.

    Args:
        source_dirs    List of root namespace directories to parse.
        include_dirs   List of root namespace directories with referenced types (possibly empty). This list is
                       automaitcally extended with source_dirs.
        output_dir     Output directory path. Will be created if doesn't exist.
    '''
    assert isinstance(source_dirs, list)
    assert isinstance(include_dirs, list)
    output_dir = str(output_dir)

    types = run_parser(source_dirs, include_dirs + source_dirs)
    if not types:
        die('No type definitions were found')

    logger.info('%d types total', len(types))
    run_generator(types, output_dir)

# -----------------

def pretty_filename(filename):
    try:
        a = os.path.abspath(filename)
        r = os.path.relpath(filename)
        return a if '..' in r else r
    except ValueError:
        return filename

def type_output_filename(t):
    assert t.category == t.CATEGORY_COMPOUND
    return t.full_name.replace('.', os.path.sep) + '.' + OUTPUT_FILE_EXTENSION

def makedirs(path):
    try:
        try:
            os.makedirs(path, exist_ok=True)  # May throw "File exists" when executed as root, which is wrong
        except TypeError:
            os.makedirs(path)  # Python 2.7 compatibility
    except OSError as ex:
        if ex.errno != errno.EEXIST:  # http://stackoverflow.com/questions/12468022
            raise

def die(text):
    raise DsdlCompilerException(str(text))

def run_parser(source_dirs, search_dirs):
    try:
        types = dsdl.parse_namespaces(source_dirs, search_dirs)
    except dsdl.DsdlException as ex:
        logger.info('Parser failure', exc_info=True)
        die(ex)
    return types

def run_generator(types, dest_dir):
    try:
        template_expander = make_template_expander(TEMPLATE_FILENAME)
        dest_dir = os.path.abspath(dest_dir)  # Removing '..'
        makedirs(dest_dir)
        for t in types:
            logger.info('Generating type %s', t.full_name)
            filename = os.path.join(dest_dir, type_output_filename(t))
            text = generate_one_type(template_expander, t)
            write_generated_data(filename, text)
    except Exception as ex:
        logger.info('Generator failure', exc_info=True)
        die(ex)

def write_generated_data(filename, data):
    dirname = os.path.dirname(filename)
    makedirs(dirname)

    # Lazy update - file will not be rewritten if its content is not going to change
    if os.path.exists(filename):
        with open(filename) as f:
            existing_data = f.read()
        if data == existing_data:
            logger.info('Up to date [%s]', pretty_filename(filename))
            return
        logger.info('Rewriting [%s]', pretty_filename(filename))
        os.remove(filename)
    else:
        logger.info('Creating [%s]', pretty_filename(filename))

    # Full rewrite
    with open(filename, 'w') as f:
        f.write(data)
    try:
        os.chmod(filename, OUTPUT_FILE_PERMISSIONS)
    except (OSError, IOError) as ex:
        logger.warning('Failed to set permissions for %s: %s', pretty_filename(filename), ex)

def type_to_cpp_type(t):
    if t.category == t.CATEGORY_PRIMITIVE:
        cast_mode = {
            t.CAST_MODE_SATURATED: '::uavcan::CastModeSaturate',
            t.CAST_MODE_TRUNCATED: '::uavcan::CastModeTruncate',
        }[t.cast_mode]
        if t.kind == t.KIND_FLOAT:
            return '::uavcan::FloatSpec< %d, %s >' % (t.bitlen, cast_mode)
        else:
            signedness = {
                t.KIND_BOOLEAN: '::uavcan::SignednessUnsigned',
                t.KIND_UNSIGNED_INT: '::uavcan::SignednessUnsigned',
                t.KIND_SIGNED_INT: '::uavcan::SignednessSigned',
            }[t.kind]
            return '::uavcan::IntegerSpec< %d, %s, %s >' % (t.bitlen, signedness, cast_mode)
    elif t.category == t.CATEGORY_ARRAY:
        value_type = type_to_cpp_type(t.value_type)
        mode = {
            t.MODE_STATIC: '::uavcan::ArrayModeStatic',
            t.MODE_DYNAMIC: '::uavcan::ArrayModeDynamic',
        }[t.mode]
        return '::uavcan::Array< %s, %s, %d >' % (value_type, mode, t.max_size)
    elif t.category == t.CATEGORY_COMPOUND:
        return '::' + t.full_name.replace('.', '::')
    elif t.category == t.CATEGORY_VOID:
        return '::uavcan::IntegerSpec< %d, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >' % t.bitlen
    else:
        raise DsdlCompilerException('Unknown type category: %s' % t.category)

def generate_one_type(template_expander, t):
    t.short_name = t.full_name.split('.')[-1]
    t.cpp_type_name = t.short_name + '_'
    t.cpp_full_type_name = '::' + t.full_name.replace('.', '::')
    t.include_guard = t.full_name.replace('.', '_').upper() + '_HPP_INCLUDED'

    # Dependencies (no duplicates)
    def fields_includes(fields):
        def detect_include(t):
            if t.category == t.CATEGORY_COMPOUND:
                return type_output_filename(t)
            if t.category == t.CATEGORY_ARRAY:
                return detect_include(t.value_type)
        return list(sorted(set(filter(None, [detect_include(x.type) for x in fields]))))

    if t.kind == t.KIND_MESSAGE:
        t.cpp_includes = fields_includes(t.fields)
    else:
        t.cpp_includes = fields_includes(t.request_fields + t.response_fields)

    t.cpp_namespace_components = t.full_name.split('.')[:-1]
    t.has_default_dtid = t.default_dtid is not None

    # Attribute types
    def inject_cpp_types(attributes):
        void_index = 0
        for a in attributes:
            a.cpp_type = type_to_cpp_type(a.type)
            a.void = a.type.category == a.type.CATEGORY_VOID
            if a.void:
                assert not a.name
                a.name = '_void_%d' % void_index
                void_index += 1

    if t.kind == t.KIND_MESSAGE:
        inject_cpp_types(t.fields)
        inject_cpp_types(t.constants)
        t.all_attributes = t.fields + t.constants
        t.union = t.union and len(t.fields)
    else:
        inject_cpp_types(t.request_fields)
        inject_cpp_types(t.request_constants)
        inject_cpp_types(t.response_fields)
        inject_cpp_types(t.response_constants)
        t.all_attributes = t.request_fields + t.request_constants + t.response_fields + t.response_constants
        t.request_union = t.request_union and len(t.request_fields)
        t.response_union = t.response_union and len(t.response_fields)

    # Constant properties
    def inject_constant_info(constants):
        for c in constants:
            if c.type.kind == c.type.KIND_FLOAT:
                float(c.string_value)  # Making sure that this is a valid float literal
                c.cpp_value = c.string_value
            else:
                int(c.string_value)  # Making sure that this is a valid integer literal
                c.cpp_value = c.string_value
                if c.type.kind == c.type.KIND_UNSIGNED_INT:
                    c.cpp_value += 'U'

    if t.kind == t.KIND_MESSAGE:
        inject_constant_info(t.constants)
    else:
        inject_constant_info(t.request_constants)
        inject_constant_info(t.response_constants)

    # Data type kind
    t.cpp_kind = {
        t.KIND_MESSAGE: '::uavcan::DataTypeKindMessage',
        t.KIND_SERVICE: '::uavcan::DataTypeKindService',
    }[t.kind]

    # Generation
    text = template_expander(t=t)  # t for Type
    text = '\n'.join(x.rstrip() for x in text.splitlines())
    text = text.replace('\n\n\n\n\n', '\n\n').replace('\n\n\n\n', '\n\n').replace('\n\n\n', '\n\n')
    text = text.replace('{\n\n ', '{\n ')
    return text

def make_template_expander(filename):
    '''
    Templating is based on pyratemp (http://www.simple-is-better.org/template/pyratemp.html).
    The pyratemp's syntax is rather verbose and not so human friendly, so we define some
    custom extensions to make it easier to read and write.
    The resulting syntax somewhat resembles Mako (which was used earlier instead of pyratemp):
        Substitution:
            ${expression}
        Line joining through backslash (replaced with a single space):
            ${foo(bar(very_long_arument=42, \
                      second_line=72))}
        Blocks:
            % for a in range(10):
                % if a == 5:
                    ${foo()}
                % endif
            % endfor
    The extended syntax is converted into pyratemp's through regexp substitution.
    '''
    with open(filename) as f:
        template_text = f.read()

    # Backslash-newline elimination
    template_text = re.sub(r'\\\r{0,1}\n\ *', r' ', template_text)

    # Substitution syntax transformation: ${foo} ==> $!foo!$
    template_text = re.sub(r'([^\$]{0,1})\$\{([^\}]+)\}', r'\1$!\2!$', template_text)

    # Flow control expression transformation: % foo: ==> <!--(foo)-->
    template_text = re.sub(r'(?m)^(\ *)\%\ *(.+?):{0,1}$', r'\1<!--(\2)-->', template_text)

    # Block termination transformation: <!--(endfoo)--> ==> <!--(end)-->
    template_text = re.sub(r'\<\!--\(end[a-z]+\)--\>', r'<!--(end)-->', template_text)

    # Pyratemp workaround.
    # The problem is that if there's no empty line after a macro declaration, first line will be doubly indented.
    # Workaround:
    #  1. Remove trailing comments
    #  2. Add a newline after each macro declaration
    template_text = re.sub(r'\ *\#\!.*', '', template_text)
    template_text = re.sub(r'(\<\!--\(macro\ [a-zA-Z0-9_]+\)--\>.*?)', r'\1\n', template_text)

    # Preprocessed text output for debugging
#   with open(filename + '.d', 'w') as f:
#       f.write(template_text)

    template = Template(template_text)

    def expand(**args):
        # This function adds one indentation level (4 spaces); it will be used from the template
        args['indent'] = lambda text, idnt = '    ': idnt + text.replace('\n', '\n' + idnt)
        # This function works like enumerate(), telling you whether the current item is the last one
        def enum_last_value(iterable, start=0):
            it = iter(iterable)
            count = start
            try:
                last = next(it)
            except StopIteration:
                return
            for val in it:
                yield count, False, last
                last = val
                count += 1
            yield count, True, last
        args['enum_last_value'] = enum_last_value
        return template(**args)

    return expand
