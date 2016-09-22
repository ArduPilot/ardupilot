# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

"""
Loader for messages and :class:`MsgContext` that assumes a
dictionary-based search path scheme (keys are the package/namespace,
values are the paths).  Compatible with ROS package system and other
possible layouts.
"""

import os
import sys

try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x

from . base import InvalidMsgSpec, log, SEP, COMMENTCHAR, CONSTCHAR, IODELIM, EXT_MSG, EXT_SRV
from . msgs import MsgSpec, TIME, TIME_MSG, DURATION, DURATION_MSG, HEADER, HEADER_FULL_NAME, \
     is_builtin, is_valid_msg_field_name, is_valid_msg_type, bare_msg_type, is_valid_constant_type, \
     Field, Constant, resolve_type
from . names import normalize_package_context, package_resource_name
from . srvs import SrvSpec

class MsgNotFound(Exception):
    pass

def get_msg_file(package, base_type, search_path, ext=EXT_MSG):
    """
    Determine the file system path for the specified ``.msg`` on
    *search_path*.

    :param package: name of package file is in, ``str``
    :param base_type: type name of message, e.g. 'Point2DFloat32', ``str``
    :param search_path: dictionary mapping message namespaces to a directory locations
    :param ext: msg file extension.  Override with EXT_SRV to search for services instead.

    :returns: filesystem path of requested file, ``str``
    :raises: :exc:`MsgNotFound` If message cannot be located.
    """
    log("msg_file(%s, %s, %s)" % (package, base_type, str(search_path)))
    if not isinstance(search_path, dict):
        raise ValueError("search_path must be a dictionary of {namespace: dirpath}")
    if not package in search_path:
        raise MsgNotFound("Cannot locate message [%s]: unknown package [%s] on search path [%s]" \
                          % (base_type, package, search_path))
    else:
        for path_tmp in search_path[package]:
            path = os.path.join(path_tmp, "%s%s"%(base_type, ext))
            if os.path.isfile(path):
                return path
        raise MsgNotFound("Cannot locate message [%s] in package [%s] with paths [%s]"%
                                                (base_type, package, str(search_path[package])))

def get_srv_file(package, base_type, search_path):
    """
    Determine the file system path for the specified .srv on path.

    :param package: name of package ``.srv`` file is in, ``str`` 
    :param base_type: type name of service, e.g. 'Empty', ``str``
    :param search_path: dictionary mapping message namespaces to a directory locations

    :returns: file path of ``.srv`` file in specified package, ``str``
    :raises: :exc:`MsgNotFound` If service file cannot be located.
    """
    return get_msg_file(package, base_type, search_path, ext=EXT_SRV)

def load_msg_by_type(msg_context, msg_type, search_path):
    """
    Load message specification for specified type.

    NOTE: this will register the message in the *msg_context*.
    
    :param msg_context: :class:`MsgContext` for finding loaded dependencies
    :param msg_type: relative or full message type.
    :param search_path: dictionary mapping message namespaces to a directory locations

    :returns: :class:`MsgSpec` instance, ``(str, MsgSpec)``
    :raises: :exc:`MsgNotFound` If message cannot be located.
    """
    log("load_msg_by_type(%s, %s)" % (msg_type, str(search_path)))
    if not isinstance(search_path, dict):
        raise ValueError("search_path must be a dictionary of {namespace: dirpath}")
    if msg_type == HEADER:
        msg_type = HEADER_FULL_NAME
    package_name, base_type = package_resource_name(msg_type)
    file_path = get_msg_file(package_name, base_type, search_path)
    log("file_path", file_path)

    spec = load_msg_from_file(msg_context, file_path, msg_type)
    msg_context.set_file(msg_type, file_path)
    return spec

def load_srv_by_type(msg_context, srv_type, search_path):
    """
    Load service specification for specified type.

    NOTE: services are *never* registered in a :class:`MsgContext`.
    
    :param msg_context: :class:`MsgContext` for finding loaded dependencies
    :param srv_type: relative or full message type.
    :param search_path: dictionary mapping message namespaces to a directory locations

    :returns: :class:`MsgSpec` instance, ``(str, MsgSpec)``
    :raises: :exc:`MsgNotFound` If message cannot be located.
    """
    log("load_srv_by_type(%s, %s)" % (srv_type, str(search_path)))
    if not isinstance(search_path, dict):
        raise ValueError("search_path must be a dictionary of {namespace: dirpath}")
    package_name, base_type = package_resource_name(srv_type)
    file_path = get_srv_file(package_name, base_type, search_path)
    log("file_path", file_path)
    return load_srv_from_file(msg_context, file_path, srv_type)

def convert_constant_value(field_type, val):
    """
    Convert constant value declaration to python value. Does not do
    type-checking, so ValueError or other exceptions may be raised.
    
    :param field_type: ROS field type, ``str``
    :param val: string representation of constant, ``str``
    :raises: :exc:`ValueError` If unable to convert to python representation
    :raises: :exc:`InvalidMsgSpec` If value exceeds specified integer width
    """
    if field_type in ['float32','float64']:
        return float(val)
    elif field_type in ['string']:
        return val.strip() #string constants are always stripped 
    elif field_type in ['int8', 'uint8', 'int16','uint16','int32','uint32','int64','uint64', 'char', 'byte']:
        # bounds checking
        bits = [('int8', 8), ('uint8', 8), ('int16', 16),('uint16', 16),\
                ('int32', 32),('uint32', 32), ('int64', 64),('uint64', 64),\
                ('byte', 8), ('char', 8)]
        b = [b for t, b in bits if t == field_type][0]
        import math
        if field_type[0] == 'u' or field_type == 'char':
            lower = 0
            upper = int(math.pow(2, b)-1)
        else:
            upper = int(math.pow(2, b-1)-1)   
            lower = -upper - 1 #two's complement min
        val = int(val) #python will autocast to long if necessary
        if val > upper or val < lower:
            raise InvalidMsgSpec("cannot coerce [%s] to %s (out of bounds)"%(val, field_type))
        return val
    elif field_type == 'bool':
        # TODO: need to nail down constant spec for bool
        return True if eval(val) else False
    raise InvalidMsgSpec("invalid constant type: [%s]"%field_type)

def _load_constant_line(orig_line):
    """
    :raises: :exc:`InvalidMsgSpec`
    """
    clean_line = _strip_comments(orig_line)
    line_splits = [s for s in [x.strip() for x in clean_line.split(" ")] if s] #split type/name, filter out empties
    field_type = line_splits[0]
    if not is_valid_constant_type(field_type):
        raise InvalidMsgSpec("%s is not a legal constant type"%field_type)

    if field_type == 'string':
        # strings contain anything to the right of the equals sign, there are no comments allowed
        idx = orig_line.find(CONSTCHAR)
        name = orig_line[orig_line.find(' ')+1:idx]
        val = orig_line[idx+1:]
    else:
        line_splits = [x.strip() for x in ' '.join(line_splits[1:]).split(CONSTCHAR)] #resplit on '='
        if len(line_splits) != 2:
            raise InvalidMsgSpec("Invalid constant declaration: %s"%l)
        name = line_splits[0]
        val = line_splits[1]

    try:
        val_converted = convert_constant_value(field_type, val)
    except Exception as e:
        raise InvalidMsgSpec("Invalid constant value: %s"%e)
    return Constant(field_type, name, val_converted, val.strip())

def _load_field_line(orig_line, package_context):
    """
    :returns: (field_type, name) tuple, ``(str, str)``
    :raises: :exc:`InvalidMsgSpec`
    """
    #log("_load_field_line", orig_line, package_context)
    clean_line = _strip_comments(orig_line)
    line_splits = [s for s in [x.strip() for x in clean_line.split(" ")] if s] #split type/name, filter out empties
    if len(line_splits) != 2:
        raise InvalidMsgSpec("Invalid declaration: %s"%(orig_line))
    field_type, name = line_splits
    if not is_valid_msg_field_name(name):
        raise InvalidMsgSpec("%s is not a legal message field name"%name)
    if not is_valid_msg_type(field_type):
        raise InvalidMsgSpec("%s is not a legal message field type"%field_type)
    if package_context and not SEP in field_type:
        if field_type == HEADER:
            field_type = HEADER_FULL_NAME
        elif not is_builtin(bare_msg_type(field_type)):
            field_type = "%s/%s"%(package_context, field_type)
    elif field_type == HEADER:
        field_type = HEADER_FULL_NAME
    return field_type, name

def _strip_comments(line):
    return line.split(COMMENTCHAR)[0].strip() #strip comments
    
def load_msg_from_string(msg_context, text, full_name):
    """
    Load message specification from a string.

    NOTE: this will register the message in the *msg_context*.
    
    :param msg_context: :class:`MsgContext` for finding loaded dependencies
    :param text: .msg text , ``str``
    :returns: :class:`MsgSpec` specification
    :raises: :exc:`InvalidMsgSpec` If syntax errors or other problems are detected in file
    """
    log("load_msg_from_string", full_name)
    package_name, short_name = package_resource_name(full_name)
    types = []
    names = []
    constants = []
    for orig_line in text.split('\n'):
        clean_line = _strip_comments(orig_line)
        if not clean_line:
            continue #ignore empty lines
        if CONSTCHAR in clean_line:
            constants.append(_load_constant_line(orig_line))
        else:
            field_type, name = _load_field_line(orig_line, package_name)
            types.append(field_type)
            names.append(name)
    spec = MsgSpec(types, names, constants, text, full_name, package_name)
    msg_context.register(full_name, spec)
    return spec

def load_msg_from_file(msg_context, file_path, full_name):
    """
    Convert the .msg representation in the file to a :class:`MsgSpec` instance.

    NOTE: this will register the message in the *msg_context*.
    
    :param file_path: path of file to load from, ``str``
    :returns: :class:`MsgSpec` instance
    :raises: :exc:`InvalidMsgSpec`: if syntax errors or other problems are detected in file
    """
    log("Load spec from", file_path)
    with open(file_path, 'r') as f:
        text = f.read()
    try:
        return load_msg_from_string(msg_context, text, full_name)
    except InvalidMsgSpec as e:
        raise InvalidMsgSpec('%s: %s'%(file_path, e))

def load_msg_depends(msg_context, spec, search_path):
    """
    Add the list of message types that spec depends on to depends.

    :param msg_context: :class:`MsgContext` instance to load dependencies into/from.
    :param spec: message to compute dependencies for, :class:`MsgSpec`/:class:`SrvSpec`
    :param search_path: dictionary mapping message namespaces to a directory locations
    :param deps: for recursion use only, do not set

    :returns: list of dependency names, ``[str]``
    :raises: :exc:`MsgNotFound` If dependency cannot be located.
    """
    package_context = spec.package
    log("load_msg_depends <spec>", spec.full_name, package_context)
    depends = []
    # Iterate over each field, loading as necessary
    for unresolved_type in spec.types:
        bare_type = bare_msg_type(unresolved_type)
        resolved_type = resolve_type(bare_type, package_context)
        if is_builtin(resolved_type):
            continue

        # Retrieve the MsgSpec instance of the field
        if msg_context.is_registered(resolved_type):
            depspec = msg_context.get_registered(resolved_type)
        else:
            # load and register on demand
            depspec = load_msg_by_type(msg_context, resolved_type, search_path)
            msg_context.register(resolved_type, depspec)

        # Update dependencies
        depends.append(resolved_type)
        #  - check to see if we have compute dependencies of field
        dep_dependencies = msg_context.get_depends(resolved_type)
        if dep_dependencies is None:
            load_msg_depends(msg_context, depspec, search_path)

    assert spec.full_name, "MsgSpec must have a properly set full name"
    msg_context.set_depends(spec.full_name, depends)
    # have to copy array in order to prevent inadvertent mutation (we've stored this list in set_dependencies)
    return depends[:]
            
def load_depends(msg_context, spec, msg_search_path):
    """
    Compute dependencies of *spec* and load their MsgSpec dependencies
    into *msg_context*.

    NOTE: *msg_search_path* is only for finding .msg files.  ``.srv``
    files have a separate and distinct search path.  As services
    cannot depend on other services, it is not necessary to provide
    the srv search path here.

    :param msg_context: :class:`MsgContext` instance to load dependencies into/from.
    :param spec: :class:`MsgSpec` or :class:`SrvSpec` instance to load dependencies for.
    :param msg_search_path: dictionary mapping message namespaces to a directory locations.
    :raises: :exc:`MsgNotFound` If dependency cannot be located.
    """
    if isinstance(spec, MsgSpec):
        return load_msg_depends(msg_context, spec, msg_search_path)
    elif isinstance(spec, SrvSpec):
        depends = load_msg_depends(msg_context, spec.request, msg_search_path)
        depends.extend(load_msg_depends(msg_context, spec.response, msg_search_path))
        return depends
    else:
        raise ValueError("spec does not appear to be a message or service")

class MsgContext(object):
    """
    Context object for storing :class:`MsgSpec` instances and related
    metadata.

    NOTE: All APIs work on :class:`MsgSpec` instance information.
    Thus, for services, there is information for the request and
    response messages, but there is no direct information about the
    :class:`SrvSpec` instance.
    """

    def __init__(self):
        self._registered_packages = {}
        self._files = {}
        self._dependencies = {}

    def set_file(self, full_msg_type, file_path):
        self._files[full_msg_type] = file_path
        
    def get_file(self, full_msg_type):
        return self._files.get(full_msg_type, None)

    def set_depends(self, full_msg_type, dependencies):
        """
        :param dependencies: direct first order
        dependencies for  *full_msg_type*
        """
        log("set_depends", full_msg_type, dependencies)
        self._dependencies[full_msg_type] = dependencies
    
    def get_depends(self, full_msg_type):
        """
        :returns: List of dependencies for *full_msg_type*,
          only first order dependencies
        """
        return self._dependencies.get(full_msg_type, None)

    def get_all_depends(self, full_msg_type):
        all_deps = []
        depends = self.get_depends(full_msg_type)
        if depends is None:
            raise KeyError(full_msg_type)
        for d in depends:
            all_deps.extend([d])
            all_deps.extend(self.get_all_depends(d))
        return all_deps

    @staticmethod
    def create_default():
        msg_context = MsgContext()
        # register builtins (needed for serialization).  builtins have no package.
        load_msg_from_string(msg_context, TIME_MSG, TIME)
        load_msg_from_string(msg_context, DURATION_MSG, DURATION)
        return msg_context
        
    def register(self, full_msg_type, msgspec):
        full_msg_type = bare_msg_type(full_msg_type)
        package, base_type = package_resource_name(full_msg_type)
        if package not in self._registered_packages:
            self._registered_packages[package] = {}
        self._registered_packages[package][base_type] = msgspec

    def is_registered(self, full_msg_type):
        """
        :param full_msg_type: Fully resolve message type
        :param default_package: default package namespace to resolve
          in.  May be ignored by special types (e.g. time/duration).
          
        :returns: ``True`` if :class:`MsgSpec` instance has been loaded for the requested type.
        """
        full_msg_type = bare_msg_type(full_msg_type)
        package, base_type = package_resource_name(full_msg_type)
        if package in self._registered_packages:
            return base_type in self._registered_packages[package]
        else:
            return False

    def get_registered(self, full_msg_type):
        """
        :raises: :exc:`KeyError` If not registered
        """
        full_msg_type = bare_msg_type(full_msg_type)
        if self.is_registered(full_msg_type):
            package, base_type = package_resource_name(full_msg_type)
            return self._registered_packages[package][base_type]
        else:
            raise KeyError(full_msg_type)

    def __str__(self):
        return str(self._registered_packages)

def load_srv_from_string(msg_context, text, full_name):
    """
    Load :class:`SrvSpec` from the .srv file.
    
    :param msg_context: :class:`MsgContext` instance to load request/response messages into.
    :param text: .msg text , ``str``
    :param package_name: context to use for msg type name, i.e. the package name,
      or '' to use local naming convention. ``str``
    :returns: :class:`SrvSpec` instance
    :raises :exc:`InvalidMsgSpec` If syntax errors or other problems are detected in file
    """
    text_in  = StringIO()
    text_out = StringIO()
    accum = text_in
    for l in text.split('\n'):
        l = l.split(COMMENTCHAR)[0].strip() #strip comments        
        if l.startswith(IODELIM): #lenient, by request
            accum = text_out
        else:
            accum.write(l+'\n')

    # create separate MsgSpec objects for each half of file
    msg_in = load_msg_from_string(msg_context, text_in.getvalue(), '%sRequest'%(full_name))
    msg_out = load_msg_from_string(msg_context, text_out.getvalue(), '%sResponse'%(full_name))
    return SrvSpec(msg_in, msg_out, text, full_name)

def load_srv_from_file(msg_context, file_path, full_name):
    """
    Convert the .srv representation in the file to a :class:`SrvSpec` instance.

    :param msg_context: :class:`MsgContext` instance to load request/response messages into.
    :param file_name: name of file to load from, ``str``
    :returns: :class:`SrvSpec` instance
    :raise: :exc:`InvalidMsgSpec` If syntax errors or other problems are detected in file
    """
    log("Load spec from %s %s\n"%(file_path, full_name))
    with open(file_path, 'r') as f:
        text = f.read()
    spec = load_srv_from_string(msg_context, text, full_name)
    msg_context.set_file('%sRequest'%(full_name), file_path)
    msg_context.set_file('%sResponse'%(full_name), file_path)
    return spec
