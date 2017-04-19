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
ROS msg library for Python

Implements: U{http://ros.org/wiki/msg}
"""

import os
import sys

from . base import InvalidMsgSpec, EXT_MSG, MSG_DIR, SEP, log
from . names import is_legal_resource_name, is_legal_resource_base_name, package_resource_name, resource_name

#TODOXXX: unit test
def bare_msg_type(msg_type):
    """
    Compute the bare data type, e.g. for arrays, get the underlying array item type
    
    :param msg_type: ROS msg type (e.g. 'std_msgs/String'), ``str``
    :returns: base type, ``str``
    """
    if msg_type is None:
        return None
    if '[' in msg_type:
        return msg_type[:msg_type.find('[')]
    return msg_type

def resolve_type(msg_type, package_context):
    """
    Resolve type name based on current package context.

    NOTE: in ROS Diamondback, 'Header' resolves to
    'std_msgs/Header'. In previous releases, it resolves to
    'roslib/Header' (REP 100).

    e.g.::
      resolve_type('String', 'std_msgs') -> 'std_msgs/String'
      resolve_type('String[]', 'std_msgs') -> 'std_msgs/String[]'
      resolve_type('std_msgs/String', 'foo') -> 'std_msgs/String'    
      resolve_type('uint16', 'std_msgs') -> 'uint16'
      resolve_type('uint16[]', 'std_msgs') -> 'uint16[]'
    """
    bt = bare_msg_type(msg_type)
    if bt in BUILTIN_TYPES:
        return msg_type
    elif bt == HEADER:
        return HEADER_FULL_NAME
    elif SEP in msg_type:
        return msg_type
    else:
        return "%s%s%s"%(package_context, SEP, msg_type)    

#NOTE: this assumes that we aren't going to support multi-dimensional

def parse_type(msg_type):
    """
    Parse ROS message field type
    :param msg_type: ROS field type, ``str``
    :returns: base_type, is_array, array_length, ``(str, bool, int)``
    :raises: :exc:`ValueError` If *msg_type* cannot be parsed
    """
    if not msg_type:
        raise ValueError("Invalid empty type")
    if '[' in msg_type:
        var_length = msg_type.endswith('[]')
        splits = msg_type.split('[')
        if len(splits) > 2:
            raise ValueError("Currently only support 1-dimensional array types: %s"%msg_type)
        if var_length:
            return msg_type[:-2], True, None
        else:
            try:
                length = int(splits[1][:-1])
                return splits[0], True, length
            except ValueError:
                raise ValueError("Invalid array dimension: [%s]"%splits[1][:-1])
    else:
        return msg_type, False, None
   
################################################################################
# name validation 

def is_valid_msg_type(x):
    """
    :returns: True if the name is a syntatically legal message type name, ``bool``
    """
    if not x or len(x) != len(x.strip()):
        return False
    base = bare_msg_type(x)
    if not is_legal_resource_name(base):
        return False
    #parse array indices
    x = x[len(base):]
    state = 0
    i = 0
    for c in x:
        if state == 0:
            if c != '[':
                return False
            state = 1 #open
        elif state == 1:
            if c == ']':
                state = 0 #closed
            else:
                try:
                    int(c)
                except:
                    return False
    return state == 0

def is_valid_constant_type(x):
    """
    :returns: ``True`` if the name is a legal constant type. Only simple types are allowed, ``bool``
    """
    return x in PRIMITIVE_TYPES

def is_valid_msg_field_name(x):
    """
    :returns: ``True`` if the name is a syntatically legal message field name, ``bool``
    """
    return is_legal_resource_base_name(x)

# msg spec representation ##########################################

class Constant(object):
    """
    Container class for holding a Constant declaration

    Attributes:

    - ``type``
    - ``name``
    - ``val``    
    - ``val_text`` 
    """
    __slots__ = ['type', 'name', 'val', 'val_text']
    
    def __init__(self, type_, name, val, val_text):
        """
        :param type_: constant type, ``str``
        :param name: constant name, ``str``
        :param val: constant value, ``str``
        :param val_text: Original text definition of *val*, ``str``
        """
        if type is None or name is None or val is None or val_text is None:
            raise ValueError('Constant must have non-None parameters')
        self.type = type_
        self.name = name.strip() #names are always stripped of whitespace
        self.val = val
        self.val_text = val_text

    def __eq__(self, other):
        if not isinstance(other, Constant):
            return False
        return self.type == other.type and self.name == other.name and self.val == other.val

    def __repr__(self):
        return "%s %s=%s"%(self.type, self.name, self.val)

    def __str__(self):
        return "%s %s=%s"%(self.type, self.name, self.val)

class Field(object):
    """
    Container class for storing information about a single field in a MsgSpec
    
    Attributes:
    
    - ``name``
    - ``type``
    - ``base_type``
    - ``is_array``
    - ``array_len``
    - ``is_builtin``
    - ``is_header``
    """
    
    def __init__(self, name, type):
        self.name = name
        self.type = type
        (self.base_type, self.is_array, self.array_len) = parse_type(type)
        self.is_header = is_header_type(self.type)
        self.is_builtin = is_builtin(self.base_type)

    def __eq__(self, other):
        if not isinstance(other, Field):
            return False
        else:
            return self.name == other.name and \
                   self.type == other.type
    
    def __repr__(self):
        return "[%s, %s, %s, %s, %s]"%(self.name, self.type, self.base_type, self.is_array, self.array_len)

class MsgSpec(object):
    """
    Container class for storing loaded msg description files. Field
    types and names are stored in separate lists with 1-to-1
    correspondence. MsgSpec can also return an md5 of the source text.
    """

    def __init__(self, types, names, constants, text, full_name, package = '', short_name = ''):
        """
        :param types: list of field types, in order of declaration, ``[str]]``
        :param names: list of field names, in order of declaration, ``[str]]``
        :param constants: List of :class:`Constant` declarations, ``[Constant]``
        :param text: text of declaration, ``str`
        :raises: :exc:`InvalidMsgSpec` If spec is invalid (e.g. fields with the same name)
        """
        alt_package, alt_short_name = package_resource_name(full_name)
        if not package:
            package = alt_package
        if not short_name:
            short_name = alt_short_name
            
        self.types = types
        if len(set(names)) != len(names):
            raise InvalidMsgSpec("Duplicate field names in message: %s"%names)
        self.names = names
        self.constants = constants
        assert len(self.types) == len(self.names), "len(%s) != len(%s)"%(self.types, self.names)
        #Header.msg support
        if (len(self.types)):
            self.header_present = self.types[0] == HEADER_FULL_NAME and self.names[0] == 'header'
        else:
            self.header_present = False
        self.text = text
        self.full_name = full_name
        self.short_name = short_name
        self.package = package
        try:
            self._parsed_fields = [Field(name, type) for (name, type) in zip(self.names, self.types)]
        except ValueError as e:
            raise InvalidMsgSpec("invalid field: %s"%(e))
        
    def fields(self):
        """
        :returns: zip list of types and names (e.g. [('int32', 'x'), ('int32', 'y')], ``[(str,str),]``
        """
        return list(zip(self.types, self.names)) #py3k
    
    def parsed_fields(self):
        """
        :returns: list of :class:`Field` classes, ``[Field,]``
        """
        return self._parsed_fields

    def has_header(self):
        """
        :returns: ``True`` if msg decription contains a 'Header header'
          declaration at the beginning, ``bool``
        """
        return self.header_present
    
    def __eq__(self, other):
        if not other or not isinstance(other, MsgSpec):
            return False 
        return self.types == other.types and self.names == other.names and \
               self.constants == other.constants and self.text == other.text and \
               self.full_name == other.full_name and self.short_name == other.short_name and \
               self.package == other.package

    def __ne__(self, other):
        if not other or not isinstance(other, MsgSpec):
            return True
        return not self.__eq__(other)

    def __repr__(self):
        if self.constants:
            return "MsgSpec[%s, %s, %s]"%(repr(self.constants), repr(self.types), repr(self.names))
        else:
            return "MsgSpec[%s, %s]"%(repr(self.types), repr(self.names))        

    def __str__(self):
        return self.text
    
# .msg file routines ##############################################################       

# adjustable constants, in case we change our minds
HEADER   = 'Header'
TIME     = 'time'
DURATION = 'duration'
HEADER_FULL_NAME = 'std_msgs/Header'

def is_header_type(msg_type):
    """
    :param msg_type: message type name, ``str``
    :returns: ``True`` if *msg_type* refers to the ROS Header type, ``bool``
    """
    # for backwards compatibility, include roslib/Header. REP 100
    return msg_type in [HEADER, HEADER_FULL_NAME, 'roslib/Header']
       
# time and duration types are represented as aggregate data structures
# for the purposes of serialization from the perspective of
# roslib.msgs. genmsg_py will do additional special handling is required
# to convert them into rospy.msg.Time/Duration instances.

## time as msg spec. time is unsigned 
TIME_MSG     = "uint32 secs\nuint32 nsecs"
## duration as msg spec. duration is just like time except signed
DURATION_MSG = "int32 secs\nint32 nsecs"

## primitive types are those for which we allow constants, i.e. have  primitive representation
PRIMITIVE_TYPES = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64',
                   'string',
                   'bool',
                   # deprecated:
                   'char','byte']
BUILTIN_TYPES = PRIMITIVE_TYPES + [TIME, DURATION]

def is_builtin(msg_type_name):
    """
    :param msg_type_name: name of message type, ``str``
    :returns: True if msg_type_name is a builtin/primitive type, ``bool``
    """
    return msg_type_name in BUILTIN_TYPES
