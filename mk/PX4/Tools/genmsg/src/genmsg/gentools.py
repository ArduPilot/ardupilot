#! /usr/bin/env python
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

"""
Library for supporting message and service generation for all ROS
client libraries. This is mainly responsible for calculating the
md5sums and message definitions of classes.
"""

# NOTE: this should not contain any rospy-specific code. The rospy
# generator library is rospy.genpy.

import sys
import hashlib

try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x

from . import msgs

from .msgs import InvalidMsgSpec, MsgSpec, bare_msg_type, is_builtin
from .msg_loader import load_depends
from .srvs import SrvSpec
from . import names
from . import base

def compute_md5_text(msg_context, spec):
    """
    Compute the text used for md5 calculation. MD5 spec states that we
    removes comments and non-meaningful whitespace. We also strip
    packages names from type names. For convenience sake, constants are
    reordered ahead of other declarations, in the order that they were
    originally defined.

    :returns: text for ROS MD5-processing, ``str``
    """
    package = spec.package

    buff = StringIO()    

    for c in spec.constants:
        buff.write("%s %s=%s\n"%(c.type, c.name, c.val_text))
    for type_, name in zip(spec.types, spec.names):
        msg_type = bare_msg_type(type_)
        # md5 spec strips package names
        if is_builtin(msg_type):
            buff.write("%s %s\n"%(type_, name))
        else:
            # recursively generate md5 for subtype.  have to build up
            # dependency representation for subtype in order to
            # generate md5
            sub_pkg, _ = names.package_resource_name(msg_type)
            sub_pkg = sub_pkg or package
            sub_spec = msg_context.get_registered(msg_type)
            sub_md5 = compute_md5(msg_context, sub_spec)
            buff.write("%s %s\n"%(sub_md5, name))
    
    return buff.getvalue().strip() # remove trailing new line

def _compute_hash(msg_context, spec, hash):
    """
    subroutine of compute_md5()

    :param msg_context: :class:`MsgContext` instance to load dependencies into/from.
    :param spec: :class:`MsgSpec` to compute hash for.
    :param hash: hash instance  
    """
    # accumulate the hash
    # - root file
    if isinstance(spec, MsgSpec):
        hash.update(compute_md5_text(msg_context, spec).encode())
    elif isinstance(spec, SrvSpec):
        hash.update(compute_md5_text(msg_context, spec.request).encode())
        hash.update(compute_md5_text(msg_context, spec.response).encode())
    else:
        raise Exception("[%s] is not a message or service"%spec)   
    return hash.hexdigest()

def compute_md5(msg_context, spec):
    """
    Compute md5 hash for message/service

    :param msg_context: :class:`MsgContext` instance to load dependencies into/from.
    :param spec: :class:`MsgSpec` to compute md5 for.
    :returns: md5 hash, ``str``
    """
    return _compute_hash(msg_context, spec, hashlib.md5())

## alias
compute_md5_v2 = compute_md5

def _unique_deps(dep_list):
    uniques = []
    for d in dep_list:
        if d not in uniques:
            uniques.append(d)
    return uniques
        
def compute_full_text(msg_context, spec):
    """
    Compute full text of message/service, including text of embedded
    types.  The text of the main msg/srv is listed first. Embedded
    msg/srv files are denoted first by an 80-character '=' separator,
    followed by a type declaration line,'MSG: pkg/type', followed by
    the text of the embedded type.

    :param msg_context: :class:`MsgContext` instance to load dependencies into/from.
    :param spec: :class:`MsgSpec` to compute full text for.
    :returns: concatenated text for msg/srv file and embedded msg/srv types, ``str``
    """
    buff = StringIO()
    sep = '='*80+'\n'

    # write the text of the top-level type
    buff.write(spec.text)
    buff.write('\n')    
    # append the text of the dependencies (embedded types).  Can't use set() as we have to preserve order.
    for d in _unique_deps(msg_context.get_all_depends(spec.full_name)):
        buff.write(sep)
        buff.write("MSG: %s\n"%d)
        buff.write(msg_context.get_registered(d).text)
        buff.write('\n')
    # #1168: remove the trailing \n separator that is added by the concatenation logic
    return buff.getvalue()[:-1]

def compute_full_type_name(package_name, file_name):
    """
    Compute the full type name of message/service 'pkg/type'.

    :param package_name: name of package file is in, ``str``
    :file_name: name of the msg og srv file, ``str``
    :returns: typename in format 'pkg/type'
    :raises: :exc:`MsgGenerationException` if file_name ends with an unknown file extension
    """
    # strip extension
    for ext in (base.EXT_MSG, base.EXT_SRV):
        if file_name.endswith(ext):
            short_name = file_name[:-len(ext)]
            break
    else:
        raise base.MsgGenerationException("Processing file: '%s' - unknown file extension"% (file_name))
    return "%s/%s"%(package_name, short_name)

