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

PRN_SEPARATOR = '/'

import re

def normalize_package_context(package_context):
    package_context = package_context.strip()
    while package_context.endswith(PRN_SEPARATOR):
        package_context = package_context[:-1]
    return package_context
    
#######################################################################
# RESOURCE NAMES
# resource names refer to entities in a file system

def resource_name(res_pkg_name, name, my_pkg=None):
    """
    Convert package name + resource into a fully qualified resource name

    @param res_pkg_name: name of package resource is located in
    @type  res_pkg_name: str
    @param name: resource base name
    @type  name: str
    @param my_pkg: name of package resource is being referred to
        in. If specified, name will be returned in local form if 
        res_pkg_name is my_pkg
    @type  my_pkg: str
    @return: name for resource 
    @rtype: str
    """    
    if res_pkg_name != my_pkg:
        return res_pkg_name+PRN_SEPARATOR+name
    return name

def resource_name_base(name):
    """
    pkg/typeName -> typeName, typeName -> typeName
    
    Convert fully qualified resource name into the package-less resource name
    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: resource name sans package-name scope
    @rtype: str
    """    

    return name[name.rfind(PRN_SEPARATOR)+1:]

def resource_name_package(name):
    """
    pkg/typeName -> pkg, typeName -> None
    
    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: package name of resource
    @rtype: str
    """    

    if not PRN_SEPARATOR in name:
        return None
    return name[:name.find(PRN_SEPARATOR)]

def package_resource_name(name):
    """
    Split a name into its package and resource name parts, e.g. 'std_msgs/String -> std_msgs, String'

    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: package name, resource name
    @rtype: str
    @raise ValueError: if name is invalid
    """    
    if PRN_SEPARATOR in name:
        val = tuple(name.split(PRN_SEPARATOR))
        if len(val) != 2:
            raise ValueError("invalid name [%s]"%name)
        else:
            return val
    else:
        return '', name

################################################################################
# NAME VALIDATORS

#ascii char followed by (alphanumeric, _, /)
RESOURCE_NAME_LEGAL_CHARS_P = re.compile('^[A-Za-z][\w_\/]*$') 
def is_legal_resource_name(name):
    """
    Check if name is a legal ROS name for filesystem resources
    (alphabetical character followed by alphanumeric, underscore, or
    forward slashes). This constraint is currently not being enforced,
    but may start getting enforced in later versions of ROS.

    @param name: Name
    @type  name: str
    """
    # resource names can be unicode due to filesystem
    if name is None:
        return False
    m = RESOURCE_NAME_LEGAL_CHARS_P.match(name)
    # '//' check makes sure there isn't double-slashes
    return m is not None and m.group(0) == name and not '//' in name

BASE_RESOURCE_NAME_LEGAL_CHARS_P = re.compile('^[A-Za-z][\w_]*$') #ascii char followed by (alphanumeric, _)
def is_legal_resource_base_name(name):
    """
    Validates that name is a legal resource base name. A base name has
    no package context, e.g. "String".
    """
    # resource names can be unicode due to filesystem
    if name is None:
        return False
    m = BASE_RESOURCE_NAME_LEGAL_CHARS_P.match(name)
    return m is not None and m.group(0) == name

