# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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

import os
import genmsg.msg_loader
import genmsg

# pkg_name - string
# msg_file - string full path
# search_paths -  dict of {'pkg':'msg_dir'}
def find_msg_dependencies_with_type(pkg_name, msg_file, search_paths):

    # Read and parse the source msg file
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(pkg_name, os.path.basename(msg_file))
    spec = genmsg.msg_loader.load_msg_from_file(msg_context, msg_file, full_type_name)

    try:
        genmsg.msg_loader.load_depends(msg_context, spec, search_paths)
    except genmsg.InvalidMsgSpec as e:
        raise genmsg.MsgGenerationException("Cannot read .msg for %s: %s"%(full_type_name, str(e)))

    deps = set()
    for dep_type_name in msg_context.get_all_depends(full_type_name):
        deps.add((dep_type_name, msg_context.get_file(dep_type_name)))

    return list(deps)


def find_msg_dependencies(pkg_name, msg_file, search_paths):
    deps = find_msg_dependencies_with_type(pkg_name, msg_file, search_paths)
    return [d[1] for d in deps]


def find_srv_dependencies_with_type(pkg_name, msg_file, search_paths):

    # Read and parse the source msg file
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(pkg_name, os.path.basename(msg_file))

    spec = genmsg.msg_loader.load_srv_from_file(msg_context, msg_file, full_type_name)

    try:
        genmsg.msg_loader.load_depends(msg_context, spec, search_paths)
    except genmsg.InvalidMsgSpec as e:
        raise genmsg.MsgGenerationException("Cannot read .msg for %s: %s"%(full_type_name, str(e)))

    deps = set()

    for dep_type_name in msg_context.get_all_depends(spec.request.full_name):
        deps.add((dep_type_name, msg_context.get_file(dep_type_name)))

    for dep_type_name in msg_context.get_all_depends(spec.response.full_name):
        deps.add((dep_type_name, msg_context.get_file(dep_type_name)))

    return list(deps)


def find_srv_dependencies(pkg_name, msg_file, search_paths):
    deps = find_srv_dependencies_with_type(pkg_name, msg_file, search_paths)
    return [d[1] for d in deps]

#paths = {'std_msgs':'/u/mkjargaard/repositories/mkjargaard/dist-sandbox/std_msgs/msg'}
#file = '/u/mkjargaard/repositories/mkjargaard/dist-sandbox/quux_msgs/msg/QuuxString.msg'
#find_msg_dependencies('quux_msgs', file, paths)
