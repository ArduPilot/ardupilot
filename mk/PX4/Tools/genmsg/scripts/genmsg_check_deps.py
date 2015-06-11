#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
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

import os
import sys

from genmsg import EXT_MSG, EXT_SRV, MsgContext
from genmsg.gentools import compute_full_type_name
from genmsg.msg_loader import load_msg_from_file, load_srv_from_file
from genmsg.msgs import bare_msg_type, is_builtin, resolve_type

pkg_name = sys.argv[1]
msg_file = sys.argv[2]
deps = sys.argv[3].split(':') if len(sys.argv) > 3 else []

msg_context = MsgContext.create_default()
full_type_name = compute_full_type_name(pkg_name, os.path.basename(msg_file))

if msg_file.endswith(EXT_MSG):
    spec = load_msg_from_file(msg_context, msg_file, full_type_name)
    unresolved_types = spec.types
elif msg_file.endswith(EXT_SRV):
    spec = load_srv_from_file(msg_context, msg_file, full_type_name)
    unresolved_types = spec.request.types + spec.response.types
else:
    print("Processing file: '%s' - unknown file extension" % msg_file, file=sys.stderr)
    sys.exit(1)

package_context = spec.package
for unresolved_type in unresolved_types:
    bare_type = bare_msg_type(unresolved_type)
    resolved_type = resolve_type(bare_type, package_context)
    if not is_builtin(resolved_type) and resolved_type not in deps:
        print("The dependencies of the message/service '%s' have changed. Please rerun cmake." % spec.full_name, file=sys.stderr)
        sys.exit(1)
