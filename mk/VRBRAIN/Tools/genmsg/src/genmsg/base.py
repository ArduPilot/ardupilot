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

from __future__ import print_function
import os, sys

SEP = '/'

MSG_DIR = 'msg'
SRV_DIR = 'srv'

EXT_MSG = '.msg'
EXT_SRV = '.srv'

## character that designates a constant assignment rather than a field
CONSTCHAR   = '='
COMMENTCHAR = '#'
IODELIM   = '---'


verbose = False

import inspect, pprint

def log_verbose(value):
    global verbose
    verbose = value

def log(*args):
    global verbose
    if verbose:
        print("%s:%d" % inspect.stack()[1][1:3], file=sys.stderr)
        print(' '.join([str(x) for x in args]), file=sys.stderr)

def plog(msg, obj):
    if verbose:
        print("%s:%d" % inspect.stack()[1][1:3], file=sys.stderr)
        print(msg, " ", file=sys.stderr)
        pprint.pprint(obj, file=sys.stderr)

class InvalidMsgSpec(Exception):
    pass

class MsgGenerationException(Exception):
    pass

