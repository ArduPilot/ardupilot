#!/usr/bin/env python
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

import os
import sys 
        
TEST_CTX = 'rosgraph_msgs'

def get_test_dir():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 'md5tests'))

def get_test_msg_dir():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 'files'))

def get_search_path():
    test_dir = get_test_msg_dir()
    search_path = {}
    for pkg in ['std_msgs', 'rosgraph_msgs', 'test_ros', 'geometry_msgs']:
        search_path[pkg] = [ os.path.join(test_dir, pkg, 'msg') ]
    return search_path

def _load_md5_tests(dir_name):
    test_dir = os.path.join(get_test_dir(), dir_name)
    tests = {}
    for f in os.listdir(test_dir):
        path = os.path.join(test_dir, f)
        if not f.endswith('.txt'):
            continue
        name = f[:-4]
        while name and name[-1].isdigit():
            name = name[:-1]
        assert bool(name)
        if name in tests:
            tests[name].append(path)
        else:
            tests[name] = [path]
    return tests
    
def _compute_md5(msg_context, f):
    from genmsg import load_depends, compute_md5
    from genmsg.msg_loader import load_msg_from_string

    text = open(f, 'r').read()
    short_name = os.path.basename(f)[:-len('.msg')]
    full_name = "%s/%s"%(TEST_CTX, short_name)
    spec = load_msg_from_string(msg_context, text, full_name)
    search_path = get_search_path()
    load_depends(msg_context, spec, search_path)
    return compute_md5(msg_context, spec)
        
def _compute_md5_text(msg_context, f):
    from genmsg import compute_md5_text, load_depends
    from genmsg.msg_loader import load_msg_from_string

    text = open(f, 'r').read()
    short_name = os.path.basename(f)[:-len('.msg')]
    full_name = "%s/%s"%(TEST_CTX, short_name)
    spec = load_msg_from_string(msg_context, text, full_name)
    search_path = get_search_path()
    load_depends(msg_context, spec, search_path)
    return compute_md5_text(msg_context, spec)

def test_compute_md5_text():
    from genmsg import MsgContext
    msg_context = MsgContext.create_default()
    
    # this test is just verifying that the md5sum is what it was for cturtle->electric
    Header_md5 = "2176decaecbce78abc3b96ef049fabed"
    rg_msg_dir = os.path.join(get_test_msg_dir(), TEST_CTX, 'msg')
    clock_msg = os.path.join(rg_msg_dir, 'Clock.msg')
    # a bit gory, but go ahead and regression test these important messages
    assert "time clock" == _compute_md5_text(msg_context, clock_msg)
    log_msg = os.path.join(rg_msg_dir, 'Log.msg')
    assert "byte DEBUG=1\nbyte INFO=2\nbyte WARN=4\nbyte ERROR=8\nbyte FATAL=16\n%s header\nbyte level\nstring name\nstring msg\nstring file\nstring function\nuint32 line\nstring[] topics"%Header_md5 == _compute_md5_text(msg_context, log_msg)

    tests = _load_md5_tests('md5text')
    # text file #1 is the reference
    for k, files in tests.items():
        print("running tests", k)
        ref_file = [f for f in files if f.endswith('%s1.txt'%k)]
        if not ref_file:
            assert False, "failed to load %s"%k
        ref_file = ref_file[0]
        ref_text = open(ref_file, 'r').read().strip()
        print("KEY", k)
        files = [f for f in files if not f.endswith('%s1.txt'%k)]
        for f in files[1:]:
            f_text = _compute_md5_text(msg_context, f)
            assert ref_text == f_text, "failed on %s\n%s\n%s: \n[%s]\nvs.\n[%s]\n"%(k, ref_file, f, ref_text, f_text)
        
def test_md5_equals():
    from genmsg import MsgContext
    msg_context = MsgContext.create_default()

    search_path = get_search_path()
    tests = _load_md5_tests('same')
    for k, files in tests.items():
        print("running tests", k)
        md5sum = _compute_md5(msg_context, files[0])
        for f in files[1:]:
            assert md5sum == _compute_md5(msg_context, f), "failed on %s: \n[%s]\nvs.\n[%s]\n"%(k, _compute_md5_text(msg_context, files[0]), _compute_md5_text(msg_context, f))
    
def test_md5_not_equals():
    from genmsg import MsgContext
    msg_context = MsgContext.create_default()

    tests = _load_md5_tests('different')
    for k, files in tests.items():
        print("running tests", k)
        md5s = set()
        md6md5sum = _compute_md5(msg_context, files[0])
        for f in files:
            md5s.add(_compute_md5(msg_context, f))
        # each md5 should be unique
        assert len(md5s) == len(files)
    
twist_with_covariance_stamped_full_text = """# This represents an estimate twist with reference coordinate frame and timestamp.
Header header
TwistWithCovariance twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertianty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into it's linear and angular parts. 
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z"""

log_full_text = """##
## Severity level constants
##
byte DEBUG=1 #debug level
byte INFO=2  #general level
byte WARN=4  #warning level
byte ERROR=8 #error level
byte FATAL=16 #fatal/critical level
##
## Fields
##
Header header
byte level
string name # name of the node
string msg # message 
string file # file the message came from
string function # function the message came from
uint32 line # line the message came from
string[] topics # topic names that the node publishes

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""

def test_compute_full_text():
    from genmsg import MsgContext, compute_full_text, load_msg_by_type, load_depends
    msg_context = MsgContext.create_default()

    search_path = get_search_path()
    
    # regression test against values used for cturtle-electric
    
    spec = load_msg_by_type(msg_context, 'rosgraph_msgs/Log', search_path)
    load_depends(msg_context, spec, search_path)
    val = compute_full_text(msg_context, spec)
    assert val == log_full_text, "[%s][%s]"%(val, log_full_text)

    spec = load_msg_by_type(msg_context, 'geometry_msgs/TwistWithCovarianceStamped', search_path)
    load_depends(msg_context, spec, search_path)
    val = compute_full_text(msg_context, spec)
    assert val == twist_with_covariance_stamped_full_text, "[%s][%s]"%(val, twist_with_covariance_stamped_full_text)
