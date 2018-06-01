# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import random

def get_test_dir():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 'files'))

def test_exceptions():
    from genmsg import MsgNotFound
    try:
        raise MsgNotFound('hello')
    except MsgNotFound:
        pass

def test__convert_constant_value():
    from genmsg.msg_loader import convert_constant_value
    from genmsg import InvalidMsgSpec
    assert 0. == convert_constant_value('float32', '0.0')
    assert 0. == convert_constant_value('float64', '0.0')
    
    assert 'fo o' == convert_constant_value('string', '   fo o ')

    assert 1 == convert_constant_value('byte', '1')
    assert 1 == convert_constant_value('char', '1')
    assert 1 == convert_constant_value('int8', '1')
    assert 12 == convert_constant_value('int16', '12')
    assert -13 == convert_constant_value('int32', '-13')
    assert 14 == convert_constant_value('int64', '14')
    assert 0 == convert_constant_value('uint8', '0')
    assert 18 == convert_constant_value('uint16', '18')
    assert 19 == convert_constant_value('uint32', '19')
    assert 20 == convert_constant_value('uint64', '20')

    assert True == convert_constant_value('bool', '1')
    assert False == convert_constant_value('bool', '0')    

    width_fail = [('int8', '129'), ('uint8', '256'),
                  ('int16', '35536'), ('uint16', '-1'),('uint16', '65536'),
                  ('int32', '3000000000'),('int32', '-2700000000'),
                  ('uint32', '-1'),('uint32', '41000000000'),
                  ('uint64', '-1')]
    for t, v in width_fail:
        try:
            convert_constant_value(t, v)
            assert False, "should have failed width check: %s, %s"%(t, v)
        except InvalidMsgSpec:
            pass
    type_fail = [('int32', 'f'), ('float32', 'baz')]
    for t, v in type_fail:
        try:
            convert_constant_value(t, v)
            assert False, "should have failed type check: %s, %s"%(t, v)
        except ValueError:
            pass
    try:
        convert_constant_value('foo', '1')
        assert False, "should have failed invalid type"
    except InvalidMsgSpec:
        pass

def test__load_constant_line():
    from genmsg.msgs import Constant, InvalidMsgSpec
    from genmsg.msg_loader import _load_constant_line
    try:
        _load_constant_line("int8 field=alpha")
        assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    try:
        _load_constant_line("int8 field=")
        assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    try:
        _load_constant_line("faketype field=1")
        assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    
    c = _load_constant_line("int8 field=1")
    assert c == Constant('int8', 'field', 1, '1')
    c = _load_constant_line("string val=hello #world")
    assert c == Constant('string', 'val', 'hello #world', 'hello #world')
    
def test__load_field_line():
    from genmsg.msgs import InvalidMsgSpec, Field
    from genmsg.msg_loader import _load_field_line, InvalidMsgSpec, Field, is_valid_msg_field_name
    try:
       _load_field_line("string", 'foo')
       assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    assert not is_valid_msg_field_name('string[')
    try:
       _load_field_line("string data!", 'foo')
       assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    try:
       _load_field_line("string[ data", 'foo')
       assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    
    f =_load_field_line("string str", 'foo')
    assert f == ('string', 'str')
    
    f =_load_field_line("string str #nonsense", 'foo')
    assert f == ('string', 'str')

    f =_load_field_line("String str #nonsense", '')
    assert f == ('String', 'str')
    f =_load_field_line("String str #nonsense", 'foo')
    assert f == ('foo/String', 'str')

    # make sure Header is mapped
    f =_load_field_line("Header header #nonsense", 'somewhere')
    assert f == ('std_msgs/Header', 'header'), f
    f =_load_field_line("Header header #nonsense", '')
    assert f == ('std_msgs/Header', 'header'), f

def test_load_msg_from_string():
    # make sure Header -> std_msgs/Header conversion works
    from genmsg.msgs import Constant
    from genmsg.msg_loader import load_msg_from_string, MsgContext
    context = MsgContext.create_default()
    msgspec = load_msg_from_string(context, "Header header", 'test_pkg/HeaderTest')
    print(msgspec)
    assert msgspec.has_header()
    assert msgspec.types == ['std_msgs/Header']
    assert msgspec.names == ['header']
    assert msgspec.constants == []
    assert msgspec.short_name == 'HeaderTest'
    assert msgspec.package == 'test_pkg'
    assert msgspec.full_name == 'test_pkg/HeaderTest'
    
    msgspec = load_msg_from_string(context, "int8 c=1\nHeader header\nint64 data", 'test_pkg/HeaderValsTest')
    assert msgspec.has_header()
    assert msgspec.types == ['std_msgs/Header', 'int64']
    assert msgspec.names == ['header', 'data']
    assert msgspec.constants == [Constant('int8', 'c', 1, '1')]
    assert msgspec.short_name == 'HeaderValsTest'
    assert msgspec.package == 'test_pkg'
    assert msgspec.full_name == 'test_pkg/HeaderValsTest'
    
    msgspec = load_msg_from_string(context, "string data\nint64 data2", 'test_pkg/ValsTest')
    assert not msgspec.has_header()
    assert msgspec.types == ['string', 'int64']
    assert msgspec.names == ['data', 'data2']
    assert msgspec.constants == []
    assert msgspec.short_name == 'ValsTest'

    assert msgspec.full_name == 'test_pkg/ValsTest'

def _validate_TestString(msgspec):
    assert ['caller_id', 'orig_caller_id', 'data'] == msgspec.names, msgspec.names
    assert ['string', 'string', 'string'] == msgspec.types, msgspec.types

def test_load_msg_from_file():
    from genmsg.msgs import InvalidMsgSpec
    from genmsg.msg_loader import load_msg_from_file, MsgContext
    test_d = get_test_dir()
    test_ros_dir = os.path.join(test_d, 'test_ros', 'msg')
    test_string_path = os.path.join(test_ros_dir, 'TestString.msg')

    msg_context = MsgContext.create_default()
    spec = load_msg_from_file(msg_context, test_string_path, 'test_ros/TestString')
    assert spec.full_name == 'test_ros/TestString'
    assert spec.package == 'test_ros'
    assert spec.short_name == 'TestString'
    _validate_TestString(spec)

    # test repeat
    spec_2 = load_msg_from_file(msg_context, test_string_path, 'test_ros/TestString')
    assert spec == spec_2
    assert spec.package == spec_2.package
    assert spec.short_name == spec_2.short_name
    
    # test w/ bad file
    test_bad_path = os.path.join(test_ros_dir, 'Bad.msg')
    try:
        load_msg_from_file(msg_context, test_bad_path, 'test_ros/Bad')
        assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    
    # supposed to register
    assert msg_context.is_registered('test_ros/TestString'), msg_context
    
def test_load_msg_from_string_TestString():
    from genmsg.msg_loader import load_msg_from_string, MsgContext

    test_d = get_test_dir()
    test_ros_dir = os.path.join(test_d, 'test_ros', 'msg')
    test_string_path = os.path.join(test_ros_dir, 'TestString.msg')
    with open(test_string_path) as f:
        text = f.read()

    msg_context = MsgContext.create_default()
    _validate_TestString(load_msg_from_string(msg_context, text, 'test_ros/TestString'))
    # supposed to register
    assert msg_context.is_registered('test_ros/TestString'), msg_context

def test_load_msg_by_type():
    from genmsg.msg_loader import load_msg_by_type, MsgContext, MsgNotFound
    
    test_d = get_test_dir()
    geometry_d = os.path.join(test_d, 'geometry_msgs', 'msg')

    test_ros_dir = os.path.join(test_d, 'test_ros', 'msg')
    test_string_path = os.path.join(test_ros_dir, 'TestString.msg')
    search_path = {
        'test_ros': [ test_ros_dir ],
        'geometry_msgs': [ geometry_d ],
        }
    msg_context = MsgContext.create_default()
    msgspec = load_msg_by_type(msg_context, 'test_ros/TestString', search_path)
    _validate_TestString(msgspec)
    # supposed to register
    assert msg_context.is_registered('test_ros/TestString'), msg_context

    # test invalid search path
    try:
        load_msg_by_type(msg_context, 'test_ros/TestString', [test_string_path])
        assert False, "should have raised"
    except ValueError:
        pass
    # test not found
    try:
        load_msg_by_type(msg_context, 'test_ros/Fake', search_path)
        assert False, "should have raised"
    except MsgNotFound:
        pass

    # test all the known geometry msgs
    test_d = get_test_dir()
    for f in os.listdir(geometry_d):
        if f.endswith('.msg'):
            short = f[:-4]
            msg_type = 'geometry_msgs/%s'%short
            spec = load_msg_by_type(msg_context, msg_type, search_path)
            assert spec is not None
            assert spec.package == 'geometry_msgs'
            assert spec.full_name == msg_type
            assert spec.short_name == short
            with open(os.path.join(geometry_d, f)) as file_h:
                assert spec.text == file_h.read()
            # all types with 'Stamped' in name have headers
            if 'Stamped' in f:
                assert spec.has_header(), msg_type
    
def test_get_msg_file():
    from genmsg import MsgNotFound
    from genmsg.msg_loader import get_msg_file
    test_d = get_test_dir()
    test_ros_dir = os.path.join(test_d, 'test_ros', 'msg')
    test_string_path = os.path.join(test_ros_dir, 'TestString.msg')
    search_path = {
        'test_ros': [ test_ros_dir ],
        }
    assert test_string_path == get_msg_file('test_ros', 'TestString', search_path)
    try:
        get_msg_file('test_ros', 'DNE', search_path)
        assert False, "should have raised"
    except MsgNotFound:
        pass
    try:
        get_msg_file('bad_pkg', 'TestString', search_path)
        assert False, "should have raised"
    except MsgNotFound:
        pass

    # test with invalid search path
    try:
        get_msg_file('test_ros', 'TestString', [test_string_path])
        assert False, "should have raised"
    except ValueError:
        pass

def test_get_srv_file():
    from genmsg import MsgNotFound
    from genmsg.msg_loader import get_srv_file
    test_d = get_test_dir()
    test_ros_dir = os.path.join(test_d, 'test_ros', 'srv')
    std_srvs_dir = os.path.join(test_d, 'std_srvs', 'srv')
    empty_path = os.path.join(std_srvs_dir, 'Empty.srv')
    search_path = {
        'test_ros': [ test_ros_dir ],
        'std_srvs': [ std_srvs_dir ],
        }
    assert empty_path == get_srv_file('std_srvs', 'Empty', search_path)
    try:
        get_srv_file('test_ros', 'DNE', search_path)
        assert False, "should have raised"
    except MsgNotFound:
        pass
    try:
        get_srv_file('bad_pkg', 'TestString', search_path)
        assert False, "should have raised"
    except MsgNotFound:
        pass

    # test with invalid search path
    try:
        get_srv_file('std_srvs', 'Empty', [std_srvs_dir])
        assert False, "should have raised"
    except ValueError:
        pass

def test_MsgContext():
    from genmsg.msg_loader import MsgContext, load_msg_from_file
    msg_context = MsgContext()
    assert not msg_context.is_registered('time')
    assert not msg_context.is_registered('duration')
    
    msg_context = MsgContext.create_default()
    # tripwires
    repr(msg_context)
    str(msg_context)

    assert msg_context.is_registered('time'), msg_context._registered_packages
    assert msg_context.is_registered('duration')

    assert not msg_context.is_registered('test_ros/TestString')
    assert not msg_context.is_registered('Header')
    
    # start loading stuff into context
    test_d = get_test_dir()
    test_ros_dir = os.path.join(test_d, 'test_ros', 'msg')
    test_string_path = os.path.join(test_ros_dir, 'TestString.msg')
    spec = load_msg_from_file(msg_context, test_string_path, 'test_ros/TestString')
    msg_context.register('test_ros/TestString', spec)
    assert msg_context.get_registered('test_ros/TestString') == spec
    try:
        msg_context.get_registered('bad/TestString')
        assert False, 'should have raised'
    except KeyError:
        pass
    
    assert msg_context.is_registered('test_ros/TestString')

    # test Header
    assert not msg_context.is_registered('Header')
    assert not msg_context.is_registered('std_msgs/Header')
    
    msg_context.register('std_msgs/Header', spec)
    assert msg_context.is_registered('std_msgs/Header')

def test_load_srv_from_file():
    from genmsg.msg_loader import MsgContext, load_srv_from_file
        
    msg_context = MsgContext.create_default()
    
    d = get_test_dir()
    filename = os.path.join(d, 'test_ros', 'srv', 'AddTwoInts.srv')
    with open(filename, 'r') as f:
        text = f.read()
        
    full_name = 'test_ros/AddTwoInts'
    spec = load_srv_from_file(msg_context, filename, full_name)
    assert spec == load_srv_from_file(msg_context, filename, full_name)
    assert ['int64', 'int64'] == spec.request.types, spec.request.types
    assert ['a', 'b'] == spec.request.names
    assert text == spec.text
    assert full_name == spec.full_name

def test_load_msg_depends():
    #TODO: should there just be a 'load_msg, implicit=True?'
    from genmsg.msg_loader import MsgContext, load_msg_by_type, load_msg_depends, MsgNotFound
    test_d = get_test_dir()
    search_path = {
        'test_ros': [ os.path.join(test_d, 'test_ros', 'msg') ],
        'std_msgs': [ os.path.join(test_d, 'std_msgs', 'msg') ],
        'geometry_msgs': [ os.path.join(test_d, 'geometry_msgs', 'msg') ],
        'sensor_msgs': [ os.path.join(test_d, 'sensor_msgs', 'msg') ],
        'invalid': [ os.path.join(test_d, 'invalid', 'msg') ],
        }
    
    # Test not found
    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'invalid/BadDepend', search_path)
    try:
        load_msg_depends(msg_context, root_spec, search_path)
        assert False, "should have raised MsgNotFound"
    except MsgNotFound:
        pass
    root_spec = load_msg_by_type(msg_context, 'invalid/BadLocalDepend', search_path)
    try:
        load_msg_depends(msg_context, root_spec, search_path)
        assert False, "should have raised MsgNotFound"
    except MsgNotFound:
        pass

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'std_msgs/Int32', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'std_msgs', 'msg', 'Int32.msg')
    assert file_p == msg_context.get_file('std_msgs/Int32')
    assert [] == msg_context.get_depends('std_msgs/Int32')

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'std_msgs/Header', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'std_msgs', 'msg', 'Header.msg')
    assert file_p == msg_context.get_file('std_msgs/Header')
    assert [] == msg_context.get_depends('std_msgs/Header')

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'Header', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'std_msgs', 'msg', 'Header.msg')
    assert file_p == msg_context.get_file('std_msgs/Header')
    assert [] == msg_context.get_depends('std_msgs/Header')

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'std_msgs/Int32MultiArray', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'std_msgs', 'msg', 'Int32MultiArray.msg')
    assert file_p == msg_context.get_file('std_msgs/Int32MultiArray')
    val = msg_context.get_all_depends('std_msgs/Int32MultiArray')
    assert set(['std_msgs/MultiArrayLayout', 'std_msgs/MultiArrayDimension']) == set(val), val
    assert 2 == len(val), val

    val = msg_context.get_depends('std_msgs/Int32MultiArray')
    assert set(['std_msgs/MultiArrayLayout']) == set(val), val
    for s in ['MultiArrayLayout', 'MultiArrayDimension']:
        file_p = os.path.join(test_d, 'std_msgs', 'msg', '%s.msg'%s)
        assert file_p == msg_context.get_file('std_msgs/%s'%s)

def test_load_msg_depends_stamped():
    #TODO: should there just be a 'load_msg, implicit=True?'
    from genmsg.msg_loader import MsgContext, load_msg_by_type, load_msg_depends
    test_d = get_test_dir()
    geometry_d = os.path.join(test_d, 'geometry_msgs', 'msg')
    search_path = {
        'test_ros': [ os.path.join(test_d, 'test_ros', 'msg') ],
        'std_msgs': [ os.path.join(test_d, 'std_msgs', 'msg') ],
        'geometry_msgs': [ geometry_d ],
        'sensor_msgs': [ os.path.join(test_d, 'sensor_msgs', 'msg') ],
        }

    # Test with Stamped and deeper hierarchies, Header

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'geometry_msgs/PoseStamped', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'geometry_msgs', 'msg', 'PoseStamped.msg')
    assert file_p == msg_context.get_file('geometry_msgs/PoseStamped')
    val = msg_context.get_all_depends('geometry_msgs/PoseStamped')
    assert set(['std_msgs/Header', 'geometry_msgs/Pose', 'geometry_msgs/Point', 'geometry_msgs/Quaternion']) == set(val), val
    val = msg_context.get_depends('geometry_msgs/PoseStamped')
    assert set(['std_msgs/Header', 'geometry_msgs/Pose']) == set(val), val
    for s in ['Header']:
        file_p = os.path.join(test_d, 'std_msgs', 'msg', '%s.msg'%s)
        assert file_p == msg_context.get_file('std_msgs/%s'%s)
    for s in ['Pose', 'Point', 'Quaternion']:
        file_p = os.path.join(geometry_d, '%s.msg'%s)
        assert file_p == msg_context.get_file('geometry_msgs/%s'%s)

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'geometry_msgs/TwistWithCovarianceStamped', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'geometry_msgs', 'msg', 'TwistWithCovarianceStamped.msg')
    assert file_p == msg_context.get_file('geometry_msgs/TwistWithCovarianceStamped')
    val = msg_context.get_all_depends('geometry_msgs/TwistWithCovarianceStamped')
    assert set(['std_msgs/Header', 'geometry_msgs/TwistWithCovariance', 'geometry_msgs/Twist', 'geometry_msgs/Vector3']) == set(val), val
    val = msg_context.get_depends('geometry_msgs/TwistWithCovarianceStamped')
    assert set(['std_msgs/Header', 'geometry_msgs/TwistWithCovariance']) == set(val), val

    for s in ['Header']:
        file_p = os.path.join(test_d, 'std_msgs', 'msg', '%s.msg'%s)
        assert file_p == msg_context.get_file('std_msgs/%s'%s)
    for s in ['TwistWithCovariance', 'Twist', 'Vector3']:
        file_p = os.path.join(geometry_d, '%s.msg'%s)
        assert file_p == msg_context.get_file('geometry_msgs/%s'%s)

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'sensor_msgs/Imu', search_path)
    load_msg_depends(msg_context, root_spec, search_path)
    file_p = os.path.join(test_d, 'sensor_msgs', 'msg', 'Imu.msg')
    assert file_p == msg_context.get_file('sensor_msgs/Imu')
    val = msg_context.get_all_depends('sensor_msgs/Imu')
    assert set(['std_msgs/Header', 'geometry_msgs/Quaternion', 'geometry_msgs/Vector3']) == set(val), val
    val = msg_context.get_depends('sensor_msgs/Imu')
    assert set(['std_msgs/Header', 'geometry_msgs/Quaternion', 'geometry_msgs/Vector3']) == set(val), val
    for s in ['Header']:
        file_p = os.path.join(test_d, 'std_msgs', 'msg', '%s.msg'%s)
        assert file_p == msg_context.get_file('std_msgs/%s'%s)
    for s in ['Quaternion', 'Vector3']:
        file_p = os.path.join(geometry_d, '%s.msg'%s)
        assert file_p == msg_context.get_file('geometry_msgs/%s'%s)


def test_load_depends_msg():
    from genmsg.msg_loader import MsgContext, load_msg_by_type, load_depends, MsgNotFound, load_srv_by_type
    test_d = get_test_dir()
    geometry_d = os.path.join(test_d, 'geometry_msgs', 'msg')
    msg_search_path = {
        'test_ros': [ os.path.join(test_d, 'test_ros', 'msg') ],
        'std_msgs': [ os.path.join(test_d, 'std_msgs', 'msg') ],
        'geometry_msgs': [ geometry_d ],
        'sensor_msgs': [ os.path.join(test_d, 'sensor_msgs', 'msg') ],
        'invalid': [ os.path.join(test_d, 'invalid', 'msg') ],
        }

    # Test not found
    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'invalid/BadDepend', msg_search_path)
    try:
        load_depends(msg_context, root_spec, msg_search_path)
        assert False, "should have raised MsgNotFound"
    except MsgNotFound:
        pass
    root_spec = load_msg_by_type(msg_context, 'invalid/BadLocalDepend', msg_search_path)
    try:
        load_depends(msg_context, root_spec, msg_search_path)
        assert False, "should have raised MsgNotFound"
    except MsgNotFound:
        pass
    
    # Test with msgs
    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'geometry_msgs/PoseStamped', msg_search_path)
    load_depends(msg_context, root_spec, msg_search_path)
    file_p = os.path.join(test_d, 'geometry_msgs', 'msg', 'PoseStamped.msg')
    assert file_p == msg_context.get_file('geometry_msgs/PoseStamped')
    val = msg_context.get_all_depends('geometry_msgs/PoseStamped')
    assert set(['std_msgs/Header', 'geometry_msgs/Pose', 'geometry_msgs/Point', 'geometry_msgs/Quaternion']) == set(val), val
    val = msg_context.get_depends('geometry_msgs/PoseStamped')
    assert set(['std_msgs/Header', 'geometry_msgs/Pose']) == set(val), val

    for s in ['Header']:
        file_p = os.path.join(test_d, 'std_msgs', 'msg', '%s.msg'%s)
        assert file_p == msg_context.get_file('std_msgs/%s'%s)
    for s in ['Pose', 'Point', 'Quaternion']:
        file_p = os.path.join(geometry_d, '%s.msg'%s)
        assert file_p == msg_context.get_file('geometry_msgs/%s'%s)

    msg_context = MsgContext.create_default()
    root_spec = load_msg_by_type(msg_context, 'sensor_msgs/Imu', msg_search_path)
    load_depends(msg_context, root_spec, msg_search_path)
    file_p = os.path.join(test_d, 'sensor_msgs', 'msg', 'Imu.msg')
    assert file_p == msg_context.get_file('sensor_msgs/Imu')
    val = msg_context.get_depends('sensor_msgs/Imu')
    assert set(['std_msgs/Header', 'geometry_msgs/Quaternion', 'geometry_msgs/Vector3']) == set(val), val
    for s in ['Header']:
        file_p = os.path.join(test_d, 'std_msgs', 'msg', '%s.msg'%s)
        assert file_p == msg_context.get_file('std_msgs/%s'%s)
    for s in ['Quaternion', 'Vector3']:
        file_p = os.path.join(geometry_d, '%s.msg'%s)
        assert file_p == msg_context.get_file('geometry_msgs/%s'%s)


def test_load_depends_srv():
    from genmsg.msg_loader import MsgContext, load_msg_by_type, load_depends, MsgNotFound, load_srv_by_type
    test_d = get_test_dir()
    geometry_d = os.path.join(test_d, 'geometry_msgs', 'msg')
    msg_search_path = {
        'test_ros': [ os.path.join(test_d, 'test_ros', 'msg') ],
        'std_msgs': [ os.path.join(test_d, 'std_msgs', 'msg') ],
        'geometry_msgs': [ geometry_d ],
        'sensor_msgs': [ os.path.join(test_d, 'sensor_msgs', 'msg') ],
        'invalid': [ os.path.join(test_d, 'invalid', 'msg') ],
        }

    # Test with srvs
    srv_search_path = {
        'test_ros': [ os.path.join(test_d, 'test_ros', 'srv') ],
        'std_srvs': [ os.path.join(test_d, 'std_srvs', 'srv') ],
        }

    msg_context = MsgContext.create_default()
    root_spec = load_srv_by_type(msg_context, 'test_ros/AddTwoInts', srv_search_path)
    load_depends(msg_context, root_spec, msg_search_path)
    val = msg_context.get_depends('test_ros/AddTwoIntsRequest')
    assert val == [], val
    val = msg_context.get_depends('test_ros/AddTwoIntsResponse')    
    assert val == [], val

    # test with srv that has depends
    msg_context = MsgContext.create_default()
    response_deps = ['std_msgs/Header', 'geometry_msgs/Pose', 'geometry_msgs/PoseStamped', 'geometry_msgs/Point', 'geometry_msgs/Quaternion']
    root_spec = load_srv_by_type(msg_context, 'test_ros/GetPoseStamped', srv_search_path)
    load_depends(msg_context, root_spec, msg_search_path)
    for d in response_deps:
        assert msg_context.is_registered(d)
    val = msg_context.get_depends('test_ros/GetPoseStampedRequest')
    assert val == [], val
    val = msg_context.get_depends('test_ros/GetPoseStampedResponse')    
    assert val == ['geometry_msgs/PoseStamped']
    
    # Test with nonsense
    class Foo(object): pass
    try:
        load_depends(msg_context, Foo(), msg_search_path)
        assert False, "should have raised"
    except ValueError:
        pass
    
def test_load_srv_by_type():
    from genmsg.msg_loader import load_srv_by_type, MsgContext, MsgNotFound
    
    test_d = get_test_dir()
    test_ros_dir = os.path.join(test_d, 'test_ros', 'srv')
    std_srvs_dir = os.path.join(test_d, 'std_srvs', 'srv')
    empty_path = os.path.join(std_srvs_dir, 'Empty.srv')
    a2i_path = os.path.join(std_srvs_dir, 'AddTwoInts.srv')
    search_path = {
        'test_ros': [ test_ros_dir ],
        'std_srvs': [ std_srvs_dir ],
        }
    msg_context = MsgContext.create_default()
    spec = load_srv_by_type(msg_context, 'std_srvs/Empty', search_path)
    assert msg_context.is_registered('std_srvs/EmptyRequest')
    assert msg_context.is_registered('std_srvs/EmptyResponse')
    assert msg_context.get_registered('std_srvs/EmptyRequest') == spec.request
    assert msg_context.get_registered('std_srvs/EmptyResponse') == spec.response
    assert msg_context.get_file('std_srvs/EmptyRequest') == empty_path, msg_context.get_file('std_srvs/EmptyRequest')
    assert msg_context.get_file('std_srvs/EmptyResponse') == empty_path,msg_context.get_file('std_srvs/EmptyResponse')
    assert spec.request.full_name == 'std_srvs/EmptyRequest'
    assert spec.response.full_name == 'std_srvs/EmptyResponse'
    assert spec.request.short_name == 'EmptyRequest'
    assert spec.response.short_name == 'EmptyResponse'
    assert spec.request.package == 'std_srvs'
    assert spec.response.package == 'std_srvs'
    for f in [spec.request.names, spec.request.types, spec.response.names, spec.response.types]:
        assert [] == f

    spec = load_srv_by_type(msg_context, 'test_ros/AddTwoInts', search_path)
    assert msg_context.is_registered('test_ros/AddTwoIntsRequest')
    assert msg_context.is_registered('test_ros/AddTwoIntsResponse')
    assert msg_context.get_registered('test_ros/AddTwoIntsRequest') == spec.request
    assert msg_context.get_registered('test_ros/AddTwoIntsResponse') == spec.response
    assert spec.request.types == ['int64', 'int64'], spec.request.types
    assert spec.request.names == ['a', 'b'], spec.request.names
    assert spec.response.types == ['int64'], spec.response.types
    assert spec.response.names == ['sum'], spec.response.names

    # test invalid search path
    try:
        load_srv_by_type(msg_context, 'test_ros/AddTwoInts', [std_srvs_dir])
        assert False, "should have raised"
    except ValueError:
        pass
    # test not found
    try:
        load_srv_by_type(msg_context, 'test_ros/Fake', search_path)
        assert False, "should have raised"
    except MsgNotFound:
        pass

