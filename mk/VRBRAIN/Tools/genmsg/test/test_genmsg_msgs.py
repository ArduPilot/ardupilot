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

def test_bare_msg_type():
    import genmsg.msgs    
    tests = [(None, None), ('String', 'String'), ('std_msgs/String', 'std_msgs/String'),
             ('String[10]', 'String'), ('string[10]', 'string'), ('std_msgs/String[10]', 'std_msgs/String'),
             ]
    for val, res in tests:
      assert res == genmsg.msgs.bare_msg_type(val)

PKG = 'genmsg'

def test_resolve_type():
    from genmsg.msgs import resolve_type, bare_msg_type
    for t in ['string', 'string[]', 'string[14]', 'int32', 'int32[]']:
        bt = bare_msg_type(t)
        t == resolve_type(t, PKG)
      
    assert 'foo/string' == resolve_type('foo/string', PKG)
    assert 'std_msgs/Header' == resolve_type('Header', 'roslib')
    assert 'std_msgs/Header' == resolve_type('std_msgs/Header', 'roslib')
    assert 'std_msgs/Header' == resolve_type('Header', 'stereo_msgs')
    assert 'std_msgs/String' == resolve_type('String', 'std_msgs')
    assert 'std_msgs/String' == resolve_type('std_msgs/String', 'std_msgs')
    assert 'std_msgs/String' == resolve_type('std_msgs/String', PKG) 
    assert 'std_msgs/String[]' == resolve_type('std_msgs/String[]', PKG)
    
def test_parse_type():
    import genmsg.msgs
    tests = [
        ('a', ('a', False, None)),
        ('int8', ('int8', False, None)),      
        ('std_msgs/String', ('std_msgs/String', False, None)),
        ('a[]', ('a', True, None)),
        ('int8[]', ('int8', True, None)),      
        ('std_msgs/String[]', ('std_msgs/String', True, None)),
        ('a[1]', ('a', True, 1)),
        ('int8[1]', ('int8', True, 1)),      
        ('std_msgs/String[1]', ('std_msgs/String', True, 1)),
        ('a[11]', ('a', True, 11)),
        ('int8[11]', ('int8', True, 11)),      
        ('std_msgs/String[11]', ('std_msgs/String', True, 11)),
        ]
    for val, res in tests:
        assert res == genmsg.msgs.parse_type(val)
      
    fail = ['a[1][2]', 'a[][]', '', None, 'a[', 'a[[1]', 'a[1]]']
    for f in fail:
        try:
            genmsg.msgs.parse_type(f)
            assert False, "should have failed on %s"%f
        except ValueError as e:
            pass

def test_Constant():
    import genmsg.msgs    
    vals = [random.randint(0, 1000) for i in range(0, 3)]
    type_, name, val = [str(x) for x in vals]
    x = genmsg.msgs.Constant(type_, name, val, str(val))
    assert type_ == x.type
    assert name == x.name
    assert val == x.val
    assert x == genmsg.msgs.Constant(type_, name, val, str(val))

    assert x != 1
    assert not x == 1
    assert x != genmsg.msgs.Constant('baz', name, val, str(val))
    assert x != genmsg.msgs.Constant(type_, 'foo', val, str(val))
    assert x != genmsg.msgs.Constant(type_, name, 'foo', 'foo')

    # tripwire
    assert repr(x)
    assert str(x)
    
    try:
        genmsg.msgs.Constant(None, name, val, str(val))
        assert False, "should have raised"
    except: pass
    try:
        genmsg.msgs.Constant(type_, None, val, str(val))
        assert False, "should have raised"        
    except: pass
    try:
        genmsg.msgs.Constant(type_, name, None, 'None')
        assert False, "should have raised"        
    except: pass
    try:
        genmsg.msgs.Constant(type_, name, val, None)
        assert False, "should have raised"        
    except: pass
    
    try:
        x.foo = 'bar'
        assert False, 'Constant should not allow arbitrary attr assignment'
    except: pass
    
def test_MsgSpec():
    def sub_test_MsgSpec(types, names, constants, text, full_name, has_header):
        m = MsgSpec(types, names, constants, text, full_name)
        assert m.types == types
        assert m.names == names
        assert m.text == text
        assert has_header == m.has_header()
        assert m.constants == constants
        assert list(zip(types, names)) == m.fields()
        assert m == MsgSpec(types, names, constants, text, full_name)
        return m
    
    from genmsg import MsgSpec, InvalidMsgSpec
    from genmsg.msgs import Field

    # don't allow duplicate fields
    try:
        MsgSpec(['int32', 'int64'], ['x', 'x'], [], 'int32 x\nint64 x', 'x/DupFields')
        assert False, "should have raised"
    except InvalidMsgSpec:
        pass
    # don't allow invalid fields
    try:
        MsgSpec(['string['], ['x'], [], 'int32 x\nint64 x', 'x/InvalidFields')
        assert False, "should have raised"
    except InvalidMsgSpec:
        pass

    # allow empty msg
    empty = sub_test_MsgSpec([], [], [], '', 'x/Nothing', False)
    assert [] == empty.fields()
    assert [] == empty.parsed_fields()
    assert 'x/Nothing' == empty.full_name
    assert 'x' == empty.package
    assert 'Nothing' == empty.short_name    

    # one-field
    one_field = sub_test_MsgSpec(['int32'], ['x'], [], 'int32 x', 'x/OneInt', False)
    # make sure that equals tests every declared field
    assert one_field == MsgSpec(['int32'], ['x'], [], 'int32 x', 'x/OneInt')
    assert one_field != MsgSpec(['uint32'], ['x'], [], 'int32 x', 'x/OneInt')
    assert one_field != MsgSpec(['int32'], ['y'], [], 'int32 x', 'x/OneInt')
    assert one_field != MsgSpec(['int32'], ['x'], [], 'uint32 x', 'x/OneInt')
    assert one_field != MsgSpec(['int32'], ['x'], [], 'int32 x', 'x/OneIntBad')
    # test against __ne__ as well
    assert one_field != MsgSpec(['int32'], ['x'], [], 'uint32 x', 'x/OneInt')
    assert [Field('x', 'int32')] == one_field.parsed_fields(), "%s vs %s"%([Field('x', 'int32')], one_field.parsed_fields())
    #test str
    assert "int32 x" == str(one_field).strip()
    
    # test variations of multiple fields and headers
    two_fields = sub_test_MsgSpec(['int32', 'string'], ['x', 'str'], [], 'int32 x\nstring str', 'x/TwoFields', False)
    assert [Field('x', 'int32'), Field('str', 'string')] == two_fields.parsed_fields()
    
    one_header = sub_test_MsgSpec(['std_msgs/Header'], ['header'], [], 'Header header', 'x/OneHeader', True)
    header_and_fields = sub_test_MsgSpec(['std_msgs/Header', 'int32', 'string'], ['header', 'x', 'str'], [], 'Header header\nint32 x\nstring str', 'x/HeaderAndFields', True)
    embed_types = sub_test_MsgSpec(['std_msgs/Header', 'std_msgs/Int32', 'string'], ['header', 'x', 'str'], [], 'Header header\nstd_msgs/Int32 x\nstring str', 'x/EmbedTypes', True)
    #test strify
    assert "int32 x\nstring str" == str(two_fields).strip()

    # types and names mismatch
    try:
        MsgSpec(['int32', 'int32'], ['intval'], [], 'int32 intval\int32 y')
        assert False, "types and names must align"
    except: pass

    # test (not) equals against non msgspec
    assert not (one_field == 1)
    assert one_field != 1

    # test constants
    from genmsg.msgs import Constant
    msgspec = MsgSpec(['int32'], ['x'], [Constant('int8', 'c', 1, '1')], 'int8 c=1\nuint32 x', 'x/Constants')
    assert msgspec.constants == [Constant('int8', 'c', 1, '1')]
    # tripwire
    str(msgspec)
    repr(msgspec)

    # test that repr doesn't throw an error
    [repr(x) for x in [empty, one_field, one_header, two_fields, embed_types]]

def test_Field():
    from genmsg.msgs import Field

    field = Field('foo', 'string')
    assert field == Field('foo', 'string')
    assert field != Field('bar', 'string')
    assert field != Field('foo', 'int32')
    assert field != 1
    assert not field == 1    

    assert field.name == 'foo'
    assert field.type == 'string'
    assert field.base_type == 'string'
    assert field.is_array == False
    assert field.array_len == None
    assert field.is_header == False
    assert field.is_builtin == True

    field = Field('foo', 'std_msgs/String')
    assert field.type == 'std_msgs/String'
    assert field.base_type == 'std_msgs/String'
    assert field.is_array == False
    assert field.array_len == None
    assert field.is_header == False
    assert field.is_builtin == False

    field = Field('foo', 'std_msgs/String[5]')
    assert field.type == 'std_msgs/String[5]'
    assert field.base_type == 'std_msgs/String'
    assert field.is_array == True
    assert field.array_len == 5
    assert field.is_header == False
    assert field.is_builtin == False

    field = Field('foo', 'std_msgs/String[]')
    assert field.type == 'std_msgs/String[]'
    assert field.base_type == 'std_msgs/String'
    assert field.is_array == True
    assert field.array_len == None
    assert field.is_header == False
    assert field.is_builtin == False

    field = Field('foo', 'std_msgs/Header')
    assert field.type == 'std_msgs/Header'
    assert field.is_header == True
    assert field.is_builtin == False

    field = Field('foo', 'std_msgs/Header[]')
    assert field.type == 'std_msgs/Header[]'
    assert field.is_header == False

    #tripwire
    repr(field)
    
def test_is_valid_msg_type():
    import genmsg.msgs
    vals = [
        #basic
        'F', 'f', 'Foo', 'Foo1',
        'std_msgs/String',
        # arrays
        'Foo[]', 'Foo[1]', 'Foo[10]',
        ]
    for v in vals:
        assert genmsg.msgs.is_valid_msg_type(v), "genmsg.msgs.is_valid_msg_type should have returned True for '%s'"%v
 
    # bad cases
    vals = [None, '', '#', '%', 'Foo%', 'Woo Woo',
            '/', '/String', 
            'Foo[f]', 'Foo[1d]', 'Foo[-1]', 'Foo[1:10]', 'Foo[', 'Foo]', 'Foo[]Bar']
    for v in vals:
        assert not genmsg.msgs.is_valid_msg_type(v), "genmsg.msgs.is_valid_msg_type should have returned False for '%s'"%v
      
def test_is_valid_constant_type():
    import genmsg.msgs
    valid = ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', \
             'uint64', 'float32', 'float64', 'char', 'byte', 'string']
    invalid = [
        'std_msgs/String', '/', 'String',
        'time', 'duration','header',
    ]
    for v in valid:
        assert genmsg.msgs.is_valid_constant_type(v), "genmsg.msgs.is_valid_constant_type should have returned True for '%s'"%v
    for v in invalid:
        assert not genmsg.msgs.is_valid_constant_type(v), "genmsg.msgs.is_valid_constant_type should have returned False for '%s'"%v

