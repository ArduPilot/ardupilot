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

def test_normalize_package_context():
    # normalize isn't meant to be fancy; just correct minor differences
    from genmsg.names import normalize_package_context
    assert 'foo' == normalize_package_context('foo')
    assert 'foo' == normalize_package_context(' foo ')
    assert 'foo' == normalize_package_context('foo/')
    assert 'foo' == normalize_package_context('foo//')
    assert 'foo' == normalize_package_context('foo///')
    
def test_resource_name():
    from genmsg import resource_name
    assert 'foo/bar' == resource_name('foo', 'bar')
    assert 'bar' == resource_name('foo', 'bar', my_pkg='foo')
    assert 'foo/bar' == resource_name('foo', 'bar', my_pkg='bar')
    assert 'foo/bar' == resource_name('foo', 'bar', my_pkg='')
    assert 'foo/bar' == resource_name('foo', 'bar', my_pkg=None)

def test_resource_name_base():
    from genmsg import resource_name_base
    assert '' == resource_name_base('')
    assert 'bar' == resource_name_base('bar') 
    assert 'bar' == resource_name_base('foo/bar')
    assert 'bar' == resource_name_base('/bar')
    assert '' == resource_name_base('foo/')

def test_resource_name_package():
    from genmsg import resource_name_package
    assert None == resource_name_package('')
    assert None == resource_name_package('foo')   
    assert 'foo' == resource_name_package('foo/')
    assert 'foo' == resource_name_package('foo/bar')

def test_package_resource_name():
    from genmsg import package_resource_name
    assert ('', '') == package_resource_name('')
    assert ('', 'foo') == package_resource_name('foo')
    assert ('foo', 'bar') == package_resource_name('foo/bar')
    assert ('foo', '') == package_resource_name('foo/')
    try:
        # only allowed single separator
        package_resource_name("foo/bar/baz")
        assert False, "should have raised ValueError"
    except ValueError:
        pass
      

def test_is_legal_resource_name():
    from genmsg import is_legal_resource_name
    failures = [None, '', 'hello\n', '\t', 'foo++', 'foo-bar', '#foo', 
                ' name', 'name ',
                '~name', '/name',
                '1name', 'foo\\']
    for f in failures:
        assert not is_legal_resource_name(f), f
    tests = ['f', 'f1', 'f_', 'foo', 'foo_bar', 'foo/bar', 'roslib/Log']
    for t in tests:
        assert is_legal_resource_name(t), t

def test_is_legal_resource_base_name():
    from genmsg import is_legal_resource_base_name
    failures = [None, '', 'hello\n', '\t', 'foo++', 'foo-bar', '#foo',
                'f/', 'foo/bar', '/', '/a',
                'f//b',
                '~f', '~a/b/c',
                '~/f',
                ' name', 'name ',
                '1name', 'foo\\']
    for f in failures:
        assert not is_legal_resource_base_name(f), f
    tests = ['f', 'f1', 'f_', 'foo', 'foo_bar']
    for t in tests:
        assert is_legal_resource_base_name(t), "[%s]"%t
      
