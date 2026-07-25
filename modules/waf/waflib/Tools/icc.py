#!/usr/bin/env python
# encoding: utf-8
# Stian Selnes 2008
# Thomas Nagy 2009-2018 (ita)

"""
Detects the Intel C compiler
"""

from waflib import Utils
from waflib.Tools import ccroot, ar, gcc
from waflib.Configure import conf
from waflib.Tools import msvc

@conf
def find_icc(conf):
	"""
	Finds the program icc and execute it to ensure it really is icc
	"""
	if Utils.is_win32:
		conf.find_program(['icx-cl'], var='ICXCL', mandatory=False)
		if conf.env.ICXCL:
			conf.env.INTEL_CLANG_COMPILER = True
			conf.env.CC = conf.env.ICXCL

	if not conf.env.ICXCL:
		cc = conf.find_program(['icx', 'icc', 'ICL'], var='CC')
		conf.get_cc_version(cc, icc=True)

	conf.env.CC_NAME = 'icc'

def configure(conf):
	conf.find_icc()
	if conf.env.ICXCL and Utils.is_win32:
		conf.find_msvc()
		conf.find_program('MT', var='MT')
		conf.env.MTFLAGS = ['/nologo']
		conf.env.MSVC_MANIFEST = True

		conf.msvc_common_flags()

		conf.env.CFLAGS = []
		conf.cc_load_tools()
		conf.cc_add_flags()
		conf.link_add_flags()

		conf.visual_studio_add_flags()
		conf.env.CC_TGT_F = ['/FC', '/c', '/Fo']
		conf.env.CPPPATH_ST = '/I%s'
	else:
		conf.find_ar()
		conf.gcc_common_flags()
		conf.gcc_modifier_platform()
		conf.cc_load_tools()
		conf.cc_add_flags()
		conf.link_add_flags()
