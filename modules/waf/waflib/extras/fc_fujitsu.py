#! /usr/bin/env python
# encoding: utf-8
# Detection of the Fujitsu Fortran compiler for ARM64FX

import re
from waflib.Tools import fc,fc_config,fc_scan
from waflib.Configure import conf
from waflib.Tools.compiler_fc import fc_compiler
fc_compiler['linux'].append('fc_fujitsu')

@conf
def find_fujitsu(conf):
	fc=conf.find_program(['frtpx'],var='FC')
	conf.get_fujitsu_version(fc)
	conf.env.FC_NAME='FUJITSU'
	conf.env.FC_MOD_CAPITALIZATION='lower'

@conf
def fujitsu_flags(conf):
	v=conf.env
	v['_FCMODOUTFLAGS']=[]
	v['FCFLAGS_DEBUG']=[]
	v['FCFLAGS_fcshlib']=[]
	v['LINKFLAGS_fcshlib']=[]
	v['FCSTLIB_MARKER']=''
	v['FCSHLIB_MARKER']=''

@conf
def get_fujitsu_version(conf,fc):
	version_re=re.compile(r"frtpx\s*\(FRT\)\s*(?P<major>\d+)\.(?P<minor>\d+)\.",re.I).search
	cmd=fc+['--version']
	out,err=fc_config.getoutput(conf,cmd,stdin=False)
	if out:
		match=version_re(out)
	else:
		match=version_re(err)
	if not match:
		return(False)
		conf.fatal('Could not determine the Fujitsu FRT Fortran compiler version.')
	else:
		k=match.groupdict()
		conf.env['FC_VERSION']=(k['major'],k['minor'])

def configure(conf):
	conf.find_fujitsu()
	conf.find_program('ar',var='AR')
	conf.add_os_flags('ARFLAGS')
	if not conf.env.ARFLAGS:
		conf.env.ARFLAGS=['rcs']
	conf.fc_flags()
	conf.fc_add_flags()
	conf.fujitsu_flags()
