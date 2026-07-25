#! /usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2005-2018

"""
to make a custom waf file use the option --tools

To add a tool that does not exist in the folder compat15, pass an absolute path:
./waf-light  --tools=compat15,/comp/waf/aba.py --prelude=$'\tfrom waflib.extras import aba\n\taba.foo()'
"""

from __future__ import with_statement

VERSION="2.0.27"
APPNAME='waf'
REVISION=''

top = '.'
out = 'build'

zip_types = ['bz2', 'gz', 'xz']

PRELUDE = ''

import os, sys, re, io, optparse, tokenize
from hashlib import md5

from waflib import Errors, Utils, Options, Logs, Scripting
from waflib import Configure
Configure.autoconfig = 1

def sub_file(fname, lst):
	with open(fname, 'r') as f:
		txt = f.read()

	for (key, val) in lst:
		re_pat = re.compile(key, re.M)
		txt = re_pat.sub(val, txt)

	with open(fname, 'w') as f:
		f.write(txt)

def to_bytes(x):
	if sys.hexversion>0x300000f:
		return x.encode()
	return x

Logs.warn('------> Executing code from the top-level wscript <-----')
def init(ctx):
	if Options.options.setver: # maintainer only (ita)
		ver = Options.options.setver
		hexver = Utils.num2ver(ver)
		hexver = '0x%x'%hexver
		sub_file('wscript', (('^VERSION=(.*)', 'VERSION="%s"' % ver), ))
		sub_file('waf-light', (('^VERSION=(.*)', 'VERSION="%s"' % ver), ))

		pats = []
		pats.append(('^WAFVERSION=(.*)', 'WAFVERSION="%s"' % ver))
		pats.append(('^HEXVERSION(.*)', 'HEXVERSION=%s' % hexver))

		try:
			rev = ctx.cmd_and_log("git rev-parse HEAD").strip()
		except Errors.WafError:
			rev = ''
		else:
			pats.append(('^WAFREVISION(.*)', 'WAFREVISION="%s"' % rev))

		sub_file('waflib/Context.py', pats)

		sys.exit(0)

def check(ctx):
	Logs.warn('Nothing to do')

# this function is called before any other for parsing the command-line
def options(opt):

	# generate waf
	opt.add_option('--make-waf', action='store_true', default=True,
		help='creates the waf script', dest='waf')

	opt.add_option('--interpreter', action='store', default=None,
		help='specify the #! line on top of the waf file', dest='interpreter')

	opt.add_option('--sign', action='store_true', default=False, help='make a signed file', dest='signed')

	default_zip = 'bz2'
	if os.name == 'java':
		default_zip = 'gz'
	opt.add_option('--zip-type', action='store', default=default_zip,
		help='specify the zip type [Allowed values: %s]' % ' '.join(zip_types), dest='zip')

	opt.add_option('--make-batch', action='store_true', default=False,
		help='creates a convenience waf.bat file (done automatically on win32 systems)',
		dest='make_batch')

	opt.add_option('--yes', action='store_true', default=False,
		help=optparse.SUPPRESS_HELP,
		dest='yes')

	# those ones are not too interesting
	opt.add_option('--set-version', default='',
		help='sets the version number for waf releases (for the maintainer)', dest='setver')
	opt.add_option('--set-name', default='waf', help=optparse.SUPPRESS_HELP, dest='wafname')

	opt.add_option('--strip', action='store_true', default=True,
		help='shrinks waf (strip docstrings, saves 33kb)',
		dest='strip_comments')
	opt.add_option('--nostrip', action='store_false', help='no shrinking',
		dest='strip_comments')
	opt.add_option('--tools', action='store', help='Comma-separated 3rd party tools to add, eg: "compat,ocaml" [Default: "compat15"]',
		dest='add3rdparty', default='compat15')
	opt.add_option('--coretools', action='store', help='Comma-separated core tools to add, eg: "vala,tex" [Default: all of them]',
		dest='coretools', default='default')
	opt.add_option('--prelude', action='store', help='Code to execute before calling waf', dest='prelude', default=PRELUDE)
	opt.add_option('--namesfrom', action='store', help='Obtain the file names from a model archive', dest='namesfrom', default=None)
	opt.load('python')

def process_tokens(tokens, filename):
	accu = []
	prev = tokenize.NEWLINE

	indent = 0
	line_buf = []

	for (type, token, start, end, line) in tokens:
		token = token.replace('\r\n', '\n')
		if type == tokenize.NEWLINE:
			if line_buf:
				accu.append(indent * '\t')
				ln = "".join(line_buf)
				if ln == 'if __name__=="__main__":': break
				#ln = ln.replace('\n', '')
				accu.append(ln)
				accu.append('\n')
				line_buf = []
				prev = tokenize.NEWLINE
		elif type == tokenize.INDENT:
			indent += 1
		elif type == tokenize.DEDENT:
			indent -= 1
		elif type == tokenize.NAME:
			if prev == tokenize.NAME or prev == tokenize.NUMBER: line_buf.append(' ')
			line_buf.append(token)
		elif type == tokenize.NUMBER:
			if prev == tokenize.NAME or prev == tokenize.NUMBER: line_buf.append(' ')
			line_buf.append(token)
		elif type == tokenize.STRING:
			if not line_buf and token.startswith('"'): pass
			else:
				if token.lower().startswith('f'):
					raise ValueError('Found f-strings in %s which require Python >= 3.6, use "waf-light --nostrip"' % filename)
				line_buf.append(token)
		elif type == tokenize.COMMENT:
			pass
		elif type == tokenize.OP:
			line_buf.append(token)
		else:
			if token != "\n": line_buf.append(token)

		if token != '\n':
			prev = type

	body = ''.join(accu)
	return body

deco_re = re.compile('(def|class)\\s+(\\w+)\\(.*')
def process_decorators(body):
	lst = body.splitlines()
	accu = []
	all_deco = []
	buf = [] # put the decorator lines
	for line in lst:
		if line.startswith('@'):
			buf.append(line[1:])
		elif buf:
			name = deco_re.sub('\\2', line)
			if not name:
				raise IOError("decorator not followed by a function!" + line)
			for x in buf:
				all_deco.append('%s(%s)' % (x, name))
			accu.append(line)
			buf = []
		else:
			accu.append(line)
	return '\n'.join(accu+all_deco)

def sfilter(path):
	if path.endswith('.py') :
		if Options.options.strip_comments:
			if sys.version_info[0] >= 3:
				with open(path, 'rb') as f:
					tk = tokenize.tokenize(f.readline)
					next(tk) # the first one is always tokenize.ENCODING for Python 3, ignore it
					cnt = process_tokens(tk, path)
			else:
				with open(path, 'r') as f:
					cnt = process_tokens(tokenize.generate_tokens(f.readline), path)
		else:
			with open(path, 'r') as f:
				cnt = f.read()
		# WARNING: since python >= 2.5 is required, decorators are not processed anymore
		# uncomment the following to enable decorator replacement:
		#cnt = process_decorators(cnt)
		#if cnt.find('set(') > -1:
		#	cnt = 'import sys\nif sys.hexversion < 0x020400f0: from sets import Set as set\n' + cnt
		cnt = '#! /usr/bin/env python\n# encoding: utf-8\n# WARNING! Do not edit! https://waf.io/book/index.html#_obtaining_the_waf_file\n\n' + cnt

	else:
		with open(path, 'r') as f:
			cnt = f.read()

	if sys.hexversion > 0x030000f0:
		return (io.BytesIO(cnt.encode('utf-8')), len(cnt.encode('utf-8')), cnt)
	return (io.BytesIO(cnt), len(cnt), cnt)

def create_waf(self, *k, **kw):
	mw = 'tmp-waf-'+VERSION
	print('-> preparing %r' % mw)

	import tarfile, zipfile

	zipType = Options.options.zip.strip().lower()
	if zipType not in zip_types:
		zipType = zip_types[0]

	directory_files = {}
	files = []
	add3rdparty = []
	for x in Options.options.add3rdparty.split(','):
		if os.path.isdir(x):
			# Create mapping from files absolute path to path in module
			# directory (for module mylib):
			#
			#     {"/home/path/mylib/__init__.py": "mylib/__init__.py",
			#      "/home/path/mylib/lib.py": "mylib/lib.py",
			#      "/home/path/mylib/sub/sub.py": "mylib/sub/lib.py"
			#     }
			#
			x_dir = self.generator.bld.root.find_dir(
				os.path.abspath(os.path.expanduser(x)))

			file_list = x_dir.ant_glob('**/*.py')

			for f in file_list:

				file_from = f.abspath()
				file_to = os.path.join(x_dir.name, f.path_from(x_dir))

				# If this is executed on Windows, then file_to will contain
				# '\' path separators. These should be changed to '/', otherwise
				# the added tools will not be accessible on Unix systems.
				directory_files[file_from] = file_to.replace('\\', '/')
				files.append(file_from)

		elif os.path.isabs(x):
			files.append(x)
		else:
			add3rdparty.append(x + '.py')

	coretools = []
	for x in Options.options.coretools.split(','):
		coretools.append(x + '.py')

	up_node = self.generator.bld.path
	for node in up_node.find_dir('waflib').ant_glob(incl=['*.py', 'Tools/*.py', 'extras/*.py']):
		relpath = node.path_from(up_node)
		if node.name == '__init__.py':
			files.append(relpath)
			continue
		if node.parent.name == 'Tools' and Options.options.coretools != 'default':
			if node.name not in coretools:
				continue
		if node.parent.name == 'extras':
			if node.name not in add3rdparty:
				continue
		files.append(relpath)

	if Options.options.namesfrom:
		with tarfile.open(Options.options.namesfrom) as tar:
			oldfiles = files
			files = [x.name for x in tar.getmembers()]
			if set(files) ^ set(oldfiles):
				Logs.warn('The archive model has differences:')
				Logs.warn('- Added %r', list(set(files) - set(oldfiles)))
				Logs.warn('- Removed %r', list(set(oldfiles) - set(files)))

	#open a file as tar.[extension] for writing
	tar = tarfile.open('%s.tar.%s' % (mw, zipType), "w:%s" % zipType)
	z = zipfile.ZipFile("zip/waflib.zip", "w", compression=zipfile.ZIP_DEFLATED)
	for x in files:
		try:
			tarinfo = tar.gettarinfo(x, x)
		except NotImplementedError:
			# jython 2.7.0 workaround
			tarinfo = tarfile.TarInfo(x)
		tarinfo.uid   = tarinfo.gid   = 0
		tarinfo.uname = tarinfo.gname = 'root'
		if os.environ.get('SOURCE_DATE_EPOCH'):
			tarinfo.mtime = int(os.environ.get('SOURCE_DATE_EPOCH'))
		(code, size, cnt) = sfilter(x)
		tarinfo.size = size

		if x in directory_files:
			tarinfo.name = 'waflib/extras/' + directory_files[x]
		elif os.path.isabs(x):
			tarinfo.name = 'waflib/extras/' + os.path.split(x)[1]

		print('   adding %s as %s' % (x, tarinfo.name))
		def dest(x):
			if x in directory_files:
				return os.path.join('waflib', 'extras', directory_files[x])
			elif os.path.isabs(x):
				return os.path.join('waflib', 'extras', os.path.basename(x))
			else:
				return os.path.normpath(os.path.relpath(x, "."))

		z.write(x, dest(x))
		tar.addfile(tarinfo, code)
	tar.close()
	z.close()

	with open('waf-light', 'r') as f:
		code1 = f.read()

	# tune the application name if necessary
	if Options.options.wafname != 'waf':
		Options.options.prelude = '\tfrom waflib import Context\n\tContext.WAFNAME=%r\n' % Options.options.wafname + Options.options.prelude

	# now store the revision unique number in waf
	code1 = code1.replace("if sys.hexversion<0x206000f:\n\traise ImportError('Python >= 2.6 is required to create the waf file')\n", '')
	code1 = code1.replace('\t#import waflib.extras.compat15#PRELUDE', Options.options.prelude)

	# when possible, set the git revision in the waf file
	bld = self.generator.bld
	try:
		rev = bld.cmd_and_log('git rev-parse HEAD', quiet=0).strip()
	except Errors.WafError:
		rev = ''
	else:
		reg = re.compile('^GIT(.*)', re.M)
		code1 = reg.sub('GIT="%s"' % rev, code1)

	# if the waf file is installed somewhere... but do not do that
	prefix = ''
	reg = re.compile('^INSTALL=(.*)', re.M)
	code1 = reg.sub(r'INSTALL=%r' % prefix, code1)
	#change the tarfile extension in the waf script
	reg = re.compile('bz2', re.M)
	code1 = reg.sub(zipType, code1)
	if zipType == 'gz':
		code1 = code1.replace('bunzip2', 'gzip -d')
	elif zipType == 'xz':
		code1 = code1.replace('bunzip2', 'xz -d')

	with open('%s.tar.%s' % (mw, zipType), 'rb') as f:
		cnt = f.read()

	# the REVISION value is the md5 sum of the compressed data (facilitate audits)
	m = md5()
	m.update(cnt)
	REVISION = m.hexdigest()
	reg = re.compile('^REVISION=(.*)', re.M)
	code1 = reg.sub(r'REVISION="%s"' % REVISION, code1)

	def find_unused(kd, ch):
		for i in range(35, 125):
			for j in range(35, 125):
				if i==j: continue
				if i == 39 or j == 39: continue
				if i == 92 or j == 92: continue
				s = chr(i) + chr(j)
				if -1 == kd.find(s.encode()):
					return (kd.replace(ch.encode(), s.encode()), s)
		raise ValueError('Could not find a proper encoding')

	# The reverse order prevents collisions
	(cnt, C3) = find_unused(cnt, '\x00')
	(cnt, C2) = find_unused(cnt, '\r')
	(cnt, C1) = find_unused(cnt, '\n')
	ccc = code1.replace("C1='x'", "C1='%s'" % C1).replace("C2='x'", "C2='%s'" % C2).replace("C3='x'", "C3='%s'" % C3)

	if getattr(Options.options, 'interpreter', None):
		ccc = ccc.replace('#!/usr/bin/env python', Options.options.interpreter)

	with open('waf', 'wb') as f:
		f.write(ccc.encode())
		f.write(to_bytes('#==>\n#'))
		f.write(cnt)
		f.write(to_bytes('\n#<==\n'))

		if Options.options.signed:
			f.flush()
			try:
				os.remove('waf.asc')
			except OSError:
				pass
			ret = Utils.subprocess.Popen('gpg -bass waf', shell=True).wait()
			if ret:
				raise ValueError('Could not sign the waf file!')

			sig = Utils.readf('waf.asc')
			sig = sig.replace('\r', '').replace('\n', '\\n')
			f.write(to_bytes('#'))
			f.write(to_bytes(sig))
			f.write(to_bytes('\n'))
			os.remove('waf.asc')


	if sys.platform == 'win32' or Options.options.make_batch:
		with open('waf.bat', 'w') as f:
			f.write('@setlocal\n@set PYEXE=python\n@where %PYEXE% 1>NUL 2>NUL\n@if %ERRORLEVEL% neq 0 set PYEXE=py\n@%PYEXE% -x "%~dp0waf" %*\n@exit /b %ERRORLEVEL%\n')

	if sys.platform != 'win32':
		os.chmod('waf', Utils.O755)
	os.remove('%s.tar.%s' % (mw, zipType))

def configure(conf):
	conf.load('python')

def build(bld):
	waf = bld.path.make_node('waf') # do not use a build directory for this file
	bld(name='create_waf', rule=create_waf, target=waf, always=True, color='PINK')

class Dist(Scripting.Dist):
	def get_excl(self):
		return super(self.__class__, self).get_excl() + ' **/waflib.zip'
