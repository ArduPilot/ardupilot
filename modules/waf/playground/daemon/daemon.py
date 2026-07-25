#!/usr/bin/env python
# encoding: utf-8
# Matthias Jahn 2006
# rewritten by Thomas Nagy 2009

"""
Start a new build as soon as something changes in the build directory.

PyInotify, Fam, Gamin or time-threshold are used for the detection

For now only PyInotify and time threshold are supported
Watching for new svn revisions could be added too
"""

import select, errno, os, time
from waflib import Utils, Scripting, Logs, Build, Node, Context, Options

w_pyinotify = w_fam = w_gamin = None
def check_support():
	global w_pyinotify, w_fam, w_gamin
	try:
		import pyinotify as w_pyinotify
	except ImportError:
		w_pyinotify = None
	else:
		try:
			wm = w_pyinotify.WatchManager()
			wm = w_pyinotify.Notifier(wm)
			wm = None
		except:
			raise
			w_pyinotify = None

	try:
		import gamin as w_gamin
	except ImportError:
		w_gamin = None
	else:
		try:
			test = w_gamin.WatchMonitor()
			test.disconnect()
			test = None
		except:
			w_gamin = None

	try:
		import _fam as w_fam
	except ImportError:
		w_fam = None
	else:
		try:
			test = w_fam.open()
			test.close()
			test = None
		except:
			w_fam = None

def daemon(ctx):
	"""waf command: rebuild as soon as something changes"""
	bld = None
	while True:
		bld = Context.create_context('build')
		try:
			bld.options = Options.options
			bld.cmd = 'build'
			bld.execute()
		except ctx.errors.WafError as e:
			print(e)
		except KeyboardInterrupt:
			Utils.pprint('RED', 'interrupted')
			break

		try:
			x = ctx.state
		except AttributeError:
			setattr(ctx, 'state', DirWatch())
			x = ctx.state

		x.wait(bld)

def options(opt):
	"""So this shows how to add new commands from tools"""
	Context.g_module.__dict__['daemon'] = daemon

class DirWatch(object):
	def __init__(self):
		check_support()
		if w_pyinotify:
			self.sup = 'pyinotify'
		elif w_gamin:
			self.sup = 'gamin'
		elif w_fam:
			self.sup = 'fam'
		else:
			self.sup = 'dumb'
		#self.sup = 'dumb'

	def wait(self, bld):
		return getattr(self.__class__, 'wait_' + self.sup)(self, bld)

	def enumerate(self, node):
		if os.path.exists(node.abspath()):
			yield node.abspath()
		try:
			for x in node.children.values():
				for k in self.enumerate(x):
					yield k
		except AttributeError:
			pass

	def wait_pyinotify(self, bld):

		class PE(w_pyinotify.ProcessEvent):
			def stop(self, event):
				self.notif.ev = True
				self.notif.stop()
				raise ValueError("stop for delete")

			process_IN_DELETE = stop
			process_IN_CLOSE = stop
			process_default = stop

		proc = PE()
		wm = w_pyinotify.WatchManager()
		notif = w_pyinotify.Notifier(wm, proc)
		proc.notif = notif

		# well, we should add all the folders to watch here
		for x in self.enumerate(bld.srcnode):
			wm.add_watch(x, w_pyinotify.IN_DELETE | w_pyinotify.IN_CLOSE_WRITE)

		try:
			# pyinotify uses an infinite loop ... not too nice, so we have to use an exception
			notif.loop()
		except ValueError:
			pass
		if not hasattr(notif, 'ev'):
			raise KeyboardInterrupt

	def wait_dumb(self, bld):
		time.sleep(5)

	def wait_gamin(self, bld):
		time.sleep(5)

	def wait_fam(self, bld):
		time.sleep(5)

