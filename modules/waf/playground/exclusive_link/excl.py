#! /usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2011-2015 (ita)

"""
Prevents link tasks from executing in parallel. This can be used to
improve the linker execution, which may use a lot of memory.

The variable 'MAX' represents the tasks able to run
concurrently (just one by default). The variable 'count'
is the amount of tasks of one kind being executed.

Different constraints can be enforced by changing the scope
of some variables. Remember that:

* the counter could be set on the class level
* the MAX amount of concurrent tasks can be more than 1
"""

from waflib.Utils import threading
from waflib import Task
lock = threading.Lock()
count = 0
MAX = 1

def make_exclusive(cls):

	old_runnable_status = cls.runnable_status
	def runnable_status(self):
		global count, lock, MAX
		ret = Task.ASK_LATER
		if count >= MAX:
			return ret

		self.m1_excl = getattr(self, 'm1_excl', 0) + 1
		ret = old_runnable_status(self)
		self.m1_excl -= 1

		if ret == Task.RUN_ME and not self.m1_excl:
			lock.acquire()
			count += 1
			lock.release()
		return ret
	cls.runnable_status = runnable_status

	old_run = cls.run
	def run(self):
		global count, lock
		try:
			self.m2_excl = getattr(self, 'm2_excl', 0) + 1
			ret = old_run(self)
		finally:
			self.m2_excl -= 1
			if not self.m2_excl:
				lock.acquire()
				count -= 1
				lock.release()
		return ret
	cls.run = run

for x in 'cprogram cxxprogram cshlib cxxshlib cstlib cxxstlib fcprogram fcshlib fcstlib'.split():
	if x in Task.classes:
		make_exclusive(Task.classes[x])

