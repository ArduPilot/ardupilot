#!/usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2021 (ita)

from waflib import Utils, Runner

"""
Re-enable the classic threading system from waf 1.x

def configure(conf):
	conf.load('classic_runner')
"""

class TaskConsumer(Utils.threading.Thread):
	"""
	Task consumers belong to a pool of workers

	They wait for tasks in the queue and then use ``task.process(...)``
	"""
	def __init__(self, spawner):
		Utils.threading.Thread.__init__(self)
		"""
		Obtain :py:class:`waflib.Task.TaskBase` instances from this queue.
		"""
		self.spawner = spawner
		self.daemon = True
		self.start()

	def run(self):
		"""
		Loop over the tasks to execute
		"""
		try:
			self.loop()
		except Exception:
			pass

	def loop(self):
		"""
		Obtain tasks from :py:attr:`waflib.Runner.TaskConsumer.ready` and call
		:py:meth:`waflib.Task.TaskBase.process`. If the object is a function, execute it.
		"""
		master = self.spawner.master
		while 1:
			if not master.stop:
				try:
					tsk = master.ready.get()
					if tsk:
						tsk.log_display(tsk.generator.bld)
						master.process_task(tsk)
					else:
						break
				finally:
					master.out.put(tsk)

class Spawner(object):
	"""
	Daemon thread that consumes tasks from :py:class:`waflib.Runner.Parallel` producer and
	spawns a consuming thread :py:class:`waflib.Runner.Consumer` for each
	:py:class:`waflib.Task.Task` instance.
	"""
	def __init__(self, master):
		self.master = master
		""":py:class:`waflib.Runner.Parallel` producer instance"""

		self.pool = [TaskConsumer(self) for i in range(master.numjobs)]

Runner.Spawner = Spawner
