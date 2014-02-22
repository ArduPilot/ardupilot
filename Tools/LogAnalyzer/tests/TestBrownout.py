from LogAnalyzer import Test,TestResult
import DataflashLog

import collections

class TestBrownout(Test):
	'''test for a log that has been truncated in flight'''

	def __init__(self):
		self.name = "Brownout"

	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		if "EV" in logdata.channels:
			# step through the arm/disarm events in order, to see if they're symmetrical
			# note: it seems landing detection isn't robust enough to rely upon here, so we'll only consider arm+disarm, not takeoff+land
			evData = logdata.channels["EV"]["Id"]
			orderedEVData = collections.OrderedDict(sorted(evData.dictData.items(), key=lambda t: t[0]))
			isArmed = False
			for line,ev in orderedEVData.iteritems():
				if ev == 10:
					isArmed = True
				elif ev == 11:
					isArmed = False

		if "CTUN" not in logdata.channels:
			self.result.status = TestResult.StatusType.UNKNOWN
			self.result.statusMessage = "No CTUN log data"
			return

		# check for relative altitude at end
		if "CTUN" in logdata.channels and "BarAlt" in logdata.channels["CTUN"]:
			(finalAlt,finalAltLine) = logdata.channels["CTUN"]["BarAlt"].getNearestValue(logdata.lineCount, lookForwards=False)

			finalAltMax = 3.0   # max alt offset that we'll still consider to be on the ground
			if isArmed and finalAlt > finalAltMax:
				self.result.status = TestResult.StatusType.FAIL
				self.result.statusMessage = "Truncated Log? Ends while armed at altitude %.2fm" % finalAlt
