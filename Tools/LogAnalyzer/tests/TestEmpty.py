from LogAnalyzer import Test,TestResult
import DataflashLog


class TestEmpty(Test):
	'''test for empty or near-empty logs'''

	def __init__(self):
		Test.__init__(self)
		self.name = "Empty"
		
	def run(self, logdata, verbose):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.GOOD

		# all the logic for this test is in the helper function, as it can also be called up front as an early exit
		emptyErr = DataflashLog.DataflashLogHelper.isLogEmpty(logdata)
		if emptyErr:
			self.result.status = TestResult.StatusType.FAIL
			self.result.statusMessage = "Empty log? " + emptyErr
