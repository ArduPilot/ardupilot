from LogAnalyzer import Test,TestResult
import DataflashLog


class TestPerformance(Test):
	'''check performance monitoring messages (PM) for issues with slow loops, etc'''

	def __init__(self):
		self.name = "Performance"

	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS


		# TODO: test for performance warning messages, slow loops, etc
