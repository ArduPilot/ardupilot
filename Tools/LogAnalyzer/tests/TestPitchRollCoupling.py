from LogAnalyzer import Test,TestResult
import DataflashLog


class TestPitchRollCoupling(Test):
	'''test for divergence between input and output pitch/roll, i.e. mechanical failure or bad PID tuning'''

	def __init__(self):
		self.name = "Pitch/Roll"

	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		# TODO: implement pitch/roll input/output divergence testing - 

		# note: names changed from PitchIn to DesPitch at some point, check for both
		