from LogAnalyzer import Test,TestResult
import DataflashLog


class TestBalanceTwist(Test):
	'''test for badly unbalanced copter, including yaw twist'''

	def __init__(self):
		self.name = "Balance/Twist"
		self.enable = False   # TEMP
		
	def run(self, logdata, verbose):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		if logdata.vehicleType != "ArduCopter":
			self.result.status = TestResult.StatusType.NA
			return

		# TODO: implement copter test for unbalanced or twisted copter frame