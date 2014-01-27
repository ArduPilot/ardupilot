from LogAnalyzer import Test,TestResult
import DataflashLog


class TestBalanceTwist(Test):
	'''test for badly unbalanced copter, including yaw twist'''

	def __init__(self):
		self.name = "Balance/Twist"
		
	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		if logdata.vehicleType == "ArduPlane":
			self.result.status = TestResult.StatusType.NA

		# TODO: implement copter test for unbalanced or twisted copter frame