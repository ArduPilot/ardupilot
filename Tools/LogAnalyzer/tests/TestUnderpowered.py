from LogAnalyzer import Test,TestResult
import DataflashLog


class TestUnderpowered(Test):
	'''test for underpowered copter'''

	def __init__(self):
		self.name = "Underpowered"
		
	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		if logdata.vehicleType == "ArduPlane":
			self.result.status = TestResult.StatusType.NA

		# TODO: implement test for underpowered copter (use log Randy provided)
