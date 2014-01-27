from LogAnalyzer import Test,TestResult
import DataflashLog


class TestGPSGlitch(Test):
	'''test for bad GPS data (satellite count, hdop), and later for sudden repositioning beyond what the drone could do'''

	def __init__(self):
		self.name = "GPS"

	def run(self, logdata):
		self.result = TestResult()
		# define and check different thresholds for WARN level and FAIL level
		minSatsWARN = 6
		minSatsFAIL = 5
		maxHDopWARN = 3.0
		maxHDopFAIL = 10.0
		foundBadSatsWarn = logdata.channels["GPS"]["NSats"].min() < minSatsWARN
		foundBadHDopWarn = logdata.channels["GPS"]["HDop"].max()  > maxHDopWARN
		foundBadSatsFail = logdata.channels["GPS"]["NSats"].min() < minSatsFAIL
		foundBadHDopFail = logdata.channels["GPS"]["HDop"].max()  > maxHDopFAIL
		if foundBadSatsFail or foundBadHDopFail:
			self.result.status = TestResult.StatusType.FAIL
			self.result.statusMessage = "Min satellites: %s, Max HDop: %s" % (logdata.channels["GPS"]["NSats"].min(), logdata.channels["GPS"]["HDop"].max())
		elif foundBadSatsWarn or foundBadHDopWarn:
			self.result.status = TestResult.StatusType.WARN
			self.result.statusMessage = "Min satellites: %s, Max HDop: %s" % (logdata.channels["GPS"]["NSats"].min(), logdata.channels["GPS"]["HDop"].max())
		else:
			self.result.status = TestResult.StatusType.PASS

		# TODO: also test for sudden repositioning beyond what the drone could reasonably achieve
		# ...
