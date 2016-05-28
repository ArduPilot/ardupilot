from LogAnalyzer import Test,TestResult
import DataflashLog


class TestGPSGlitch(Test):
	'''test for GPS glitch reporting or bad GPS data (satellite count, hdop)'''

	def __init__(self):
		Test.__init__(self)
		self.name = "GPS"

	def run(self, logdata, verbose):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.GOOD

		if "GPS" not in logdata.channels:
			self.result.status = TestResult.StatusType.UNKNOWN
			self.result.statusMessage = "No GPS log data"
			return

		# glitch protection is currently copter-only, but might be added to other vehicle types later and there's no harm in leaving the test in for all
		gpsGlitchCount = 0
		if "ERR" in logdata.channels:
			assert(len(logdata.channels["ERR"]["Subsys"].listData) == len(logdata.channels["ERR"]["ECode"].listData))
			for i in range(len(logdata.channels["ERR"]["Subsys"].listData)):
				subSys = logdata.channels["ERR"]["Subsys"].listData[i][1]
				eCode  = logdata.channels["ERR"]["ECode"].listData[i][1]
			 	if subSys == 11 and (eCode == 2):
			 		gpsGlitchCount += 1
 		if gpsGlitchCount:
			self.result.status = TestResult.StatusType.FAIL
			self.result.statusMessage = "GPS glitch errors found (%d)" % gpsGlitchCount

		# define and check different thresholds for WARN level and FAIL level
		# TODO: for plane, only check after first instance of throttle > 0, or after takeoff if we can reliably detect it
		minSatsWARN = 6
		minSatsFAIL = 5
		maxHDopWARN = 3.0
		maxHDopFAIL = 10.0
		foundBadSatsWarn = logdata.channels["GPS"]["NSats"].min() < minSatsWARN
		foundBadHDopWarn = logdata.channels["GPS"]["HDop"].max()  > maxHDopWARN
		foundBadSatsFail = logdata.channels["GPS"]["NSats"].min() < minSatsFAIL
		foundBadHDopFail = logdata.channels["GPS"]["HDop"].max()  > maxHDopFAIL
		satsMsg = "Min satellites: %s, Max HDop: %s" % (logdata.channels["GPS"]["NSats"].min(), logdata.channels["GPS"]["HDop"].max())	
		if gpsGlitchCount:
			self.result.statusMessage = self.result.statusMessage + "\n" + satsMsg
		if foundBadSatsFail or foundBadHDopFail:
			if not gpsGlitchCount:
				self.result.status = TestResult.StatusType.FAIL
				self.result.statusMessage = satsMsg
		elif foundBadSatsWarn or foundBadHDopWarn:
			if not gpsGlitchCount:
				self.result.status = TestResult.StatusType.WARN
				self.result.statusMessage = satsMsg


