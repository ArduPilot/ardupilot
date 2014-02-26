from LogAnalyzer import Test,TestResult
import DataflashLog


class TestPitchRollCoupling(Test):
	'''test for divergence between input and output pitch/roll, i.e. mechanical failure or bad PID tuning'''

	def __init__(self):
		self.name = "Pitch/Roll"
		self.enable = False   # TEMP

	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		if logdata.vehicleType != "ArduCopter":
			self.result.status = TestResult.StatusType.NA

		if not "ATT" in logdata.channels:
			self.result.status = TestResult.StatusType.UNKNOWN
			self.result.statusMessage = "No ATT log data"
			return

		# TODO: implement pitch/roll input/output divergence testing - 

		# note: names changed from PitchIn to DesPitch at some point, check for both

		# what to test for?
		# - only analyse while we're airborne 
		# - absolute diff between in+out?
		# - accumulated diff between in+out?
		# - slope diff between in+out curves?
		# - roll/pitch over max in non-acro modes?
		# - if direct control use CTUN roll+pitch, if auto mode use NTUN data


		# figure out where each mode begins and ends, so we can treat auto and manual modes differently
		autoModes   = ["RTL","AUTO","LAND","LOITER","GUIDED","CIRCLE","OF_LOITER"]     # use NTUN DRol+DPit
		manualModes = ["STABILIZE","DRIFT","ALT_HOLD"]                                 # use CTUN RollIn/DesRoll + PitchIn/DesPitch
		ignoreModes = ["ACRO","SPORT","FLIP","AUTOTUNE"]                               # ignore data from these modes
		autoSegments   = []  # list of (startLine,endLine) pairs
		manualSegments = []  # list of (startLine,endLine) pairs
		orderedModes = collections.OrderedDict(sorted(logdata.modeChanges.items(), key=lambda t: t[0]))
		isAuto = False # always start in a manual control mode
		prevLine = 1
		for line,modepair in orderedModes.iteritems():
			mode = modepair[0].upper()
			if mode in autoModes:
				print "On line %d mode changed to %s (AUTO)" % (line,mode)   # TEMP
				if not isAuto:
					manualSegments.append((prevLine,line-1))
					print "  Previous manual segment: " + `(prevLine,line-1)`   # TEMP
					prevLine = line
				isAuto = True
			elif mode in manualModes:
				print "On line %d mode changed to %s (MANUAL)" % (line,mode)   # TEMP
				if isAuto:
					autoSegments.append((prevLine,line-1))
					print "  Previous auto segment: " + `(prevLine,line-1)`   # TEMP
					prevLine = line
				isAuto = False
			elif mode in ignoreModes:
				pass
			else:
				raise Exception("Unknown mode in TestPitchRollCoupling: %s" % mode)

		# look through manual segments
		for startLine,endLine in manualSegments:
			(value,attLine) = logdata.channels["ATT"]["Roll"].getNearestValue(startLine, lookForwards=True)
			print "Nearest ATT line after %d is %d" % (startLine,attLine)
			index = logdata.channels["ATT"]["Roll"].getIndexOf(attLine)
			print "First ATT line in manual segment (%d,%d) is line %d" % (startLine,endLine,logdata.channels["ATT"]["Roll"].listData[index][0])







		# look through auto segments
		# ...









