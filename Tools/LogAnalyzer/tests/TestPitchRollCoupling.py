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
		# TODO: fill in all known modes here 
		autoModes   = ["RTL","AUTO","LAND","LOITER"]     # use NTUN DRol+DPit
		manualModes = ["STABILIZE","DRIFT","ALT_HOLD"]   # use CTUN RollIn/DesRoll + PitchIn/DesPitch
		acroModes   = ["ACRO","SPORT"]                   # ignore data from acro modes
		autoSegments   = []  # list of (startLine,endLine) pairs
		manualSegments = []  # list of (startLine,endLine) pairs
		orderedModes = collections.OrderedDict(sorted(logdata.modeChanges.items(), key=lambda t: t[0]))
		isAuto = False # always start in a manual control mode
		prevLine = 1
		for line,modepair in orderedModes.iteritems():
			mode = modepair[0].upper()
			if mode in autoModes:
				print "On line %d mode changed to %s (AUTO)" % (line,mode)
				if not isAuto:
					manualSegments.append((prevLine,line-1))
					print "  Previous manual segment: " + `(prevLine,line-1)`
				isAuto = True
			elif mode in manualModes:
				print "On line %d mode changed to %s (MANUAL)" % (line,mode)
				if isAuto:
					autoSegments.append((prevLine,line-1))
					print "  Previous auto segment: " + `(prevLine,line-1)`
				isAuto = False
			elif mode in acroModes:
				pass
			else:
				raise Exception("Unknown mode: %s" % mode)
			prevLine = line
			if isAuto:
				autoSegments.append((prevLine,logdata.lineCount))
				print "  Previous auto segment: " + `(prevLine,logdata.lineCount)`
			else:
				manualSegments.append((prevLine,logdata.lineCount))
				print "  Previous manual segment: " + `(prevLine,logdata.lineCount)`
