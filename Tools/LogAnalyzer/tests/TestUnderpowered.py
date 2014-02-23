from LogAnalyzer import Test,TestResult
import DataflashLog


class TestUnderpowered(Test):
	'''test for underpowered copter'''

	def __init__(self):
		self.name = "Underpowered"
		
	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		if logdata.vehicleType != "ArduCopter":
			self.result.status = TestResult.StatusType.NA
			return

		if not "CTUN" in logdata.channels:
			self.result.status = TestResult.StatusType.UNKNOWN
			self.result.statusMessage = "No CTUN log data"
			return
		if not "ATT" in logdata.channels:
			self.result.status = TestResult.StatusType.UNKNOWN
			self.result.statusMessage = "No ATT log data"
			return

		# check for throttle (CTUN.ThrOut) above 700 for a chunk of time with copter not rising

		highThrottleThreshold = 700
		tiltThreshold = 20 # ignore high throttle when roll or tilt is above this value
		climbThresholdWARN = 100
		climbThresholdFAIL = 50
		minSampleLength = 50

		highThrottleSegments = []

		# find any contiguous chunks where CTUN.ThrOut > highThrottleThreshold, ignore high throttle if tilt > tiltThreshold, and discard any segments shorter than minSampleLength
		start = None
		data = logdata.channels["CTUN"]["ThrOut"].listData
		for i in range(0,len(data)):
			(lineNumber,value) = data[i]
			isBelowTiltThreshold = True
			if value > highThrottleThreshold:
				(roll,meh)  = logdata.channels["ATT"]["Roll"].getNearestValue(lineNumber)
				(pitch,meh) = logdata.channels["ATT"]["Pitch"].getNearestValue(lineNumber)
				if (abs(roll) > tiltThreshold) or (abs(pitch) > tiltThreshold):
					isBelowTiltThreshold = False
			if (value > highThrottleThreshold) and isBelowTiltThreshold:
				if start == None:
					start = i
			elif start != None:
				if (i-start) > minSampleLength:
					#print "Found high throttle chunk from line %d to %d (%d samples)" % (data[start][0],data[i][0],i-start+1)
					highThrottleSegments.append((start,i))
				start = None

		# loop through each checking CTUN.CRate, if < 50 FAIL, if < 100 WARN
		# TODO: we should filter CRate and use its slope rather than value for this test
		for seg in highThrottleSegments:
			(startLine,endLine) = (data[seg[0]][0], data[seg[1]][0])
			avgClimbRate = logdata.channels["CTUN"]["CRate"].getSegment(startLine,endLine).avg()
			avgThrOut    = logdata.channels["CTUN"]["ThrOut"].getSegment(startLine,endLine).avg()
			#print "    Average CRate for this chunk is %.2f" % avgClimbRate
			if avgClimbRate < climbThresholdFAIL:
				self.result.status = TestResult.StatusType.FAIL
				self.result.statusMessage = "Avg climb rate %.2f cm/s for throttle avg %d" % (avgClimbRate,avgThrOut)
				return
			if avgClimbRate < climbThresholdWARN:
				self.result.status = TestResult.StatusType.WARN
				self.result.statusMessage = "Avg climb rate %.2f  cm/s for throttle avg %d" % (avgClimbRate,avgThrOut)



