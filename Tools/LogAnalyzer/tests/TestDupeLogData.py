from __future__ import print_function

from LogAnalyzer import Test,TestResult
import DataflashLog


class TestDupeLogData(Test):
	'''test for duplicated data in log, which has been happening on PX4/Pixhawk'''

	def __init__(self):
		Test.__init__(self)
		self.name = "Dupe Log Data"
		
	def __matchSample(self, sample, sampleStartIndex, logdata):
		'''return the line number where a match is found, otherwise return False'''

		# ignore if all data in sample is the same value
		nSame = 0
		for s in sample:
			if s[1] == sample[0][1]:
				nSame += 1
		if nSame == 20:
			return False

		# c
		data = logdata.channels["ATT"]["Pitch"].listData
		for i in range(sampleStartIndex, len(data)):
			#print("Checking against index %d" % i)
			if i == sampleStartIndex:
				continue # skip matching against ourselves
			j = 0
			while j<20 and (i+j)<len(data) and data[i+j][1] == sample[j][1]:
				#print("### Match found, j=%d, data=%f, sample=%f, log data matched to sample at line %d" % (j,data[i+j][1],sample[j][1],data[i+j][0]))
				j += 1
			if j == 20: # all samples match
				return data[i][0]

		return False

	def run(self, logdata, verbose):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.GOOD

		# this could be made more flexible by not hard-coding to use ATT data, could make it dynamic based on whatever is available as long as it is highly variable
		if "ATT" not in logdata.channels:
			self.result.status = TestResult.StatusType.UNKNOWN
			self.result.statusMessage = "No ATT log data"
			return

		# pick 10 sample points within the range of ATT data we have
		sampleStartIndices = []
		attStartIndex = 0
		attEndIndex   = len(logdata.channels["ATT"]["Pitch"].listData)-1
		step = attEndIndex / 11
		for i in range(step,attEndIndex-step,step):
			sampleStartIndices.append(i)
			#print("Dupe data sample point index %d at line %d" % (i, logdata.channels["ATT"]["Pitch"].listData[i][0]))

		# get 20 datapoints of pitch from each sample location and check for a match elsewhere
		sampleIndex = 0
		for i in range(sampleStartIndices[0], len(logdata.channels["ATT"]["Pitch"].listData)):
			if i == sampleStartIndices[sampleIndex]:
				#print("Checking sample %d" % i)
				sample = logdata.channels["ATT"]["Pitch"].listData[i:i+20]
				matchedLine = self.__matchSample(sample, i, logdata)
				if matchedLine:
					#print("Data from line %d found duplicated at line %d" % (sample[0][0],matchedLine))
					self.result.status = TestResult.StatusType.FAIL
					self.result.statusMessage = "Duplicate data chunks found in log (%d and %d)" % (sample[0][0],matchedLine)
					return
				sampleIndex += 1
				if sampleIndex >= len(sampleStartIndices):
					break






