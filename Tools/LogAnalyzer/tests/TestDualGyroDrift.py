from LogAnalyzer import Test,TestResult
import DataflashLog

# import scipy
# import pylab  ####   TEMP!!! only for dev
# from scipy import signal


class TestDualGyroDrift(Test):
	'''test for gyro drift between dual IMU data'''

	def __init__(self):
		Test.__init__(self)
		self.name = "Gyro Drift"
		self.enable = False 
		
	def run(self, logdata, verbose):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.GOOD

		# if "IMU" not in logdata.channels or "IMU2" not in logdata.channels:
		# 	self.result.status = TestResult.StatusType.NA
		# 	return

		# imuX  = logdata.channels["IMU"]["GyrX"].listData
		# imu2X = logdata.channels["IMU2"]["GyrX"].listData

		# # NOTE: weird thing about Holger's log is that the counts of IMU+IMU2 are different
		# print "length 1: %.2f, length 2: %.2f" % (len(imuX),len(imu2X))
		# #assert(len(imuX) == len(imu2X))

		# # divide the curve into segments and get the average of each segment
		# # we will get the diff between those averages, rather than a per-sample diff as the IMU+IMU2 arrays are often not the same length
		# diffThresholdWARN = 0.03
		# diffThresholdFAIL = 0.05
		# nSamples = 10
		# imu1XAverages, imu1YAverages, imu1ZAverages, imu2XAverages, imu2YAverages, imu2ZAverages = ([],[],[],[],[],[])
		# imuXDiffAverages, imuYDiffAverages, imuZDiffAverages = ([],[],[])
		# maxDiffX, maxDiffY, maxDiffZ = (0,0,0)
		# sliceLength1 = len(logdata.channels["IMU"]["GyrX"].dictData.values())  / nSamples
		# sliceLength2 = len(logdata.channels["IMU2"]["GyrX"].dictData.values()) / nSamples
		# for i in range(0,nSamples):
		# 	imu1XAverages.append(numpy.mean(logdata.channels["IMU"]["GyrX"].dictData.values()[i*sliceLength1:i*sliceLength1+sliceLength1]))
		# 	imu1YAverages.append(numpy.mean(logdata.channels["IMU"]["GyrY"].dictData.values()[i*sliceLength1:i*sliceLength1+sliceLength1]))
		# 	imu1ZAverages.append(numpy.mean(logdata.channels["IMU"]["GyrZ"].dictData.values()[i*sliceLength1:i*sliceLength1+sliceLength1]))
		# 	imu2XAverages.append(numpy.mean(logdata.channels["IMU2"]["GyrX"].dictData.values()[i*sliceLength2:i*sliceLength2+sliceLength2]))
		# 	imu2YAverages.append(numpy.mean(logdata.channels["IMU2"]["GyrY"].dictData.values()[i*sliceLength2:i*sliceLength2+sliceLength2]))
		# 	imu2ZAverages.append(numpy.mean(logdata.channels["IMU2"]["GyrZ"].dictData.values()[i*sliceLength2:i*sliceLength2+sliceLength2]))
		# 	imuXDiffAverages.append(imu2XAverages[-1]-imu1XAverages[-1])
		# 	imuYDiffAverages.append(imu2YAverages[-1]-imu1YAverages[-1])
		# 	imuZDiffAverages.append(imu2ZAverages[-1]-imu1ZAverages[-1])
		# 	if abs(imuXDiffAverages[-1]) > maxDiffX:
		# 		maxDiffX = imuXDiffAverages[-1]
		# 	if abs(imuYDiffAverages[-1]) > maxDiffY:
		# 		maxDiffY = imuYDiffAverages[-1]
		# 	if abs(imuZDiffAverages[-1]) > maxDiffZ:
		# 		maxDiffZ = imuZDiffAverages[-1]
		
		# if max(maxDiffX,maxDiffY,maxDiffZ) > diffThresholdFAIL:
		# 	self.result.status = TestResult.StatusType.FAIL
		# 	self.result.statusMessage = "IMU/IMU2 gyro averages differ by more than %s radians" % diffThresholdFAIL
		# elif max(maxDiffX,maxDiffY,maxDiffZ) > diffThresholdWARN:
		# 	self.result.status = TestResult.StatusType.WARN
		# 	self.result.statusMessage = "IMU/IMU2 gyro averages differ by more than %s radians" % diffThresholdWARN

		# # pylab.plot(zip(*imuX)[0], zip(*imuX)[1], 'g')
		# # pylab.plot(zip(*imu2X)[0], zip(*imu2X)[1], 'r')

		# #pylab.plot(range(0,(nSamples*sliceLength1),sliceLength1), imu1ZAverages, 'b')

		# print "Gyro averages1X: " + `imu1XAverages`
		# print "Gyro averages1Y: " + `imu1YAverages`
		# print "Gyro averages1Z: " + `imu1ZAverages` + "\n"
		# print "Gyro averages2X: " + `imu2XAverages`
		# print "Gyro averages2Y: " + `imu2YAverages`
		# print "Gyro averages2Z: " + `imu2ZAverages` + "\n"
		# print "Gyro averages diff X: " + `imuXDiffAverages`
		# print "Gyro averages diff Y: " + `imuYDiffAverages`
		# print "Gyro averages diff Z: " + `imuZDiffAverages`

		# # lowpass filter using numpy
		# # cutoff = 100
		# # fs = 10000.0
		# # b,a = scipy.signal.filter_design.butter(5,cutoff/(fs/2))
		# # imuXFiltered  = scipy.signal.filtfilt(b,a,zip(*imuX)[1])
		# # imu2XFiltered = scipy.signal.filtfilt(b,a,zip(*imu2X)[1])
		# #pylab.plot(imuXFiltered, 'r')


		# # TMP: DISPLAY BEFORE+AFTER plots
		# pylab.show()

		# # print "imuX  average before lowpass filter: %.8f" % logdata.channels["IMU"]["GyrX"].avg()
		# # print "imuX  average after  lowpass filter: %.8f" % numpy.mean(imuXFiltered)
		# # print "imu2X average before lowpass filter: %.8f" % logdata.channels["IMU2"]["GyrX"].avg()
		# # print "imu2X average after  lowpass filter: %.8f" % numpy.mean(imu2XFiltered)

		# avg1X = logdata.channels["IMU"]["GyrX"].avg()
		# avg1Y = logdata.channels["IMU"]["GyrY"].avg()
		# avg1Z = logdata.channels["IMU"]["GyrZ"].avg()
		# avg2X = logdata.channels["IMU2"]["GyrX"].avg()
		# avg2Y = logdata.channels["IMU2"]["GyrY"].avg()
		# avg2Z = logdata.channels["IMU2"]["GyrZ"].avg()

		# avgRatioX = (max(avg1X,avg2X) - min(avg1X,avg2X)) /     #abs(max(avg1X,avg2X) / min(avg1X,avg2X))
		# avgRatioY = abs(max(avg1Y,avg2Y) / min(avg1Y,avg2Y))
		# avgRatioZ = abs(max(avg1Z,avg2Z) / min(avg1Z,avg2Z))

		# self.result.statusMessage = "IMU  gyro avg: %.4f,%.4f,%.4f\nIMU2 gyro avg: %.4f,%.4f,%.4f\nAvg ratio: %.4f,%.4f,%.4f" % (avg1X,avg1Y,avg1Z, avg2X,avg2Y,avg2Z, avgRatioX,avgRatioY,avgRatioZ)










