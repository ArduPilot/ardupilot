from LogAnalyzer import Test,TestResult
import DataflashLog


class TestCompass(Test):
	'''test for compass offsets and throttle interference'''

	def __init__(self):
		self.name = "Compass"

	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		# test that compass offset parameters are within recommended range (+/- 150)
		maxOffset = 150
		if logdata.hardwareType == "PX4":
			maxOffset = 250 # Pixhawks have their offsets scaled larger
		compassOfsX = logdata.parameters["COMPASS_OFS_X"]
		compassOfsY = logdata.parameters["COMPASS_OFS_Y"]
		compassOfsZ = logdata.parameters["COMPASS_OFS_Z"]
		#print "MAG params: %.2f %.2f %.2f" % (compassOfsX,compassOfsY,compassOfsZ)
		if abs(compassOfsX) > maxOffset or abs(compassOfsY) > maxOffset or abs(compassOfsZ) > maxOffset:
			self.result.status = TestResult.StatusType.FAIL
			self.result.statusMessage = "Large compass off params (X:%.2f, Y:%.2f, Z:%.2f)\n" % (compassOfsX,compassOfsY,compassOfsZ)

		# check for excessive compass offsets using MAG data if present (it can change during flight is compass learn is on)
		if "MAG" in logdata.channels:
			errMsg = ""
			if logdata.channels["MAG"]["OfsX"].max() >  maxOffset:
				errMsg  = errMsg + "\nX: %.2f\n" % logdata.channels["MAG"]["OfsX"].max()
				err = True
			if logdata.channels["MAG"]["OfsX"].min() < -maxOffset:
				errMsg = errMsg + "\nX: %.2f\n" % logdata.channels["MAG"]["OfsX"].min()
				err = True
			if logdata.channels["MAG"]["OfsY"].max() >  maxOffset:
				errMsg = errMsg + "\nY: %.2f\n" % logdata.channels["MAG"]["OfsY"].max()
				err = True
			if logdata.channels["MAG"]["OfsY"].min() < -maxOffset:
				errMsg = errMsg + "\nY: %.2f\n" % logdata.channels["MAG"]["OfsY"].min()
				err = True
			if logdata.channels["MAG"]["OfsZ"].max() >  maxOffset:
				errMsg = errMsg + "\nZ: %.2f\n" % logdata.channels["MAG"]["OfsZ"].max()
				err = True
			if logdata.channels["MAG"]["OfsZ"].min() < -maxOffset:
				errMsg = errMsg + "\nZ: %.2f\n" % logdata.channels["MAG"]["OfsZ"].min()
				err = True
			if errMsg:
				self.result.status = TestResult.StatusType.FAIL
				self.result.statusMessage = self.result.statusMessage + "Large compass offset in MAG data:" + errMsg

		# TODO: check for compass/throttle interference. Need to add mag_field to logging, or calc our own from MAG data if present
		# TODO: can we derive a compassmot percentage from the params? Fail if >30%?				
