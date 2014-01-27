from LogAnalyzer import Test,TestResult
import DataflashLog


class TestCompass(Test):
	'''test for compass offsets and throttle interference'''

	def __init__(self):
		self.name = "Compass"

	def run(self, logdata):
		self.result = TestResult()
		self.result.status = TestResult.StatusType.PASS

		# quick test that compass offset parameters are within recommended range (+/- 150)
		maxOffset = 150
		if logdata.hardwareType == "PX4":
			maxOffset = 250 # Pixhawks have their offsets scaled larger
		compassOfsX = logdata.parameters["COMPASS_OFS_X"]
		compassOfsY = logdata.parameters["COMPASS_OFS_Y"]
		compassOfsZ = logdata.parameters["COMPASS_OFS_Z"]
		#print "MAG params: %.2f %.2f %.2f" % (compassOfsX,compassOfsY,compassOfsZ)
		if abs(compassOfsX) > maxOffset or abs(compassOfsY) > maxOffset or abs(compassOfsZ) > maxOffset:
			self.result.status = TestResult.StatusType.FAIL
			self.result.statusMessage = "Large compass off params (X:%.2f, Y:%.2f, Z:%.2f)" % (compassOfsX,compassOfsY,compassOfsZ)

		# check for excessive compass offsets using MAG data if present (it can change during flight is compass learn is on)
		if "MAG" in logdata.channels:
			err = False
			#print "MAG min/max xyz... %.2f %.2f %.2f %.2f %.2f %.2f " % (logdata.channels["MAG"]["OfsX"].min(), logdata.channels["MAG"]["OfsX"].max(), logdata.channels["MAG"]["OfsY"].min(), logdata.channels["MAG"]["OfsY"].min(), logdata.channels["MAG"]["OfsZ"].min(), logdata.channels["MAG"]["OfsZ"].max())
			if logdata.channels["MAG"]["OfsX"].max() >  maxOffset:
				self.result.extraFeedback = self.result.extraFeedback + "X: %.2f\n" % logdata.channels["MAG"]["OfsX"].max()
				err = True
			if logdata.channels["MAG"]["OfsX"].min() < -maxOffset:
				self.result.extraFeedback = self.result.extraFeedback + "X: %.2f\n" % logdata.channels["MAG"]["OfsX"].min()
				err = True
			if logdata.channels["MAG"]["OfsY"].max() >  maxOffset:
				self.result.extraFeedback = self.result.extraFeedback + "Y: %.2f\n" % logdata.channels["MAG"]["OfsY"].max()
				err = True
			if logdata.channels["MAG"]["OfsY"].min() < -maxOffset:
				self.result.extraFeedback = self.result.extraFeedback + "Y: %.2f\n" % logdata.channels["MAG"]["OfsY"].min()
				err = True
			if logdata.channels["MAG"]["OfsZ"].max() >  maxOffset:
				self.result.extraFeedback = self.result.extraFeedback + "Z: %.2f\n" % logdata.channels["MAG"]["OfsZ"].max()
				err = True
			if logdata.channels["MAG"]["OfsZ"].min() < -maxOffset:
				self.result.extraFeedback = self.result.extraFeedback + "Z: %.2f\n" % logdata.channels["MAG"]["OfsZ"].min()
				err = True
			if err:
				self.result.status = TestResult.StatusType.FAIL
				self.result.statusMessage = "Large compass offset in MAG data"

		# TODO: check for compass/throttle interference. Need to add mag_field to logging, or calc our own from MAG data if present
		# TODO: can we derive a compassmot percentage from the params? Fail if >30%?				
