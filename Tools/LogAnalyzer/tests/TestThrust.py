# AP_FLAKE8_CLEAN


from __future__ import print_function

from LogAnalyzer import Test, TestResult
from VehicleType import VehicleType


class TestThrust(Test):
    '''test for sufficient thrust (copter only for now)'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Thrust"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if logdata.vehicleType != VehicleType.Copter:
            self.result.status = TestResult.StatusType.NA
            return

        if "CTUN" not in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No CTUN log data"
            return
        if "ATT" not in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No ATT log data"
            return

        throut_key = None
        for key in "ThO", "ThrOut":
            if key in logdata.channels["CTUN"]:
                throut_key = key
                break
        if throut_key is None:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "Could not find throttle out column"
            return

        # check for throttle (CTUN.ThrOut) above 700 for a chunk of time with copter not rising

        highThrottleThreshold = 700
        tiltThreshold = 20  # ignore high throttle when roll or tilt is above this value
        climbThresholdWARN = 100
        climbThresholdFAIL = 50
        minSampleLength = 50

        highThrottleSegments = []

        # find any contiguous chunks where CTUN.ThrOut > highThrottleThreshold, ignore high throttle if
        # tilt > tiltThreshold, and discard any segments shorter than minSampleLength
        start = None
        data = logdata.channels["CTUN"][throut_key].listData
        for i in range(0, len(data)):
            (lineNumber, value) = data[i]
            isBelowTiltThreshold = True
            if value > highThrottleThreshold:
                (roll, meh) = logdata.channels["ATT"]["Roll"].getNearestValue(lineNumber)
                (pitch, meh) = logdata.channels["ATT"]["Pitch"].getNearestValue(lineNumber)
                if (abs(roll) > tiltThreshold) or (abs(pitch) > tiltThreshold):
                    isBelowTiltThreshold = False
            if (value > highThrottleThreshold) and isBelowTiltThreshold:
                if start is None:
                    start = i
            elif start is not None:
                if (i - start) > minSampleLength:
                    highThrottleSegments.append((start, i))
                start = None

        climbRate = "CRate"
        if "CRate" not in logdata.channels["CTUN"]:
            climbRate = "CRt"

        # loop through each checking climbRate, if < 50 FAIL, if < 100 WARN
        # TODO: we should filter climbRate and use its slope rather than value for this test
        for seg in highThrottleSegments:
            (startLine, endLine) = (data[seg[0]][0], data[seg[1]][0])
            avgClimbRate = logdata.channels["CTUN"][climbRate].getSegment(startLine, endLine).avg()
            avgThrOut = logdata.channels["CTUN"][throut_key].getSegment(startLine, endLine).avg()
            if avgClimbRate < climbThresholdFAIL:
                self.result.status = TestResult.StatusType.FAIL
                self.result.statusMessage = "Avg climb rate %.2f cm/s for throttle avg %d" % (avgClimbRate, avgThrOut)
                return
            if avgClimbRate < climbThresholdWARN:
                self.result.status = TestResult.StatusType.WARN
                self.result.statusMessage = "Avg climb rate %.2f  cm/s for throttle avg %d" % (avgClimbRate, avgThrOut)
