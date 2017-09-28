from __future__ import print_function

from LogAnalyzer import Test,TestResult
import DataflashLog

import numpy


class TestVibration(Test):
    '''test for accelerometer vibration (accX/accY/accZ) within recommendations'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Vibration"


    def run(self, logdata, verbose):
        self.result = TestResult()

        if logdata.vehicleType != VehicleType.Copter:
            self.result.status = TestResult.StatusType.NA
            return

        # constants
        gravity        = -9.81
        aimRangeWarnXY = 1.5
        aimRangeFailXY = 3.0
        aimRangeWarnZ  = 2.0  # gravity +/- aim range
        aimRangeFailZ  = 5.0  # gravity +/- aim range

        if not "IMU" in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No IMU log data"
            return

        # find some stable LOITER data to analyze, at least 10 seconds
        chunks = DataflashLog.DataflashLogHelper.findLoiterChunks(logdata, minLengthSeconds=10, noRCInputs=True)
        if not chunks:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No stable LOITER log data found"       
            return

        # for now we'll just use the first (largest) chunk of LOITER data
        # TODO: ignore the first couple of secs to avoid bad data during transition - or can we check more analytically that we're stable?
        # TODO: accumulate all LOITER chunks over min size, or just use the largest one?
        startLine = chunks[0][0]
        endLine   = chunks[0][1]
        #print("TestVibration using LOITER chunk from lines %s to %s" % (repr(startLine), repr(endLine)))

        def getStdDevIMU(logdata, channelName, startLine,endLine):
            loiterData = logdata.channels["IMU"][channelName].getSegment(startLine,endLine)
            numpyData  = numpy.array(loiterData.dictData.values())
            return numpy.std(numpyData)

        # use 2x standard deviations as the metric, so if 95% of samples lie within the aim range we're good
        stdDevX = abs(2 * getStdDevIMU(logdata,"AccX",startLine,endLine))
        stdDevY = abs(2 * getStdDevIMU(logdata,"AccY",startLine,endLine))
        stdDevZ = abs(2 * getStdDevIMU(logdata,"AccZ",startLine,endLine))
        if (stdDevX > aimRangeFailXY) or (stdDevY > aimRangeFailXY) or (stdDevZ > aimRangeFailZ):
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = "Vibration too high (X:%.2fg, Y:%.2fg, Z:%.2fg)" % (stdDevX,stdDevY,stdDevZ)
        elif (stdDevX > aimRangeWarnXY) or (stdDevY > aimRangeWarnXY) or (stdDevZ > aimRangeWarnZ):
            self.result.status = TestResult.StatusType.WARN
            self.result.statusMessage = "Vibration slightly high (X:%.2fg, Y:%.2fg, Z:%.2fg)" % (stdDevX,stdDevY,stdDevZ)
        else:
            self.result.status = TestResult.StatusType.GOOD
            self.result.statusMessage = "Good vibration values (X:%.2fg, Y:%.2fg, Z:%.2fg)" % (stdDevX,stdDevY,stdDevZ)

        
