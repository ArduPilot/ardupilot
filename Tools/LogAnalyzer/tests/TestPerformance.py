# AP_FLAKE8_CLEAN


from __future__ import print_function

from LogAnalyzer import Test, TestResult
from VehicleType import VehicleType


class TestPerformance(Test):
    '''check performance monitoring messages (PM) for issues with slow loops, etc'''

    def __init__(self):
        Test.__init__(self)
        self.name = "PM"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        # this test should be valid for all vehicle types, just need to figure out why PM logging data is different in each
        if logdata.vehicleType != VehicleType.Copter:
            self.result.status = TestResult.StatusType.NA
            return

        # NOTE: we'll ignore MaxT altogether for now, it seems there are quite regularly one or two high values in
        # there, even ignoring the ones expected after arm/disarm events.
        # gather info on arm/disarm lines, we will ignore the MaxT data from the first line found after each of these
        # armingLines = []
        # for line,ev in logdata.channels["EV"]["Id"].listData:
        #   if (ev == 10) or (ev == 11):
        #       armingLines.append(line)
        # ignoreMaxTLines = []
        # for maxT in logdata.channels["PM"]["MaxT"].listData:
        #   if not armingLines:
        #       break
        #   if maxT[0] > armingLines[0]:
        #       ignoreMaxTLines.append(maxT[0])
        #       armingLines.pop(0)

        if "PM" not in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No PM log data"
            return

        # check for slow loops, i.e. NLon greater than 6% of NLoop
        maxPercentSlow = 0
        maxPercentSlowLine = 0
        slowLoopLineCount = 0
        for i in range(len(logdata.channels["PM"]["NLon"].listData)):
            (line, nLon) = logdata.channels["PM"]["NLon"].listData[i]
            (line, nLoop) = logdata.channels["PM"]["NLoop"].listData[i]
            (line, maxT) = logdata.channels["PM"]["MaxT"].listData[i]
            percentSlow = (nLon / float(nLoop)) * 100
            if percentSlow > 6.0:
                slowLoopLineCount = slowLoopLineCount + 1
                if percentSlow > maxPercentSlow:
                    maxPercentSlow = percentSlow
                    maxPercentSlowLine = line
        if (maxPercentSlow > 10) or (slowLoopLineCount > 6):
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = "%d slow loop lines found, max %.2f%% on line %d" % (
                slowLoopLineCount,
                maxPercentSlow,
                maxPercentSlowLine,
            )
        elif maxPercentSlow > 6:
            self.result.status = TestResult.StatusType.WARN
            self.result.statusMessage = "%d slow loop lines found, max %.2f%% on line %d" % (
                slowLoopLineCount,
                maxPercentSlow,
                maxPercentSlowLine,
            )
