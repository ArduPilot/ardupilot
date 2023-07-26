# AP_FLAKE8_CLEAN

from LogAnalyzer import Test, TestResult


class TestBrownout(Test):
    '''test for a log that has been truncated in flight'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Brownout"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        isArmed = False
        # FIXME: cope with LOG_ARM_DISARM_MSG message
        if "EV" in logdata.channels:
            # step through the arm/disarm events in order, to see if they're symmetrical
            # note: it seems landing detection isn't robust enough to rely upon here, so we'll only consider arm+disarm,
            # not takeoff+land
            for line, ev in logdata.channels["EV"]["Id"].listData:
                if ev == 10:
                    isArmed = True
                elif ev == 11:
                    isArmed = False

        if "CTUN" not in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No CTUN log data"
            return

        if "BarAlt" in logdata.channels['CTUN']:
            self.ctun_baralt_att = 'BarAlt'
        else:
            self.ctun_baralt_att = 'BAlt'

        # check for relative altitude at end
        (finalAlt, finalAltLine) = logdata.channels["CTUN"][self.ctun_baralt_att].getNearestValue(
            logdata.lineCount, lookForwards=False
        )

        finalAltMax = 3.0  # max alt offset that we'll still consider to be on the ground
        if isArmed and finalAlt > finalAltMax:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = "Truncated Log? Ends while armed at altitude %.2fm" % finalAlt
