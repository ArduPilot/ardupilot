# AP_FLAKE8_CLEAN

from LogAnalyzer import Test, TestResult


class TestEvents(Test):
    '''test for erroneous events and failsafes'''

    # TODO: need to check for vehicle-specific codes

    def __init__(self):
        Test.__init__(self)
        self.name = "Event/Failsafe"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        errors = set()

        if "ERR" in logdata.channels:
            assert len(logdata.channels["ERR"]["Subsys"].listData) == len(logdata.channels["ERR"]["ECode"].listData)
            for i in range(len(logdata.channels["ERR"]["Subsys"].listData)):
                subSys = logdata.channels["ERR"]["Subsys"].listData[i][1]
                eCode = logdata.channels["ERR"]["ECode"].listData[i][1]
                if subSys == 2 and (eCode == 1):
                    errors.add("PPM")
                elif subSys == 3 and (eCode == 1 or eCode == 2):
                    errors.add("COMPASS")
                elif subSys == 5 and (eCode == 1):
                    errors.add("FS_THR")
                elif subSys == 6 and (eCode == 1):
                    errors.add("FS_BATT")
                elif subSys == 7 and (eCode == 1):
                    errors.add("GPS")
                elif subSys == 8 and (eCode == 1):
                    errors.add("GCS")
                elif subSys == 9 and (eCode == 1 or eCode == 2):
                    errors.add("FENCE")
                elif subSys == 10:
                    errors.add("FLT_MODE")
                elif subSys == 11 and (eCode == 2):
                    errors.add("GPS_GLITCH")
                elif subSys == 12 and (eCode == 1):
                    errors.add("CRASH")

        if errors:
            if len(errors) == 1 and "FENCE" in errors:
                self.result.status = TestResult.StatusType.WARN
            else:
                self.result.status = TestResult.StatusType.FAIL
                if len(errors) == 1:
                    self.result.statusMessage = "ERR found: "
                else:
                    self.result.statusMessage = "ERRs found: "
            for err in errors:
                self.result.statusMessage = self.result.statusMessage + err + " "
