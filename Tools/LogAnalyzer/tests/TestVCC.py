# AP_FLAKE8_CLEAN

from LogAnalyzer import Test, TestResult


class TestVCC(Test):
    '''test for VCC within recommendations, or abrupt end to log in flight'''

    def __init__(self):
        Test.__init__(self)
        self.name = "VCC"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if "CURR" not in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No CURR log data"
            return

        # just a naive min/max test for now
        try:
            vccMin = logdata.channels["CURR"]["Vcc"].min()
            vccMax = logdata.channels["CURR"]["Vcc"].max()
        except KeyError:
            vccMin = logdata.channels["POWR"]["Vcc"].min()
            vccMax = logdata.channels["POWR"]["Vcc"].max()
            vccMin *= 1000
            vccMax *= 1000

        vccDiff = vccMax - vccMin
        vccMinThreshold = 4.6 * 1000
        vccMaxDiff = 0.3 * 1000
        if vccDiff > vccMaxDiff:
            self.result.status = TestResult.StatusType.WARN
            self.result.statusMessage = "VCC min/max diff %sv, should be <%sv" % (vccDiff / 1000.0, vccMaxDiff / 1000.0)
        elif vccMin < vccMinThreshold:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = "VCC below minimum of %sv (%sv)" % (
                repr(vccMinThreshold / 1000.0),
                repr(vccMin / 1000.0),
            )
