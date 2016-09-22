from LogAnalyzer import Test,TestResult
import DataflashLog

import collections


class TestVCC(Test):
    '''test for VCC within recommendations, or abrupt end to log in flight'''

    def __init__(self):
        Test.__init__(self)
        self.name = "VCC"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if not "CURR" in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No CURR log data"
            return

        # just a naive min/max test for now
        try:
            vccMin  = logdata.channels["CURR"]["Vcc"].min()
            vccMax  = logdata.channels["CURR"]["Vcc"].max()
        except KeyError as e:
            # Vcc was renamed to Volts at some stage...
            vccMin  = logdata.channels["CURR"]["Volt"].min()
            vccMax  = logdata.channels["CURR"]["Volt"].max()
            # apparently it also changed to mV:
            vccMin *= 1000
            vccMax *= 1000

        vccDiff = vccMax - vccMin;
        vccMinThreshold = 4.6 * 1000;
        vccMaxDiff      = 0.3 * 1000;
        if vccDiff > vccMaxDiff:
            self.result.status = TestResult.StatusType.WARN
            self.result.statusMessage = "VCC min/max diff %sv, should be <%sv" % (vccDiff/1000.0, vccMaxDiff/1000.0)
        elif vccMin < vccMinThreshold:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = "VCC below minimum of %sv (%sv)" % (`vccMinThreshold/1000.0`,`vccMin/1000.0`)
        
