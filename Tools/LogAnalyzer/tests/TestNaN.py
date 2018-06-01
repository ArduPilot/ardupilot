from LogAnalyzer import Test,TestResult
import math

class TestNaN(Test):
    '''test for NaNs present in log'''

    def __init__(self):
        Test.__init__(self)
        self.name = "NaNs"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        def FAIL():
            self.result.status = TestResult.StatusType.FAIL


        for channel in logdata.channels.keys():
            for field in logdata.channels[channel].keys():
                try:
                    for tupe in logdata.channels[channel][field].listData:
                        (ts, val) = tupe
                        if isinstance(val, float) and math.isnan(val):
                            FAIL()
                            self.result.statusMessage += "Found NaN in %s.%s\n" % (channel, field,)
                            raise ValueError()
                except ValueError as e:
                    continue
