from LogAnalyzer import Test,TestResult
import DataflashLog

import math # for isnan()


class TestParams(Test):
    '''test for any obviously bad parameters in the config'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Parameters"


    # helper functions
    def __checkParamIsEqual(self, paramName, expectedValue, logdata):
        value = logdata.parameters[paramName]
        if value != expectedValue:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = self.result.statusMessage + "%s set to %s, expecting %s\n" % (paramName, `value`, `expectedValue`)
    def __checkParamIsLessThan(self, paramName, maxValue, logdata):
        value = logdata.parameters[paramName]
        if value >= maxValue:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = self.result.statusMessage + "%s set to %s, expecting less than %s\n" % (paramName, `value`, `maxValue`)
    def __checkParamIsMoreThan(self, paramName, minValue, logdata):
        value = logdata.parameters[paramName]
        if value <= minValue:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = self.result.statusMessage + "%s set to %s, expecting less than %s\n" % (paramName, `value`, `minValue`)


    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD  # GOOD by default, tests below will override it if they fail

        # check all params for NaN
        for name,value in logdata.parameters.iteritems():
            if math.isnan(value):
                self.result.status = TestResult.StatusType.FAIL
                self.result.statusMessage = self.result.statusMessage + name + " is NaN\n"

        try:
            # add parameter checks below using the helper functions, any failures will trigger a FAIL status and accumulate info in statusMessage
            # if more complex checking or correlations are required you can access parameter values directly using the logdata.parameters[paramName] dict
            if logdata.vehicleType == "ArduCopter":
                self.__checkParamIsEqual   ("MAG_ENABLE",   1, logdata)
                self.__checkParamIsLessThan("THR_MIN",    200, logdata)
                self.__checkParamIsLessThan("THR_MID",    701, logdata)
                self.__checkParamIsMoreThan("THR_MID",    299, logdata)
                # TODO: add more parameter tests, these are just an example...
            elif logdata.vehicleType == "ArduPlane":
                # TODO: add parameter checks for plane...
                pass
            elif logdata.vehicleType == "ArduRover":
                # TODO: add parameter checks for rover...
                pass

            if self.result.status == TestResult.StatusType.FAIL:
                self.result.statusMessage = "Bad parameters found:\n" + self.result.statusMessage
        except KeyError as e:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = str(e) + ' not found'