# AP_FLAKE8_CLEAN


import math  # for isnan()

from LogAnalyzer import Test, TestResult
from VehicleType import VehicleType


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
            self.result.statusMessage = self.result.statusMessage + "%s set to %s, expecting %s\n" % (
                paramName,
                repr(value),
                repr(expectedValue),
            )

    def __checkParamIsLessThan(self, paramName, maxValue, logdata):
        value = logdata.parameters[paramName]
        if value >= maxValue:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = self.result.statusMessage + "%s set to %s, expecting less than %s\n" % (
                paramName,
                repr(value),
                repr(maxValue),
            )

    def __checkParamIsMoreThan(self, paramName, minValue, logdata):
        value = logdata.parameters[paramName]
        if value <= minValue:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = self.result.statusMessage + "%s set to %s, expecting less than %s\n" % (
                paramName,
                repr(value),
                repr(minValue),
            )

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD  # GOOD by default, tests below will override it if they fail

        # check all params for NaN
        for name, value in logdata.parameters.items():
            if math.isnan(value):
                self.result.status = TestResult.StatusType.FAIL
                self.result.statusMessage = self.result.statusMessage + name + " is NaN\n"

        try:
            # add parameter checks below using the helper functions, any failures will trigger a FAIL status and
            # accumulate info in statusMessage.
            # If more complex checking or correlations are required you can access parameter values directly using the
            # logdata.parameters[paramName] dict
            if logdata.vehicleType == VehicleType.Copter:
                self.__checkParamIsEqual("MAG_ENABLE", 1, logdata)
                if "THR_MIN" in logdata.parameters:
                    self.__checkParamIsLessThan("THR_MIN", 200, logdata)
                    self.__checkParamIsLessThan("THR_MID", 701, logdata)
                    self.__checkParamIsMoreThan("THR_MID", 299, logdata)
                # TODO: add more parameter tests, these are just an example...
            elif logdata.vehicleType == VehicleType.Plane:
                # TODO: add parameter checks for plane...
                pass
            elif logdata.vehicleType == VehicleType.Rover:
                # TODO: add parameter checks for rover...
                pass

            if self.result.status == TestResult.StatusType.FAIL:
                self.result.statusMessage = "Bad parameters found:\n" + self.result.statusMessage
        except KeyError as e:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = str(e) + ' not found'
