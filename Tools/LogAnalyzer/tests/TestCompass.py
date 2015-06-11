from LogAnalyzer import Test,TestResult
import DataflashLog

import math


class TestCompass(Test):
    '''test for compass offsets and throttle interference'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Compass"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        def vec_len(x):
            return sqrt(x[0]**2+x[1]**2+x[2]**2)

        def FAIL():
            self.result.status = TestResult.StatusType.FAIL

        def WARN():
            if self.result.status != TestResult.StatusType.FAIL:
                self.result.status = TestResult.StatusType.WARN

        try:
            warnOffset = 300
            failOffset = 500
            param_offsets = (
                logdata.parameters["COMPASS_OFS_X"],
                logdata.parameters["COMPASS_OFS_Y"],
                logdata.parameters["COMPASS_OFS_Z"]
                )

            if vec_len(param_offsets) > failOffset:
                FAIL()
                self.result.statusMessage = "FAIL: Large compass offset params (X:%.2f, Y:%.2f, Z:%.2f)\n" % (param_offsets[0],param_offsets[1],param_offsets[2])
            elif vec_len(param_offsets) > warnOffset:
                WARN()
                self.result.statusMessage = "WARN: Large compass offset params (X:%.2f, Y:%.2f, Z:%.2f)\n" % (param_offsets[0],param_offsets[1],param_offsets[2])

            if "MAG" in logdata.channels:
                max_log_offsets = zip(
                    map(lambda x: x[1],logdata.channels["MAG"]["OfsX"].listData),
                    map(lambda x: x[1],logdata.channels["MAG"]["OfsY"].listData),
                    map(lambda x: x[1],logdata.channels["MAG"]["OfsZ"].listData)
                    )
                max_log_offsets = reduce(lambda x,y: x if vec_len(x) > vec_len(y) else y, max_log_offsets)

                if vec_len(max_log_offsets) > failOffset:
                    FAIL()
                    self.result.statusMessage += "FAIL: Large compass offset in MAG data (X:%.2f, Y:%.2f, Z:%.2f)\n" % (max_log_offsets[0],max_log_offsets[1],max_log_offsets[2])
                elif vec_len(max_log_offsets) > warnOffset:
                    WARN()
                    self.result.statusMessage += "WARN: Large compass offset in MAG data (X:%.2f, Y:%.2f, Z:%.2f)\n" % (max_log_offsets[0],max_log_offsets[1],max_log_offsets[2])

            # check for mag field length change, and length outside of recommended range
            if "MAG" in logdata.channels:
                percentDiffThresholdWARN = 0.25
                percentDiffThresholdFAIL = 0.35
                minMagFieldThreshold = 120.0
                maxMagFieldThreshold = 550.0
                index = 0
                length = len(logdata.channels["MAG"]["MagX"].listData)
                magField = []
                (minMagField, maxMagField) = (None,None)
                (minMagFieldLine, maxMagFieldLine) = (None,None)
                zerosFound = False
                while index<length:
                    mx = logdata.channels["MAG"]["MagX"].listData[index][1]
                    my = logdata.channels["MAG"]["MagY"].listData[index][1]
                    mz = logdata.channels["MAG"]["MagZ"].listData[index][1]
                    if ((mx==0) and (my==0) and (mz==0)): # sometimes they're zero, not sure why, same reason as why we get NaNs as offsets?
                        zerosFound = True
                    else:
                        mf = math.sqrt(mx*mx + my*my + mz*mz)
                        magField.append(mf)
                        if mf<minMagField:
                            minMagField = mf
                            minMagFieldLine = logdata.channels["MAG"]["MagX"].listData[index][0]
                        if mf>maxMagField:
                            maxMagField = mf
                            maxMagFieldLine = logdata.channels["MAG"]["MagX"].listData[index][0]
                        if index == 0:
                            (minMagField, maxMagField) = (mf,mf)
                    index += 1
                percentDiff = (maxMagField-minMagField) / minMagField
                if percentDiff > percentDiffThresholdFAIL:
                    FAIL()
                    self.result.statusMessage = self.result.statusMessage + "Large change in mag_field (%.2f%%)\n" % (percentDiff*100)
                elif percentDiff > percentDiffThresholdWARN:
                    WARN()
                    self.result.statusMessage = self.result.statusMessage + "Moderate change in mag_field (%.2f%%)\n" % (percentDiff*100)
                else:
                    self.result.statusMessage = self.result.statusMessage + "mag_field interference within limits (%.2f%%)\n" % (percentDiff*100)
                if minMagField < minMagFieldThreshold:
                    self.result.statusMessage = self.result.statusMessage + "Min mag field length (%.2f) < recommended (%.2f)\n" % (minMagField,minMagFieldThreshold)
                if maxMagField > maxMagFieldThreshold:
                    self.result.statusMessage = self.result.statusMessage + "Max mag field length (%.2f) > recommended (%.2f)\n" % (maxMagField,maxMagFieldThreshold)
                if zerosFound:
                    WARN()
                    self.result.statusMessage = self.result.statusMessage + "All zeros found in MAG X/Y/Z log data\n"
                if verbose:
                    self.result.statusMessage = self.result.statusMessage + "Min mag_field of %.2f on line %d\n" % (minMagField,minMagFieldLine)
                    self.result.statusMessage = self.result.statusMessage + "Max mag_field of %.2f on line %d\n" % (maxMagField,maxMagFieldLine)

            else:
                self.result.statusMessage = self.result.statusMessage + "No MAG data, unable to test mag_field interference\n"

        except KeyError as e:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = str(e) + ' not found'










