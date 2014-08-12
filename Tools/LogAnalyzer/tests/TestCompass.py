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

        try:
            # test that compass offset parameters are within recommended range (+/- 150)
            maxOffset = 150
            if logdata.hardwareType == "PX4":
                maxOffset = 250 # Pixhawks have their offsets scaled larger
            compassOfsX = logdata.parameters["COMPASS_OFS_X"]
            compassOfsY = logdata.parameters["COMPASS_OFS_Y"]
            compassOfsZ = logdata.parameters["COMPASS_OFS_Z"]
            if abs(compassOfsX) > maxOffset or abs(compassOfsY) > maxOffset or abs(compassOfsZ) > maxOffset:
                self.result.status = TestResult.StatusType.FAIL
                self.result.statusMessage = "Large compass off params (X:%.2f, Y:%.2f, Z:%.2f)\n" % (compassOfsX,compassOfsY,compassOfsZ)

            # check for excessive compass offsets using MAG data if present (it can change during flight is compass learn is on)
            if "MAG" in logdata.channels:
                errMsg = ""
                if logdata.channels["MAG"]["OfsX"].max() >  maxOffset:
                    errMsg  = errMsg + "\nX: %.2f" % logdata.channels["MAG"]["OfsX"].max()
                    err = True
                if logdata.channels["MAG"]["OfsX"].min() < -maxOffset:
                    errMsg = errMsg + "\nX: %.2f" % logdata.channels["MAG"]["OfsX"].min()
                    err = True
                if logdata.channels["MAG"]["OfsY"].max() >  maxOffset:
                    errMsg = errMsg + "\nY: %.2f" % logdata.channels["MAG"]["OfsY"].max()
                    err = True
                if logdata.channels["MAG"]["OfsY"].min() < -maxOffset:
                    errMsg = errMsg + "\nY: %.2f" % logdata.channels["MAG"]["OfsY"].min()
                    err = True
                if logdata.channels["MAG"]["OfsZ"].max() >  maxOffset:
                    errMsg = errMsg + "\nZ: %.2f" % logdata.channels["MAG"]["OfsZ"].max()
                    err = True
                if logdata.channels["MAG"]["OfsZ"].min() < -maxOffset:
                    errMsg = errMsg + "\nZ: %.2f" % logdata.channels["MAG"]["OfsZ"].min()
                    err = True
                if errMsg:
                    self.result.status = TestResult.StatusType.FAIL
                    self.result.statusMessage = self.result.statusMessage + "Large compass offset in MAG data:" + errMsg + "\n"

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
                    self.result.status = TestResult.StatusType.FAIL
                    self.result.statusMessage = self.result.statusMessage + "Large change in mag_field (%.2f%%)\n" % (percentDiff*100)
                elif percentDiff > percentDiffThresholdWARN:
                    if self.result.status != TestResult.StatusType.FAIL:
                        self.result.status = TestResult.StatusType.WARN
                    self.result.statusMessage = self.result.statusMessage + "Moderate change in mag_field (%.2f%%)\n" % (percentDiff*100)
                else:
                    self.result.statusMessage = self.result.statusMessage + "mag_field interference within limits (%.2f%%)\n" % (percentDiff*100)
                if minMagField < minMagFieldThreshold:
                    self.result.statusMessage = self.result.statusMessage + "Min mag field length (%.2f) < recommended (%.2f)\n" % (minMagField,minMagFieldThreshold)
                if maxMagField > maxMagFieldThreshold:
                    self.result.statusMessage = self.result.statusMessage + "Max mag field length (%.2f) > recommended (%.2f)\n" % (maxMagField,maxMagFieldThreshold)
                if zerosFound:
                    if self.result.status != TestResult.StatusType.FAIL:
                        self.result.status = TestResult.StatusType.WARN
                    self.result.statusMessage = self.result.statusMessage + "All zeros found in MAG X/Y/Z log data\n"
                if verbose:
                    self.result.statusMessage = self.result.statusMessage + "Min mag_field of %.2f on line %d\n" % (minMagField,minMagFieldLine)
                    self.result.statusMessage = self.result.statusMessage + "Max mag_field of %.2f on line %d\n" % (maxMagField,maxMagFieldLine)

            else:
                self.result.statusMessage = self.result.statusMessage + "No MAG data, unable to test mag_field interference\n"

        except KeyError as e:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = str(e) + ' not found'










