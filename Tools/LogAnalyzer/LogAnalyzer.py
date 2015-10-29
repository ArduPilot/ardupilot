#!/usr/bin/env python
#
# A module to analyze and identify any common problems which can be determined from log files
#
# Initial code by Andrew Chapman (amchapman@gmail.com), 16th Jan 2014
#


# some logging oddities noticed while doing this, to be followed up on:
#   - tradheli MOT labels Mot1,Mot2,Mot3,Mot4,GGain
#   - Pixhawk doesn't output one of the FMT labels... forget which one
#   - MAG offsets seem to be constant (only seen data on Pixhawk)
#   - MAG offsets seem to be cast to int before being output? (param is -84.67, logged as -84)
#   - copter+plane use 'V' in their vehicle type/version/build line, rover uses lower case 'v'. Copter+Rover give a build number, plane does not
#   - CTUN.ThrOut on copter is 0-1000, on plane+rover it is 0-100

# TODO: add test for noisy baro values
# TODO: support loading binary log files (use Tridge's mavlogdump?)


import DataflashLog

import pprint  # temp
import imp
import glob
import inspect
import os, sys
import argparse
import datetime
import time
from xml.sax.saxutils import escape

from VehicleType import VehicleType

class TestResult(object):
    '''all tests return a standardized result type'''
    class StatusType:
        # NA means not applicable for this log (e.g. copter tests against a plane log), UNKNOWN means it is missing data required for the test
        GOOD, FAIL, WARN, UNKNOWN, NA = range(5)
    status = None
    statusMessage = "" # can be multi-line


class Test(object):
    '''base class to be inherited by log tests. Each test should be quite granular so we have lots of small tests with clear results'''
    def __init__(self):
        self.name     = ""
        self.result   = None   # will be an instance of TestResult after being run
        self.execTime = None
        self.enable   = True

    def run(self, logdata, verbose=False):
        pass


class TestSuite(object):
    '''registers test classes, loading using a basic plugin architecture, and can run them all in one run() operation'''
    def __init__(self):
        self.tests   = []
        self.logfile = None
        self.logdata = None  
        # dynamically load in Test subclasses from the 'tests' folder
        # to prevent one being loaded, move it out of that folder, or set that test's .enable attribute to False
        dirName = os.path.dirname(os.path.abspath(__file__))
        testScripts = glob.glob(dirName + '/tests/*.py')
        testClasses = []
        for script in testScripts:
            m = imp.load_source("m",script)
            for name, obj in inspect.getmembers(m, inspect.isclass):
                if name not in testClasses and inspect.getsourcefile(obj) == script:
                    testClasses.append(name)
                    self.tests.append(obj())

        # and here's an example of explicitly loading a Test class if you wanted to do that
        # m = imp.load_source("m", dirName + '/tests/TestBadParams.py')
        # self.tests.append(m.TestBadParams())

    def run(self, logdata, verbose):
        '''run all registered tests in a single call, gathering execution timing info'''
        self.logdata = logdata
        if 'GPS' not in self.logdata.channels and 'GPS2' in self.logdata.channels:
            # *cough*
            self.logdata.channels['GPS'] = self.logdata.channels['GPS2']

        self.logfile = logdata.filename
        for test in self.tests:
            # run each test in turn, gathering timing info
            if test.enable:
                startTime = time.time()
                test.run(self.logdata, verbose)  # RUN THE TEST
                endTime = time.time()
                test.execTime = 1000 * (endTime-startTime)

    def outputPlainText(self, outputStats):
        '''output test results in plain text'''
        print 'Dataflash log analysis report for file: ' + self.logfile
        print 'Log size: %.2fmb (%d lines)' % (self.logdata.filesizeKB / 1024.0, self.logdata.lineCount)
        print 'Log duration: %s' % str(datetime.timedelta(seconds=self.logdata.durationSecs)) + '\n'

        if self.logdata.vehicleType == VehicleType.Copter and self.logdata.getCopterType():
            print 'Vehicle Type: %s (%s)' % (self.logdata.vehicleTypeString, self.logdata.getCopterType())
        else:
            print 'Vehicle Type: %s' % self.logdata.vehicleTypeString
        print 'Firmware Version: %s (%s)' % (self.logdata.firmwareVersion, self.logdata.firmwareHash)
        print 'Hardware: %s' % self.logdata.hardwareType
        print 'Free RAM: %s' % self.logdata.freeRAM
        if self.logdata.skippedLines:
            print "\nWARNING: %d malformed log lines skipped during read" % self.logdata.skippedLines
        print '\n'

        print "Test Results:"
        for test in self.tests:
            if not test.enable:
                continue
            statusMessageFirstLine = test.result.statusMessage.strip('\n\r').split('\n')[0]
            statusMessageExtra     = test.result.statusMessage.strip('\n\r').split('\n')[1:]
            execTime = ""
            if outputStats:
                execTime = "  (%6.2fms)" % (test.execTime)
            if test.result.status == TestResult.StatusType.GOOD:
                print "  %20s:  GOOD       %-55s%s" % (test.name, statusMessageFirstLine, execTime)
            elif test.result.status == TestResult.StatusType.FAIL:
                print "  %20s:  FAIL       %-55s%s    [GRAPH]" % (test.name, statusMessageFirstLine, execTime)
            elif test.result.status == TestResult.StatusType.WARN:
                print "  %20s:  WARN       %-55s%s    [GRAPH]" % (test.name, statusMessageFirstLine, execTime)
            elif test.result.status == TestResult.StatusType.NA:
                # skip any that aren't relevant for this vehicle/hardware/etc
                continue
            else:
                print "  %20s:  UNKNOWN    %-55s%s" % (test.name, statusMessageFirstLine, execTime)
            #if statusMessageExtra:
            for line in statusMessageExtra:
                print "  %29s     %s" % ("",line)

        print '\n'
        print 'The Log Analyzer is currently BETA code.\nFor any support or feedback on the log analyzer please email Andrew Chapman (amchapman@gmail.com)'
        print '\n'

    def outputXML(self, xmlFile):
        '''output test results to an XML file'''

        # open the file for writing
        xml = None
        try:
            if xmlFile == '-':
                xml = sys.stdout
            else:
                xml = open(xmlFile, 'w')
        except:
            sys.stderr.write("Error opening output xml file: %s" % xmlFile)
            sys.exit(1)


        # output header info
        print >>xml, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        print >>xml, "<loganalysis>"
        print >>xml, "<header>"
        print >>xml, "  <logfile>"   + escape(self.logfile) + "</logfile>"
        print >>xml, "  <sizekb>"    + escape(`self.logdata.filesizeKB`) + "</sizekb>"
        print >>xml, "  <sizelines>" + escape(`self.logdata.lineCount`) + "</sizelines>"
        print >>xml, "  <duration>"  + escape(str(datetime.timedelta(seconds=self.logdata.durationSecs))) + "</duration>"
        print >>xml, "  <vehicletype>" + escape(self.logdata.vehicleTypeString) + "</vehicletype>"
        if self.logdata.vehicleType == VehicleType.Copter and self.logdata.getCopterType():
            print >>xml, "  <coptertype>"  + escape(self.logdata.getCopterType()) + "</coptertype>"
        print >>xml, "  <firmwareversion>" + escape(self.logdata.firmwareVersion) + "</firmwareversion>"
        print >>xml, "  <firmwarehash>" + escape(self.logdata.firmwareHash) + "</firmwarehash>"
        print >>xml, "  <hardwaretype>" + escape(self.logdata.hardwareType) + "</hardwaretype>"
        print >>xml, "  <freemem>" + escape(`self.logdata.freeRAM`) + "</freemem>"
        print >>xml, "  <skippedlines>" + escape(`self.logdata.skippedLines`) + "</skippedlines>"
        print >>xml, "</header>"

        # output parameters
        print >>xml, "<params>"
        for param, value in self.logdata.parameters.items():
            print >>xml, "  <param name=\"%s\" value=\"%s\" />" % (param,escape(`value`))
        print >>xml, "</params>"

        # output test results
        print >>xml, "<results>"
        for test in self.tests:
            if not test.enable:
                continue
            print >>xml, "  <result>"
            if test.result.status == TestResult.StatusType.GOOD:
                print >>xml, "    <name>" + escape(test.name) + "</name>"
                print >>xml, "    <status>GOOD</status>"
                print >>xml, "    <message>" + escape(test.result.statusMessage) + "</message>"
            elif test.result.status == TestResult.StatusType.FAIL:
                print >>xml, "    <name>" + escape(test.name) + "</name>"
                print >>xml, "    <status>FAIL</status>"
                print >>xml, "    <message>" + escape(test.result.statusMessage) + "</message>"
                print >>xml, "    <data>(test data will be embeded here at some point)</data>"
            elif test.result.status == TestResult.StatusType.WARN:
                print >>xml, "    <name>" + escape(test.name) + "</name>"
                print >>xml, "    <status>WARN</status>"
                print >>xml, "    <message>" + escape(test.result.statusMessage) + "</message>"
                print >>xml, "    <data>(test data will be embeded here at some point)</data>"
            elif test.result.status == TestResult.StatusType.NA:
                print >>xml, "    <name>" + escape(test.name) + "</name>"
                print >>xml, "    <status>NA</status>"
            else:
                print >>xml, "    <name>" + escape(test.name) + "</name>"
                print >>xml, "    <status>UNKNOWN</status>"
                print >>xml, "    <message>" + escape(test.result.statusMessage) + "</message>"
            print >>xml, "  </result>"
        print >>xml, "</results>"

        print >>xml, "</loganalysis>"

        xml.close()


def main():
    dirName = os.path.dirname(os.path.abspath(__file__))

    # deal with command line arguments
    parser = argparse.ArgumentParser(description='Analyze an APM Dataflash log for known issues')
    parser.add_argument('logfile', type=argparse.FileType('r'), help='path to Dataflash log file (or - for stdin)')
    parser.add_argument('-f', '--format',  metavar='', type=str, action='store', choices=['bin','log','auto'], default='auto', help='log file format: \'bin\',\'log\' or \'auto\'')
    parser.add_argument('-q', '--quiet',  metavar='', action='store_const', const=True, help='quiet mode, do not print results')
    parser.add_argument('-p', '--profile', metavar='', action='store_const', const=True, help='output performance profiling data')
    parser.add_argument('-s', '--skip_bad', metavar='', action='store_const', const=True, help='skip over corrupt dataflash lines')
    parser.add_argument('-e', '--empty',  metavar='', action='store_const', const=True, help='run an initial check for an empty log')
    parser.add_argument('-x', '--xml', type=str, metavar='XML file', nargs='?', const='', default='', help='write output to specified XML file (or - for stdout)')
    parser.add_argument('-v', '--verbose', metavar='', action='store_const', const=True, help='verbose output')
    args = parser.parse_args()

    # load the log
    startTime = time.time()
    logdata = DataflashLog.DataflashLog(args.logfile.name, format=args.format, ignoreBadlines=args.skip_bad) # read log
    endTime = time.time()
    if args.profile:
        print "Log file read time: %.2f seconds" % (endTime-startTime)

    # check for empty log if requested
    if args.empty:
        emptyErr = DataflashLog.DataflashLogHelper.isLogEmpty(logdata)
        if emptyErr:
            sys.stderr.write("Empty log file: %s, %s" % (logdata.filename, emptyErr))
            sys.exit(1)

    #run the tests, and gather timings
    testSuite = TestSuite()
    startTime = time.time()
    testSuite.run(logdata, args.verbose)  # run tests
    endTime = time.time()
    if args.profile:
        print "Test suite run time: %.2f seconds" % (endTime-startTime)

    # deal with output
    if not args.quiet:
        testSuite.outputPlainText(args.profile)
    if args.xml:
        testSuite.outputXML(args.xml)
        if not args.quiet:
            print "XML output written to file: %s\n" % args.xml


if __name__ == "__main__":
    main()

