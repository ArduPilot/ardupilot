#!/usr/bin/env python
#
# A module to analyze and identify any common problems which can be determined from log files
#
# Initial code by Andrew Chapman (chapman@skymount.com), 16th Jan 2014
#


# some logging oddities noticed while doing this, to be followed up on:
#   - tradheli MOT labels Mot1,Mot2,Mot3,Mot4,GGain
#   - Pixhawk doesn't output one of the FMT labels... forget which one
#   - MAG offsets seem to be constant (only seen data on Pixhawk)
#   - MAG offsets seem to be cast to int before being output? (param is -84.67, logged as -84)

# TODO - unify result statusMessage and extraOutput. simple tests set statusMessage, complex ones append to it with newlines


import DataflashLog

import pprint  # temp
import imp
import glob
import inspect
import os, sys
import argparse
import datetime
import time


class TestResult:
	'''all tests pass back a standardized result'''
	class StatusType:
		# NA means not applicable for this log (e.g. copter tests against a plane log), UNKNOWN means it is missing data required for the test
		PASS, FAIL, WARN, UNKNOWN, NA = range(5)
	status = None
	statusMessage = ""
	extraFeedback = ""


class Test:
	'''base class to be inherited by each specific log test. Each test should be quite granular so we have lots of small tests with clear results'''
	name = ""
	result = None   # will be an instance of TestResult after being run
	execTime = None
	enable = True
	def run(self, logdata):
		pass


class TestSuite:
	'''registers test classes'''
	tests   = []
	logfile = None
	logdata = None

	def __init__(self):
		# dynamically load in Test subclasses from the 'tests' folder
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


	def run(self, logdata):
		'''run all registered tests in a single call'''
		self.logdata = logdata
		self.logfile = logdata.filename
		for test in self.tests:
			# run each test in turn, gathering timing info
			if test.enable:
				startTime = time.time()
				test.run(self.logdata)  # RUN THE TEST
				endTime = time.time()
				test.execTime = endTime-startTime

	def outputPlainText(self, outputStats):
		print 'Dataflash log analysis report for file: ' + self.logfile
		print 'Log size: %.2fmb (%d lines)' % (self.logdata.filesizeKB / 1024.0, self.logdata.lineCount)
		print 'Log duration: %s' % str(datetime.timedelta(seconds=self.logdata.durationSecs)) + '\n'

		if self.logdata.vehicleType == "ArduCopter" and self.logdata.getCopterType():
			print 'Vehicle Type: %s (%s)' % (self.logdata.vehicleType, self.logdata.getCopterType())
		else:
			print 'Vehicle Type: %s' % self.logdata.vehicleType
		print 'Firmware Version: %s (%s)' % (self.logdata.firmwareVersion, self.logdata.firmwareHash)
		print 'Hardware: %s' % self.logdata.hardwareType
		print 'Free RAM: %s' % self.logdata.freeRAM
		print '\n'
		
		print "Test Results:"
		for test in self.tests:
			if not test.enable:
				continue
			execTime = ""
			if outputStats:
				execTime = "  (%.2fms)" % (test.execTime)
			if test.result.status == TestResult.StatusType.PASS:
				print "  %20s:  PASS       %-50s%s" % (test.name, test.result.statusMessage,execTime)
			elif test.result.status == TestResult.StatusType.FAIL:
				print "  %20s:  FAIL       %-50s%s    [GRAPH]" % (test.name, test.result.statusMessage,execTime)
			elif test.result.status == TestResult.StatusType.WARN:
				print "  %20s:  WARN       %-50s%s    [GRAPH]" % (test.name, test.result.statusMessage,execTime)
			elif test.result.status == TestResult.StatusType.NA:
				# skip any that aren't relevant for this vehicle/hardware/etc
				continue
			else:
				print "  %20s:  UNKNOWN    %-50s%s" % (test.name, test.result.statusMessage,execTime)
			if test.result.extraFeedback:
				for line in test.result.extraFeedback.strip().split('\n'):
					print "  %20s     %s" % ("",line)
		print '\n'


		# temp - test some spot values
		#print "GPS abs alt on line 24126 is " + `self.logdata.channels["GPS"]["Alt"].dictData[24126]`   # 52.03
		#print "ATT pitch on line 22153 is " + `self.logdata.channels["ATT"]["Pitch"].dictData[22153]`   # -7.03
		#gpsAlt = self.logdata.channels["GPS"]["Alt"]
		#print "All GPS Alt data: %s\n\n" % gpsAlt.dictData
		#gpsAltSeg = gpsAlt.getSegment(426,711)
		#print "Segment of GPS Alt data from %d to %d: %s\n\n" % (426,711,gpsAltSeg.dictData)

	def outputXML(self, xmlFile):
		# TODO: implement XML output
		# ...
		raise Exception("outputXML() not implemented yet")


def main():
	dirName = os.path.dirname(os.path.abspath(__file__))

	# deal with command line arguments
	parser = argparse.ArgumentParser(description='Analyze an APM Dataflash log for known issues')
	parser.add_argument('logfile', type=argparse.FileType('r'), help='path to Dataflash log file')
	parser.add_argument('-q', '--quiet',  metavar='', action='store_const', const=True, help='quiet mode, do not print results')
	parser.add_argument('-s', '--stats',  metavar='', action='store_const', const=True, help='output performance stats')
	parser.add_argument('-i', '--ignore', metavar='', action='store_const', const=True, help='ignore bad data lines')
	parser.add_argument('-e', '--empty',  metavar='', action='store_const', const=True, help='run an initial check for an empty log')
	parser.add_argument('-x', '--xml', type=str, metavar='XML file', nargs='?', const='', default='', help='write output to specified XML file')
	args = parser.parse_args()

	# load the log
	startTime = time.time()	
	logdata = DataflashLog.DataflashLog(args.logfile.name, ignoreBadlines=args.ignore)  # read log
	endTime = time.time()
	if args.stats:
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
	testSuite.run(logdata)  # run tests
	endTime = time.time()
	if args.stats:
		print "Test suite run time: %.2f seconds" % (endTime-startTime)

	# deal with output
	if not args.quiet:
		testSuite.outputPlainText(args.stats)
	if args.xml:
		testSuite.outputXML(args.xml)
		if not args.quiet:
			print "XML output written to file: %s\n" % args.xml


if __name__ == "__main__":
	main()

