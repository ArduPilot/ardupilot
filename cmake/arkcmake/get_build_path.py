#!/usr/bin/python
# Author: Lenna X. Peterson (github.com/lennax)
# Determines appropriate path for CMake
#  Looks for "CMakeLists.txt" and "src/"
#   ( BUILDFILE and SRCDIR in find_build_dir() )
#  Searches the following paths:
#   1 Path of call
#   2 Path where script is located
#   3 Path above 2 (parent directory)
#   4 Path above 3 (grandparent directory)

import os # for getcwd(), os.path

def get_build_path():

	build_dir=""

	## Initialize search paths
	call_dir = os.getcwd()
	script_dir = os.path.dirname(os.path.abspath(__file__))
	script_mom = os.path.abspath(script_dir + os.sep + os.pardir)
	script_grandma = os.path.abspath(script_mom + os.sep + os.pardir)
	if script_mom == call_dir:
		script_mom = ""
	if script_grandma == call_dir:
		script_grandma = ""

	## Define function to search for required components for build
	def find_build_dir(search_dir):
		BUILDFILE = "CMakeLists.txt"
		#SRCDIR = "src"
		os.chdir(search_dir)
		if os.path.isfile(BUILDFILE):
			return search_dir
		return False

	## Class to emulate if temp = x 
	#      (checking equality of x while assigning it to temp)
	#      Borrowed from Alex Martelli
	class Holder(object):
		def set(self, value):
			self.value = value
			return value
		def get(self):
			return self.value

	temp = Holder()

	## Search paths for build components
	if temp.set(find_build_dir(call_dir)): 
		build_dir = temp.get()
	elif temp.set(find_build_dir(script_dir)): 
		build_dir = temp.get()
	elif script_mom and temp.set(find_build_dir(script_mom)): 
		build_dir = temp.get()
	elif script_grandma and temp.set(find_build_dir(script_grandma)):
		build_dir = temp.get()
	else: 
		return 0
	
	print "I go now. Good luck, everybody!"
	return build_dir
