#!/usr/bin/python 
# Author: Lenna X. Peterson (github.com/lennax)
# based on bash script by James Goppert (github.com/jgoppert)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# USAGE:                                                                      #
# $ ./autobuild.py [1-9]                                                      #
#   Then follow menu                                                          #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# TODO: Error handling: 
## check cmake success, etc.
## catch CMake Warning:
  #Manually-specified variables were not used by the project:
		  #BUILD_TYPE
#     (missing gprof flags)

import sys # for sys.argv[] and sys.platform
import os # for chdir()
import subprocess # for check_call()
import shutil # for rmtree()
from optparse import OptionParser # for parsing options
try: 
	from get_build_path import get_build_path
except ImportError: 
	print "Could not find 'get_build_path.py' "
	print "in '%s'" % os.path.dirname(os.path.abspath(__file__))
	print "This module is required."
	raise SystemExit

## Move to directory containing CMakeLists.txt and src/
build_path = get_build_path()
if build_path:
	os.chdir(build_path)
else: 
	print "The script was unable to find a build directory."
	raise SystemExit

makeargs = "-j8"
cmakecall = ["cmake", ".."]
build_dir = "build"

## Parse command line options
## TODO: add makeargs/cmakeargs etc.
## (with 'action="append"' to append arg(s) to list)
usage = "usage: %prog [options] [1-9]"
parser = OptionParser(usage=usage)
parser.set_defaults(verbose=False, makeargs="-j8")
parser.add_option("-v", "--verbose",
				action="store_true", dest="verbose",
				help="Verbose mode")
parser.add_option("-c", "--cmake",
				action="store", dest="cm",
				help="Specify one or more arguments for CMake")
#parser.add_option("--makeargs", 
#                  action="store", type="string", dest="makeargs",
#                  help="Argument to `make` [default '-j8']")
(options, args) = parser.parse_args()
if options.verbose:
    os.environ['VERBOSE'] = "1"
### Split cmakeargs, reassemble, and insert into cmake call
if options.cm:
	cm_raw = options.cm
	cm_list = cm_raw.split("-D")
	if cm_list[0] == "":
		cm_list.pop(0)
	for cm in cm_list:
		cm = cm.lstrip()
		cmakearg = "-D" + cm
		cmakecall.insert(1, cmakearg)
#if options.makeargs:
#    makeargs = options.makeargs


def install_build(cmakecall, exitVal=True):
    if not os.path.isdir(build_dir): 
        os.mkdir(build_dir)
    os.chdir(build_dir)
    subprocess.check_call(cmakecall)
    subprocess.check_call(["make", makeargs])
    if exitVal == True:
        raise SystemExit
	
def dev_build():
	cmakecall.insert(1, "-DDEV_MODE::bool=TRUE")
	install_build(cmakecall)

def grab_deps():
	if 'linux' in sys.platform:
		try: 
			subprocess.check_call('sudo apt-get install cmake', shell=True)
		except: 
			print "Error installing dependencies: ", sys.exc_info()[0]
			print "apt-get is available on Debian and Ubuntu" 
			raise SystemExit
	elif 'darwin' in sys.platform:
		try: 
			subprocess.check_call('sudo port install cmake', shell=True)
		except: 
			print "Error installing dependencies: ", sys.exc_info()[0]
			print "Please install Macports (http://www.macports.org)"
			raise SystemExit
	else: 
		print "Platform not recognized (did not match linux or darwin)"
		print "Script doesn't download dependencies for this platform"
        raise SystemExit

def package_source():
    install_build(cmakecall, exitVal=False)
    subprocess.check_call(["make", "package_source"])
    raise SystemExit

def package():
    install_build(cmakecall, exitVal=False)
    subprocess.check_call(["make", "package"])
    raise SystemExit

def remake():
    if not os.path.isdir(build_dir): 
        print "Directory '%s' does not exist" % build_dir
        print "You must make before you can remake."
        return 1
    os.chdir(build_dir)
    subprocess.check_call(["make", makeargs])
    raise SystemExit

def clean():
	if 'posix' in os.name: 
		print "Cleaning '%s' with rm -rf" % build_dir
		subprocess.check_call(["rm", "-rf", build_dir])
	else: 
		print "Cleaning '%s' with shutil.rmtree()" % build_dir
		print "(may be very slow)"
		shutil.rmtree(build_dir, ignore_errors=True)
	print "Build cleaned"

# requires PROFILE definition in CMakeLists.txt:
# set(CMAKE_BUILD_TYPE PROFILE)
# set(CMAKE_CXX_FLAGS_PROFILE "-g -pg")
# set(CMAKE_C_FLAGS_PROFILE "-g -pg")
def profile():
	cmakecall.insert(1, "-DDEV_MODE::bool=TRUE")
	cmakecall.insert(2, "-DBUILD_TYPE=PROFILE")
	install_build(cmakecall)
	
def menu():
	print "1. developer build: used for development."
	print "2. install build: used for building before final installation to the system."
	print "3. grab dependencies: installs all the required packages for debian based systems (ubuntu maverick/ debian squeeze,lenny) or darwin with macports."
	print "4. package source: creates a source package for distribution."
	print "5. package: creates binary packages for distribution."
	print "6. remake: calls make again after project has been configured as install or in source build."
	print "7. clean: removes the build directory."
	print "8. profile: compiles for gprof."
	print "9. end."
	opt = raw_input("Please choose an option: ")
	return opt

try: 
    loop_num = 0
    # continues until a function raises system exit or ^C
    while (1): 	
        if len(args) == 1 and loop_num == 0:
            opt = args[0]
            loop_num += 1
        else:
            opt = menu()

        try:
            opt = int(opt)
        except ValueError:
            pass
            
        if   opt == 1:
            print "You chose developer build"
            dev_build()
        elif opt == 2:
            print "You chose install build"
            install_build(cmakecall)
        elif opt == 3: 
            print "You chose to install dependencies"
            grab_deps()
        elif opt == 4:
            print "You chose to package the source"
            package_source()
        elif opt == 5:
            print "You chose to package the binary"
            package()
        elif opt == 6:
            print "You chose to re-call make on the previously configured build"
            remake()
        elif opt == 7:
            print "You chose to clean the build"
            clean()
        elif opt == 8:
            # requires definition in CMakeLists.txt (see def above)
            print "You chose to compile for gprof"
            profile()
        elif opt == 9:
            raise SystemExit
        else:
            print "Invalid option. Please try again: " 
		
except KeyboardInterrupt: 
	print "\n" 
