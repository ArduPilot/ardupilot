#!/usr/bin/python
# Author: Lenna X. Peterson (github.com/lennax)
# Based on bash script by James Goppert (github.com/jgoppert)
#
# script used to update cmake modules from git repo, can't make this
# a submodule otherwise it won't know how to interpret the CMakeLists.txt
# # # # # # subprocess# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import os # for os.path
import subprocess # for check_call()

clone_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(clone_path)
os.chdir(clone_path)
subprocess.check_call(["git", "clone", "git://github.com/arktools/arkcmake.git","arkcmake_tmp"])
subprocess.check_call(["rm", "-rf", "arkcmake_tmp/.git"])
if os.path.isdir("arkcmake"):
	subprocess.check_call(["rm", "-rf", "arkcmake"])
subprocess.check_call(["mv", "arkcmake_tmp", "arkcmake"])
