#!/usr/bin/env python
"""
    This script aims to parser autotest output file and provide a html.

    It is a brief MVP in order to test the concept.

    TO-DO:
    - Automate the download of the last file of https://autotest.ardupilot.org/history/2019-07-01-02:07/autotest-output.txt
    - Create a link to history of generated files
    - Work in input and output redirects for files
    - Insert other simple tests
    - Inset other autotest results as presented in https://autotest.ardupilot.org?

    Build notes: 
    - As the autotest output file is a not small text file, we read it at once and store in memory for all steps.

"""

import sys
import shutil
import argparse
import re
import io
import datetime
from pysim import util
import os


#from autotest import write_webresults
#from autotest import TestResults
#import glob
#from pysim import util
#from pymavlink.generator import mavtemplate

parser = argparse.ArgumentParser(description="python3 generate_.py [options]")
parser.add_argument("--verbose", dest='verbose', action='store_false', default=True, help="show debugging output")
parser.add_argument("--filein", dest='file_to_parse', default="autotest-output-sample.txt", help="autotest-output.txt file")
parser.add_argument("--fileout", dest='file_out', default="index_errors.html", help="HTML output file.")
args = parser.parse_args()

error_count = 0


def debug(str_to_print):
    """ Debug output if verbose is set. """
    if args.verbose:
        print(str_to_print)

def error(str_to_print):
    """ Show and count the errors. """
    global error_count
    error_count += 1
    print(str_to_print)

def all_vehicles():
    """ Original method from autotest.py """
    return ('ArduPlane',
            'ArduCopter',
            'APMrover2',
            'AntennaTracker',
            'ArduSub')

def get_file_to_parse():
    """ Return the log file as an array of strings """

    autotest_log = "autotest-output-sample.txt"
    try:
        return open(autotest_log, 'r').readlines()
    except Exception as e:
        error("Error on open the log file: %s)" % e)
        sys.exit(1)


def find_board_build_errors(autotest_log):
    """ Return a list of board build erros """
    regex = re.compile("^BB..Failed.build.of")
    matches = []
    for line in autotest_log:
        if regex.search(line):
            matches.append(str(line[4:-1]).replace("Failed", "FAILED")) # Removes 'BB..', change Failed to css style and remove the last '\n'
    return matches


def find_autotest_steps_errors(autotest_log):
    """ Return a list of failed errors """
    regex = re.compile("^>>>>.FAILED.STEP:")
    matches = []
    for line in autotest_log:
        if regex.search(line):
            matches.append("autotest.py FAILED on step "  +  line[17:-1 ]) # Replaces the beginning the removes last \n
    return matches

def write_results(results):

    # HTML header
    shutil.copy(util.reltopdir('Tools/autotest/web/index-new-header.html'),  util.reltopdir('Tools/autotest/web/index-new-tmp.html') )

    with open(util.reltopdir('Tools/autotest/web/index-new-tmp.html'), "a") as generate_file:
        now = datetime.datetime.now()
        generate_file.write("<h2>Automatic autotest results run at " + now.strftime("%Y-%m-%d %H:%M:%S") +  "</h2>\n\n") 
        global githash
        generate_file.write("<div id=\"git\"> Test run on git commit  <a href=\"https://github.com/ArduPilot/ardupilot/commit/%s\">%s</a> </div>\n\n" % (githash,githash))

        for chapter in results:
            
            generate_file.write("<h2> %s </h2>\n\n" % chapter)
        
            #generate_file.write("<ul id=\"%s\">\n" % str(chapter).lower().replace(" ",""))
            generate_file.write("<ul id=\"testresults\">\n") # TO-DO create a proper css tag.

            for item in results[chapter]:
                generate_file.write("<li>" + str(item).replace("FAILED" , "<span class=\"failed-text\">FAILED</span>") + "</li>\n")

            generate_file.write("</ul>\n\n")

        with open(util.reltopdir('Tools/autotest/web/index-new-footer.html'), "r") as footer:
            for line in footer:
                generate_file.write(line)


def get_githash(autotest_log):
    """ Return the first occurency of a declared git hash"""
    regex = re.compile("^BB:.git.hash:.")
    for line in autotest_log:
        if regex.search(line):
            return line[14:-1] # Clen the begining and the last \n


def run_step(step):
    """ Run a step method """

    global results
    autotest_logfile = get_file_to_parse()
    global githash
    githash = get_githash(autotest_logfile)

    if step == 'Board build errors':      
        results['Board build errors'] = find_board_build_errors(autotest_logfile) 

    if step == 'Failed autotest steps':
        results['Failed autotest steps'] = find_autotest_steps_errors(autotest_logfile)

    if step == 'Template for something else':
        results['Template for something else'] = ["Template",  "for",  "something", "else"]

    try:
        autotest_logfile.close()
    except:
        pass

            
if __name__ == "__main__":
    """ main program """

    steps = [
            'Board build errors',
            'Failed autotest steps',
            'Template for something else'
    ]

    results = {} # Dict os list. The key is presented as a chapter and each item as a line ainda fancy table.


    for step in steps:
        print(step)
        run_step(step)
    
    try:
        if os.path.isfile(util.reltopdir("Tools/autotest/web/index-new-tmp.html")):
            os.remove(util.reltopdir("Tools/autotest/web/index-new-tmp.html"))
    except:
        pass

    write_results(results)

    try: 
        if os.path.isfile(util.reltopdir("Tools/autotest/web/index-new.html")):
            os.remove(util.reltopdir("Tools/autotest/web/index-new.html"))
        shutil.move(util.reltopdir("Tools/autotest/web/index-new-tmp.html") , util.reltopdir("Tools/autotest/web/index-new.html"))
    except Exception as e:
        error("Erro while moving results: %s" % e)
