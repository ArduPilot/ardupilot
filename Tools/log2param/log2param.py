#!/usr/bin/env python3

"""
log2param.py by André Kjellstrup

On execution, it will parse specified file, recreate .param file, and save it.
"""

import subprocess
import os.path
import sys
import argparse

parser = argparse.ArgumentParser(description="This program will extract parameters from dataflashlog (.BIN) or telemetry log (.tlog) and save it in APMPlanner2 format (.param). You will also, see parameter changes and updates made during the logged events. Usage example: log2param.py 0000231.BIN" )
parser.add_argument("filename")
parser.add_argument("-v", "--verbose", action="store_true",
                    help="increase output verbosity")
args = parser.parse_args()

params=[]  #0 = list of lists, 0=names 1=values  like: [["A",5],["B",6]]
paramname=""
paramvalue=""

##################### FUNCTIONS #######################

def find_between( s, first, last ):
    try:
        start = s.index( first ) + len( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""

def find_between_r( s, first, last ):
    try:
        start = s.rindex( first ) + len( first )
        end = s.rindex( last, start )
        return s[start:end]
    except ValueError:
        return ""

def process_params():
    global params
    global paramname
    global paramvalue
    if paramvalue.endswith(".0"):  #remove trailing ".0" from integers
        paramvalue=paramvalue[:-2]
    i=0
    while i < len(params):  #loop pointer thru known params
        if paramname == params[i][0]:  #if already exists
            if params[i][1] != paramvalue:  #update only if changed, no action if unchanged.
                print (paramname+','+paramvalue," #updated from", params[i][1])
                params[i][1] = paramvalue #update instead
            paramvalue = "-USED-"
        i+=1
    if paramvalue != "-USED-":  #new paramter, were not found in table
        params.append([paramname, paramvalue])
        print (paramname+','+paramvalue)



##################### MAIN #####################

if os.path.isfile(args.filename):

    print ("\n\n======================= Parsing file: ", args.filename, " ======================")
    print ("Listing parameters in the found order:")
    todo = 'mavlogdump.py' + ' "' + args.filename + '"'
    cmd = subprocess.Popen(todo, shell=True, stdout=subprocess.PIPE, universal_newlines=True)

    for line in cmd.stdout:

        if args.filename.lower().endswith(".tlog"):
            if "{param_id : " in line:
                paramname= find_between( line, "{param_id : ",", param_value" )
                paramvalue= find_between( line, ", param_value : ",", param_type" )
                paramvalue=str(round(float(paramvalue),8)) #reduce floats decimals
                paramtot= find_between( line, " param_count : ",", param_index" )  #if we got this many, we got all
                process_params()

        if args.filename.lower().endswith(".bin"):
            #print (line)
            #processing every line
            if "PARM {" in line:
                paramname= find_between( line, "Name : ",", Value" )
                paramvalue= find_between( line, ", Value : ","}" )
                paramvalue=str(round(float(paramvalue),8)) #reduce floats decimals
                process_params()

else:
    print ("Error : file not found")

outfile = open(args.filename+".param", 'w')

if len(params) < 1:
    print ("Failed to find any paramteres in",args.filename)
    exit()

print ("\n\n\n\n\n\n#NOTE: APMPlanner2 style parameter recovery from",args.filename)
outfile.write("#NOTE: APMPlanner2 style parameter recovery from "+args.filename+"\n")
params.sort()
i=0
while i < len(params):
    print (params[i][0]+","+str(params[i][1]))
    outfile.write(params[i][0]+","+str(params[i][1])+"\n")
    i+=1

print("#NOTE: Exported",i,"parameters to file", args.filename+".param")
outfile.close()
