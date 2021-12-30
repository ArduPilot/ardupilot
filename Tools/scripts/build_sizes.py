#!/usr/bin/env python3
'''
script to build a html file showing flash free for current builds

AP_FLAKE8_CLEAN
'''

import os
import argparse
import fnmatch
import json
from datetime import datetime

parser = argparse.ArgumentParser(description='create builds.html for list of builds')
parser.add_argument('basedir', default=None, help='base directory (binaries directory)')
parser.add_argument('--outfile', default="builds.html", help='output file')

build_dirs = ['latest', 'beta', 'stable']
builds = ['Plane', 'Copter', 'Rover', 'Sub', 'Blimp', 'AP_Periph']

args = parser.parse_args()


class APJInfo:
    def __init__(self, vehicle, board, githash, mtime, flash_free):
        self.vehicle = vehicle
        self.board = board
        self.githash = githash
        self.mtime = mtime
        self.flash_free = flash_free


def apj_list(basedir):
    '''list of APJInfo for one directory'''
    boards = []
    for root, subdirs, files in os.walk(basedir):
        for f in files:
            if not fnmatch.fnmatch(f, "*.apj"):
                continue
            fname = os.path.join(root, f)
            board = os.path.basename(root)
            vehicle = fname.split('/')[-4]
            fw_json = json.load(open(fname, "r"))
            githash = fw_json['git_identity']
            flash_free = fw_json.get('flash_free', -1)
            mtime = os.stat(fname).st_mtime
            apjinfo = APJInfo(vehicle, board, githash, mtime, flash_free)
            boards.append(apjinfo)
    return boards


def write_headers(h):
    '''write html headers'''
    h.write('''
<!DOCTYPE html>
<html>
<head>
<script src="sorttable.js"></script>
<style>
td {
 border-left:1px solid black;
 border-top:1px solid black;
}
th {
 text-align: left;
 border-left:1px solid black;
 border-top:1px solid black;
}
table {
 border-right:1px solid black;
 border-bottom:1px solid black;
}
</style>
<title>Build List</title>
</head>
<body>
<h1>Build list</h1>
This is an auto-generated list of current builds allowing us to quickly see how close we are to running out of flash space
<ul>
<li>Jump to <a href='#latest'>latest</a></li>
<li>Jump to <a href='#beta'>beta</a></li>
<li>Jump to <a href='#stable'>stable</a></li>
</ul>
''')


def write_footer(h):
    '''write html footer'''
    h.write('''
</body>
</html>
''')


def write_table(h, build_type):
    '''write table for one build type'''
    boards = []

    for build in builds:
        boards.extend(apj_list(os.path.join(args.basedir, build, build_type)))

    h.write('''
<h1 id='%s'>%s builds</h1>
<table class="sortable">
<tr><th>Vehicle</th><th>Board</th><th>Build Date</th><th>git hash</th><th>Flash Free</th></tr>
''' % (build_type, build_type))

    for apjinfo in boards:
        h.write('''<tr><td>%s</td><td>%s</td><td>%s</td>
   <td><a href="https://github.com/ArduPilot/ardupilot/commit/%s">%s</a></td><td>%u</td></tr>\n''' % (
            apjinfo.vehicle, apjinfo.board, datetime.fromtimestamp(apjinfo.mtime).strftime("%F %k:%M"),
            apjinfo.githash, apjinfo.githash, apjinfo.flash_free))

    h.write('''
</table>
''')


h = open(args.outfile, "w")
write_headers(h)
for t in build_dirs:
    write_table(h, t)
write_footer(h)
