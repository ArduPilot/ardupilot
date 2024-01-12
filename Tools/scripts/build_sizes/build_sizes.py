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

build_dirs = ['latest', 'beta', 'beta-4.3', 'stable']
builds = ['Plane', 'Copter', 'Rover', 'Sub', 'Blimp', 'AntennaTracker', 'AP_Periph']

args = parser.parse_args()

warning_flash_free = 5000
warning_build_days = 3


class APJInfo:
    def __init__(self, vehicle, board, githash, mtime, flash_free):
        self.vehicle = vehicle
        self.board = board
        self.githash = githash
        self.mtime = mtime
        self.flash_free = flash_free
        self.warning = 0


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
<script src="filtertable.js"></script>
<link href="../../css/main.css" rel="stylesheet" type="text/css" />
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
a {
 color: inherit;
}
</style>
<title>Build List</title>
</head>
<body>
<div id="main">
<a href="https://firmware.ardupilot.org/">
<div id="logo" style="text-align:center;">
</div>
</a>
<h2 id='top'>Build List</h2>
<p>This is an auto-generated list of current builds
    allowing us to quickly see how close we are to running out of flash space.
<br>This page was most recently regenerated on %s UTC.</p>
<p>Builds that are coloured red haven't built recently. Builds that are coloured yellow have low flash space remaining.</p>
<p>Click on any column header to sort by that column, or filter by entering a search term in the box above each table.</p>
<ul>
<li>Jump to <a href='#latest'>latest</a></li>
<li>Jump to <a href='#beta'>beta</a></li>
<li>Jump to <a href='#beta-4.3'>beta-4.3</a></li>
<li>Jump to <a href='#stable'>stable</a></li>
</ul>
''' % datetime.now().strftime("%F %k:%M"))


def write_footer(h):
    '''write html footer'''
    h.write('''
</div>
</body>
</html>
''')


def write_table(h, build_type):
    '''write table for one build type'''
    boards = []

    for build in builds:
        boards.extend(apj_list(os.path.join(args.basedir, build, build_type)))

    max_mtime = 0
    for apjinfo in boards:
        if apjinfo.mtime > max_mtime:
            max_mtime = apjinfo.mtime

    for apjinfo in boards:
        if apjinfo.flash_free < warning_flash_free and not apjinfo.flash_free == -1:
            apjinfo.warning = 1
        if int(apjinfo.mtime) < int(max_mtime)-warning_build_days*86400 and build_type == "latest":
            apjinfo.warning = 2

    boards.sort(key=lambda board: board.warning, reverse=True)

    try:
        idxs1 = [i for (i, e) in enumerate(boards) if e.warning == 1]
        boards[min(idxs1):max(idxs1)] = sorted(boards[min(idxs1):max(idxs1)], key=lambda board: board.flash_free)
    except ValueError:
        pass

    try:
        idxs2 = [i for (i, e) in enumerate(boards) if e.warning == 2]
        boards[min(idxs2):max(idxs2)] = sorted(boards[min(idxs2):max(idxs2)], key=lambda board: board.mtime)
    except ValueError:
        pass

    h.write('''
<h3 id='%s'>%s builds</h3>
<p><input type="text" id="search_%s" onkeyup="searchFunc('%s')" placeholder="Filter..."></p>
<table class="sortable" id="table_%s">
<tr><th style="width:90px">Vehicle</th><th style="width:250px">Board</th><th style="width:140px">Build Date</th>
    <th style="width:100px">git hash</th><th style="width:100px">Flash Free</th></tr>
''' % (build_type, build_type.capitalize(), build_type, build_type, build_type))

    for apjinfo in boards:
        if apjinfo.warning == 1:
            h.write('<tr style="color:#E6B800;">')
        elif apjinfo.warning == 2:
            h.write('<tr style="color:#FF0000;">')
        else:
            h.write('<tr>')
        h.write('''<td>%s</td>
    <td><a href="https://firmware.ardupilot.org/%s/%s/%s">%s</a></td>
    <td>%s</td>
    <td><a href="https://github.com/ArduPilot/ardupilot/commit/%s">%s</a></td>
    <td>%u</td></tr>\n''' % (
            apjinfo.vehicle, apjinfo.vehicle, build_type, apjinfo.board, apjinfo.board,
            datetime.fromtimestamp(apjinfo.mtime).strftime("%F %k:%M"),
            apjinfo.githash, apjinfo.githash, apjinfo.flash_free))

    h.write('''
</table>
<p>Return to <a href='#top'>top</a></p>
''')


h = open(args.outfile, "w")
write_headers(h)
for t in build_dirs:
    write_table(h, t)
write_footer(h)
