#!/usr/bin/env python

import re, sys, operator, os

code_line = re.compile("^\s*\d+:/")
frame_line = re.compile("^\s*\d+\s+/\* frame size = (\d+) \*/")

class frame(object):
    def __init__(self, code, frame_size):
        self.code = code
        self.frame_size = int(frame_size)

frames = []

def process_lst(filename):
    '''process one lst file'''
    last_code = ''
    h = open(filename, mode='r')
    for line in h:
        if code_line.match(line):
            last_code = line.strip()
        elif frame_line.match(line):
            frames.append(frame(last_code, frame_line.match(line).group(1)))
    h.close()

if len(sys.argv) > 1:
    dname = sys.argv[1]
else:
    dname = '.'

for root, dirs, files in os.walk(dname):
    for f in files:
        if f.endswith(".lst"):
            process_lst(os.path.join(root, f))

sorted_frames =  sorted(frames,
                        key=operator.attrgetter('frame_size'),
                        reverse=True)

print("FrameSize   Code")
for frame in sorted_frames:
    if frame.frame_size > 0:
        print("%9u   %s" % (frame.frame_size, frame.code))
    
