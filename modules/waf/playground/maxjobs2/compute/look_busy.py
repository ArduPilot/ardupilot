#! /usr/bin/env python

import sys, time
loops = int(sys.argv[1])

if not loops:
	time.sleep(1)
	pass
else:
	for i in range(loops):
		time.sleep(1)

