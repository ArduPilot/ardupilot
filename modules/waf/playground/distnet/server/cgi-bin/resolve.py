#! /usr/bin/env python

import os, sys
import cgi, cgitb
cgitb.enable()

PKGDIR = os.environ.get('PKGDIR', os.path.abspath('../packages'))
if not 'DISTNETCACHE' in os.environ:
	os.environ['DISTNETCACHE'] = PKGDIR

d = os.path.dirname
base = d(d(d(d(d(os.path.abspath(__file__))))))
sys.path.append(base)

from waflib.extras import distnet

form = cgi.FieldStorage()

text = form.getvalue('text')
distnet.packages.debug = 0
distnet.packages.constraints = distnet.packages.local_resolve(text)
results = distnet.packages.get_results()

print("Content-Type: text/plain")
print( "")
print("")
print(results)

