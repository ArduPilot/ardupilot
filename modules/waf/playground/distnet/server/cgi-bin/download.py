#! /usr/bin/env python

import os, sys
import cgi, cgitb
cgitb.enable()

PKGDIR = os.environ.get('PKGDIR', os.path.abspath('../packages'))

form = cgi.FieldStorage()
def getvalue(x):
	v = form.getvalue(x)
	if not v:
		print("Status: 413\ncontent-type: text/plain\n\nmissing %s\n" % x)
	return v

pkgname = getvalue('pkgname')
pkgver = getvalue('pkgver')
pkgfile = getvalue('pkgfile')

filename = os.path.join(PKGDIR, pkgname, pkgver, pkgfile)
if not os.path.exists(filename):
	filename = filename + '.tarfile'

if not os.path.exists(filename):
	print("Status: 404\ncontent-type: text/plain\n\nInvalid package %r\n" % filename)

length = os.stat(filename).st_size

print("Content-Type: application/octet-stream")
print("Content-Disposition: attachment; filename=f.bin")
print("Content-length: %s" % length)
print("")

with open(filename, 'rb') as f:
	while True:
		buf = f.read(8192)
		if buf:
			sys.stdout.write(buf)
		else:
			break

