#! /usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2014-2015

"""
A simple file for verifying signatures in signed waf files
This script is meant for Python >= 2.6 and the encoding is bytes - latin-1

Distributing detached signatures is boring
"""

import sys, os, re, subprocess

if __name__ == '__main__':
	try:
		infile = sys.argv[1]
	except IndexError:
		infile = 'waf'

	try:
		outfile1 = sys.argv[2]
	except IndexError:
		outfile1 = infile + '-sig'

	try:
		outfile2 = sys.argv[3]
	except IndexError:
		outfile2 = outfile1 + '.asc'

	f1 = open(outfile1, 'wb')
	f2 = open(outfile2, 'wb')
	f = open(infile, 'rb')
	try:
		txt = f.read()

		lastline = txt.decode('latin-1').splitlines()[-1] # just the last line
		if not lastline.startswith('#-----BEGIN PGP SIGNATURE-----'):
			print("ERROR: there is no signature to verify in %r :-/" % infile)
			sys.exit(1)

		sigtext = lastline.replace('\\n', '\n') # convert newlines
		sigtext = sigtext[1:] # omit the '# character'
		sigtext = sigtext.encode('latin-1') # python3

		f2.write(sigtext)
		f1.write(txt[:-len(lastline) - 1]) # one newline character was eaten from splitlines()
	finally:
		f.close()
		f1.close()
		f2.close()

	cmd = 'gpg --verify %s' % outfile2
	print("-> %r" % cmd)
	ret = subprocess.Popen(cmd, shell=True).wait()
	sys.exit(ret)

