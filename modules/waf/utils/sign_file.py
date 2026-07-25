#! /usr/bin/env python

"""
A script that creates signed Python files.
"""

from __future__ import print_function

import os
import argparse
import subprocess


def get_file_encoding(filename):
    """
    Get the file encoding for the file with the given filename
    """

    with open(filename, 'rb') as fp:
        # The encoding is usually specified on the second line
        txt = fp.read().splitlines()[1]
        txt = txt.decode('utf-8')
        if 'encoding' in txt:
            encoding = txt.split()[-1]
        else:
            encoding = 'utf-8' # default

    return str(encoding)


def sign_file_and_get_sig(filename, encoding):
    """
    Sign the file and get the signature
    """

    cmd = 'gpg -bass {}'.format(filename)
    ret = subprocess.Popen(cmd, shell=True).wait()
    print ('-> %r' % cmd)

    if ret:
        raise ValueError('Could not sign the file!')

    with open('{}.asc'.format(filename), 'rb') as fp:
        sig = fp.read()

    try:
        os.remove('{}.asc'.format(filename))
    except OSError:
        pass

    sig = sig.decode(encoding)
    sig = sig.replace('\r', '').replace('\n', '\\n')
    sig = sig.encode(encoding)

    return sig


def sign_original_file(filename, encoding):
    """
    Sign the original file
    """

    sig = sign_file_and_get_sig(filename, encoding)

    with open(filename, 'ab') as outfile:
        outfile.write('#'.encode(encoding))
        outfile.write(sig)
        outfile.write('\n'.encode(encoding))


def create_signed_file(filename, encoding):
    """
    Create a signed file
    """

    sig = sign_file_and_get_sig(filename, encoding)
    name, extension = os.path.splitext(filename)

    new_file_name = '{}_signed{}'.format(name, extension)

    with open(new_file_name, 'wb') as outfile, \
         open(filename, 'rb') as infile:
        txt = infile.read()
        outfile.write(txt)
        outfile.write('#'.encode(encoding))
        outfile.write(sig)
        outfile.write('\n'.encode(encoding))


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('filenames', action='store', nargs='+',
                        help='Files you wish to sign')

    parser.add_argument('--overwrite', action='store_true',
                        dest='overwrite', default=False, 
                        help='Overwrite the original file'
                             ' (sign the original file)')

    opts = parser.parse_args()

    return opts


if __name__ == '__main__':
    opts = parse_args()

    for filename in opts.filenames:
        encoding = get_file_encoding(filename)
        if opts.overwrite:
            sign_original_file(filename, encoding)
        else:
            create_signed_file(filename, encoding)

