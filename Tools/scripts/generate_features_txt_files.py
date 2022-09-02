#!/usr/bin/env python

'''
recurse through directory tree rooted at supplied path.  For every elf file found, write out next to it a features.txt
'''

import optparse
import os
import sys

import extract_features

class GenerateFeatureTxtFiles(object):
    def __init__(self, path):
        self.path = path

    def write_features_txt_for_filepath(self, filepath):
        ef = extract_features.ExtractFeatures(filepath)
        text = ef.create_string()
        features_txt_filepath = os.path.join(os.path.dirname(filepath), "features.txt")
        with open(features_txt_filepath, "w") as fh:
            fh.write(text)

    def run(self):
        done_dirpaths = dict()
        for (dirpath, dirnames, filenames) in os.walk(self.path):
            for filename in filenames:
                if os.path.splitext(filename)[1].upper() != ".ELF":
                    continue
                if dirpath in done_dirpaths:
                    raise ValueError(
                        "Already processed elf (%s) in dirpath (%s) but also found elf (%s)" %
                        (done_dirpaths[dirpath], dirpath, filename))
                done_dirpaths[dirpath] = filename
                filepath = os.path.join(dirpath, filename)
                self.write_features_txt_for_filepath(filepath)


if __name__ == '__main__':

    parser = optparse.OptionParser("generate_features_txt_files.py DIRPATH")

    cmd_opts, cmd_args = parser.parse_args()

    if len(cmd_args) < 1:
        parser.print_help()
        sys.exit(1)

    dirpath = cmd_args[0]

    gen = GenerateFeatureTxtFiles(dirpath)
    gen.run()
