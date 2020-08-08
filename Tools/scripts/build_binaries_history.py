#!/usr/bin/env python

from __future__ import print_function

import os
import sqlite3


class BuildBinariesHistory():
    def __init__(self, db_filepath):
        self.db_filepath = db_filepath
        self.assure_db_present()

    def progress(self, msg):
        print("BBHIST: %s" % msg)

    def conn(self):
        return sqlite3.connect(self.db_filepath)

    def create_schema(self, c):
        '''create our tables and whatnot'''
        schema_version = 1
        c.execute("create table version (version integer)")
        c.execute("insert into version (version) values (?)", (schema_version,))
        # at some stage we should probably directly associate build with runs....
        c.execute("create table build (hash text, tag text, vehicle text, board text, "
                  "frame text, text integer, data integer, bss integer, start_time real, duration real)")
        c.execute("create table run (hash text, tag text, start_time real, duration real)")

    def sizes_for_file(self, filepath):
        cmd = "size %s" % (filepath,)
        stuff = os.popen(cmd).read()
        lines = stuff.split("\n")
        sizes = lines[1].split("\t")
        text = int(sizes[0])
        data = int(sizes[1])
        bss = int(sizes[2])
        print("text=%u" % text)
        print("data=%u" % data)
        print("bss=%u" % bss)
        return (text, data, bss)

    def assure_db_present(self):
        c = self.conn()
        need_schema_create = False
        try:
            version_cursor = c.execute("select version from version")
        except sqlite3.OperationalError as e:
            if "no such table" in str(e): # FIXME: do better here?  what's in "e"?
                print("need schema create")
                need_schema_create = True

        if need_schema_create:
            self.create_schema(c)
            version_cursor = c.execute("select version from version")

        version_results = version_cursor.fetchall()

        if len(version_results) == 0:
            raise IOError("No version number?")
        if len(version_results) > 1:
            raise IOError("More than one version result?")
        first = version_results[0]
        want_version = 1
        got_version = first[0]
        if got_version != want_version:
            raise IOError("Bad version number (want=%u got=%u" %
                          (want_version, got_version))
        self.progress("Got history version %u" % got_version)

    def record_build(self, hash, tag, vehicle, board, frame, bare_path, start_time, duration):
        if bare_path is None:
            (text, data, bss) = (None, None, None)
        else:
            (text, data, bss) = self.sizes_for_file(bare_path)
        c = self.conn()
        c.execute("replace into build (hash, tag, vehicle, board, frame, text, data, bss, start_time, duration) "
                  "values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                  (hash, tag, vehicle, board, frame, text, data, bss, start_time, duration))
        c.commit()

    def record_run(self, hash, tag, start_time, duration):
        c = self.conn()
        c.execute("replace into run (hash, tag, start_time, duration) "
                  "values (?, ?, ?, ?)",
                  (hash, tag, start_time, duration))
        c.commit()
