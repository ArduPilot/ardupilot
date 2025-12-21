#!/usr/bin/env python3
#
# Script to find data size at the function level. Basically just a big wrapper
# around nm with some extra conveniences for comparing builds. Heavily inspired
# by Linux's Bloat-O-Meter.
#
# Example:
# ./scripts/data.py lfs.o lfs_util.o -Ssize
#
# Copyright (c) 2022, The littlefs authors.
# Copyright (c) 2020, Arm Limited. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import collections as co
import csv
import difflib
import itertools as it
import math as m
import os
import re
import shlex
import subprocess as sp


NM_PATH = ['nm']
NM_TYPES = 'dDbB'
OBJDUMP_PATH = ['objdump']


# integer fields
class Int(co.namedtuple('Int', 'x')):
    __slots__ = ()
    def __new__(cls, x=0):
        if isinstance(x, Int):
            return x
        if isinstance(x, str):
            try:
                x = int(x, 0)
            except ValueError:
                # also accept +-∞ and +-inf
                if re.match('^\s*\+?\s*(?:∞|inf)\s*$', x):
                    x = m.inf
                elif re.match('^\s*-\s*(?:∞|inf)\s*$', x):
                    x = -m.inf
                else:
                    raise
        assert isinstance(x, int) or m.isinf(x), x
        return super().__new__(cls, x)

    def __str__(self):
        if self.x == m.inf:
            return '∞'
        elif self.x == -m.inf:
            return '-∞'
        else:
            return str(self.x)

    def __int__(self):
        assert not m.isinf(self.x)
        return self.x

    def __float__(self):
        return float(self.x)

    none = '%7s' % '-'
    def table(self):
        return '%7s' % (self,)

    diff_none = '%7s' % '-'
    diff_table = table

    def diff_diff(self, other):
        new = self.x if self else 0
        old = other.x if other else 0
        diff = new - old
        if diff == +m.inf:
            return '%7s' % '+∞'
        elif diff == -m.inf:
            return '%7s' % '-∞'
        else:
            return '%+7d' % diff

    def ratio(self, other):
        new = self.x if self else 0
        old = other.x if other else 0
        if m.isinf(new) and m.isinf(old):
            return 0.0
        elif m.isinf(new):
            return +m.inf
        elif m.isinf(old):
            return -m.inf
        elif not old and not new:
            return 0.0
        elif not old:
            return 1.0
        else:
            return (new-old) / old

    def __add__(self, other):
        return self.__class__(self.x + other.x)

    def __sub__(self, other):
        return self.__class__(self.x - other.x)

    def __mul__(self, other):
        return self.__class__(self.x * other.x)

# data size results
class DataResult(co.namedtuple('DataResult', [
        'file', 'function',
        'size'])):
    _by = ['file', 'function']
    _fields = ['size']
    _sort = ['size']
    _types = {'size': Int}

    __slots__ = ()
    def __new__(cls, file='', function='', size=0):
        return super().__new__(cls, file, function,
            Int(size))

    def __add__(self, other):
        return DataResult(self.file, self.function,
            self.size + other.size)


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)

def collect(obj_paths, *,
        nm_path=NM_PATH,
        nm_types=NM_TYPES,
        objdump_path=OBJDUMP_PATH,
        sources=None,
        everything=False,
        **args):
    size_pattern = re.compile(
        '^(?P<size>[0-9a-fA-F]+)' +
        ' (?P<type>[%s])' % re.escape(nm_types) +
        ' (?P<func>.+?)$')
    line_pattern = re.compile(
        '^\s+(?P<no>[0-9]+)'
            '(?:\s+(?P<dir>[0-9]+))?'
            '\s+.*'
            '\s+(?P<path>[^\s]+)$')
    info_pattern = re.compile(
        '^(?:.*(?P<tag>DW_TAG_[a-z_]+).*'
            '|.*DW_AT_name.*:\s*(?P<name>[^:\s]+)\s*'
            '|.*DW_AT_decl_file.*:\s*(?P<file>[0-9]+)\s*)$')

    results = []
    for path in obj_paths:
        # guess the source, if we have debug-info we'll replace this later
        file = re.sub('(\.o)?$', '.c', path, 1)

        # find symbol sizes
        results_ = []
        # note nm-path may contain extra args
        cmd = nm_path + ['--size-sort', path]
        if args.get('verbose'):
            print(' '.join(shlex.quote(c) for c in cmd))
        proc = sp.Popen(cmd,
            stdout=sp.PIPE,
            stderr=sp.PIPE if not args.get('verbose') else None,
            universal_newlines=True,
            errors='replace',
            close_fds=False)
        for line in proc.stdout:
            m = size_pattern.match(line)
            if m:
                func = m.group('func')
                # discard internal functions
                if not everything and func.startswith('__'):
                    continue
                results_.append(DataResult(
                    file, func,
                    int(m.group('size'), 16)))
        proc.wait()
        if proc.returncode != 0:
            if not args.get('verbose'):
                for line in proc.stderr:
                    sys.stdout.write(line)
            sys.exit(-1)


        # try to figure out the source file if we have debug-info
        dirs = {}
        files = {}
        # note objdump-path may contain extra args
        cmd = objdump_path + ['--dwarf=rawline', path]
        if args.get('verbose'):
            print(' '.join(shlex.quote(c) for c in cmd))
        proc = sp.Popen(cmd,
            stdout=sp.PIPE,
            stderr=sp.PIPE if not args.get('verbose') else None,
            universal_newlines=True,
            errors='replace',
            close_fds=False)
        for line in proc.stdout:
            # note that files contain references to dirs, which we
            # dereference as soon as we see them as each file table follows a
            # dir table
            m = line_pattern.match(line)
            if m:
                if not m.group('dir'):
                    # found a directory entry
                    dirs[int(m.group('no'))] = m.group('path')
                else:
                    # found a file entry
                    dir = int(m.group('dir'))
                    if dir in dirs:
                        files[int(m.group('no'))] = os.path.join(
                            dirs[dir],
                            m.group('path'))
                    else:
                        files[int(m.group('no'))] = m.group('path')
        proc.wait()
        if proc.returncode != 0:
            if not args.get('verbose'):
                for line in proc.stderr:
                    sys.stdout.write(line)
            # do nothing on error, we don't need objdump to work, source files
            # may just be inaccurate
            pass

        defs = {}
        is_func = False
        f_name = None
        f_file = None
        # note objdump-path may contain extra args
        cmd = objdump_path + ['--dwarf=info', path]
        if args.get('verbose'):
            print(' '.join(shlex.quote(c) for c in cmd))
        proc = sp.Popen(cmd,
            stdout=sp.PIPE,
            stderr=sp.PIPE if not args.get('verbose') else None,
            universal_newlines=True,
            errors='replace',
            close_fds=False)
        for line in proc.stdout:
            # state machine here to find definitions
            m = info_pattern.match(line)
            if m:
                if m.group('tag'):
                    if is_func:
                        defs[f_name] = files.get(f_file, '?')
                    is_func = (m.group('tag') == 'DW_TAG_subprogram')
                elif m.group('name'):
                    f_name = m.group('name')
                elif m.group('file'):
                    f_file = int(m.group('file'))
        if is_func:
            defs[f_name] = files.get(f_file, '?')
        proc.wait()
        if proc.returncode != 0:
            if not args.get('verbose'):
                for line in proc.stderr:
                    sys.stdout.write(line)
            # do nothing on error, we don't need objdump to work, source files
            # may just be inaccurate
            pass

        for r in results_:
            # find best matching debug symbol, this may be slightly different
            # due to optimizations
            if defs:
                # exact match? avoid difflib if we can for speed
                if r.function in defs:
                    file = defs[r.function]
                else:
                    _, file = max(
                        defs.items(),
                        key=lambda d: difflib.SequenceMatcher(None,
                            d[0],
                            r.function, False).ratio())
            else:
                file = r.file

            # ignore filtered sources
            if sources is not None:
                if not any(
                        os.path.abspath(file) == os.path.abspath(s)
                        for s in sources):
                    continue
            else:
                # default to only cwd
                if not everything and not os.path.commonpath([
                        os.getcwd(),
                        os.path.abspath(file)]) == os.getcwd():
                    continue

            # simplify path
            if os.path.commonpath([
                    os.getcwd(),
                    os.path.abspath(file)]) == os.getcwd():
                file = os.path.relpath(file)
            else:
                file = os.path.abspath(file)

            results.append(r._replace(file=file))

    return results


def fold(Result, results, *,
        by=None,
        defines=None,
        **_):
    if by is None:
        by = Result._by

    for k in it.chain(by or [], (k for k, _ in defines or [])):
        if k not in Result._by and k not in Result._fields:
            print("error: could not find field %r?" % k)
            sys.exit(-1)

    # filter by matching defines
    if defines is not None:
        results_ = []
        for r in results:
            if all(getattr(r, k) in vs for k, vs in defines):
                results_.append(r)
        results = results_

    # organize results into conflicts
    folding = co.OrderedDict()
    for r in results:
        name = tuple(getattr(r, k) for k in by)
        if name not in folding:
            folding[name] = []
        folding[name].append(r)

    # merge conflicts
    folded = []
    for name, rs in folding.items():
        folded.append(sum(rs[1:], start=rs[0]))

    return folded

def table(Result, results, diff_results=None, *,
        by=None,
        fields=None,
        sort=None,
        summary=False,
        all=False,
        percent=False,
        **_):
    all_, all = all, __builtins__.all

    if by is None:
        by = Result._by
    if fields is None:
        fields = Result._fields
    types = Result._types

    # fold again
    results = fold(Result, results, by=by)
    if diff_results is not None:
        diff_results = fold(Result, diff_results, by=by)

    # organize by name
    table = {
        ','.join(str(getattr(r, k) or '') for k in by): r
        for r in results}
    diff_table = {
        ','.join(str(getattr(r, k) or '') for k in by): r
        for r in diff_results or []}
    names = list(table.keys() | diff_table.keys())

    # sort again, now with diff info, note that python's sort is stable
    names.sort()
    if diff_results is not None:
        names.sort(key=lambda n: tuple(
            types[k].ratio(
                getattr(table.get(n), k, None),
                getattr(diff_table.get(n), k, None))
            for k in fields),
            reverse=True)
    if sort:
        for k, reverse in reversed(sort):
            names.sort(
                key=lambda n: tuple(
                    (getattr(table[n], k),)
                    if getattr(table.get(n), k, None) is not None else ()
                    for k in ([k] if k else [
                        k for k in Result._sort if k in fields])),
                reverse=reverse ^ (not k or k in Result._fields))


    # build up our lines
    lines = []

    # header
    header = []
    header.append('%s%s' % (
        ','.join(by),
        ' (%d added, %d removed)' % (
            sum(1 for n in table if n not in diff_table),
            sum(1 for n in diff_table if n not in table))
            if diff_results is not None and not percent else '')
        if not summary else '')
    if diff_results is None:
        for k in fields:
            header.append(k)
    elif percent:
        for k in fields:
            header.append(k)
    else:
        for k in fields:
            header.append('o'+k)
        for k in fields:
            header.append('n'+k)
        for k in fields:
            header.append('d'+k)
    header.append('')
    lines.append(header)

    def table_entry(name, r, diff_r=None, ratios=[]):
        entry = []
        entry.append(name)
        if diff_results is None:
            for k in fields:
                entry.append(getattr(r, k).table()
                    if getattr(r, k, None) is not None
                    else types[k].none)
        elif percent:
            for k in fields:
                entry.append(getattr(r, k).diff_table()
                    if getattr(r, k, None) is not None
                    else types[k].diff_none)
        else:
            for k in fields:
                entry.append(getattr(diff_r, k).diff_table()
                    if getattr(diff_r, k, None) is not None
                    else types[k].diff_none)
            for k in fields:
                entry.append(getattr(r, k).diff_table()
                    if getattr(r, k, None) is not None
                    else types[k].diff_none)
            for k in fields:
                entry.append(types[k].diff_diff(
                        getattr(r, k, None),
                        getattr(diff_r, k, None)))
        if diff_results is None:
            entry.append('')
        elif percent:
            entry.append(' (%s)' % ', '.join(
                '+∞%' if t == +m.inf
                else '-∞%' if t == -m.inf
                else '%+.1f%%' % (100*t)
                for t in ratios))
        else:
            entry.append(' (%s)' % ', '.join(
                    '+∞%' if t == +m.inf
                    else '-∞%' if t == -m.inf
                    else '%+.1f%%' % (100*t)
                    for t in ratios
                    if t)
                if any(ratios) else '')
        return entry

    # entries
    if not summary:
        for name in names:
            r = table.get(name)
            if diff_results is None:
                diff_r = None
                ratios = None
            else:
                diff_r = diff_table.get(name)
                ratios = [
                    types[k].ratio(
                        getattr(r, k, None),
                        getattr(diff_r, k, None))
                    for k in fields]
                if not all_ and not any(ratios):
                    continue
            lines.append(table_entry(name, r, diff_r, ratios))

    # total
    r = next(iter(fold(Result, results, by=[])), None)
    if diff_results is None:
        diff_r = None
        ratios = None
    else:
        diff_r = next(iter(fold(Result, diff_results, by=[])), None)
        ratios = [
            types[k].ratio(
                getattr(r, k, None),
                getattr(diff_r, k, None))
            for k in fields]
    lines.append(table_entry('TOTAL', r, diff_r, ratios))

    # find the best widths, note that column 0 contains the names and column -1
    # the ratios, so those are handled a bit differently
    widths = [
        ((max(it.chain([w], (len(l[i]) for l in lines)))+1+4-1)//4)*4-1
        for w, i in zip(
            it.chain([23], it.repeat(7)),
            range(len(lines[0])-1))]

    # print our table
    for line in lines:
        print('%-*s  %s%s' % (
            widths[0], line[0],
            ' '.join('%*s' % (w, x)
                for w, x in zip(widths[1:], line[1:-1])),
            line[-1]))


def main(obj_paths, *,
        by=None,
        fields=None,
        defines=None,
        sort=None,
        **args):
    # find sizes
    if not args.get('use', None):
        results = collect(obj_paths, **args)
    else:
        results = []
        with openio(args['use']) as f:
            reader = csv.DictReader(f, restval='')
            for r in reader:
                try:
                    results.append(DataResult(
                        **{k: r[k] for k in DataResult._by
                            if k in r and r[k].strip()},
                        **{k: r['data_'+k] for k in DataResult._fields
                            if 'data_'+k in r and r['data_'+k].strip()}))
                except TypeError:
                    pass

    # fold
    results = fold(DataResult, results, by=by, defines=defines)

    # sort, note that python's sort is stable
    results.sort()
    if sort:
        for k, reverse in reversed(sort):
            results.sort(
                key=lambda r: tuple(
                    (getattr(r, k),) if getattr(r, k) is not None else ()
                    for k in ([k] if k else DataResult._sort)),
                reverse=reverse ^ (not k or k in DataResult._fields))

    # write results to CSV
    if args.get('output'):
        with openio(args['output'], 'w') as f:
            writer = csv.DictWriter(f,
                (by if by is not None else DataResult._by)
                + ['data_'+k for k in (
                    fields if fields is not None else DataResult._fields)])
            writer.writeheader()
            for r in results:
                writer.writerow(
                    {k: getattr(r, k) for k in (
                        by if by is not None else DataResult._by)}
                    | {'data_'+k: getattr(r, k) for k in (
                        fields if fields is not None else DataResult._fields)})

    # find previous results?
    if args.get('diff'):
        diff_results = []
        try:
            with openio(args['diff']) as f:
                reader = csv.DictReader(f, restval='')
                for r in reader:
                    if not any('data_'+k in r and r['data_'+k].strip()
                            for k in DataResult._fields):
                        continue
                    try:
                        diff_results.append(DataResult(
                            **{k: r[k] for k in DataResult._by
                                if k in r and r[k].strip()},
                            **{k: r['data_'+k] for k in DataResult._fields
                                if 'data_'+k in r and r['data_'+k].strip()}))
                    except TypeError:
                        pass
        except FileNotFoundError:
            pass

        # fold
        diff_results = fold(DataResult, diff_results, by=by, defines=defines)

    # print table
    if not args.get('quiet'):
        table(DataResult, results,
            diff_results if args.get('diff') else None,
            by=by if by is not None else ['function'],
            fields=fields,
            sort=sort,
            **args)


if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser(
        description="Find data size at the function level.",
        allow_abbrev=False)
    parser.add_argument(
        'obj_paths',
        nargs='*',
        help="Input *.o files.")
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help="Output commands that run behind the scenes.")
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help="Don't show anything, useful with -o.")
    parser.add_argument(
        '-o', '--output',
        help="Specify CSV file to store results.")
    parser.add_argument(
        '-u', '--use',
        help="Don't parse anything, use this CSV file.")
    parser.add_argument(
        '-d', '--diff',
        help="Specify CSV file to diff against.")
    parser.add_argument(
        '-a', '--all',
        action='store_true',
        help="Show all, not just the ones that changed.")
    parser.add_argument(
        '-p', '--percent',
        action='store_true',
        help="Only show percentage change, not a full diff.")
    parser.add_argument(
        '-b', '--by',
        action='append',
        choices=DataResult._by,
        help="Group by this field.")
    parser.add_argument(
        '-f', '--field',
        dest='fields',
        action='append',
        choices=DataResult._fields,
        help="Show this field.")
    parser.add_argument(
        '-D', '--define',
        dest='defines',
        action='append',
        type=lambda x: (lambda k,v: (k, set(v.split(','))))(*x.split('=', 1)),
        help="Only include results where this field is this value.")
    class AppendSort(argparse.Action):
        def __call__(self, parser, namespace, value, option):
            if namespace.sort is None:
                namespace.sort = []
            namespace.sort.append((value, True if option == '-S' else False))
    parser.add_argument(
        '-s', '--sort',
        nargs='?',
        action=AppendSort,
        help="Sort by this field.")
    parser.add_argument(
        '-S', '--reverse-sort',
        nargs='?',
        action=AppendSort,
        help="Sort by this field, but backwards.")
    parser.add_argument(
        '-Y', '--summary',
        action='store_true',
        help="Only show the total.")
    parser.add_argument(
        '-F', '--source',
        dest='sources',
        action='append',
        help="Only consider definitions in this file. Defaults to anything "
            "in the current directory.")
    parser.add_argument(
        '--everything',
        action='store_true',
        help="Include builtin and libc specific symbols.")
    parser.add_argument(
        '--nm-types',
        default=NM_TYPES,
        help="Type of symbols to report, this uses the same single-character "
            "type-names emitted by nm. Defaults to %r." % NM_TYPES)
    parser.add_argument(
        '--nm-path',
        type=lambda x: x.split(),
        default=NM_PATH,
        help="Path to the nm executable, may include flags. "
            "Defaults to %r." % NM_PATH)
    parser.add_argument(
        '--objdump-path',
        type=lambda x: x.split(),
        default=OBJDUMP_PATH,
        help="Path to the objdump executable, may include flags. "
            "Defaults to %r." % OBJDUMP_PATH)
    sys.exit(main(**{k: v
        for k, v in vars(parser.parse_intermixed_args()).items()
        if v is not None}))
