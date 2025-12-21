#!/usr/bin/env python3
#
# Script to find coverage info after running tests.
#
# Example:
# ./scripts/cov.py \
#     lfs.t.a.gcda lfs_util.t.a.gcda \
#     -Flfs.c -Flfs_util.c -slines
#
# Copyright (c) 2022, The littlefs authors.
# Copyright (c) 2020, Arm Limited. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import collections as co
import csv
import itertools as it
import json
import math as m
import os
import re
import shlex
import subprocess as sp

# TODO use explode_asserts to avoid counting assert branches?
# TODO use dwarf=info to find functions for inline functions?

GCOV_PATH = ['gcov']


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

# fractional fields, a/b
class Frac(co.namedtuple('Frac', 'a,b')):
    __slots__ = ()
    def __new__(cls, a=0, b=None):
        if isinstance(a, Frac) and b is None:
            return a
        if isinstance(a, str) and b is None:
            a, b = a.split('/', 1)
        if b is None:
            b = a
        return super().__new__(cls, Int(a), Int(b))

    def __str__(self):
        return '%s/%s' % (self.a, self.b)

    def __float__(self):
        return float(self.a)

    none = '%11s %7s' % ('-', '-')
    def table(self):
        t = self.a.x/self.b.x if self.b.x else 1.0
        return '%11s %7s' % (
            self,
            '∞%' if t == +m.inf
            else '-∞%' if t == -m.inf
            else '%.1f%%' % (100*t))

    diff_none = '%11s' % '-'
    def diff_table(self):
        return '%11s' % (self,)

    def diff_diff(self, other):
        new_a, new_b = self if self else (Int(0), Int(0))
        old_a, old_b = other if other else (Int(0), Int(0))
        return '%11s' % ('%s/%s' % (
            new_a.diff_diff(old_a).strip(),
            new_b.diff_diff(old_b).strip()))

    def ratio(self, other):
        new_a, new_b = self if self else (Int(0), Int(0))
        old_a, old_b = other if other else (Int(0), Int(0))
        new = new_a.x/new_b.x if new_b.x else 1.0
        old = old_a.x/old_b.x if old_b.x else 1.0
        return new - old

    def __add__(self, other):
        return self.__class__(self.a + other.a, self.b + other.b)

    def __sub__(self, other):
        return self.__class__(self.a - other.a, self.b - other.b)

    def __mul__(self, other):
        return self.__class__(self.a * other.a, self.b + other.b)

    def __lt__(self, other):
        self_t = self.a.x/self.b.x if self.b.x else 1.0
        other_t = other.a.x/other.b.x if other.b.x else 1.0
        return (self_t, self.a.x) < (other_t, other.a.x)

    def __gt__(self, other):
        return self.__class__.__lt__(other, self)

    def __le__(self, other):
        return not self.__gt__(other)

    def __ge__(self, other):
        return not self.__lt__(other)

# coverage results
class CovResult(co.namedtuple('CovResult', [
        'file', 'function', 'line',
        'calls', 'hits', 'funcs', 'lines', 'branches'])):
    _by = ['file', 'function', 'line']
    _fields = ['calls', 'hits', 'funcs', 'lines', 'branches']
    _sort = ['funcs', 'lines', 'branches', 'hits', 'calls']
    _types = {
        'calls': Int, 'hits': Int,
        'funcs': Frac, 'lines': Frac, 'branches': Frac}

    __slots__ = ()
    def __new__(cls, file='', function='', line=0,
            calls=0, hits=0, funcs=0, lines=0, branches=0):
        return super().__new__(cls, file, function, int(Int(line)),
            Int(calls), Int(hits), Frac(funcs), Frac(lines), Frac(branches))

    def __add__(self, other):
        return CovResult(self.file, self.function, self.line,
            max(self.calls, other.calls),
            max(self.hits, other.hits),
            self.funcs + other.funcs,
            self.lines + other.lines,
            self.branches + other.branches)


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)

def collect(gcda_paths, *,
        gcov_path=GCOV_PATH,
        sources=None,
        everything=False,
        **args):
    results = []
    for path in gcda_paths:
        # get coverage info through gcov's json output
        # note, gcov-path may contain extra args
        cmd = GCOV_PATH + ['-b', '-t', '--json-format', path]
        if args.get('verbose'):
            print(' '.join(shlex.quote(c) for c in cmd))
        proc = sp.Popen(cmd,
            stdout=sp.PIPE,
            stderr=sp.PIPE if not args.get('verbose') else None,
            universal_newlines=True,
            errors='replace',
            close_fds=False)
        data = json.load(proc.stdout)
        proc.wait()
        if proc.returncode != 0:
            if not args.get('verbose'):
                for line in proc.stderr:
                    sys.stdout.write(line)
            sys.exit(-1)

        # collect line/branch coverage
        for file in data['files']:
            # ignore filtered sources
            if sources is not None:
                if not any(
                        os.path.abspath(file['file']) == os.path.abspath(s)
                        for s in sources):
                    continue
            else:
                # default to only cwd
                if not everything and not os.path.commonpath([
                        os.getcwd(),
                        os.path.abspath(file['file'])]) == os.getcwd():
                    continue

            # simplify path
            if os.path.commonpath([
                    os.getcwd(),
                    os.path.abspath(file['file'])]) == os.getcwd():
                file_name = os.path.relpath(file['file'])
            else:
                file_name = os.path.abspath(file['file'])

            for func in file['functions']:
                func_name = func.get('name', '(inlined)')
                # discard internal functions (this includes injected test cases)
                if not everything:
                    if func_name.startswith('__'):
                        continue

                # go ahead and add functions, later folding will merge this if
                # there are other hits on this line
                results.append(CovResult(
                    file_name, func_name, func['start_line'],
                    func['execution_count'], 0,
                    Frac(1 if func['execution_count'] > 0 else 0, 1),
                    0,
                    0))

            for line in file['lines']:
                func_name = line.get('function_name', '(inlined)')
                # discard internal function (this includes injected test cases)
                if not everything:
                    if func_name.startswith('__'):
                        continue

                # go ahead and add lines, later folding will merge this if
                # there are other hits on this line
                results.append(CovResult(
                    file_name, func_name, line['line_number'],
                    0, line['count'],
                    0,
                    Frac(1 if line['count'] > 0 else 0, 1),
                    Frac(
                        sum(1 if branch['count'] > 0 else 0
                            for branch in line['branches']),
                        len(line['branches']))))

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


def annotate(Result, results, *,
        annotate=False,
        lines=False,
        branches=False,
        **args):
    # if neither branches/lines specified, color both
    if annotate and not lines and not branches:
        lines, branches = True, True

    for path in co.OrderedDict.fromkeys(r.file for r in results).keys():
        # flatten to line info
        results = fold(Result, results, by=['file', 'line'])
        table = {r.line: r for r in results if r.file == path}

        # calculate spans to show
        if not annotate:
            spans = []
            last = None
            func = None
            for line, r in sorted(table.items()):
                if ((lines and int(r.hits) == 0)
                        or (branches and r.branches.a < r.branches.b)):
                    if last is not None and line - last.stop <= args['context']:
                        last = range(
                            last.start,
                            line+1+args['context'])
                    else:
                        if last is not None:
                            spans.append((last, func))
                        last = range(
                            line-args['context'],
                            line+1+args['context'])
                        func = r.function
            if last is not None:
                spans.append((last, func))

        with open(path) as f:
            skipped = False
            for i, line in enumerate(f):
                # skip lines not in spans?
                if not annotate and not any(i+1 in s for s, _ in spans):
                    skipped = True
                    continue

                if skipped:
                    skipped = False
                    print('%s@@ %s:%d: %s @@%s' % (
                        '\x1b[36m' if args['color'] else '',
                        path,
                        i+1,
                        next(iter(f for _, f in spans)),
                        '\x1b[m' if args['color'] else ''))

                # build line
                if line.endswith('\n'):
                    line = line[:-1]

                if i+1 in table:
                    r = table[i+1]
                    line = '%-*s // %s hits%s' % (
                        args['width'],
                        line,
                        r.hits,
                        ', %s branches' % (r.branches,)
                            if int(r.branches.b) else '')

                    if args['color']:
                        if lines and int(r.hits) == 0:
                            line = '\x1b[1;31m%s\x1b[m' % line
                        elif branches and r.branches.a < r.branches.b:
                            line = '\x1b[35m%s\x1b[m' % line

                print(line)


def main(gcda_paths, *,
        by=None,
        fields=None,
        defines=None,
        sort=None,
        hits=False,
        **args):
    # figure out what color should be
    if args.get('color') == 'auto':
        args['color'] = sys.stdout.isatty()
    elif args.get('color') == 'always':
        args['color'] = True
    else:
        args['color'] = False

    # find sizes
    if not args.get('use', None):
        results = collect(gcda_paths, **args)
    else:
        results = []
        with openio(args['use']) as f:
            reader = csv.DictReader(f, restval='')
            for r in reader:
                if not any('cov_'+k in r and r['cov_'+k].strip()
                        for k in CovResult._fields):
                    continue
                try:
                    results.append(CovResult(
                        **{k: r[k] for k in CovResult._by
                            if k in r and r[k].strip()},
                        **{k: r['cov_'+k]
                            for k in CovResult._fields
                            if 'cov_'+k in r
                                and r['cov_'+k].strip()}))
                except TypeError:
                    pass

    # fold
    results = fold(CovResult, results, by=by, defines=defines)

    # sort, note that python's sort is stable
    results.sort()
    if sort:
        for k, reverse in reversed(sort):
            results.sort(
                key=lambda r: tuple(
                    (getattr(r, k),) if getattr(r, k) is not None else ()
                    for k in ([k] if k else CovResult._sort)),
                reverse=reverse ^ (not k or k in CovResult._fields))

    # write results to CSV
    if args.get('output'):
        with openio(args['output'], 'w') as f:
            writer = csv.DictWriter(f,
                (by if by is not None else CovResult._by)
                + ['cov_'+k for k in (
                    fields if fields is not None else CovResult._fields)])
            writer.writeheader()
            for r in results:
                writer.writerow(
                    {k: getattr(r, k) for k in (
                        by if by is not None else CovResult._by)}
                    | {'cov_'+k: getattr(r, k) for k in (
                        fields if fields is not None else CovResult._fields)})

    # find previous results?
    if args.get('diff'):
        diff_results = []
        try:
            with openio(args['diff']) as f:
                reader = csv.DictReader(f, restval='')
                for r in reader:
                    if not any('cov_'+k in r and r['cov_'+k].strip()
                            for k in CovResult._fields):
                        continue
                    try:
                        diff_results.append(CovResult(
                            **{k: r[k] for k in CovResult._by
                                if k in r and r[k].strip()},
                            **{k: r['cov_'+k]
                                for k in CovResult._fields
                                if 'cov_'+k in r
                                    and r['cov_'+k].strip()}))
                    except TypeError:
                        pass
        except FileNotFoundError:
            pass

        # fold
        diff_results = fold(CovResult, diff_results,
            by=by, defines=defines)

    # print table
    if not args.get('quiet'):
        if (args.get('annotate')
                or args.get('lines')
                or args.get('branches')):
            # annotate sources
            annotate(CovResult, results, **args)
        else:
            # print table
            table(CovResult, results,
                diff_results if args.get('diff') else None,
                by=by if by is not None else ['function'],
                fields=fields if fields is not None
                    else ['lines', 'branches'] if not hits
                    else ['calls', 'hits'],
                sort=sort,
                **args)

    # catch lack of coverage
    if args.get('error_on_lines') and any(
            r.lines.a < r.lines.b for r in results):
        sys.exit(2)
    elif args.get('error_on_branches') and any(
            r.branches.a < r.branches.b for r in results):
        sys.exit(3)


if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser(
        description="Find coverage info after running tests.",
        allow_abbrev=False)
    parser.add_argument(
        'gcda_paths',
        nargs='*',
        help="Input *.gcda files.")
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
        choices=CovResult._by,
        help="Group by this field.")
    parser.add_argument(
        '-f', '--field',
        dest='fields',
        action='append',
        choices=CovResult._fields,
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
        '--hits',
        action='store_true',
        help="Show total hits instead of coverage.")
    parser.add_argument(
        '-A', '--annotate',
        action='store_true',
        help="Show source files annotated with coverage info.")
    parser.add_argument(
        '-L', '--lines',
        action='store_true',
        help="Show uncovered lines.")
    parser.add_argument(
        '-B', '--branches',
        action='store_true',
        help="Show uncovered branches.")
    parser.add_argument(
        '-c', '--context',
        type=lambda x: int(x, 0),
        default=3,
        help="Show n additional lines of context. Defaults to 3.")
    parser.add_argument(
        '-W', '--width',
        type=lambda x: int(x, 0),
        default=80,
        help="Assume source is styled with this many columns. Defaults to 80.")
    parser.add_argument(
        '--color',
        choices=['never', 'always', 'auto'],
        default='auto',
        help="When to use terminal colors. Defaults to 'auto'.")
    parser.add_argument(
        '-e', '--error-on-lines',
        action='store_true',
        help="Error if any lines are not covered.")
    parser.add_argument(
        '-E', '--error-on-branches',
        action='store_true',
        help="Error if any branches are not covered.")
    parser.add_argument(
        '--gcov-path',
        default=GCOV_PATH,
        type=lambda x: x.split(),
        help="Path to the gcov executable, may include paths. "
            "Defaults to %r." % GCOV_PATH)
    sys.exit(main(**{k: v
        for k, v in vars(parser.parse_intermixed_args()).items()
        if v is not None}))
