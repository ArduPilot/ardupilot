#!/usr/bin/env python3
#
# Script to compile and runs benches.
#
# Example:
# ./scripts/bench.py runners/bench_runner -b
#
# Copyright (c) 2022, The littlefs authors.
# SPDX-License-Identifier: BSD-3-Clause
#

import collections as co
import csv
import errno
import glob
import itertools as it
import math as m
import os
import pty
import re
import shlex
import shutil
import signal
import subprocess as sp
import threading as th
import time
import toml


RUNNER_PATH = './runners/bench_runner'
HEADER_PATH = 'runners/bench_runner.h'

GDB_PATH = ['gdb']
VALGRIND_PATH = ['valgrind']
PERF_SCRIPT = ['./scripts/perf.py']


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)

class BenchCase:
    # create a BenchCase object from a config
    def __init__(self, config, args={}):
        self.name = config.pop('name')
        self.path = config.pop('path')
        self.suite = config.pop('suite')
        self.lineno = config.pop('lineno', None)
        self.if_ = config.pop('if', None)
        if isinstance(self.if_, bool):
            self.if_ = 'true' if self.if_ else 'false'
        self.code = config.pop('code')
        self.code_lineno = config.pop('code_lineno', None)
        self.in_ = config.pop('in',
            config.pop('suite_in', None))

        # figure out defines and build possible permutations
        self.defines = set()
        self.permutations = []

        # defines can be a dict or a list or dicts
        suite_defines = config.pop('suite_defines', {})
        if not isinstance(suite_defines, list):
            suite_defines = [suite_defines]
        defines = config.pop('defines', {})
        if not isinstance(defines, list):
            defines = [defines]

        def csplit(v):
            # split commas but only outside of parens
            parens = 0
            i_ = 0
            for i in range(len(v)):
                if v[i] == ',' and parens == 0:
                    yield v[i_:i]
                    i_ = i+1
                elif v[i] in '([{':
                    parens += 1
                elif v[i] in '}])':
                    parens -= 1
            if v[i_:].strip():
                yield v[i_:]

        def parse_define(v):
            # a define entry can be a list
            if isinstance(v, list):
                for v_ in v:
                    yield from parse_define(v_)
            # or a string
            elif isinstance(v, str):
                # which can be comma-separated values, with optional
                # range statements. This matches the runtime define parser in
                # the runner itself.
                for v_ in csplit(v):
                    m = re.search(r'\brange\b\s*\('
                        '(?P<start>[^,\s]*)'
                        '\s*(?:,\s*(?P<stop>[^,\s]*)'
                        '\s*(?:,\s*(?P<step>[^,\s]*)\s*)?)?\)',
                        v_)
                    if m:
                        start = (int(m.group('start'), 0)
                            if m.group('start') else 0)
                        stop = (int(m.group('stop'), 0)
                            if m.group('stop') else None)
                        step = (int(m.group('step'), 0)
                            if m.group('step') else 1)
                        if m.lastindex <= 1:
                            start, stop = 0, start
                        for x in range(start, stop, step):
                            yield from parse_define('%s(%d)%s' % (
                                v_[:m.start()], x, v_[m.end():]))
                    else:
                        yield v_
            # or a literal value
            elif isinstance(v, bool):
                yield 'true' if v else 'false'
            else:
                yield v

        # build possible permutations
        for suite_defines_ in suite_defines:
            self.defines |= suite_defines_.keys()
            for defines_ in defines:
                self.defines |= defines_.keys()
                self.permutations.extend(dict(perm) for perm in it.product(*(
                    [(k, v) for v in parse_define(vs)]
                    for k, vs in sorted((suite_defines_ | defines_).items()))))

        for k in config.keys():
            print('%swarning:%s in %s, found unused key %r' % (
                '\x1b[01;33m' if args['color'] else '',
                '\x1b[m' if args['color'] else '',
                self.name,
                k),
                file=sys.stderr)


class BenchSuite:
    # create a BenchSuite object from a toml file
    def __init__(self, path, args={}):
        self.path = path
        self.name = os.path.basename(path)
        if self.name.endswith('.toml'):
            self.name = self.name[:-len('.toml')]

        # load toml file and parse bench cases
        with open(self.path) as f:
            # load benches
            config = toml.load(f)

            # find line numbers
            f.seek(0)
            case_linenos = []
            code_linenos = []
            for i, line in enumerate(f):
                match = re.match(
                    '(?P<case>\[\s*cases\s*\.\s*(?P<name>\w+)\s*\])'
                        '|' '(?P<code>code\s*=)',
                    line)
                if match and match.group('case'):
                    case_linenos.append((i+1, match.group('name')))
                elif match and match.group('code'):
                    code_linenos.append(i+2)

            # sort in case toml parsing did not retain order
            case_linenos.sort()

            cases = config.pop('cases')
            for (lineno, name), (nlineno, _) in it.zip_longest(
                    case_linenos, case_linenos[1:],
                    fillvalue=(float('inf'), None)):
                code_lineno = min(
                    (l for l in code_linenos if l >= lineno and l < nlineno),
                    default=None)
                cases[name]['lineno'] = lineno
                cases[name]['code_lineno'] = code_lineno

            self.if_ = config.pop('if', None)
            if isinstance(self.if_, bool):
                self.if_ = 'true' if self.if_ else 'false'

            self.code = config.pop('code', None)
            self.code_lineno = min(
                (l for l in code_linenos
                    if not case_linenos or l < case_linenos[0][0]),
                default=None)

            # a couple of these we just forward to all cases
            defines = config.pop('defines', {})
            in_ = config.pop('in', None)

            self.cases = []
            for name, case in sorted(cases.items(),
                    key=lambda c: c[1].get('lineno')):
                self.cases.append(BenchCase(config={
                    'name': name,
                    'path': path + (':%d' % case['lineno']
                        if 'lineno' in case else ''),
                    'suite': self.name,
                    'suite_defines': defines,
                    'suite_in': in_,
                    **case},
                    args=args))

            # combine per-case defines
            self.defines = set.union(*(
                set(case.defines) for case in self.cases))

        for k in config.keys():
            print('%swarning:%s in %s, found unused key %r' % (
                '\x1b[01;33m' if args['color'] else '',
                '\x1b[m' if args['color'] else '',
                self.name,
                k),
                file=sys.stderr)



def compile(bench_paths, **args):
    # find .toml files
    paths = []
    for path in bench_paths:
        if os.path.isdir(path):
            path = path + '/*.toml'

        for path in glob.glob(path):
            paths.append(path)

    if not paths:
        print('no bench suites found in %r?' % bench_paths)
        sys.exit(-1)

    # load the suites
    suites = [BenchSuite(path, args) for path in paths]
    suites.sort(key=lambda s: s.name)

    # check for name conflicts, these will cause ambiguity problems later
    # when running benches
    seen = {}
    for suite in suites:
        if suite.name in seen:
            print('%swarning:%s conflicting suite %r, %s and %s' % (
                '\x1b[01;33m' if args['color'] else '',
                '\x1b[m' if args['color'] else '',
                suite.name,
                suite.path,
                seen[suite.name].path),
                file=sys.stderr)
        seen[suite.name] = suite

        for case in suite.cases:
            # only allow conflicts if a case and its suite share a name
            if case.name in seen and not (
                    isinstance(seen[case.name], BenchSuite)
                    and seen[case.name].cases == [case]):
                print('%swarning:%s conflicting case %r, %s and %s' % (
                    '\x1b[01;33m' if args['color'] else '',
                    '\x1b[m' if args['color'] else '',
                    case.name,
                    case.path,
                    seen[case.name].path),
                    file=sys.stderr)
            seen[case.name] = case

    # we can only compile one bench suite at a time
    if not args.get('source'):
        if len(suites) > 1:
            print('more than one bench suite for compilation? (%r)' % bench_paths)
            sys.exit(-1)

        suite = suites[0]

    # write generated bench source
    if 'output' in args:
        with openio(args['output'], 'w') as f:
            _write = f.write
            def write(s):
                f.lineno += s.count('\n')
                _write(s)
            def writeln(s=''):
                f.lineno += s.count('\n') + 1
                _write(s)
                _write('\n')
            f.lineno = 1
            f.write = write
            f.writeln = writeln

            f.writeln("// Generated by %s:" % sys.argv[0])
            f.writeln("//")
            f.writeln("// %s" % ' '.join(sys.argv))
            f.writeln("//")
            f.writeln()

            # include bench_runner.h in every generated file
            f.writeln("#include \"%s\"" % args['include'])
            f.writeln()

            # write out generated functions, this can end up in different
            # files depending on the "in" attribute
            #
            # note it's up to the specific generated file to declare
            # the bench defines
            def write_case_functions(f, suite, case):
                    # create case define functions
                    if case.defines:
                        # deduplicate defines by value to try to reduce the
                        # number of functions we generate
                        define_cbs = {}
                        for i, defines in enumerate(case.permutations):
                            for k, v in sorted(defines.items()):
                                if v not in define_cbs:
                                    name = ('__bench__%s__%s__%d'
                                        % (case.name, k, i))
                                    define_cbs[v] = name
                                    f.writeln('intmax_t %s('
                                        '__attribute__((unused)) '
                                        'void *data) {' % name)
                                    f.writeln(4*' '+'return %s;' % v)
                                    f.writeln('}')
                                    f.writeln()
                        f.writeln('const bench_define_t '
                            '__bench__%s__defines[]['
                            'BENCH_IMPLICIT_DEFINE_COUNT+%d] = {'
                            % (case.name, len(suite.defines)))
                        for defines in case.permutations:
                            f.writeln(4*' '+'{')
                            for k, v in sorted(defines.items()):
                                f.writeln(8*' '+'[%-24s] = {%s, NULL},' % (
                                    k+'_i', define_cbs[v]))
                            f.writeln(4*' '+'},')
                        f.writeln('};')
                        f.writeln()

                    # create case filter function
                    if suite.if_ is not None or case.if_ is not None:
                        f.writeln('bool __bench__%s__filter(void) {'
                            % (case.name))
                        f.writeln(4*' '+'return %s;'
                            % ' && '.join('(%s)' % if_
                                for if_ in [suite.if_, case.if_]
                                if if_ is not None))
                        f.writeln('}')
                        f.writeln()

                    # create case run function
                    f.writeln('void __bench__%s__run('
                        '__attribute__((unused)) struct lfs_config *cfg) {'
                        % (case.name))
                    f.writeln(4*' '+'// bench case %s' % case.name)
                    if case.code_lineno is not None:
                        f.writeln(4*' '+'#line %d "%s"'
                            % (case.code_lineno, suite.path))
                    f.write(case.code)
                    if case.code_lineno is not None:
                        f.writeln(4*' '+'#line %d "%s"'
                            % (f.lineno+1, args['output']))
                    f.writeln('}')
                    f.writeln()

            if not args.get('source'):
                if suite.code is not None:
                    if suite.code_lineno is not None:
                        f.writeln('#line %d "%s"'
                            % (suite.code_lineno, suite.path))
                    f.write(suite.code)
                    if suite.code_lineno is not None:
                        f.writeln('#line %d "%s"'
                            % (f.lineno+1, args['output']))
                    f.writeln()

                if suite.defines:
                    for i, define in enumerate(sorted(suite.defines)):
                        f.writeln('#ifndef %s' % define)
                        f.writeln('#define %-24s '
                            'BENCH_IMPLICIT_DEFINE_COUNT+%d' % (define+'_i', i))
                        f.writeln('#define %-24s '
                            'BENCH_DEFINE(%s)' % (define, define+'_i'))
                        f.writeln('#endif')
                    f.writeln()

                # create case functions
                for case in suite.cases:
                    if case.in_ is None:
                        write_case_functions(f, suite, case)
                    else:
                        if case.defines:
                            f.writeln('extern const bench_define_t '
                                '__bench__%s__defines[]['
                                'BENCH_IMPLICIT_DEFINE_COUNT+%d];'
                                % (case.name, len(suite.defines)))
                        if suite.if_ is not None or case.if_ is not None:
                            f.writeln('extern bool __bench__%s__filter('
                                'void);'
                                % (case.name))
                        f.writeln('extern void __bench__%s__run('
                            'struct lfs_config *cfg);'
                            % (case.name))
                        f.writeln()

                # create suite struct
                #
                # note we place this in the custom bench_suites section with
                # minimum alignment, otherwise GCC ups the alignment to
                # 32-bytes for some reason
                f.writeln('__attribute__((section("_bench_suites"), '
                    'aligned(1)))')
                f.writeln('const struct bench_suite __bench__%s__suite = {'
                    % suite.name)
                f.writeln(4*' '+'.name = "%s",' % suite.name)
                f.writeln(4*' '+'.path = "%s",' % suite.path)
                f.writeln(4*' '+'.flags = 0,')
                if suite.defines:
                    # create suite define names
                    f.writeln(4*' '+'.define_names = (const char *const['
                        'BENCH_IMPLICIT_DEFINE_COUNT+%d]){' % (
                        len(suite.defines)))
                    for k in sorted(suite.defines):
                        f.writeln(8*' '+'[%-24s] = "%s",' % (k+'_i', k))
                    f.writeln(4*' '+'},')
                    f.writeln(4*' '+'.define_count = '
                        'BENCH_IMPLICIT_DEFINE_COUNT+%d,' % len(suite.defines))
                f.writeln(4*' '+'.cases = (const struct bench_case[]){')
                for case in suite.cases:
                    # create case structs
                    f.writeln(8*' '+'{')
                    f.writeln(12*' '+'.name = "%s",' % case.name)
                    f.writeln(12*' '+'.path = "%s",' % case.path)
                    f.writeln(12*' '+'.flags = 0,')
                    f.writeln(12*' '+'.permutations = %d,'
                        % len(case.permutations))
                    if case.defines:
                        f.writeln(12*' '+'.defines '
                            '= (const bench_define_t*)__bench__%s__defines,'
                            % (case.name))
                    if suite.if_ is not None or case.if_ is not None:
                        f.writeln(12*' '+'.filter = __bench__%s__filter,'
                            % (case.name))
                    f.writeln(12*' '+'.run = __bench__%s__run,'
                        % (case.name))
                    f.writeln(8*' '+'},')
                f.writeln(4*' '+'},')
                f.writeln(4*' '+'.case_count = %d,' % len(suite.cases))
                f.writeln('};')
                f.writeln()

            else:
                # copy source
                f.writeln('#line 1 "%s"' % args['source'])
                with open(args['source']) as sf:
                    shutil.copyfileobj(sf, f)
                f.writeln()

                # write any internal benches
                for suite in suites:
                    for case in suite.cases:
                        if (case.in_ is not None
                                and os.path.normpath(case.in_)
                                    == os.path.normpath(args['source'])):
                            # write defines, but note we need to undef any
                            # new defines since we're in someone else's file
                            if suite.defines:
                                for i, define in enumerate(
                                        sorted(suite.defines)):
                                    f.writeln('#ifndef %s' % define)
                                    f.writeln('#define %-24s '
                                        'BENCH_IMPLICIT_DEFINE_COUNT+%d' % (
                                        define+'_i', i))
                                    f.writeln('#define %-24s '
                                        'BENCH_DEFINE(%s)' % (
                                        define, define+'_i'))
                                    f.writeln('#define '
                                        '__BENCH__%s__NEEDS_UNDEF' % (
                                        define))
                                    f.writeln('#endif')
                                f.writeln()

                            write_case_functions(f, suite, case)

                            if suite.defines:
                                for define in sorted(suite.defines):
                                    f.writeln('#ifdef __BENCH__%s__NEEDS_UNDEF'
                                        % define)
                                    f.writeln('#undef __BENCH__%s__NEEDS_UNDEF'
                                        % define)
                                    f.writeln('#undef %s' % define)
                                    f.writeln('#undef %s' % (define+'_i'))
                                    f.writeln('#endif')
                                f.writeln()

def find_runner(runner, **args):
    cmd = runner.copy()

    # run under some external command?
    if args.get('exec'):
        cmd[:0] = args['exec']

    # run under valgrind?
    if args.get('valgrind'):
        cmd[:0] = args['valgrind_path'] + [
            '--leak-check=full',
            '--track-origins=yes',
            '--error-exitcode=4',
            '-q']

    # run under perf?
    if args.get('perf'):
        cmd[:0] = args['perf_script'] + list(filter(None, [
            '-R',
            '--perf-freq=%s' % args['perf_freq']
                if args.get('perf_freq') else None,
            '--perf-period=%s' % args['perf_period']
                if args.get('perf_period') else None,
            '--perf-events=%s' % args['perf_events']
                if args.get('perf_events') else None,
            '--perf-path=%s' % args['perf_path']
                if args.get('perf_path') else None,
            '-o%s' % args['perf']]))

    # other context
    if args.get('geometry'):
        cmd.append('-G%s' % args['geometry'])
    if args.get('disk'):
        cmd.append('-d%s' % args['disk'])
    if args.get('trace'):
        cmd.append('-t%s' % args['trace'])
    if args.get('trace_backtrace'):
        cmd.append('--trace-backtrace')
    if args.get('trace_period'):
        cmd.append('--trace-period=%s' % args['trace_period'])
    if args.get('trace_freq'):
        cmd.append('--trace-freq=%s' % args['trace_freq'])
    if args.get('read_sleep'):
        cmd.append('--read-sleep=%s' % args['read_sleep'])
    if args.get('prog_sleep'):
        cmd.append('--prog-sleep=%s' % args['prog_sleep'])
    if args.get('erase_sleep'):
        cmd.append('--erase-sleep=%s' % args['erase_sleep'])

    # defines?
    if args.get('define'):
        for define in args.get('define'):
            cmd.append('-D%s' % define)

    return cmd

def list_(runner, bench_ids=[], **args):
    cmd = find_runner(runner, **args) + bench_ids
    if args.get('summary'):          cmd.append('--summary')
    if args.get('list_suites'):      cmd.append('--list-suites')
    if args.get('list_cases'):       cmd.append('--list-cases')
    if args.get('list_suite_paths'): cmd.append('--list-suite-paths')
    if args.get('list_case_paths'):  cmd.append('--list-case-paths')
    if args.get('list_defines'):     cmd.append('--list-defines')
    if args.get('list_permutation_defines'):
                                     cmd.append('--list-permutation-defines')
    if args.get('list_implicit_defines'):
                                     cmd.append('--list-implicit-defines')
    if args.get('list_geometries'):  cmd.append('--list-geometries')

    if args.get('verbose'):
        print(' '.join(shlex.quote(c) for c in cmd))
    return sp.call(cmd)


def find_perms(runner_, ids=[], **args):
    case_suites = {}
    expected_case_perms = co.defaultdict(lambda: 0)
    expected_perms = 0
    total_perms = 0

    # query cases from the runner
    cmd = runner_ + ['--list-cases'] + ids
    if args.get('verbose'):
        print(' '.join(shlex.quote(c) for c in cmd))
    proc = sp.Popen(cmd,
        stdout=sp.PIPE,
        stderr=sp.PIPE if not args.get('verbose') else None,
        universal_newlines=True,
        errors='replace',
        close_fds=False)
    pattern = re.compile(
        '^(?P<case>[^\s]+)'
            '\s+(?P<flags>[^\s]+)'
            '\s+(?P<filtered>\d+)/(?P<perms>\d+)')
    # skip the first line
    for line in it.islice(proc.stdout, 1, None):
        m = pattern.match(line)
        if m:
            filtered = int(m.group('filtered'))
            perms = int(m.group('perms'))
            expected_case_perms[m.group('case')] += filtered
            expected_perms += filtered
            total_perms += perms
    proc.wait()
    if proc.returncode != 0:
        if not args.get('verbose'):
            for line in proc.stderr:
                sys.stdout.write(line)
        sys.exit(-1)

    # get which suite each case belongs to via paths
    cmd = runner_ + ['--list-case-paths'] + ids
    if args.get('verbose'):
        print(' '.join(shlex.quote(c) for c in cmd))
    proc = sp.Popen(cmd,
        stdout=sp.PIPE,
        stderr=sp.PIPE if not args.get('verbose') else None,
        universal_newlines=True,
        errors='replace',
        close_fds=False)
    pattern = re.compile(
        '^(?P<case>[^\s]+)'
            '\s+(?P<path>[^:]+):(?P<lineno>\d+)')
    # skip the first line
    for line in it.islice(proc.stdout, 1, None):
        m = pattern.match(line)
        if m:
            path = m.group('path')
            # strip path/suffix here
            suite = os.path.basename(path)
            if suite.endswith('.toml'):
                suite = suite[:-len('.toml')]
            case_suites[m.group('case')] = suite
    proc.wait()
    if proc.returncode != 0:
        if not args.get('verbose'):
            for line in proc.stderr:
                sys.stdout.write(line)
        sys.exit(-1)

    # figure out expected suite perms
    expected_suite_perms = co.defaultdict(lambda: 0)
    for case, suite in case_suites.items():
        expected_suite_perms[suite] += expected_case_perms[case]

    return (
        case_suites,
        expected_suite_perms,
        expected_case_perms,
        expected_perms,
        total_perms)

def find_path(runner_, id, **args):
    path = None
    # query from runner
    cmd = runner_ + ['--list-case-paths', id]
    if args.get('verbose'):
        print(' '.join(shlex.quote(c) for c in cmd))
    proc = sp.Popen(cmd,
        stdout=sp.PIPE,
        stderr=sp.PIPE if not args.get('verbose') else None,
        universal_newlines=True,
        errors='replace',
        close_fds=False)
    pattern = re.compile(
        '^(?P<case>[^\s]+)'
            '\s+(?P<path>[^:]+):(?P<lineno>\d+)')
    # skip the first line
    for line in it.islice(proc.stdout, 1, None):
        m = pattern.match(line)
        if m and path is None:
            path_ = m.group('path')
            lineno = int(m.group('lineno'))
            path = (path_, lineno)
    proc.wait()
    if proc.returncode != 0:
        if not args.get('verbose'):
            for line in proc.stderr:
                sys.stdout.write(line)
        sys.exit(-1)

    return path

def find_defines(runner_, id, **args):
    # query permutation defines from runner
    cmd = runner_ + ['--list-permutation-defines', id]
    if args.get('verbose'):
        print(' '.join(shlex.quote(c) for c in cmd))
    proc = sp.Popen(cmd,
        stdout=sp.PIPE,
        stderr=sp.PIPE if not args.get('verbose') else None,
        universal_newlines=True,
        errors='replace',
        close_fds=False)
    defines = co.OrderedDict()
    pattern = re.compile('^(?P<define>\w+)=(?P<value>.+)')
    for line in proc.stdout:
        m = pattern.match(line)
        if m:
            define = m.group('define')
            value = m.group('value')
            defines[define] = value
    proc.wait()
    if proc.returncode != 0:
        if not args.get('verbose'):
            for line in proc.stderr:
                sys.stdout.write(line)
        sys.exit(-1)

    return defines


# Thread-safe CSV writer
class BenchOutput:
    def __init__(self, path, head=None, tail=None):
        self.f = openio(path, 'w+', 1)
        self.lock = th.Lock()
        self.head = head or []
        self.tail = tail or []
        self.writer = csv.DictWriter(self.f, self.head + self.tail)
        self.rows = []

    def close(self):
        self.f.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.f.close()

    def writerow(self, row):
        with self.lock:
            self.rows.append(row)
            if all(k in self.head or k in self.tail for k in row.keys()):
                # can simply append
                self.writer.writerow(row)
            else:
                # need to rewrite the file
                self.head.extend(row.keys() - (self.head + self.tail))
                self.f.seek(0)
                self.f.truncate()
                self.writer = csv.DictWriter(self.f, self.head + self.tail)
                self.writer.writeheader()
                for row in self.rows:
                    self.writer.writerow(row)

# A bench failure
class BenchFailure(Exception):
    def __init__(self, id, returncode, stdout, assert_=None):
        self.id = id
        self.returncode = returncode
        self.stdout = stdout
        self.assert_ = assert_

def run_stage(name, runner_, ids, stdout_, trace_, output_, **args):
    # get expected suite/case/perm counts
    (case_suites,
        expected_suite_perms,
        expected_case_perms,
        expected_perms,
        total_perms) = find_perms(runner_, ids, **args)

    passed_suite_perms = co.defaultdict(lambda: 0)
    passed_case_perms = co.defaultdict(lambda: 0)
    passed_perms = 0
    readed = 0
    proged = 0
    erased = 0
    failures = []
    killed = False

    pattern = re.compile('^(?:'
            '(?P<op>running|finished|skipped|powerloss)'
                ' (?P<id>(?P<case>[^:]+)[^\s]*)'
                '(?: (?P<readed>\d+))?'
                '(?: (?P<proged>\d+))?'
                '(?: (?P<erased>\d+))?'
            '|' '(?P<path>[^:]+):(?P<lineno>\d+):(?P<op_>assert):'
                ' *(?P<message>.*)'
        ')$')
    locals = th.local()
    children = set()

    def run_runner(runner_, ids=[]):
        nonlocal passed_suite_perms
        nonlocal passed_case_perms
        nonlocal passed_perms
        nonlocal readed
        nonlocal proged
        nonlocal erased
        nonlocal locals

        # run the benches!
        cmd = runner_ + ids
        if args.get('verbose'):
            print(' '.join(shlex.quote(c) for c in cmd))

        mpty, spty = pty.openpty()
        proc = sp.Popen(cmd, stdout=spty, stderr=spty, close_fds=False)
        os.close(spty)
        children.add(proc)
        mpty = os.fdopen(mpty, 'r', 1)

        last_id = None
        last_stdout = co.deque(maxlen=args.get('context', 5) + 1)
        last_assert = None
        try:
            while True:
                # parse a line for state changes
                try:
                    line = mpty.readline()
                except OSError as e:
                    if e.errno != errno.EIO:
                        raise
                    break
                if not line:
                    break
                last_stdout.append(line)
                if stdout_:
                    try:
                        stdout_.write(line)
                        stdout_.flush()
                    except BrokenPipeError:
                        pass

                m = pattern.match(line)
                if m:
                    op = m.group('op') or m.group('op_')
                    if op == 'running':
                        locals.seen_perms += 1
                        last_id = m.group('id')
                        last_stdout.clear()
                        last_assert = None
                    elif op == 'finished':
                        case = m.group('case')
                        suite = case_suites[case]
                        readed_ = int(m.group('readed'))
                        proged_ = int(m.group('proged'))
                        erased_ = int(m.group('erased'))
                        passed_suite_perms[suite] += 1
                        passed_case_perms[case] += 1
                        passed_perms += 1
                        readed += readed_
                        proged += proged_
                        erased += erased_
                        if output_:
                            # get defines and write to csv
                            defines = find_defines(
                                runner_, m.group('id'), **args)
                            output_.writerow({
                                'suite': suite,
                                'case': case,
                                'bench_readed': readed_,
                                'bench_proged': proged_,
                                'bench_erased': erased_,
                                **defines})
                    elif op == 'skipped':
                        locals.seen_perms += 1
                    elif op == 'assert':
                        last_assert = (
                            m.group('path'),
                            int(m.group('lineno')),
                            m.group('message'))
                        # go ahead and kill the process, aborting takes a while
                        if args.get('keep_going'):
                            proc.kill()
        except KeyboardInterrupt:
            raise BenchFailure(last_id, 1, list(last_stdout))
        finally:
            children.remove(proc)
            mpty.close()

        proc.wait()
        if proc.returncode != 0:
            raise BenchFailure(
                last_id,
                proc.returncode,
                list(last_stdout),
                last_assert)

    def run_job(runner_, ids=[], start=None, step=None):
        nonlocal failures
        nonlocal killed
        nonlocal locals

        start = start or 0
        step = step or 1
        while start < total_perms:
            job_runner = runner_.copy()
            if args.get('isolate') or args.get('valgrind'):
                job_runner.append('-s%s,%s,%s' % (start, start+step, step))
            else:
                job_runner.append('-s%s,,%s' % (start, step))

            try:
                # run the benches
                locals.seen_perms = 0
                run_runner(job_runner, ids)
                assert locals.seen_perms > 0
                start += locals.seen_perms*step

            except BenchFailure as failure:
                # keep track of failures
                if output_:
                    case, _ = failure.id.split(':', 1)
                    suite = case_suites[case]
                    # get defines and write to csv
                    defines = find_defines(runner_, failure.id, **args)
                    output_.writerow({
                        'suite': suite,
                        'case': case,
                        **defines})

                # race condition for multiple failures?
                if failures and not args.get('keep_going'):
                    break

                failures.append(failure)

                if args.get('keep_going') and not killed:
                    # resume after failed bench
                    assert locals.seen_perms > 0
                    start += locals.seen_perms*step
                    continue
                else:
                    # stop other benches
                    killed = True
                    for child in children.copy():
                        child.kill()
                    break


    # parallel jobs?
    runners = []
    if 'jobs' in args:
        for job in range(args['jobs']):
            runners.append(th.Thread(
                target=run_job, args=(runner_, ids, job, args['jobs']),
                daemon=True))
    else:
        runners.append(th.Thread(
            target=run_job, args=(runner_, ids, None, None),
            daemon=True))

    def print_update(done):
        if not args.get('verbose') and (args['color'] or done):
            sys.stdout.write('%s%srunning %s%s:%s %s%s' % (
                '\r\x1b[K' if args['color'] else '',
                '\x1b[?7l' if not done else '',
                ('\x1b[34m' if not failures else '\x1b[31m')
                    if args['color'] else '',
                name,
                '\x1b[m' if args['color'] else '',
                ', '.join(filter(None, [
                    '%d/%d suites' % (
                        sum(passed_suite_perms[k] == v
                            for k, v in expected_suite_perms.items()),
                        len(expected_suite_perms))
                        if (not args.get('by_suites')
                            and not args.get('by_cases')) else None,
                    '%d/%d cases' % (
                        sum(passed_case_perms[k] == v
                            for k, v in expected_case_perms.items()),
                        len(expected_case_perms))
                        if not args.get('by_cases') else None,
                    '%d/%d perms' % (passed_perms, expected_perms),
                    '%s%d/%d failures%s' % (
                            '\x1b[31m' if args['color'] else '',
                            len(failures),
                            expected_perms,
                            '\x1b[m' if args['color'] else '')
                        if failures else None])),
                '\x1b[?7h' if not done else '\n'))
            sys.stdout.flush()

    for r in runners:
        r.start()

    try:
        while any(r.is_alive() for r in runners):
            time.sleep(0.01)
            print_update(False)
    except KeyboardInterrupt:
        # this is handled by the runner threads, we just
        # need to not abort here
        killed = True
    finally:
        print_update(True)

    for r in runners:
        r.join()

    return (
        expected_perms,
        passed_perms,
        readed,
        proged,
        erased,
        failures,
        killed)


def run(runner, bench_ids=[], **args):
    # query runner for benches
    runner_ = find_runner(runner, **args)
    print('using runner: %s' % ' '.join(shlex.quote(c) for c in runner_))
    (_,
        expected_suite_perms,
        expected_case_perms,
        expected_perms,
        total_perms) = find_perms(runner_, bench_ids, **args)
    print('found %d suites, %d cases, %d/%d permutations' % (
        len(expected_suite_perms),
        len(expected_case_perms),
        expected_perms,
        total_perms))
    print()

    # automatic job detection?
    if args.get('jobs') == 0:
        args['jobs'] = len(os.sched_getaffinity(0))

    # truncate and open logs here so they aren't disconnected between benches
    stdout = None
    if args.get('stdout'):
        stdout = openio(args['stdout'], 'w', 1)
    trace = None
    if args.get('trace'):
        trace = openio(args['trace'], 'w', 1)
    output = None
    if args.get('output'):
        output = BenchOutput(args['output'],
            ['suite', 'case'],
            ['bench_readed', 'bench_proged', 'bench_erased'])

    # measure runtime
    start = time.time()

    # spawn runners
    expected = 0
    passed = 0
    readed = 0
    proged = 0
    erased = 0
    failures = []
    for by in (bench_ids if bench_ids
            else expected_case_perms.keys() if args.get('by_cases')
            else expected_suite_perms.keys() if args.get('by_suites')
            else [None]):
        # spawn jobs for stage
        (expected_,
            passed_,
            readed_,
            proged_,
            erased_,
            failures_,
            killed) = run_stage(
                by or 'benches',
                runner_,
                [by] if by is not None else [],
                stdout,
                trace,
                output,
                **args)
        # collect passes/failures
        expected += expected_
        passed += passed_
        readed += readed_
        proged += proged_
        erased += erased_
        failures.extend(failures_)
        if (failures and not args.get('keep_going')) or killed:
            break

    stop = time.time()

    if stdout:
        try:
            stdout.close()
        except BrokenPipeError:
            pass
    if trace:
        try:
            trace.close()
        except BrokenPipeError:
            pass
    if output:
        output.close()

    # show summary
    print()
    print('%sdone:%s %s' % (
        ('\x1b[34m' if not failures else '\x1b[31m')
            if args['color'] else '',
        '\x1b[m' if args['color'] else '',
        ', '.join(filter(None, [
            '%d readed' % readed,
            '%d proged' % proged,
            '%d erased' % erased,
            'in %.2fs' % (stop-start)]))))
    print()

    # print each failure
    for failure in failures:
        assert failure.id is not None, '%s broken? %r' % (
            ' '.join(shlex.quote(c) for c in runner_),
            failure)

        # get some extra info from runner
        path, lineno = find_path(runner_, failure.id, **args)
        defines = find_defines(runner_, failure.id, **args)

        # show summary of failure
        print('%s%s:%d:%sfailure:%s %s%s failed' % (
            '\x1b[01m' if args['color'] else '',
            path, lineno,
            '\x1b[01;31m' if args['color'] else '',
            '\x1b[m' if args['color'] else '',
            failure.id,
            ' (%s)' % ', '.join('%s=%s' % (k,v) for k,v in defines.items())
                if defines else ''))

        if failure.stdout:
            stdout = failure.stdout
            if failure.assert_ is not None:
                stdout = stdout[:-1]
            for line in stdout[-args.get('context', 5):]:
                sys.stdout.write(line)

        if failure.assert_ is not None:
            path, lineno, message = failure.assert_
            print('%s%s:%d:%sassert:%s %s' % (
                '\x1b[01m' if args['color'] else '',
                path, lineno,
                '\x1b[01;31m' if args['color'] else '',
                '\x1b[m' if args['color'] else '',
                message))
            with open(path) as f:
                line = next(it.islice(f, lineno-1, None)).strip('\n')
                print(line)
        print()

    # drop into gdb?
    if failures and (args.get('gdb')
            or args.get('gdb_case')
            or args.get('gdb_main')):
        failure = failures[0]
        cmd = runner_ + [failure.id]

        if args.get('gdb_main'):
            # we don't really need the case breakpoint here, but it
            # can be helpful
            path, lineno = find_path(runner_, failure.id, **args)
            cmd[:0] = args['gdb_path'] + [
                '-ex', 'break main',
                '-ex', 'break %s:%d' % (path, lineno),
                '-ex', 'run',
                '--args']
        elif args.get('gdb_case'):
            path, lineno = find_path(runner_, failure.id, **args)
            cmd[:0] = args['gdb_path'] + [
                '-ex', 'break %s:%d' % (path, lineno),
                '-ex', 'run',
                '--args']
        elif failure.assert_ is not None:
            cmd[:0] = args['gdb_path'] + [
                '-ex', 'run',
                '-ex', 'frame function raise',
                '-ex', 'up 2',
                '--args']
        else:
            cmd[:0] = args['gdb_path'] + [
                '-ex', 'run',
                '--args']

        # exec gdb interactively
        if args.get('verbose'):
            print(' '.join(shlex.quote(c) for c in cmd))
        os.execvp(cmd[0], cmd)

    return 1 if failures else 0


def main(**args):
    # figure out what color should be
    if args.get('color') == 'auto':
        args['color'] = sys.stdout.isatty()
    elif args.get('color') == 'always':
        args['color'] = True
    else:
        args['color'] = False

    if args.get('compile'):
        return compile(**args)
    elif (args.get('summary')
            or args.get('list_suites')
            or args.get('list_cases')
            or args.get('list_suite_paths')
            or args.get('list_case_paths')
            or args.get('list_defines')
            or args.get('list_permutation_defines')
            or args.get('list_implicit_defines')
            or args.get('list_geometries')):
        return list_(**args)
    else:
        return run(**args)


if __name__ == "__main__":
    import argparse
    import sys
    argparse.ArgumentParser._handle_conflict_ignore = lambda *_: None
    argparse._ArgumentGroup._handle_conflict_ignore = lambda *_: None
    parser = argparse.ArgumentParser(
        description="Build and run benches.",
        allow_abbrev=False,
        conflict_handler='ignore')
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help="Output commands that run behind the scenes.")
    parser.add_argument(
        '--color',
        choices=['never', 'always', 'auto'],
        default='auto',
        help="When to use terminal colors. Defaults to 'auto'.")

    # bench flags
    bench_parser = parser.add_argument_group('bench options')
    bench_parser.add_argument(
        'runner',
        nargs='?',
        type=lambda x: x.split(),
        help="Bench runner to use for benching. Defaults to %r." % RUNNER_PATH)
    bench_parser.add_argument(
        'bench_ids',
        nargs='*',
        help="Description of benches to run.")
    bench_parser.add_argument(
        '-Y', '--summary',
        action='store_true',
        help="Show quick summary.")
    bench_parser.add_argument(
        '-l', '--list-suites',
        action='store_true',
        help="List bench suites.")
    bench_parser.add_argument(
        '-L', '--list-cases',
        action='store_true',
        help="List bench cases.")
    bench_parser.add_argument(
        '--list-suite-paths',
        action='store_true',
        help="List the path for each bench suite.")
    bench_parser.add_argument(
        '--list-case-paths',
        action='store_true',
        help="List the path and line number for each bench case.")
    bench_parser.add_argument(
        '--list-defines',
        action='store_true',
        help="List all defines in this bench-runner.")
    bench_parser.add_argument(
        '--list-permutation-defines',
        action='store_true',
        help="List explicit defines in this bench-runner.")
    bench_parser.add_argument(
        '--list-implicit-defines',
        action='store_true',
        help="List implicit defines in this bench-runner.")
    bench_parser.add_argument(
        '--list-geometries',
        action='store_true',
        help="List the available disk geometries.")
    bench_parser.add_argument(
        '-D', '--define',
        action='append',
        help="Override a bench define.")
    bench_parser.add_argument(
        '-G', '--geometry',
        help="Comma-separated list of disk geometries to bench.")
    bench_parser.add_argument(
        '-d', '--disk',
        help="Direct block device operations to this file.")
    bench_parser.add_argument(
        '-t', '--trace',
        help="Direct trace output to this file.")
    bench_parser.add_argument(
        '--trace-backtrace',
        action='store_true',
        help="Include a backtrace with every trace statement.")
    bench_parser.add_argument(
        '--trace-period',
        help="Sample trace output at this period in cycles.")
    bench_parser.add_argument(
        '--trace-freq',
        help="Sample trace output at this frequency in hz.")
    bench_parser.add_argument(
        '-O', '--stdout',
        help="Direct stdout to this file. Note stderr is already merged here.")
    bench_parser.add_argument(
        '-o', '--output',
        help="CSV file to store results.")
    bench_parser.add_argument(
        '--read-sleep',
        help="Artificial read delay in seconds.")
    bench_parser.add_argument(
        '--prog-sleep',
        help="Artificial prog delay in seconds.")
    bench_parser.add_argument(
        '--erase-sleep',
        help="Artificial erase delay in seconds.")
    bench_parser.add_argument(
        '-j', '--jobs',
        nargs='?',
        type=lambda x: int(x, 0),
        const=0,
        help="Number of parallel runners to run. 0 runs one runner per core.")
    bench_parser.add_argument(
        '-k', '--keep-going',
        action='store_true',
        help="Don't stop on first error.")
    bench_parser.add_argument(
        '-i', '--isolate',
        action='store_true',
        help="Run each bench permutation in a separate process.")
    bench_parser.add_argument(
        '-b', '--by-suites',
        action='store_true',
        help="Step through benches by suite.")
    bench_parser.add_argument(
        '-B', '--by-cases',
        action='store_true',
        help="Step through benches by case.")
    bench_parser.add_argument(
        '--context',
        type=lambda x: int(x, 0),
        default=5,
        help="Show this many lines of stdout on bench failure. "
            "Defaults to 5.")
    bench_parser.add_argument(
        '--gdb',
        action='store_true',
        help="Drop into gdb on bench failure.")
    bench_parser.add_argument(
        '--gdb-case',
        action='store_true',
        help="Drop into gdb on bench failure but stop at the beginning "
            "of the failing bench case.")
    bench_parser.add_argument(
        '--gdb-main',
        action='store_true',
        help="Drop into gdb on bench failure but stop at the beginning "
            "of main.")
    bench_parser.add_argument(
        '--gdb-path',
        type=lambda x: x.split(),
        default=GDB_PATH,
        help="Path to the gdb executable, may include flags. "
            "Defaults to %r." % GDB_PATH)
    bench_parser.add_argument(
        '--exec',
        type=lambda e: e.split(),
        help="Run under another executable.")
    bench_parser.add_argument(
        '--valgrind',
        action='store_true',
        help="Run under Valgrind to find memory errors. Implicitly sets "
            "--isolate.")
    bench_parser.add_argument(
        '--valgrind-path',
        type=lambda x: x.split(),
        default=VALGRIND_PATH,
        help="Path to the Valgrind executable, may include flags. "
            "Defaults to %r." % VALGRIND_PATH)
    bench_parser.add_argument(
        '-p', '--perf',
        help="Run under Linux's perf to sample performance counters, writing "
            "samples to this file.")
    bench_parser.add_argument(
        '--perf-freq',
        help="perf sampling frequency. This is passed directly to the perf "
            "script.")
    bench_parser.add_argument(
        '--perf-period',
        help="perf sampling period. This is passed directly to the perf "
            "script.")
    bench_parser.add_argument(
        '--perf-events',
        help="perf events to record. This is passed directly to the perf "
            "script.")
    bench_parser.add_argument(
        '--perf-script',
        type=lambda x: x.split(),
        default=PERF_SCRIPT,
        help="Path to the perf script to use. Defaults to %r." % PERF_SCRIPT)
    bench_parser.add_argument(
        '--perf-path',
        type=lambda x: x.split(),
        help="Path to the perf executable, may include flags. This is passed "
            "directly to the perf script")

    # compilation flags
    comp_parser = parser.add_argument_group('compilation options')
    comp_parser.add_argument(
        'bench_paths',
        nargs='*',
        help="Description of *.toml files to compile. May be a directory "
            "or a list of paths.")
    comp_parser.add_argument(
        '-c', '--compile',
        action='store_true',
        help="Compile a bench suite or source file.")
    comp_parser.add_argument(
        '-s', '--source',
        help="Source file to compile, possibly injecting internal benches.")
    comp_parser.add_argument(
        '--include',
        default=HEADER_PATH,
        help="Inject this header file into every compiled bench file. "
            "Defaults to %r." % HEADER_PATH)
    comp_parser.add_argument(
        '-o', '--output',
        help="Output file.")

    # runner/bench_paths overlap, so need to do some munging here
    args = parser.parse_intermixed_args()
    args.bench_paths = [' '.join(args.runner or [])] + args.bench_ids
    args.runner = args.runner or [RUNNER_PATH]

    sys.exit(main(**{k: v
        for k, v in vars(args).items()
        if v is not None}))
