#!/usr/bin/env python

"""
This script is intended to be used to run make under supervision for
the purpose of capturing its stdout/stderr and retcode to generate
a JUnit XML report. This report can be used in Jenkins or any other CI.
This report contains one test suite and as many cases as many targets
are specified.

To get help on usage, possible options and their descriptions, use
the following command:

    make.py --help

An example of running this script for building a demo project:

    make.py -C demos/STM32/RT-STM32WB55RG-NUCLEO68 -f Makefile all

This script requires the following packages to be installed:

    pip install junit-xml
    pip install pyyaml
    pip install requests
"""

import argparse
import collections
import os
import re
import subprocess
import sys
import time
from xml.etree import ElementTree

import junit_xml
import requests
import yaml


def get_jobserver_auth_fds():
    """Get "jobserver" file descriptors from MAKEFLAGS."""
    make_flags = os.environ.get('MAKEFLAGS', '')
    jobserver_auth_flag = '--jobserver-auth='

    for make_flag in make_flags.split():
        if make_flag.startswith(jobserver_auth_flag):
            flag_value = make_flag[len(jobserver_auth_flag):]
            # Starting from GNU make 4.4 the value has the "fifo:PATH" format
            # where PATH is the filesystem path to a named pipe (FIFO). In this
            # case a subprocess will be able to open this file and there is no
            # need to extract and pass FDs into it.
            if not flag_value.startswith('fifo:') and ',' in flag_value:
                fdr, _, fdw = flag_value.partition(',')
                return (int(fdr), int(fdw))

    return ()


class BaseRules:
    def __init__(self, filename):
        self.filename = filename
        self._rules = None

    def load(self):
        with open(self.filename) as fd:
            self._rules = yaml.safe_load(fd)

    @property
    def rules(self):
        if self._rules is None:
            self.load()
        return self._rules


class SkipRules(BaseRules):
    def match(self, path, stderr):
        for rule in self.rules:
            if (re.match(rule['path'], path)
                    and re.search(rule['stderr'], stderr)):
                return True, rule['reason']
        return False, None


class TargetRules(BaseRules):
    def match(self, path):
        for rule, targets in self.rules.items():
            if re.match(rule, path):
                return targets
        return None


def get_params(args):
    directory = args.directory or ''
    makefile = args.makefile or ''

    path = os.path.join(directory, makefile)
    if args.prefix:
        assert path.startswith(args.prefix)
        path = path[len(args.prefix):].lstrip(os.path.sep)

    return Params(path, directory, makefile)

Params = collections.namedtuple('Params', [
    'path',
    'directory',
    'makefile',
])


def get_targets(args, path):
    if args.target_rules:
        target_rules = TargetRules(args.target_rules)
        targets = target_rules.match(path)
        if targets is not None:
            return targets
    return args.targets


def make(args):
    if args.skip_rules:
        skip_rules = SkipRules(args.skip_rules)

    params = get_params(args) 

    package = params.path.partition(os.path.sep)[0]
    suite = junit_xml.TestSuite(
        params.path,
        timestamp=time.time(),
        file=params.path,
        package=package,
    )

    cmd = ['/usr/bin/env', 'make']
    if params.directory:
        cmd.extend(['-C', params.directory])
    if params.makefile:
        cmd.extend(['-f', params.makefile])
    if args.jobs > 1:
        cmd.extend(['-j', str(args.jobs)])

    jobserver_auth_fds = get_jobserver_auth_fds()
    targets = get_targets(args, params.path)

    for target in targets:
        if not target:
            continue

        cmd_target = cmd + [target]

        timestamp=time.time()
        start = time.monotonic()
        # To better control the use of resources by a subprocess make, pass
        # jobserver auth file descriptors in the subprocess, so it will
        # participate in the jobserver protocol and schedule its jobs
        # accordingly.
        ret = subprocess.run(cmd_target, pass_fds=jobserver_auth_fds,
                             capture_output=True, text=True)
        end = time.monotonic()

        case_name = 'make ' + target
        case_log = ' '.join(cmd_target)
        case_classname = params.path
        case = junit_xml.TestCase(
            case_name,
            classname=case_classname,
            file=params.path,
            log=case_log,
            timestamp=timestamp,
            elapsed_sec=(end - start),
            stdout=ret.stdout,
            stderr=ret.stderr,
        )
        suite.test_cases.append(case)

        if ret.returncode != 0:
            skipped, msg = skip_rules.match(params.path, ret.stderr)
            if skipped:
                case.add_skipped_info(msg)
                continue

            msg = 'Ended with non-zero exit code: {}'.format(ret.returncode)
            case.add_failure_info(msg)

    if args.result == '-':
        suite.to_file(sys.stdout, [suite])
    else:
        test_result_path = os.path.join(args.result, params.path) + '.xml'
        with open(test_result_path, 'w') as f:
            suite.to_file(f, [suite])


def check_skip_rules(args):
    def get_signature(content):
        root = ElementTree.fromstring(content)
        return (
            root.findtext('className'),
            root.findtext('stderr'),
        )

    if not args.skip_rules:
        sys.stderr.write('--skip-rules is required\n')
        sys.exit(2)

    skip_rules = SkipRules(args.skip_rules)

    url = args.check_skip_rules
    if not url.endswith('/api/xml'):
        url += '/api/xml'

    resp = requests.get(url)
    if resp.status_code != 200:
        sys.stderr.write('Got unexpected response {}\n'
                         .format(resp.status_code))
        sys.exit(3)

    path, stderr = get_signature(resp.content)
    if not (path and stderr):
        sys.stderr.write('Cannot find path and stderr\n')
        sys.exit(4)

    matched, msg = skip_rules.match(path, stderr)
    if matched:
        sys.stdout.write('Matched rule: {}\n'.format(msg))
    else:
        sys.stdout.write('Not matched!\n')
        sys.exit(1)


def check_target_rules(args):
    if not args.target_rules:
        sys.stderr.write('--target-rules is required\n')
        sys.exit(1)

    params = get_params(args) 
    targets = get_targets(args, params.path)
    sys.stdout.write(' '.join(targets))


def main():
    parser = argparse.ArgumentParser(description=(
        'Execute make targets and generate JUnit results'
    ))
    # Make specific arguments
    parser.add_argument('-C', '--directory',
                        help='Change directory to dir (equivalent to '
                             '`make -C dir`)')
    parser.add_argument('-f', '--makefile',
                        help='Use file as a makefile (equivalent to '
                             '`make -f file`)')
    parser.add_argument('-j', '--jobs',
                        type=int, default=1,
                        help='Number of simultaneous jobs (equivalent to '
                             '`make -j jobs`)')
    # Test specific arguments
    parser.add_argument('-r', '--result',
                        default='-',
                        help='Directory to store JUnit XML test result')
    parser.add_argument('-p', '--prefix',
                        help=('Prefix path which should be removed for test '
                              'result'))
    parser.add_argument('-s', '--skip-rules',
                        help='YAML-file with skip rules')
    parser.add_argument('-S', '--check-skip-rules',
                        help='URL to a test result in Jenkins job')
    parser.add_argument('-t', '--target-rules',
                        help='YAML-file with target rules')
    parser.add_argument('-T', '--check-target-rules', action='store_true',
                        help='Print target rules for a given makefile')
    parser.add_argument('targets', metavar='target', nargs='*',
                        default=['all', 'clean'],
                        help='Names of targets to run')

    args = parser.parse_args()

    if args.check_skip_rules:
        check_skip_rules(args)
    elif args.check_target_rules:
        check_target_rules(args)
    else:
        make(args)


if __name__ == '__main__':
    main()
